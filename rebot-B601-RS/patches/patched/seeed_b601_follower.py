import logging
import math
import time
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any

from lerobot.cameras import CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import MotorCalibration
from motorbridge import Controller as MotorBridgeController, Mode as MotorBridgeMode
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position


@dataclass
class SeeedB601FollowerConfigBase:
    """Base configuration for the Seeed B601 Follower arm."""

    # Communication port for CAN adapter (e.g., "can0" for SocketCAN, or "/dev/ttyACM0" for Damiao serial bridge)
    port: str
    
    # CAN adapter type:
    #   "socketcan"  - SocketCAN based adapters (PCAN, slcan, embedded can controller, etc.)
    #   "damiao"     - Damiao dedicated serial bridge
    #   "robstride"  - RobStride dedicated adapter (placeholder, not yet supported)
    can_adapter: str = "socketcan"

    # Baud rate for Damiao serial bridge (only used when can_adapter="damiao")
    dm_serial_baud: int = 921600

    disable_torque_on_disconnect: bool = True

    # Max relative target for joint movements, in degrees
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    
    # Motor configuration must be provided by concrete subclasses.
    # Maps motor names to (send_can_id, recv_can_id)
    motor_can_ids: dict[str, tuple[int, int]] = field(default_factory=dict)

    # Control parameters are defined by concrete subclasses so different motor families
    # can keep their own defaults.
    ## Default target velocity for joints running in POS_VEL mode, in degrees/s.
    pos_vel_velocity: float | list[float] = field(default_factory=list)

    ## Default torque/current ration for gripper's FORCE_POS mode, in range [0,1].
    force_pos_torque_ration: float = 0.1

    # Soft joint limits in degrees. Concrete subclasses should define defaults.
    joint_limits: dict[str, tuple[float, float]] = field(default_factory=dict)

    # Per-joint action direction/scale applied before joint-limit clipping.
    # Use -1 for sign flip, 1 for no flip, and other values when scaling is required.
    joint_directions: dict[str, float] = field(default_factory=dict)

    # Gravity-compensation feedforward added to MIT tau for the 6 arm joints.
    # Disabled by default; enable in the concrete follower config (e.g. RS).
    gravity_compensation: bool = False
    # URDF for gravity calc. Set by concrete subclasses (e.g. RS follower).
    gravity_urdf_path: str | None = None

    # Temperature protection thresholds (degrees Celsius, read from each motor's t_mos).
    # Concrete subclasses may override per motor family.
    temp_alarm_threshold_c: float = 80.0              # print HIGH TEMP warning above this
    temp_overheat_threshold_c: float = 100.0           # raise KeyboardInterrupt above this
    temp_emergency_disable_threshold_c: float = 135.0  # safe_zero emergency disable-torque limit


logger = logging.getLogger(__name__)


FOLLOWER_GRIPPER_MOTOR = "gripper"
LONG_TIMEOUT_SEC = 0.1
MEDIUM_TIMEOUT_SEC = 0.01

# ---------------------------------------------------------------------------
# PATCH (Humanoid Project): software collision guard for the 6 arm joints.
# Enabled only when env COLLISION_GUARD=1 (teleop.sh sets it). Pure logic,
# no motor I/O, so it can be unit-tested offline.
#
# Why not "|last sent goal - present| > 10 deg"? With max_relative_target=5
# the sent goal is always <= 5 deg from the present position, so that error
# can never exceed ~5 deg - and real sessions show ~4 deg in normal fast
# motion. Instead a collision is: the leader is far away from the follower
# (|mapped leader goal - present| > threshold) AND the follower makes no
# progress towards it (< min_progress deg per cycle) for N cycles.
# Optional torque trigger per joint (COLLISION_TORQ_<JOINT>, default off), with
# its own consecutive-cycle counter (COLLISION_TORQ_CYCLES, default 4).
# guard v2: MIT_KP_<JOINT>/MIT_KD_<JOINT> env overrides make single joints
# compliant (teleop.sh: shoulder_pan + wrist_yaw kp 20) so a hand can stall them.
# guard v3: soft auto-resume (COLLISION_AUTO_RESUME=1): after the frozen arm has
# stood still for COLLISION_AUTO_RESUME_S seconds it creeps towards the leader at
# COLLISION_RESUME_STEP_DEG per cycle.
# guard v3.1: creep re-freezes only on a real obstacle (lag > COLLISION_CREEP_LAG
# beyond the start sag AND no progress for COLLISION_CREEP_CYCLES=10, or torque);
# per-joint creep lead (4 deg for kp<=40 joints, else 2.5); never a dead end:
# retries after 1.5/3/5/5... s; only the collided joint must be brought back.
# Torque trigger optionally needs lead >= COLLISION_TORQ_MIN_LEAD_DEG and progress
# < COLLISION_TORQ_MAX_PROGRESS_DEG.
# guard v3.2: gravity-aware. The expected gravity torque g(q) per joint comes from the
# Seeed URDF (gravity.py, validated: lift 152/elbow 120 -> model -14.2 Nm, measured
# -14.9 Nm). Torque triggers use the residual |measured - g(q)|. The lag ("blockiert")
# triggers get a load-dependent tolerance: + COLLISION_GRAV_LAG_GAIN * g_unc/kp (deg)
# when the joint lags against gravity, and are skipped when the uncompensated gravity
# exceeds COLLISION_GRAV_SAT_FRAC of what kp can deliver inside max_relative_target
# (the joint physically cannot follow; freezing would not help). COLLISION_GRAVITY=0
# switches this off (exact v3.1 behaviour). Optional feedforward GRAVITY_FF=1 adds
# GRAVITY_FF_SCALE (default 0.8) * g(q) as MIT torque feedforward (default off).
# guard v3.3: (a) gravity-explained stall: for COLLISION_GRAV_RES_JOINTS (default shoulder_lift,
# elbow_flex; teleop.sh adds wrist_flex) a lag AGAINST gravity is not a collision when the joint pushes less
# than COLLISION_GRAV_RES_<JOINT> / COLLISION_GRAV_RES_NM (default 4.0; teleop.sh: wrist_flex 1.5) beyond the
# expected gravity torque in the direction of motion
# (measured torque minus g). That is exactly the force such a stall can put on an obstacle, so a
# lift that is only too weak for the load (11 Nm gravity vs 13 Nm kp*5deg) no longer freezes, while a
# real block with feedforward on (push ~ kp*5deg - 0.2*g >= 10 Nm) still does. (b) GRAVITY_FF feedforward
# ramps in over GRAVITY_FF_RAMP_S (default 1 s) and is capped tighter (lift 14, elbow 9, wrist_flex 3,
# rotary joints 1 Nm; logged max model torque today: lift 11.4, elbow 7.1, wrist_flex 2.4 Nm).
# teleop.sh v3.7 turns GRAVITY_FF=1 on by default.
# ---------------------------------------------------------------------------
_COLLISION_WRIST_JOINTS = ("wrist_flex", "wrist_yaw", "wrist_roll")


def _env_float(name: str, default: float) -> float:
    import os as _os
    try:
        v = _os.environ.get(name, "")
        return float(v) if v.strip() != "" else default
    except ValueError:
        return default


_MIT_GAIN_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll")


def mit_gain_overrides():
    """PATCH (Humanoid Project, guard v2): per-joint MIT gain overrides from env
    MIT_KP_<JOINT> / MIT_KD_<JOINT> (e.g. MIT_KP_SHOULDER_PAN=20). Returns
    (kp_dict, kd_dict) containing only joints with a valid value >= 0."""
    kp, kd = {}, {}
    for j in _MIT_GAIN_JOINTS:
        v = _env_float(f"MIT_KP_{j.upper()}", -1.0)
        if v >= 0:
            kp[j] = v
        v = _env_float(f"MIT_KD_{j.upper()}", -1.0)
        if v >= 0:
            kd[j] = v
    return kp, kd


def collision_guard_enabled() -> bool:
    import os as _os
    return _os.environ.get("COLLISION_GUARD", "0").strip().lower() in ("1", "true", "on", "yes")


def _env_on(name: str, default: str = "0") -> bool:
    import os as _os
    return _os.environ.get(name, default).strip().lower() in ("1", "true", "on", "yes")


def gravity_ff_enabled() -> bool:
    """PATCH (guard v3.2): GRAVITY_FF=1 adds scaled gravity torque as MIT feedforward."""
    return _env_on("GRAVITY_FF", "0")


def gravity_ff_scale() -> float:
    return max(0.0, min(1.0, _env_float("GRAVITY_FF_SCALE", 0.8)))


def guard_gravity_enabled() -> bool:
    """PATCH (guard v3.2): COLLISION_GRAVITY=0 disables the gravity-aware guard (v3.1 behaviour)."""
    return _env_on("COLLISION_GRAVITY", "1")


_GRAVITY_MOD = None


def _gravity_module():
    """PATCH (guard v3.2): the plugin's gravity.py (package import, or the sibling file when this
    module is loaded standalone, e.g. by the offline tests)."""
    global _GRAVITY_MOD
    if _GRAVITY_MOD is None:
        try:
            from . import gravity as _m
        except ImportError:
            import importlib.util as _ilu, os as _os
            _p = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "gravity.py")
            _spec = _ilu.spec_from_file_location("b601_gravity", _p)
            _m = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_m)
        _GRAVITY_MOD = _m
    return _GRAVITY_MOD


# Hard cap for the gravity feedforward per joint (N·m).
# guard v3.3: tighter caps (model max in today's logs: lift 11.4, elbow 7.1, wrist_flex 2.4 Nm; the rotary
# joints have ~0 gravity torque, so anything above 1 Nm there would be a model error).
GRAVITY_FF_MAX_NM = {"shoulder_pan": 1.0, "shoulder_lift": 14.0, "elbow_flex": 9.0,
                     "wrist_flex": 3.0, "wrist_yaw": 1.0, "wrist_roll": 1.0}


def gravity_ff_ramp_s() -> float:
    """guard v3.3: seconds over which the gravity feedforward ramps from 0 to full after connect."""
    return max(0.0, _env_float("GRAVITY_FF_RAMP_S", 1.0))


_GRAV_RES_JOINTS_DEFAULT = ("shoulder_lift", "elbow_flex")


def _grav_res_joints_env():
    import os as _os
    v = _os.environ.get("COLLISION_GRAV_RES_JOINTS", "")
    return tuple(x.strip() for x in v.split(",") if x.strip()) if v.strip() else _GRAV_RES_JOINTS_DEFAULT


JOINT_DE = {
    "shoulder_pan": "Schulter drehen (links/rechts)",
    "shoulder_lift": "Schulter heben/senken",
    "elbow_flex": "Ellbogen",
    "wrist_flex": "Handgelenk hoch/runter",
    "wrist_yaw": "Handgelenk drehen",
    "wrist_roll": "Handgelenk rollen",
}


class CollisionGuard:
    """Detect blocked arm joints during teleop and freeze the arm (v3.3, gravity-aware)."""

    def __init__(self, joints, err_deg=None, cycles=6, min_progress_deg=0.15,
                 grace_s=2.0, resume_deg=6.0, resume_cycles=10, torq_limit=None,
                 stats_period_s=5.0, warn_period_s=1.0, log=None, torq_cycles=4,
                 auto_resume=False, auto_resume_s=1.5, resume_step_deg=1.0, auto_resume_max=0,
                 creep_cycles=10, still_deg=0.3, creep_lead_deg=2.5, creep_lag_deg=2.0,
                 retry_waits=(3.0, 5.0), torq_min_lead_deg=0.0, torq_max_progress_deg=1e9,
                 kp=None, clamp_deg=None, ff_scale=0.0, grav_lag_gain=1.5, grav_sat_frac=0.85,
                 grav_min_nm=0.3, grav_res_nm=4.0, grav_res_joints=_GRAV_RES_JOINTS_DEFAULT):
        self.joints = list(joints)
        # v3.3 gravity-explained stall (see header): push beyond g below grav_res_nm -> no collision
        if isinstance(grav_res_nm, dict):
            self.grav_res_nm = {j: float(grav_res_nm.get(j, 4.0)) for j in self.joints}
        else:
            self.grav_res_nm = {j: float(grav_res_nm) for j in self.joints}
        self.grav_res_joints = tuple(grav_res_joints or ())
        self.n_grav_explained = 0
        self._grav_expl_logged = set()
        # v3.2 gravity awareness (only active when update() gets a grav dict)
        kp = kp or {}
        self.kp = {j: float(kp.get(j, 0.0)) for j in self.joints}            # Nm/rad
        if isinstance(clamp_deg, dict):
            self.clamp_deg = {j: float(clamp_deg.get(j, 1e9)) for j in self.joints}
        else:
            self.clamp_deg = {j: float(clamp_deg if clamp_deg is not None else 1e9) for j in self.joints}
        self.ff_scale = max(0.0, min(1.0, float(ff_scale)))
        self.grav_lag_gain = max(0.0, float(grav_lag_gain))
        self.grav_sat_frac = float(grav_sat_frac)
        self.grav_min_nm = float(grav_min_nm)
        self.grav = {}
        self.sat_warned = set()
        err_deg = err_deg or {}
        torq_limit = torq_limit or {}
        self.err_deg = {j: float(err_deg.get(j, 15.0 if j in _COLLISION_WRIST_JOINTS else 10.0)) for j in self.joints}
        self.torq_limit = {j: float(torq_limit.get(j, 0.0)) for j in self.joints}
        self.cycles = int(cycles)
        self.min_progress_deg = float(min_progress_deg)
        self.grace_s = float(grace_s)
        self.resume_deg = float(resume_deg)
        self.resume_cycles = int(resume_cycles)
        self.stats_period_s = float(stats_period_s)
        self.warn_period_s = float(warn_period_s)
        self.torq_cycles = max(1, int(torq_cycles))
        self.torq_min_lead_deg = float(torq_min_lead_deg)
        self.torq_max_progress_deg = float(torq_max_progress_deg)
        self.auto_resume = bool(auto_resume)
        self.auto_resume_s = float(auto_resume_s)
        self.retry_waits = tuple(float(w) for w in retry_waits)
        self.resume_step_deg = max(0.05, float(resume_step_deg))
        self.auto_resume_max = int(auto_resume_max)      # 0 = unbegrenzt (v3.1: kein Sackgassen-Stopp)
        self.creep_cycles = max(1, int(creep_cycles))
        self.still_deg = float(still_deg)
        def _per_joint(v):
            return {j: float(v.get(j, 2.5)) for j in self.joints} if isinstance(v, dict) else {j: float(v) for j in self.joints}
        self.creep_lead = _per_joint(creep_lead_deg)
        self.creep_lag = _per_joint(creep_lag_deg)
        for j in self.joints:   # lag threshold must be reachable within the lead cap
            self.creep_lag[j] = min(self.creep_lag[j], 0.8 * self.creep_lead[j])
        self.creep_gap0 = {}
        self.log = log or logger
        self.reset()

    @classmethod
    def from_env(cls, joints, log=None, gains=None, clamp_deg=None, ff_scale=0.0):
        err, tq, lead, lag = {}, {}, {}, {}
        for j in joints:
            J = j.upper()
            base = _env_float("COLLISION_ERR_WRIST_DEG", 15.0) if j in _COLLISION_WRIST_JOINTS else _env_float("COLLISION_ERR_DEG", 10.0)
            err[j] = _env_float(f"COLLISION_ERR_{J}", base)
            tq[j] = _env_float(f"COLLISION_TORQ_{J}", 0.0)
            kp = _env_float(f"MIT_KP_{J}", -1.0)
            compliant = 0 <= kp <= 40
            lead[j] = _env_float(f"COLLISION_CREEP_LEAD_{J}", _env_float("COLLISION_CREEP_LEAD_DEG", 4.0 if compliant else 2.5))
            lag[j] = _env_float(f"COLLISION_CREEP_LAG_{J}", 3.0 if compliant else 2.0)
        return cls(
            joints, err_deg=err, torq_limit=tq,
            cycles=int(_env_float("COLLISION_CYCLES", 6)),
            torq_cycles=int(_env_float("COLLISION_TORQ_CYCLES", 4)),
            torq_min_lead_deg=_env_float("COLLISION_TORQ_MIN_LEAD_DEG", 0.0),
            torq_max_progress_deg=_env_float("COLLISION_TORQ_MAX_PROGRESS_DEG", 1e9),
            auto_resume=_env_float("COLLISION_AUTO_RESUME", 0.0) > 0,
            auto_resume_s=_env_float("COLLISION_AUTO_RESUME_S", 1.5),
            resume_step_deg=_env_float("COLLISION_RESUME_STEP_DEG", 1.0),
            auto_resume_max=int(_env_float("COLLISION_AUTO_RESUME_MAX", 0)),
            creep_cycles=int(_env_float("COLLISION_CREEP_CYCLES", 10)),
            creep_lead_deg=lead, creep_lag_deg=lag,
            min_progress_deg=_env_float("COLLISION_MIN_PROGRESS_DEG", 0.15),
            grace_s=_env_float("COLLISION_GRACE_S", 2.0),
            resume_deg=_env_float("COLLISION_RESUME_DEG", 6.0),
            resume_cycles=int(_env_float("COLLISION_RESUME_CYCLES", 10)),
            stats_period_s=_env_float("COLLISION_STATS_S", 5.0),
            kp=gains, clamp_deg=clamp_deg, ff_scale=ff_scale,
            grav_lag_gain=_env_float("COLLISION_GRAV_LAG_GAIN", 1.5),
            grav_sat_frac=_env_float("COLLISION_GRAV_SAT_FRAC", 0.85),
            grav_res_nm={j: _env_float(f"COLLISION_GRAV_RES_{j.upper()}", _env_float("COLLISION_GRAV_RES_NM", 4.0)) for j in joints},
            grav_res_joints=_grav_res_joints_env(),
            log=log,
        )

    def describe(self) -> str:
        thr = ", ".join(f"{j}={self.err_deg[j]:g}" for j in self.joints)
        tq = ", ".join(f"{j}={v:g}" for j, v in self.torq_limit.items() if v > 0)
        if tq:
            tq = f"[{tq}] Nm fuer {self.torq_cycles} Zyklen"
            if self.torq_min_lead_deg > 0:
                tq += f" (nur bei Abweichung >={self.torq_min_lead_deg:g} deg"
                tq += f" und Fortschritt <{self.torq_max_progress_deg:g} deg/Zyklus)" if self.torq_max_progress_deg < 1e8 else ")"
        else:
            tq = "aus"
        if self.auto_resume:
            waits = ", ".join(f"{w:g}" for w in (self.auto_resume_s,) + self.retry_waits)
            lead = ", ".join(f"{j}={self.creep_lead[j]:g}/{self.creep_lag[j]:g}" for j in self.joints)
            ar = (f"Auto-Weiter nach {self.auto_resume_s:g} s Ruhe mit max {self.resume_step_deg:g} deg/Zyklus "
                  f"(Wartezeiten {waits} s, dann alle {self.retry_waits[-1] if self.retry_waits else self.auto_resume_s:g} s; "
                  f"Vorlauf/Haenge-Schwelle [{lead}] deg, {self.creep_cycles} Zyklen"
                  + (f", max {self.auto_resume_max} Versuche" if self.auto_resume_max > 0 else ", unbegrenzt") + ")")
        else:
            ar = "Auto-Weiter: aus"
        if any(v > 0 for v in self.kp.values()):
            ar += (f"; Schwerkraft-Modell: Rest-Drehmoment fuer Kraft-Trigger, Durchhang-Toleranz "
                   f"{self.grav_lag_gain:g}x g/kp, Feedforward-Anteil {self.ff_scale:g}")
            if self.grav_res_joints:
                ar += ("; v3.3 Schwerkraft-Stillstand: kein Alarm, solange das Gelenk weniger als ["
                       + ", ".join(f"{j}={self.grav_res_nm.get(j, 0.0):g}" for j in self.grav_res_joints if j in self.grav_res_nm)
                       + "] Nm ueber der Schwerkraft drueckt")
        return (f"Kollisionsschutz AKTIV: Leader-Abweichung >[{thr}] deg und Fortschritt "
                f"<{self.min_progress_deg:g} deg/Zyklus fuer {self.cycles} Zyklen; Drehmoment-Trigger: {tq}; "
                f"Anlaufpause {self.grace_s:g} s; Freigabe wenn Leader <{self.resume_deg:g} deg fuer {self.resume_cycles} Zyklen; " + ar)

    def reset(self):
        self.t0 = None
        self.frozen = None
        self.prev_present = None
        self.last_sent = None
        self.count = {j: 0 for j in self.joints}
        self.tq_count = {j: 0 for j in self.joints}
        self.resume_ok = 0
        self.last_warn = -1e9
        self.creep = None          # creeping targets while auto-resuming
        self.still_t0 = None
        self.auto_attempts = 0     # consecutive creeps that ended in a new block
        self.hit_joint = None
        self.n_collisions = 0
        self._reset_stats(None)

    def _reset_stats(self, now):
        self.stats_t = now
        self.st_trk = {j: 0.0 for j in self.joints}
        self.st_disc = {j: 0.0 for j in self.joints}
        self.st_tq = {j: 0.0 for j in self.joints}
        self.st_res = {j: 0.0 for j in self.joints}
        self.st_g = {j: 0.0 for j in self.joints}

    def note_sent(self, sent):
        """Record the goals actually sent this cycle (after max_relative_target)."""
        self.last_sent = {j: float(sent[j]) for j in self.joints if j in sent}

    def _stats(self, now, present, goal, torq):
        if self.stats_t is None:
            self._reset_stats(now)
        for j in self.joints:
            if j in present and self.last_sent and j in self.last_sent:
                self.st_trk[j] = max(self.st_trk[j], abs(self.last_sent[j] - present[j]))
            if j in present and j in goal:
                self.st_disc[j] = max(self.st_disc[j], abs(goal[j] - present[j]))
            if torq and torq.get(j) is not None:
                self.st_tq[j] = max(self.st_tq[j], abs(torq[j]))
                self.st_res[j] = max(self.st_res[j], abs(torq[j] - self.grav.get(j, 0.0)))
            if j in self.grav:
                self.st_g[j] = max(self.st_g[j], abs(self.grav[j]))
        if now - self.stats_t >= self.stats_period_s:
            self.log.info("Guard-Stats %gs (max): %s", self.stats_period_s, " | ".join(
                f"{j} trk={self.st_trk[j]:.1f} lead={self.st_disc[j]:.1f} tq={self.st_tq[j]:.2f}"
                + (f" g={self.st_g[j]:.2f} res={self.st_res[j]:.2f}" if self.grav else "") for j in self.joints))
            self._reset_stats(now)

    def _wait_s(self):
        """Wait before the next auto-resume: 1.5 s, then 3 s, 5 s, then every 5 s."""
        waits = (self.auto_resume_s,) + self.retry_waits
        return waits[min(self.auto_attempts, len(waits) - 1)]

    def _grav_unc(self, j):
        """Gravity torque the P-term must still deliver (the feedforward covers ff_scale of it)."""
        return self.grav.get(j, 0.0) * (1.0 - self.ff_scale)

    def _grav_tol(self, j, disc):
        """v3.2: (extra lag tolerance in deg, saturated) for a joint lagging by disc deg.
        Only when the joint lags AGAINST gravity (disc has the sign of the holding torque g):
        expected static sag = g_unc/kp; saturated = kp cannot deliver g_unc within the
        max_relative_target clamp, so the joint cannot follow at all (freezing would not help)."""
        gu = self._grav_unc(j)
        kp = self.kp.get(j, 0.0)
        if kp <= 0 or abs(gu) < self.grav_min_nm or disc == 0 or (disc > 0) != (gu > 0):
            return 0.0, False
        avail = kp * math.radians(min(self.clamp_deg.get(j, 1e9), 1e6))
        sat = abs(gu) >= self.grav_sat_frac * avail
        return self.grav_lag_gain * math.degrees(abs(gu) / kp), sat

    def _note_sat(self, j, gu):
        if j not in self.sat_warned:
            self.sat_warned.add(j)
            self.log.warning("Schwerkraft: %s braucht %.1f Nm Haltemoment – mehr als kp im 5-deg-Limit schafft; "
                             "Durchhang wird nicht als Kollision gewertet (Tipp: GRAVITY_FF=1)", JOINT_DE.get(j, j), abs(gu))

    def _tq_hit(self, j, tq, lead, progress):
        if tq is not None and self.grav:
            tq = tq - self.grav.get(j, 0.0)       # v3.2: residual torque (measured - gravity)
        if self.torq_limit[j] <= 0 or tq is None or abs(tq) <= self.torq_limit[j]:
            return False
        return abs(lead) >= self.torq_min_lead_deg and progress < self.torq_max_progress_deg

    def _grav_explained(self, j, disc, tq):
        """v3.3: True when a lag AGAINST gravity is explained by gravity alone: the joint pushes less than
        grav_res_nm beyond the expected gravity torque g in the direction of the lag. That residual push is
        the most such a stall can press on an obstacle, so freezing would add nothing (and with gravity
        feedforward on, a real block shows a push of ~kp*clamp, far above grav_res_nm)."""
        if (self.grav_res_nm.get(j, 0.0) <= 0 or j not in self.grav_res_joints or tq is None or j not in self.grav
                or self.kp.get(j, 0.0) <= 0):
            return False
        g = self.grav[j]
        if abs(g) < self.grav_min_nm or disc == 0 or (disc > 0) != (g > 0):
            return False
        push = (tq - g) * (1.0 if disc > 0 else -1.0)
        return push < self.grav_res_nm[j]

    def _note_grav_explained(self, j, disc, tq):
        self.n_grav_explained += 1
        if j not in self._grav_expl_logged:
            self._grav_expl_logged.add(j)
            g = self.grav.get(j, 0.0)
            self.log.warning("Schwerkraft: %s haengt %.1f deg hinterher, drueckt aber nur %.1f Nm ueber der Schwerkraft "
                             "(g=%.1f Nm, gemessen %.1f Nm) -> keine Kollision (v3.3)", JOINT_DE.get(j, j), abs(disc),
                             (tq - g) * (1.0 if disc > 0 else -1.0), g, tq)

    def _freeze(self, present, now, j, disc, trk, tq, progress, why):
        self.frozen = {k: float(present[k]) for k in self.joints if k in present}
        self.creep = None
        self.still_t0 = None
        self.hit_joint = j
        self.n_collisions += 1
        self.resume_ok = 0
        self.last_warn = now
        self.log.warning("KOLLISION erkannt: %s err=%.1f deg (Leader-Abweichung) track=%.1f deg torq=%s Nm "
                         "fortschritt=%.2f deg/Zyklus [%s] -> Arm eingefroren (%s).",
                         j, disc, trk, "n/a" if tq is None else f"{tq:.2f}", progress, why, JOINT_DE.get(j, j))
        if self.grav:
            g = self.grav.get(j, 0.0)
            self.log.info("Schwerkraft beim Einfrieren: %s g=%.2f Nm, Rest-Drehmoment=%s Nm", j, g,
                          "n/a" if tq is None else f"{tq - g:.2f}")
        return dict(self.frozen)

    def _clear_counts(self, present):
        self.count = {j: 0 for j in self.joints}
        self.tq_count = {j: 0 for j in self.joints}
        self.prev_present = dict(present)

    def _start_creep(self, goal, present, why):
        self.creep = dict(self.frozen)
        # sag at creep start (frozen command vs present); only lag beyond it counts
        self.creep_gap0 = {k: self.frozen[k] - present[k] for k in self.frozen if k in present}
        self.frozen = None
        self.still_t0 = None
        self.resume_ok = 0
        self._clear_counts(present)
        self.log.warning("Auto-Weiter: Arm faehrt langsam weiter zum Leader (max %.1f deg/Zyklus, %s) – Schutz bleibt aktiv",
                         self.resume_step_deg, why)
        return self._creep_step(goal, present)

    def update(self, goal, present, torq, now, grav=None):
        """goal/present in follower degrees (after directions+limits). grav (v3.2, optional):
        expected gravity holding torque per joint in N·m (motor convention). Returns a
        dict of overriding arm-joint targets while frozen or creeping, else None."""
        self.grav = dict(grav) if grav else {}
        if self.t0 is None:
            self.t0 = now
        self._stats(now, present, goal, torq)

        if self.frozen is not None:
            dev = {j: abs(goal.get(j, self.frozen[j]) - self.frozen[j]) for j in self.joints}
            hj = self.hit_joint if self.hit_joint in dev else max(dev, key=dev.get)
            if self.auto_resume:
                # v3.1: only the joint that collided has to come back; the rest follows slowly
                released = dev[hj] <= self.resume_deg
            else:
                released = max(dev.values()) <= self.resume_deg
            self.resume_ok = self.resume_ok + 1 if released else 0
            if self.resume_ok >= self.resume_cycles:
                self.auto_attempts = 0
                if self.auto_resume and max(dev.values()) > self.resume_deg:
                    return self._start_creep(goal, present, "Leader zurückgeführt")
                self.log.warning("Kollision aufgehoben, Teleop läuft weiter")
                self.frozen = None
                self.resume_ok = 0
                self.still_t0 = None
                self._clear_counts(present)
                return None
            wait = None
            if self.auto_resume and (self.auto_resume_max <= 0 or self.auto_attempts < self.auto_resume_max):
                prev = self.prev_present or present
                moving = max((abs(present[k] - prev[k]) for k in self.joints if k in present and k in prev), default=0.0)
                if moving <= self.still_deg:
                    if self.still_t0 is None:
                        self.still_t0 = now
                    wait = self._wait_s() - (now - self.still_t0)
                    if wait <= 0:
                        n = self.auto_attempts + 1
                        return self._start_creep(goal, present, f"Versuch {n}")
                else:
                    self.still_t0 = None
                    wait = self._wait_s()
            self.prev_present = dict(present)
            if now - self.last_warn >= self.warn_period_s:
                self.last_warn = now
                far = {j: d for j, d in dev.items() if d > self.resume_deg}
                show = hj if (self.auto_resume or not far) else max(far, key=far.get)
                auto = (f"Auto-Weiter in {max(0.0, wait):.1f} s, wenn der Arm ruhig steht. " if wait is not None else "")
                self.log.warning("KOLLISION aktiv – Arm steht. %sLeader: %s %.1f deg zurückführen "
                                 "(Freigabe unter %.0f deg)", auto, JOINT_DE.get(show, show), dev[show], self.resume_deg)
            return dict(self.frozen)

        if self.creep is not None:
            return self._creep_update(goal, present, torq, now)

        if now - self.t0 < self.grace_s or self.prev_present is None:
            self.prev_present = dict(present)
            return None

        hit = None
        for j in self.joints:
            if j not in present or j not in goal or j not in self.prev_present:
                self.count[j] = 0
                self.tq_count[j] = 0
                continue
            disc = goal[j] - present[j]
            progress = (present[j] - self.prev_present[j]) * (1.0 if disc >= 0 else -1.0)
            tq = torq.get(j) if torq else None
            extra, sat = self._grav_tol(j, disc)
            blocked = abs(disc) > self.err_deg[j] + extra and progress < self.min_progress_deg and not sat
            if sat and abs(disc) > self.err_deg[j]:
                self._note_sat(j, self._grav_unc(j))
            if blocked and self._grav_explained(j, disc, tq):     # v3.3
                blocked = False
                self._note_grav_explained(j, disc, tq)
            tq_hit = self._tq_hit(j, tq, disc, progress)
            self.count[j] = self.count[j] + 1 if blocked else 0
            self.tq_count[j] = self.tq_count[j] + 1 if tq_hit else 0
            by_block = self.count[j] >= self.cycles
            by_tq = self.tq_count[j] >= self.torq_cycles
            if (by_block or by_tq) and hit is None:
                trk = abs(self.last_sent[j] - present[j]) if self.last_sent and j in self.last_sent else float("nan")
                hit = (j, disc, trk, tq, progress, "blockiert" if by_block else "torque")
        self.prev_present = dict(present)
        if hit is None:
            return None
        return self._freeze(present, now, *hit)

    def _creep_step(self, goal, present=None):
        """Move the creeping targets max resume_step_deg towards the leader. The targets
        start at the frozen command (not at present), so gravity sag never makes a joint droop.
        A target never runs more than creep_lead[j] ahead of its joint (beyond the start sag):
        enough torque to beat friction (4 deg at kp 20), little extra force on an obstacle."""
        st = self.resume_step_deg
        for j in self.joints:
            if j in goal and j in self.creep:
                step = max(-st, min(st, goal[j] - self.creep[j]))
                if present is not None and j in present and step != 0:
                    sgn = 1.0 if step > 0 else -1.0
                    room = max(0.0, self.creep_gap0.get(j, 0.0) * sgn) + self.creep_lead[j] - (self.creep[j] - present[j]) * sgn
                    step = sgn * max(0.0, min(abs(step), room))
                self.creep[j] += step
        return dict(self.creep)

    def _creep_update(self, goal, present, torq, now):
        """Creeping towards the leader. Re-freeze only on a real obstacle: the joint lags
        its creep target by more than creep_lag[j] (beyond the start sag) AND makes no
        progress for creep_cycles, or the torque trigger fires."""
        hit = None
        for j in self.joints:
            if j not in present or j not in goal or j not in self.creep or j not in self.prev_present:
                self.count[j] = 0
                self.tq_count[j] = 0
                continue
            e = self.creep[j] - present[j]
            sgn = 1.0 if e >= 0 else -1.0
            lag = abs(e) - max(0.0, self.creep_gap0.get(j, 0.0) * sgn)
            progress = (present[j] - self.prev_present[j]) * sgn
            tq = torq.get(j) if torq else None
            extra, sat = self._grav_tol(j, e)
            blocked = lag >= self.creep_lag[j] + extra and progress < self.min_progress_deg and not sat
            if blocked and self._grav_explained(j, e, tq):        # v3.3
                blocked = False
                self._note_grav_explained(j, e, tq)
            tq_hit = self._tq_hit(j, tq, goal[j] - present[j], progress)
            self.count[j] = self.count[j] + 1 if blocked else 0
            self.tq_count[j] = self.tq_count[j] + 1 if tq_hit else 0
            by_block = self.count[j] >= self.creep_cycles
            if (by_block or self.tq_count[j] >= self.torq_cycles) and hit is None:
                hit = (j, goal[j] - present[j], abs(e), tq, progress,
                       "blockiert beim Weiterfahren" if by_block else "torque beim Weiterfahren")
        self.prev_present = dict(present)
        if hit is not None:
            self.auto_attempts += 1
            return self._freeze(present, now, *hit)
        if max((abs(goal[j] - self.creep[j]) for j in self.joints if j in goal and j in self.creep), default=0.0) <= self.resume_deg:
            self.log.warning("Kollision aufgehoben (Auto-Weiter), Teleop läuft weiter")
            self.creep = None
            self.auto_attempts = 0
            self._clear_counts(present)
            return None
        return self._creep_step(goal, present)

class SeeedB601FollowerBase(Robot):
    """
    Base class for Seeed B601 Follower Arms (DM and RS variants).
    Uses CAN bus communication via motorbridge.
    """

    motor_type: str = ""

    def __init__(self, config: SeeedB601FollowerConfigBase):
        super().__init__(config)
        self.config = config
        self.bus = None
        self.motors = {}
        self.motor_names = list(config.motor_can_ids.keys())
        self._in_safe_zero = False
        self._emergency_disable_requested = False
        # motor_name -> URDF q_idx, built once in connect(); None disables ff.
        self._gravity_pairing = None
        # When True, disconnect() skips safe_zero(). Set by calibrate() so the
        # lerobot-calibrate entrypoint (which calls calibrate() then immediately
        # disconnect()) does not move the arm back to zero. connect() resets this
        # after its internal calibrate() call so normal use still safe-zeros.
        self._skip_safe_zero_on_disconnect = False

        # Initialize cameras
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        """Motor features for observation and action spaces."""
        features: dict[str, type] = {}
        for motor in self.motor_names:
            features[f"{motor}.pos"] = float
            # features[f"{motor}.vel"] = float
            # features[f"{motor}.torque"] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Camera features for observation space."""
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Combined observation features from motors and cameras."""
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Action features."""
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        """Check if robot is connected."""
        return self.bus is not None and all(cam.is_connected for cam in self.cameras.values())

    def _add_motors_to_bus(self):
        """Must be implemented by subclasses to add specific motor types to self.bus."""
        raise NotImplementedError

    def connect(self, calibrate: bool = True) -> None:
        """Connect to the follower arm and optionally calibrate."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting arm on {self.config.port} (adapter={self.config.can_adapter})...")
        if self.config.can_adapter == "damiao":
            self.bus = MotorBridgeController.from_dm_serial(
                serial_port=self.config.port,
                baud=self.config.dm_serial_baud,
            )
        elif self.config.can_adapter == "robstride":
            raise NotImplementedError(
                "RobStride dedicated USB-to-CAN adapter is not yet supported in motorbridge Python SDK."
            )
        else:
            # Default: socketcan (PCAN, slcan, etc.)
            self.bus = MotorBridgeController(channel=self.config.port)
        
        self._add_motors_to_bus()

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()
            # calibrate() set _skip_safe_zero_on_disconnect for the calibrate
            # script's benefit; clear it here so a later disconnect (after
            # normal use of this connected arm) still runs safe_zero().
            self._skip_safe_zero_on_disconnect = False

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

        # Skip during the calibrate flow (calibrate=False); the gripper
        # may not be homed yet and the probe would block connect().
        if calibrate:
            self.detect_gripper_zero()

        self._build_gravity_pairing()

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        """Check if robot is calibrated."""
        return bool(self.calibration)

    def calibrate(self) -> None:
        """Calibration procedure for B601."""
        # Mark so that the disconnect() following this (as in the
        # lerobot-calibrate entrypoint) skips safe_zero(). connect() clears
        # this flag after its internal calibrate() call.
        self._skip_safe_zero_on_disconnect = True
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using calibration file associated with the id {self.id}")
                return

        logger.info(f"\nRunning calibration for {self}")
        
        self.bus.disable_all()

        print(
            "\nCalibration: Set Zero Position\n"
            "Please MANUALLY move the robot to its ZERO POSITION, and close its gripper.\n"
            "Reference the B601 manual for Zero Pose (generally the default sit-down position).\n"
        )
        input("Press ENTER when ready...")

        for motor in self.motors.values():
            motor.set_zero_position()
            time.sleep(LONG_TIMEOUT_SEC)
        
        logger.info("Arm zero position set.")

        logger.info("Setting range: -90° to +90° by default for all joints")
        self.calibration = {}
        for motor_name, (send_id, recv_id) in self.config.motor_can_ids.items():
            self.calibration[motor_name] = MotorCalibration(
                id=send_id,
                drive_mode=0,
                homing_offset=0,
                range_min=-90,
                range_max=90,
            )

        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """Configure motors with appropriate settings."""
        # Keep torque off while switching modes, then enable after all motors are configured.
        self.bus.disable_all()
        num_retry = 9
        for motor_name, motor in self.motors.items():
            target_mode = MotorBridgeMode.MIT if self.motor_type == "rs" else (
                MotorBridgeMode.FORCE_POS
                if motor_name == FOLLOWER_GRIPPER_MOTOR
                else MotorBridgeMode.POS_VEL
            )
            for _ in range(num_retry + 1):
                try:
                    motor.ensure_mode(target_mode)
                    break
                except Exception as e:
                    if _ == num_retry:
                        raise e
                    time.sleep(MEDIUM_TIMEOUT_SEC)
            logger.info(f"{motor_name} ensure mode {target_mode}")
        self.bus.enable_all()

    def disable_torque(self) -> None:
        """Disable follower motor torque so the arm can be moved by hand during read-only debugging."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disable_all()
        logger.info(f"{self} torque disabled.")

    def detect_gripper_zero(
        self,
        tolerance_deg: float = 5.0,
        force_ratio: float = 0.012,
        mit_tau: float = 0.3,
    ) -> dict:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        motor = self.motors.get(FOLLOWER_GRIPPER_MOTOR)
        if motor is None:
            return {"error": "gripper motor not found"}
        lo, hi = self.config.joint_limits[FOLLOWER_GRIPPER_MOTOR]
        closing_sign = -math.copysign(1.0, hi if abs(hi) > abs(lo) else lo)

        # PATCH (Humanoid Project): always home the gripper at its CLOSED hard
        # stop. The encoder reading can wrap after a power cycle with the
        # gripper open, so ignore the reading: ramp the target in the closing
        # direction until the gripper is blocked, then set zero there.
        # Torque stays <= kp * 10deg (~1 Nm).
        if self.motor_type == "rs":
            try:
                def _rd():
                    motor.request_feedback(); time.sleep(0.01)
                    try: self.bus.poll_feedback_once()
                    except Exception: pass
                    cs = motor.get_state()
                    return cs.pos if cs is not None else None
                cur = _rd()
                if cur is not None:
                    tgt = cur
                    step = math.radians(3.0) * closing_sign
                    blocked = 0
                    for _ in range(200):
                        cur = _rd()
                        if cur is None: continue
                        if abs(tgt - cur) < math.radians(10.0):
                            tgt += step; blocked = 0
                        else:
                            blocked += 1
                            if blocked >= 15: break
                        motor.send_mit(tgt, 0.0, 6.0, 0.3, 0.0)
                        time.sleep(0.01)
                    motor.send_mit(0.0, 0.0, 0.0, 1.5, 0.0)
                    if blocked >= 15:
                        before = math.degrees(_rd() or 0.0)
                        motor.disable(); time.sleep(0.2)
                        motor.set_zero_position(); time.sleep(0.5)
                        motor.ensure_mode(MotorBridgeMode.MIT); time.sleep(0.1)
                        motor.enable(); time.sleep(0.2)
                        motor.send_mit(0.0, 0.0, 0.0, 1.5, 0.0)
                        logger.info("gripper homed at closed stop (was %.1f deg), now %.2f deg",
                                    before, math.degrees(_rd() or float("nan")))
                    else:
                        logger.warning("gripper homing: closed stop not found")
            except Exception:
                logger.warning("gripper homing failed", exc_info=True)

        prev, stable = None, 0
        for _ in range(200):
            if self.motor_type == "rs":
                motor.send_mit(0.0, 0.0, 0.0, 1.5, closing_sign * mit_tau)
            else:
                motor.send_force_pos(
                    math.radians(closing_sign * 10.0),
                    math.radians(10.0),
                    force_ratio,
                )
            time.sleep(0.05)
            motor.request_feedback()
            try:
                self.bus.poll_feedback_once()
            except Exception:
                pass
            state = motor.get_state()
            if state is None:
                continue
            cur = math.degrees(state.pos)
            stable = stable + 1 if prev is not None and abs(cur - prev) < 0.1 else 0
            prev = cur
            if stable >= 5:
                break

        if self.motor_type == "rs":
            motor.send_mit(0.0, 0.0, 0.0, 1.5, 0.0)
        else:
            motor.send_force_pos(0.0, math.radians(10.0), force_ratio)
        if prev is not None and abs(prev) <= tolerance_deg:
            logger.info("gripper zero-detect: zero-point check passed")
        elif (prev is not None and abs(prev) < 15.0 and self.motor_type == "rs"):
            # PATCH: gripper is stalled at its closed hard stop but the encoder
            # zero has drifted (e.g. powered off while open). Re-home: set zero here.
            logger.warning("gripper zero-detect: offset %.2f deg -> re-homing gripper zero at closed stop", prev)
            try:
                motor.disable(); time.sleep(0.2)
                motor.set_zero_position(); time.sleep(0.5)
                motor.ensure_mode(MotorBridgeMode.MIT); time.sleep(0.1)
                motor.enable(); time.sleep(0.2)
                motor.send_mit(0.0, 0.0, 0.0, 1.5, 0.0)
                after = None
                for _ in range(5):
                    motor.request_feedback(); time.sleep(0.03)
                    try: self.bus.poll_feedback_once()
                    except Exception: pass
                    st = motor.get_state()
                    if st is not None: after = math.degrees(st.pos)
                logger.info("gripper re-homed, now reads %s deg", after)
                if after is None or abs(after) > tolerance_deg:
                    raise RuntimeError(f"gripper re-home failed, reads {after}")
            except Exception:
                raise
        else:
            offset = prev if prev is not None else float("nan")
            raise RuntimeError(
                "gripper zero-detect: gripper not homed, offset "
                f"{offset:.2f} deg, please confirm gripper is at home or recalibrate"
            )

    def _read_motor_temperatures(self) -> dict[str, float]:
        """Read per-motor MOS temperatures once and return available values."""
        for motor in self.motors.values():
            motor.request_feedback()
        try:
            self.bus.poll_feedback_once()
        except Exception:
            logger.warning("Temperature check poll feedback failed.")

        temps: dict[str, float] = {}
        for motor_name, motor in self.motors.items():
            state = motor.get_state()
            if state is not None:
                temps[motor_name] = state.t_mos

        return temps

    def _check_motor_temperatures(
        self,
        alarm_threshold_c: float,
        overheat_threshold_c: float,
        context: str = "",
    ) -> dict[str, float]:
        """Read motor MOS temperatures once, print a HIGH TEMP warning for any
        motor above ``alarm_threshold_c``, and raise ``KeyboardInterrupt`` if any
        motor exceeds ``overheat_threshold_c`` (aborts the control loop).

        Returns the dict of ``{motor_name: t_mos_c}`` that was read.
        """
        temperatures = self._read_motor_temperatures()
        logger.debug(
            "motor temps (°C): %s",
            " | ".join(
                f"{n}={temperatures[n]:.1f}" if n in temperatures else f"{n}=--"
                for n in self.motors
            ),
        )
        label = f" in {context}" if context else ""
        for motor_name, temp_c in temperatures.items():
            if temp_c > alarm_threshold_c:
                print(
                    f"[HIGH TEMP] {motor_name} t_mos={temp_c:.2f}C > {alarm_threshold_c:.2f}C"
                )
            if temp_c > overheat_threshold_c:
                logger.error(
                    "Overheat detected%s: %s t_mos=%.2fC > %.2fC.",
                    label,
                    motor_name,
                    temp_c,
                    overheat_threshold_c,
                )
                raise KeyboardInterrupt("Overheat detected")
        return temperatures

    def mit_output_torque_limit(
        self,
        motor: Any,
        pos_target_rad: float,
    ) -> float | None:
        """Compute MIT torque command from target position and motor state."""
        return 0.0

    def _build_gravity_pairing(self) -> None:
        """Build motor_name -> URDF q_idx pairing once at connect().

        Loads and warms the gravity model, then pairs follower arm motors
        (config order, excluding gripper) with URDF revolute joints (DFS
        order). On any failure sets self._gravity_pairing = None so
        _gravity_feedforward no-ops.
        """
        if not (self.config.gravity_compensation and self.motor_type == "rs"):
            self._gravity_pairing = None
            return
        urdf = self.config.gravity_urdf_path
        if not urdf:
            logger.warning("gravity_urdf_path not set, feedforward disabled")
            self._gravity_pairing = None
            return
        try:
            from .gravity import load_dynamics_model
        except Exception as exc:  # pragma: no cover
            logger.warning("gravity import failed, feedforward disabled: %s", exc)
            self._gravity_pairing = None
            return
        model = load_dynamics_model(urdf_path=urdf)
        arm_motors = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
        arm_qidx = [j.q_idx for j in model.joints if j.jtype in ("revolute", "continuous")]
        if len(arm_motors) != len(arm_qidx):
            logger.warning(
                "gravity feedforward disabled: arm motor count %d != URDF revolute %d",
                len(arm_motors), len(arm_qidx),
            )
            self._gravity_pairing = None
            return
        self._gravity_pairing = dict(zip(arm_motors, arm_qidx))

    def _gravity_feedforward(self) -> dict[str, float]:
        """Per-arm-motor gravity feedforward torque (N·m); {} if disabled."""
        pairing = self._gravity_pairing
        if not pairing:
            return {}
        import numpy as np
        from .gravity import load_dynamics_model, compute_generalized_gravity
        model = load_dynamics_model(urdf_path=self.config.gravity_urdf_path)
        q = np.zeros(model.nq)
        for motor_name, qi in pairing.items():
            motor = self.motors.get(motor_name)
            if motor is None:
                continue
            state = motor.get_state()
            if state is not None:
                q[qi] = float(state.pos)
        tau = compute_generalized_gravity(model=model, q=q)
        return {name: float(tau[qi]) for name, qi in pairing.items()}

    def safe_zero(self, step_interval_s: float = 0.02, exit_on_complete: bool = True) -> None:
        """Move arm joints back to zero in a safer two-stage interpolation.

        Stage 1: CAN ID 1/4/5/6 -> 0
        Stage 2: CAN ID 2/3 -> 0
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self._in_safe_zero:
            logger.warning("safe_zero skipped: already running.")
            return

        if step_interval_s < 0.0:
            raise ValueError("step_interval_s must be >= 0")

        self._in_safe_zero = True
        try:
            id_to_joint: dict[int, str] = {
                send_id: motor_name
                for motor_name, (send_id, _) in self.config.motor_can_ids.items()
                if motor_name != FOLLOWER_GRIPPER_MOTOR
            }

            stage_1 = [id_to_joint[i] for i in (1, 4, 5, 6) if i in id_to_joint]
            stage_2 = [id_to_joint[i] for i in (2, 3) if i in id_to_joint]
            controlled_joints = stage_1 + [name for name in stage_2 if name not in stage_1]

            if not controlled_joints:
                logger.warning("safe_zero skipped: no arm joints mapped to CAN IDs 1-6.")
                return

            def _read_motor_deg(joint_name: str) -> float:
                """Read a joint's current angle in real motor degrees (no direction scaling)."""
                motor = self.motors.get(joint_name)
                if motor is None:
                    raise RuntimeError(f"safe_zero failed: motor '{joint_name}' not found")

                max_retry = 10
                for attempt in range(1, max_retry + 1):
                    try:
                        motor.request_feedback()
                        self.bus.poll_feedback_once()
                    except Exception:
                        logger.debug(
                            "safe_zero feedback poll failed for %s (attempt %d/%d)",
                            joint_name,
                            attempt,
                            max_retry,
                        )

                    state = motor.get_state()
                    if state is not None:
                        return math.degrees(state.pos)

                    if attempt < max_retry:
                        time.sleep(MEDIUM_TIMEOUT_SEC)

                raise RuntimeError(
                    f"safe_zero failed: unable to read state for '{joint_name}' after {max_retry} attempts"
                )

            # PATCH v3.7 safe_zero (Humanoid Project): closed loop instead of an open-loop time ramp.
            # 2026-11-21 14:09 the old ramp ran on frozen feedback (power/CAN lost: every joint's present value
            # identical to 14 decimals for 9 s), declared "safe_zero done" and left the arm overhead. Now:
            #  - every frame re-reads the motors; the command advances max SAFE_ZERO_STEP_DEG (0.5) per frame and
            #    never more than SAFE_ZERO_LEAD_DEG (3) ahead of the MEASURED position (no stale goal as ramp base)
            #  - frozen feedback (every motor's full state bit-identical for SAFE_ZERO_STALE_S = 0.6 s while moving) -> abort
            #  - per-stage timeout SAFE_ZERO_TIMEOUT_S (20 s) and stall report (no progress for 3 s) with joint,
            #    position and gravity torque in the log
            #  - gravity feedforward is used during the rest move (SAFE_ZERO_GRAVITY_FF=1, default)
            #  - self._safe_zero_result tells disconnect() whether the arm reached rest (soft release otherwise)
            _sz_step = max(0.05, _env_float("SAFE_ZERO_STEP_DEG", 0.5))
            _sz_lead = max(0.5, _env_float("SAFE_ZERO_LEAD_DEG", 3.0))
            _sz_timeout = max(1.0, _env_float("SAFE_ZERO_TIMEOUT_S", 20.0))
            _sz_stale_s = max(0.1, _env_float("SAFE_ZERO_STALE_S", 0.6))
            _sz_stall_s = max(0.5, _env_float("SAFE_ZERO_STALL_S", 3.0))
            _sz_tol = _env_float("SAFE_ZERO_TOL_DEG", 2.0)
            _dt = step_interval_s if step_interval_s > 0 else 0.02
            self._safe_zero_result = {"feedback": True, "reached": False, "positions": {}}

            def _poll_all() -> dict[str, float]:
                for _m in self.motors.values():
                    try:
                        _m.request_feedback()
                    except Exception:
                        pass
                try:
                    self.bus.poll_feedback_once()
                except Exception:
                    logger.debug("safe_zero poll failed")
                out = {}
                for _n, _m in self.motors.items():
                    _s = _m.get_state()
                    if _s is not None:
                        out[_n] = math.degrees(_s.pos)
                return out

            def _send(cmd_motor_deg: dict[str, float]) -> None:
                action: RobotAction = {}
                for joint, v in cmd_motor_deg.items():
                    d = self.config.joint_directions.get(joint, 1.0) or 1.0
                    action[f"{joint}.pos"] = v / d
                self.send_action(action)

            def _interp_to_zero(
                active_starts: dict[str, float],
                hold_joints: dict[str, float],
                targets: dict[str, float] | None = None,
                stage: str = "",
            ) -> str:
                """Closed-loop move of active joints to targets. Returns 'ok', 'timeout', 'stalled',
                'nofeedback' or 'overtemp'."""
                if not active_starts:
                    return "ok"
                targets = targets or {}
                cmd = dict(active_starts)
                hold = dict(hold_joints)
                emergency_disable_threshold_c = self.config.temp_emergency_disable_threshold_c
                t0 = time.monotonic()
                prev_all, stale_t0 = None, None
                best = {j: abs(targets.get(j, 0.0) - v) for j, v in active_starts.items()}
                best_t = {j: t0 for j in active_starts}
                frames = 0
                while True:
                    frames += 1
                    temperatures = self._read_motor_temperatures()
                    for motor_name, temp_c in temperatures.items():
                        if temp_c > emergency_disable_threshold_c:
                            logger.error(
                                "Auto-disable on overtemperature during safe_zero: %s t_mos=%.2fC > %.2fC.",
                                motor_name, temp_c, emergency_disable_threshold_c,
                            )
                            self._emergency_disable_requested = True
                            self.disable_torque()
                            logger.error("safe_zero aborted: emergency overtemperature.")
                            return "overtemp"
                    pres = _poll_all()
                    now = time.monotonic()
                    self._safe_zero_result["positions"] = dict(pres)
                    # frozen feedback: every reported position bit-identical while we command motion
                    # full motor state (position, velocity, torque, temperature): a live motor's torque reading
                    # fluctuates even when its joint is blocked; after a power cut the driver keeps the last frame
                    snap = tuple((n, tuple(getattr(_m.get_state(), a, None) for a in ("pos", "vel", "torq", "t_mos"))
                                  if _m.get_state() is not None else None) for n, _m in sorted(self.motors.items()))
                    moving = any(abs(targets.get(j, 0.0) - pres.get(j, cmd[j])) > _sz_tol for j in cmd)
                    if snap == prev_all and moving:
                        stale_t0 = now if stale_t0 is None else stale_t0
                    else:
                        stale_t0 = None
                    prev_all = snap
                    if stale_t0 is not None and now - stale_t0 >= _sz_stale_s and frames > 3:
                        self._safe_zero_result["feedback"] = False
                        logger.error("safe_zero %s ABGEBROCHEN: keine Motor-Rueckmeldung seit %.1f s (alle Positionen "
                                     "eingefroren: %s) - Strom aus oder CAN getrennt? Arm NICHT in Ruheposition.", stage,
                                     now - stale_t0, ", ".join(f"{j}={pres.get(j, float('nan')):.1f}" for j in cmd))
                        return "nofeedback"
                    done = True
                    for j in list(cmd):
                        tgt = targets.get(j, 0.0)
                        p = pres.get(j, cmd[j])
                        dist = abs(tgt - p)
                        tol = 5.0 if j == FOLLOWER_GRIPPER_MOTOR else _sz_tol
                        if dist > tol:
                            done = False
                        if dist < best[j] - 0.2:
                            best[j], best_t[j] = dist, now
                        step = max(-_sz_step, min(_sz_step, tgt - cmd[j]))
                        nc = cmd[j] + step
                        sg = 1.0 if step >= 0 else -1.0
                        if (nc - p) * sg > _sz_lead:      # never run more than LEAD ahead of the measured position
                            nc = p + sg * _sz_lead
                        if (tgt - nc) * sg < 0:
                            nc = tgt
                        cmd[j] = nc
                    if done:
                        logger.info("safe_zero %s erreicht nach %.1f s", stage, now - t0)
                        return "ok"
                    out = dict(hold); out.update(cmd)
                    _send(out)
                    open_j = [j for j in cmd if abs(targets.get(j, 0.0) - pres.get(j, cmd[j])) > (5.0 if j == FOLLOWER_GRIPPER_MOTOR else _sz_tol)]
                    if open_j and all(now - best_t[j] >= _sz_stall_s for j in open_j):
                        g = self._gravity_torques_deg(pres) if hasattr(self, "_gravity_torques_deg") else {}
                        logger.warning("safe_zero %s STILLSTAND: %s bewegen sich seit %.0f s nicht mehr (%s)", stage,
                                       ", ".join(open_j), _sz_stall_s,
                                       ", ".join(f"{j}={pres.get(j, float('nan')):.1f} deg (Ziel {targets.get(j, 0.0):.0f}, "
                                                 f"Schwerkraft {g.get(j, 0.0):.1f} Nm)" for j in open_j))
                        return "stalled"
                    if now - t0 >= _sz_timeout:
                        logger.warning("safe_zero %s TIMEOUT nach %.0f s: %s", stage, _sz_timeout,
                                       ", ".join(f"{j}={pres.get(j, float('nan')):.1f} deg (Ziel {targets.get(j, 0.0):.0f})" for j in open_j))
                        return "timeout"
                    if step_interval_s > 0.0:
                        time.sleep(step_interval_s)

            stage_1_start = {joint: _read_motor_deg(joint) for joint in stage_1}
            stage_2_start = {joint: _read_motor_deg(joint) for joint in stage_2}

            logger.info("safe_zero stage1 start: joints=%s (Start: %s)", stage_1,
                        ", ".join(f"{j}={v:.1f}" for j, v in {**stage_1_start, **stage_2_start}.items()))
            r1 = _interp_to_zero(stage_1_start, stage_2_start, stage="stage1")
            if r1 in ("overtemp", "nofeedback"):
                return
            stage_1_hold = {j: self._safe_zero_result["positions"].get(j, 0.0) if r1 != "ok" else 0.0 for j in stage_1}

            # Stage 2: move CAN ID 2/3 back to zero, and bring the gripper back to
            # 170° if it is currently past 180° (avoids leaving it wide open).
            stage_2_active = {j: self._safe_zero_result["positions"].get(j, v) for j, v in stage_2_start.items()}
            stage_2_targets: dict[str, float] = {}
            if FOLLOWER_GRIPPER_MOTOR in self.motors:
                try:
                    gripper_motor_deg = _read_motor_deg(FOLLOWER_GRIPPER_MOTOR)
                except RuntimeError as e:
                    logger.warning("safe_zero: could not read gripper position: %s", e)
                    gripper_motor_deg = None

                if gripper_motor_deg is not None and abs(gripper_motor_deg) > 180.0:
                    # Wide open: bring it back toward 170°, preserving sign.
                    target_motor_deg = math.copysign(170.0, gripper_motor_deg)
                    stage_2_active[FOLLOWER_GRIPPER_MOTOR] = gripper_motor_deg
                    stage_2_targets[FOLLOWER_GRIPPER_MOTOR] = target_motor_deg
                    logger.info(
                        "safe_zero gripper: %.2f° (motor, abs>180°), returning to %.2f°",
                        gripper_motor_deg,
                        target_motor_deg,
                    )

            logger.info("safe_zero stage2 start: joints=%s", stage_2)
            r2 = _interp_to_zero(stage_2_active, stage_1_hold, stage_2_targets, stage="stage2")
            if r2 in ("overtemp", "nofeedback"):
                return
            pos = self._safe_zero_result["positions"]
            near = all(abs(pos.get(j, 0.0)) <= _env_float("SAFE_ZERO_REST_TOL_DEG", 8.0) for j in controlled_joints)
            self._safe_zero_result["reached"] = near and r1 == "ok" and r2 == "ok"
            if not self._safe_zero_result["reached"]:
                logger.warning("safe_zero: Arm NICHT ganz in Ruheposition (%s) - beim Ausschalten wird weich abgesenkt",
                               ", ".join(f"{j}={pos.get(j, float('nan')):.1f}" for j in controlled_joints))
            logger.info("safe_zero done (Ruheposition %s).", "erreicht" if self._safe_zero_result["reached"] else "NICHT erreicht")
            time.sleep(0.5 if self._safe_zero_result["reached"] else 0.1)
            if exit_on_complete:
                # Raise KeyboardInterrupt so upper-level control loops handle this
                # the same way as Ctrl+C.
                raise KeyboardInterrupt("safe_zero completed")
        finally:
            self._in_safe_zero = False

    def get_observation(self) -> RobotObservation:
        """Get current observation from robot."""
        start = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}

        # Request and poll feedback from motorbridge
        for motor in self.motors.values():
            motor.request_feedback()
        try:
            self.bus.poll_feedback_once()
        except:
            logger.warning(f"can bus poll feedback failed.")

        for motor_name, motor in self.motors.items():
            state = motor.get_state()
            if state is not None:
                # motorbridge works natively in radians. Convert to degrees.
                obs_dict[f"{motor_name}.pos"] = math.degrees(state.pos)
                obs_dict[f"{motor_name}.vel"] = math.degrees(state.vel)
                obs_dict[f"{motor_name}.torque"] = state.torq
            else:
                obs_dict[f"{motor_name}.pos"] = 0.0
                obs_dict[f"{motor_name}.vel"] = 0.0
                obs_dict[f"{motor_name}.torque"] = 0.0

        # Capture images
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")
        # logger.debug(f"Observation: {obs_dict}")

        return obs_dict

    def _collision_guard_get(self):
        """PATCH (Humanoid Project): lazily build the guard if COLLISION_GUARD=1."""
        if not hasattr(self, "_collision_guard"):
            self._collision_guard = None
            if collision_guard_enabled():
                joints = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
                kp, clamp, ff = None, None, 0.0
                if guard_gravity_enabled() and self._gravity_model_get() is not None:
                    kp = self._effective_kp()
                    clamp = getattr(self.config, "max_relative_target", None)
                    if getattr(self.config, "gravity_compensation", False):
                        ff = 1.0
                    elif gravity_ff_enabled():
                        ff = gravity_ff_scale()
                self._collision_guard = CollisionGuard.from_env(joints, gains=kp, clamp_deg=clamp, ff_scale=ff)
                logger.info(self._collision_guard.describe())
        return self._collision_guard

    def _gravity_model_get(self):
        """PATCH (guard v3.2): lazily load the URDF gravity model + motor pairing; None on failure."""
        if not hasattr(self, "_grav_model"):
            self._grav_model = None
            try:
                if self.motor_type != "rs":
                    raise RuntimeError("gravity model only for the RS arm")
                urdf = getattr(self.config, "gravity_urdf_path", None)
                if not urdf:
                    raise RuntimeError("gravity_urdf_path not set")
                model = _gravity_module().load_dynamics_model(urdf_path=urdf)
                arm = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
                qidx = [j.q_idx for j in model.joints if j.jtype in ("revolute", "continuous")]
                if len(arm) != len(qidx):
                    raise RuntimeError(f"arm motors {len(arm)} != URDF revolute joints {len(qidx)}")
                self._grav_model = (model, dict(zip(arm, qidx)))
                logger.info("Schwerkraft-Modell geladen (%s)", urdf)
            except Exception as exc:
                logger.warning("Schwerkraft-Modell nicht verfuegbar, Schutz ohne Schwerkraft (v3.1-Verhalten): %s", exc)
        return self._grav_model

    def _gravity_torques_deg(self, pos_deg):
        """PATCH (guard v3.2): gravity holding torque (N·m) per arm joint from positions in degrees.
        Returns {} unless all arm joints are known."""
        gm = self._gravity_model_get()
        if gm is None:
            return {}
        model, pairing = gm
        if any(n not in pos_deg for n in pairing):
            return {}
        import numpy as np
        q = np.zeros(model.nq)
        for n, qi in pairing.items():
            q[qi] = math.radians(pos_deg[n])
        tau = _gravity_module().compute_generalized_gravity(model=model, q=q)
        return {n: float(tau[qi]) for n, qi in pairing.items()}

    def _effective_kp(self):
        """MIT kp per arm joint as actually sent (config + env overrides)."""
        if getattr(self.config, "gravity_compensation", False):
            return dict(getattr(self.config, "gravity_mit_kp", {}))
        kp = dict(getattr(self.config, "mit_kp", {}))
        kp.update(self._mit_gain_overrides_get()[0])
        return kp

    def _mit_gain_overrides_get(self):
        """PATCH (Humanoid Project, guard v2): cached MIT_KP_/MIT_KD_ env overrides, logged once."""
        if not hasattr(self, "_mit_gain_overrides"):
            self._mit_gain_overrides = mit_gain_overrides()
            _kp, _kd = self._mit_gain_overrides
            if _kp or _kd:
                base_kp = getattr(self.config, "mit_kp", {})
                base_kd = getattr(self.config, "mit_kd", {})
                parts = [f"{j} kp={_kp.get(j, base_kp.get(j, 0.0)):g} kd={_kd.get(j, base_kd.get(j, 0.0)):g}"
                         for j in _MIT_GAIN_JOINTS if j in _kp or j in _kd]
                logger.info("MIT-Gain-Override (nachgiebige Gelenke): %s%s", ", ".join(parts),
                            " (inaktiv: gravity_compensation an)" if getattr(self.config, "gravity_compensation", False) else "")
        return self._mit_gain_overrides

    def send_action(
        self,
        action: RobotAction
    ) -> RobotAction:
        """Send action command to robot."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if not self._in_safe_zero:
            self._check_motor_temperatures(
                self.config.temp_alarm_threshold_c,
                self.config.temp_overheat_threshold_c,
                context="send_action",
            )

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Apply per-joint direction/scale mapping before clipping.
        for motor_name, position in goal_pos.items():
            direction = self.config.joint_directions.get(motor_name, 0.0)
            position = position * direction
            # print(f"motor_name: {motor_name}, position: {position}")
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                clipped_position = max(min_limit, min(max_limit, position))
                if clipped_position != position:
                    logger.debug(f"Clipped {motor_name} from {position:.2f} to {clipped_position:.2f}")
                position = clipped_position

            goal_pos[motor_name] = position

        # To tolerate 6-DOF leader arms that don't have a wrist_yaw joint, we can allow the follower to ignore missing wrist_yaw commands by treating them as 0.
        if 'wrist_yaw' not in goal_pos:
            goal_pos['wrist_yaw'] = 0.0

        # PATCH (Humanoid Project): collision guard (only normal teleop, never in safe_zero)
        _guard = self._collision_guard_get()
        _grav_cycle = None   # v3.2: gravity torques of this cycle (computed at most once)
        if _guard is not None:
            if self._in_safe_zero:
                _guard.reset()
            else:
                try:
                    _gp, _gt = {}, {}
                    for _n in _guard.joints:
                        _m = self.motors.get(_n)
                        _s = _m.get_state() if _m is not None else None
                        if _s is not None:
                            _gp[_n] = math.degrees(_s.pos)
                            _gt[_n] = float(_s.torq)
                    _gg = None
                    if any(v > 0 for v in _guard.kp.values()):
                        _gg = self._gravity_torques_deg(_gp)
                        _grav_cycle = _gg
                    _override = _guard.update(goal_pos, _gp, _gt, time.monotonic(), grav=_gg)
                    if _override:
                        goal_pos.update(_override)
                except Exception:
                    logger.warning("collision guard error (ignored)", exc_info=True)

        # Safety: Cap relative target
        if self.config.max_relative_target is not None:
            # We need current position in degrees to compare against relative limit safely
            present_pos = {}
            for motor_name, motor in self.motors.items():
                state = motor.get_state()
                if state is not None:
                    present_pos[motor_name] = math.degrees(state.pos)
                else:
                    present_pos[motor_name] = 0.0
            
            goal_present_pos = {key: (g_pos, present_pos.get(key, g_pos)) for key, g_pos in goal_pos.items()}
            _mrt = self.config.max_relative_target
            if isinstance(_mrt, dict):
                _mrt = {k: v for k, v in _mrt.items() if k in goal_present_pos}
            goal_pos = ensure_safe_goal_position(goal_present_pos, _mrt)

        if _guard is not None and not self._in_safe_zero:
            _guard.note_sent(goal_pos)

        # Prepare and send commands
        gravity_tau = (
            self._gravity_feedforward()
            if (self.config.gravity_compensation and self.motor_type == "rs")
            else {}
        )
        # PATCH (guard v3.2): optional gravity feedforward GRAVITY_FF=1 (keeps the tuned kp/kd)
        if not gravity_tau and self.motor_type == "rs" and not self.config.gravity_compensation and (
                gravity_ff_enabled() or (self._in_safe_zero and _env_on("SAFE_ZERO_GRAVITY_FF", "1"))):
            try:
                if not _grav_cycle:
                    _pd = {}
                    for _n in self.motor_names:
                        _m = self.motors.get(_n)
                        _s = _m.get_state() if _m is not None else None
                        if _s is not None:
                            _pd[_n] = math.degrees(_s.pos)
                    _grav_cycle = self._gravity_torques_deg(_pd)
                _sc = gravity_ff_scale()
                # v3.3: ramp in over GRAVITY_FF_RAMP_S after the first feedforward cycle (no torque step at start)
                _now = time.monotonic()
                if getattr(self, "_grav_ff_t0", None) is None and _grav_cycle:
                    self._grav_ff_t0 = _now
                _rs = gravity_ff_ramp_s()
                _ramp = 1.0 if _rs <= 0 or getattr(self, "_grav_ff_t0", None) is None else min(1.0, (_now - self._grav_ff_t0) / _rs)
                gravity_tau = {n: max(-GRAVITY_FF_MAX_NM.get(n, 1.0), min(GRAVITY_FF_MAX_NM.get(n, 1.0), _ramp * _sc * g))
                               for n, g in (_grav_cycle or {}).items()}
                if not getattr(self, "_grav_ff_logged", False):
                    self._grav_ff_logged = True
                    logger.info("Schwerkraft-Feedforward AN (Anteil %.2f, Rampe %.1f s, Grenzen %s): Start-Werte %s", _sc, _rs,
                                ", ".join(f"{n}={v:g}" for n, v in GRAVITY_FF_MAX_NM.items()),
                                ", ".join(f"{n}={v:.2f}" for n, v in gravity_tau.items()) or "kein Modell")
            except Exception:
                logger.warning("gravity feedforward error (ignored, tau=0)", exc_info=True)
                gravity_tau = {}

        for motor_name, position_degrees in goal_pos.items():
            try:
                idx = self.motor_names.index(motor_name)
            except ValueError:
                idx = 0 # Fallback

            # Convert target position from degrees to radians for motorbridge
            pos_rad = math.radians(position_degrees)
            vel_deg_s = (
                self.config.pos_vel_velocity[idx]
                if isinstance(self.config.pos_vel_velocity, list)
                else self.config.pos_vel_velocity
            )
            vel_rad = math.radians(vel_deg_s)

            motor = self.motors.get(motor_name)
            if motor is not None:
                if motor_name == FOLLOWER_GRIPPER_MOTOR:
                    if self.motor_type == "rs":
                        tau_ff = self.mit_output_torque_limit(motor, pos_rad)
                        if tau_ff is None:
                            tau_ff = 0.0
                        motor.send_mit(0, 0, 0, 1.5, tau_ff)
                        logger.debug(
                            f"Sent MIT command to {motor_name}: pos={position_degrees:.2f}°, "
                            f"tau_ff={tau_ff:.2f}"
                        )
                    else:
                        motor.send_force_pos(pos_rad, vel_rad, self.config.force_pos_torque_ration)
                        logger.debug(f"Sent FORCE_POS command to {motor_name}: pos={position_degrees:.2f}°, vel={vel_deg_s:.2f}°/s, ratio={0.1}")
                else:
                    if self.motor_type == "rs":
                        if self.config.gravity_compensation:
                            kp = getattr(self.config, "gravity_mit_kp", {}).get(motor_name, 0.0)
                            kd = getattr(self.config, "gravity_mit_kd", {}).get(motor_name, 0.0)
                        else:
                            kp = getattr(self.config, "mit_kp", {}).get(motor_name, 0.0)
                            kd = getattr(self.config, "mit_kd", {}).get(motor_name, 0.0)
                            # PATCH (Humanoid Project, guard v2): env gain overrides
                            _okp, _okd = self._mit_gain_overrides_get()
                            kp = _okp.get(motor_name, kp)
                            kd = _okd.get(motor_name, kd)
                        tau = gravity_tau.get(motor_name, 0.0)
                        motor.send_mit(pos_rad, 0, kp, kd, tau)
                        logger.debug(
                            f"Sent MIT command to {motor_name}: "
                            f"pos={position_degrees:.2f}°, kp={kp}, kd={kd}, tau_ff={tau:.3f}"
                        )
                    else:
                        motor.send_pos_vel(pos_rad, 32)
                        logger.debug(f"Sent POS_VEL command to {motor_name}: target={pos_rad:.2f},pos={position_degrees:.2f}°, vel={vel_deg_s:.2f}°/s")

        # motorbridge sends packets mostly synchronously here over loop, 
        # so we don't need a bulk send command through ctypes.

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def _soft_release(self) -> None:
        """PATCH v3.7: lower the arm gently instead of a hard torque-off away from rest. Over SOFT_RELEASE_S
        (default 3 s) the position gain goes to 0 and the gravity feedforward fades out, with strong damping
        on shoulder_lift/elbow_flex (kd >= 20) so the arm sinks slowly onto its rest; then 1.5 s damping only."""
        dur = max(0.5, _env_float("SOFT_RELEASE_S", 3.0))
        arm = [n for n in self.motor_names if n != FOLLOWER_GRIPPER_MOTOR]
        kp0 = self._effective_kp()
        kd0 = dict(getattr(self.config, "mit_kd", {})); kd0.update(self._mit_gain_overrides_get()[1])
        pos = {}
        for n in arm:
            s = self.motors[n].get_state()
            if s is not None:
                pos[n] = math.degrees(s.pos)
        g = self._gravity_torques_deg(pos) if len(pos) == len(arm) else {}
        ff0 = {n: max(-GRAVITY_FF_MAX_NM.get(n, 1.0), min(GRAVITY_FF_MAX_NM.get(n, 1.0), 0.8 * g.get(n, 0.0))) for n in arm}
        logger.warning("Weiches Absenken (%.1f s): Arm nicht in Ruheposition (%s)", dur,
                       ", ".join(f"{n}={v:.0f}" for n, v in pos.items()))
        steps = max(1, int(dur / 0.02)); tail = int(1.5 / 0.02)
        for k in range(steps + tail):
            f = max(0.0, 1.0 - k / steps)
            try:
                self.bus.poll_feedback_once()
            except Exception:
                pass
            for n in arm:
                m = self.motors.get(n); s = m.get_state() if m is not None else None
                if s is None:
                    continue
                kd = max(kd0.get(n, 2.0), 20.0) if n in ("shoulder_lift", "elbow_flex") else kd0.get(n, 2.0)
                m.send_mit(s.pos, 0.0, kp0.get(n, 0.0) * f, kd, ff0.get(n, 0.0) * f)
            time.sleep(0.02)

    def disconnect(self):
        """Disconnect from robot."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if (
            not self._in_safe_zero
            and not self._emergency_disable_requested
            and not self._skip_safe_zero_on_disconnect
        ):
            try:
                self.safe_zero(exit_on_complete=False)
            except Exception:
                logger.exception("safe_zero during disconnect failed.")
        elif self._skip_safe_zero_on_disconnect:
            logger.info(
                "safe_zero skipped on disconnect: calibrate context "
                "(_skip_safe_zero_on_disconnect=True)."
            )

        # PATCH v3.7: gentle torque-off when the rest move did not reach rest (and feedback is alive)
        _szr = getattr(self, "_safe_zero_result", None)
        if (_szr and _szr.get("feedback") and not _szr.get("reached") and self.config.disable_torque_on_disconnect
                and not self._emergency_disable_requested and self.motor_type == "rs"):
            try:
                self._soft_release()
            except Exception:
                logger.exception("soft release failed (ignored)")
        elif _szr and not _szr.get("feedback"):
            logger.error("Keine Motor-Rueckmeldung: Motoren koennen evtl. nicht ausgeschaltet werden - Strom pruefen.")
        for motor in self.motors.values():
            try:
                if self.config.disable_torque_on_disconnect:
                    motor.disable()
                motor.clear_error()
                motor.close()
            except Exception:
                logger.exception("disable/close of a motor failed (power off?)")
        
        self.bus.close()
        self.bus = None

        for cam in self.cameras.values():
            cam.disconnect()

        self._emergency_disable_requested = False
        logger.info(f"{self} disconnected.")
