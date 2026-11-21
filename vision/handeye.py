#!/usr/bin/env python3
"""Automatic hand-eye calibration for the reBot B601-RS (Humanoid Project).

Replaces the manual corner-touch step (c). The arm moves the wrist camera (Innomaker) to ~18 poses above the
taped ChArUco board, and at each pose stores the joint angles, a wrist image and an overview image. From these:
wrist intrinsics, wrist camera -> gripper (T_cam2gripper), board -> robot base (= table -> robot) and overview
camera -> robot base. See docs/OPERATING_PROCEDURE.md "Auto calibration".

v3.9.1: the taught pose is optional. `run --auto` finds the board itself: slow move (6 deg/s) from wherever the arm
is to a high search pose, board detection in the wrist camera, move 200 mm above the board centre, plan, sweep.
Motion tracks reality: each cycle the goal = measured position + a step limited by the speed AND by 0.75 x the
follower's max_relative_target, so LeRobot never clamps (3.9 crash 22:29: precomputed ramp ran ahead of the clamped
arm -> 10 deg lag abort -> cascading tracebacks). Motor communication loss -> one line "Roboterarm antwortet nicht –
Strom prüfen", torque off once, no further motion.

Commands (python = ~/humanoid/.venv/bin/python, run from anywhere):
  handeye.py plan --auto              offline preview of the automatic start (search pose, path from rest). No hardware.
  handeye.py run --auto [--yes]       automatic start + sweep. ARM MOVES.
  handeye.py run --sim --auto         the same with a simulated arm + rendered images.
  handeye.py teach                    read the CURRENT arm pose from teleop telemetry (teleop running, wrist camera
                                      aimed at the board) and save it as the start/aim pose. Read-only.
  handeye.py plan [--dry-run]         plan the poses from the taught pose, check limits/height/paths, print the list
                                      and write plan.json + plan.png. No hardware.
  handeye.py run [--yes]              execute the plan on the arm (refuses while teleop/policy/training runs),
                                      save samples, solve, apply if the quality is good. ARM MOVES.
  handeye.py run --sim                the same with a simulated arm + rendered camera images (no hardware).
  handeye.py replay <folder>          solve again from a saved sample folder (no hardware).
  handeye.py apply <folder> [--force] write the result into calibration/handeye.json and table.json.
  handeye.py status                   show the current hand-eye calibration.
Stop: Ctrl+C / SIGTERM (GUI "■ STOPP") = motors off immediately (the arm sags - it is >= 80 mm over the board).
Normal end / other errors: back to the start pose, then the plugin's rest move (safe_zero), then torque off.
Arm unresponsive / not following (stall, lag vs the sent goal): torque off once, no return or rest move.
Board not found: rest move, torque off.
"""
import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
HUM = HERE.parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HUM))

import handeye_plan as HP                       # noqa: E402
from handeye_plan import ARM_JOINTS, Kin       # noqa: E402

CAL_DIR = HERE / "calibration"
HE_DIR = Path(os.environ.get("HANDEYE_OUT_DIR", str(CAL_DIR / "handeye")))
PLAN_JSON = HP.WORK_DIR / "handeye_plan.json"
PLAN_PNG = HP.WORK_DIR / "handeye_plan.png"
RESULT_JSON = CAL_DIR / "handeye.json"
AUTO_JSON = HP.WORK_DIR / "handeye_auto.json"     # v3.9.1: preview of the automatic start
MOTORS = ARM_JOINTS + ["gripper"]
DEFAULT_DIRECTIONS = {"shoulder_pan": 1.0, "shoulder_lift": 1.0, "elbow_flex": -1.0, "wrist_flex": -1.0,
                      "wrist_yaw": -1.0, "wrist_roll": 1.0, "gripper": 6.0}
FWD_PORT = int(os.environ.get("TELEMETRY_FWD_PORT", "8766"))


def say(msg):
    print(f"[handeye {time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Stop(BaseException):
    pass


def threading_main():
    import threading
    return threading.current_thread() is threading.main_thread()


# ---------------------------------------------------------------- teach (read-only)
def teach(port=None, seconds=1.0):
    import calibrate as cal
    ports = [port] if port else [FWD_PORT, 8765]
    last = None
    for p in ports:
        try:
            q = cal.read_tap_joints(port=p, seconds=seconds, timeout=4.0)
            break
        except RuntimeError as e:
            last = e
    else:
        raise RuntimeError(f"no teleop telemetry ({last}). Start teleop and hold the arm still.")
    cfg = HP.load_config(); kin = Kin()
    P = HP.look_point(kin, q, cfg["table_z_mm"])
    if P is None:
        raise RuntimeError("the gripper does not point down at the board (finger axis must be < 60 deg from vertical)")
    T = kin.T(q)
    d = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "q": [float(v) for v in q], "look_point_mm": P.tolist(),
         "gripper_end_mm": T[:3, 3].tolist(), "height_over_table_mm": float(T[2, 3] - cfg["table_z_mm"])}
    HP.TEACH.write_text(json.dumps(d, indent=1))
    say(f"taught pose saved: joints {np.round(q, 1).tolist()}, aim point {np.round(P, 0).tolist()} mm, "
        f"gripper {d['height_over_table_mm']:.0f} mm over the board")
    return d


def load_teach():
    if not HP.TEACH.exists():
        raise RuntimeError("no taught pose yet: run `handeye.py teach` while teleop runs (wrist camera aimed at the board)")
    return json.loads(HP.TEACH.read_text())


def make_plan(q_teach=None, out_json=PLAN_JSON, out_png=PLAN_PNG, log=say):
    q_teach = q_teach if q_teach is not None else load_teach()["q"]
    plan = HP.plan_poses(q_teach)
    plan["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    Path(out_json).write_text(json.dumps(plan, indent=1))
    try:
        HP.plot_plan(plan, out_png)
    except Exception as e:
        log(f"plot failed: {e}")
    return plan


# ---------------------------------------------------------------- cameras
class ShmSource:
    """Newest frame of the GUI's shared memory (640x360), no camera access."""
    def __init__(self, role):
        import camera_feeds as cf
        self.r = cf.ShmReader(role); self.role = role; self.seq = -1

    def grab(self, after_ts=0.0, timeout=3.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            x = self.r.read(self.seq)
            if x is not None:
                self.seq, ts, img = x
                if ts >= after_ts:
                    return img, ts
            time.sleep(0.005)
        raise RuntimeError(f"no fresh frame from GUI camera '{self.role}'")

    def close(self):
        self.r.close()


class DirectSource:
    """Own camera (GUI not running). A thread keeps reading so grab() returns a frame newer than after_ts."""
    def __init__(self, source):
        import threading
        import camera_feeds as cf
        idx, info = cf.resolve_source(source)
        if idx is None:
            raise RuntimeError(info)
        self.cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {source}")
        self.frame, self.ts, self.stop = None, 0.0, False
        self.th = threading.Thread(target=self._run, daemon=True); self.th.start()

    def _run(self):
        while not self.stop:
            ok, f = self.cap.read()
            if ok:
                self.frame, self.ts = f, time.time()

    def grab(self, after_ts=0.0, timeout=3.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.frame is not None and self.ts >= after_ts:
                return self.frame.copy(), self.ts
            time.sleep(0.01)
        raise RuntimeError("no fresh camera frame")

    def close(self):
        self.stop = True; time.sleep(0.1); self.cap.release()


def open_cameras():
    import camera_feeds as cf
    cfg = cf.load_config()
    src = {fd.get("role"): fd["source"] for fd in cfg["feeds"]}
    cams = {}
    for role, key in (("gripper", "wrist"), ("overview", "overview")):
        try:
            cams[key] = ShmSource(role); cams[key].grab(timeout=2.0); say(f"{key}: GUI shared frames")
        except Exception:
            cams[key] = DirectSource(src.get(role, "name:Innomaker|U20CAM" if role == "gripper" else "name:C920"))
            cams[key].grab(timeout=5.0); say(f"{key}: camera opened directly")
    return cams


# ---------------------------------------------------------------- robot wrappers
class ArmUnresponsive(RuntimeError):
    """The motors stopped answering (power cut, CAN cable, ack timeouts, frozen feedback)."""


class MotionAbort(RuntimeError):
    """The arm does not follow the command (blocked, too weak, collision guard froze it)."""


class BoardNotFound(RuntimeError):
    """The wrist camera does not see enough of the ChArUco board."""


MSG_NO_ARM = "Roboterarm antwortet nicht – Strom prüfen"
MSG_NO_BOARD = "Brett nicht gefunden – Brett im Blick der Handkamera?"
_COMM_WORDS = ("timeout", "ack", "no feedback", "not connected", "callerror", "can bus", "comm_type")


def _is_comm_error(e):
    s = f"{type(e).__name__} {e}".lower()
    return any(w in s for w in _COMM_WORDS)


class _QuietLogs:
    """Silence the follower plugin's per-motor tracebacks (e.g. 6x 'disable failed: control ack timeout') while we
    switch the motors off; we write one concise line ourselves."""
    NAMES = ("lerobot_robot_seeed_b601", "lerobot.robots", "motorbridge")

    def __enter__(self):
        import logging
        self.old = [(logging.getLogger(n), logging.getLogger(n).level) for n in self.NAMES]
        for lg, _ in self.old:
            lg.setLevel(logging.CRITICAL + 1)
        return self

    def __exit__(self, *a):
        for lg, lv in self.old:
            lg.setLevel(lv)
        return False


class _ClampCounter:
    """logging filter for 'Relative goal position magnitude had to be clamped': counts them, lets the first 3 through
    (v3.9.1: the executor stays below max_relative_target, so any clamp is a diagnostic, not normal operation)."""
    def __init__(self):
        self.n = 0

    def filter(self, rec):
        if "clamped" in rec.getMessage():
            self.n += 1
            return self.n <= 3
        return True


class RealArm:
    """Thin wrapper around the follower (same construction + safety settings as play_policy.py).

    v3.9.1: exposes the follower's own max_relative_target (self.mrt, raw degrees per joint) so the executor can stay
    below it, and raises ArmUnresponsive when the motors stop answering: an exception with a communication error,
    a motor without any state, or the complete motor state (pos, vel, torque, temperature) bit-identical for
    frozen_s seconds (after a power cut the driver keeps returning the last frame - same test as the plugin's
    safe_zero)."""
    def __init__(self, mrt_deg, frozen_s=1.0):
        import logging
        import play_policy as pp
        self.pp = pp
        mrt = {m: (2.0 if m == "gripper" else float(mrt_deg)) for m in MOTORS}
        self.clamps = _ClampCounter()
        logging.getLogger("lerobot.robots.utils").addFilter(self.clamps)
        self.robot = pp.make_robot(mrt)
        try:
            self.robot.connect()
        except Exception as e:
            raise ArmUnresponsive(f"connect failed: {e}") from None
        try:
            self.dirs = dict(self.robot.config.joint_directions) or DEFAULT_DIRECTIONS
        except Exception:
            self.dirs = DEFAULT_DIRECTIONS
        cm = getattr(self.robot.config, "max_relative_target", None)
        if isinstance(cm, dict):
            self.mrt = np.array([float(cm.get(m, mrt[m])) for m in MOTORS])
        elif cm is not None:
            self.mrt = np.full(len(MOTORS), float(cm))
        else:
            self.mrt = np.array([mrt[m] for m in MOTORS])
        self.frozen_s = frozen_s
        self._sig, self._sig_t0, self._off = None, None, False

    def _signature(self):
        sig = []
        for n in sorted(self.robot.motors):
            st = self.robot.motors[n].get_state()
            if st is None:
                raise ArmUnresponsive(f"motor {n} reports no state")
            sig.append((st.pos, st.vel, st.torq, getattr(st, "t_mos", None)))
        return tuple(sig)

    def read(self):
        try:
            o = self.robot.get_observation()
            sig = self._signature()
        except ArmUnresponsive:
            raise
        except Exception as e:
            raise ArmUnresponsive(f"no motor feedback ({e})") from None
        now = time.monotonic()
        if sig == self._sig:
            self._sig_t0 = self._sig_t0 or now
            if now - self._sig_t0 >= self.frozen_s:
                raise ArmUnresponsive(f"motor feedback frozen for {now - self._sig_t0:.1f} s")
        else:
            self._sig, self._sig_t0 = sig, None
        return np.array([o[f"{m}.pos"] for m in MOTORS], float), np.array([o.get(f"{m}.vel", 0.0) for m in MOTORS])

    def command(self, q7):
        """q7: RAW motor degrees (observation convention) -> action values (divide by joint_directions)."""
        try:
            self.robot.send_action({f"{m}.pos": float(q7[k]) / (self.dirs.get(m, 1.0) or 1.0) for k, m in enumerate(MOTORS)})
        except Exception as e:
            if _is_comm_error(e):
                raise ArmUnresponsive(f"command not acknowledged ({e})") from None
            raise

    def stop_now(self):
        """Torque off immediately, no rest move, exactly once. Plugin tracebacks are silenced.
        -> True if the motors acknowledged, False if they did not answer (power off?)."""
        if self._off:
            return True
        self._off = True
        ok = True
        with _QuietLogs():
            try:
                self.robot._skip_safe_zero_on_disconnect = True
            except Exception:
                pass
            try:
                self.robot.bus.disable_all()
            except Exception:
                ok = False
            try:
                self.robot.disconnect()
            except Exception:
                pass
        return ok

    def finish(self):
        """Normal end: plugin rest move (safe_zero), then torque off."""
        self._off = True
        self.robot.disconnect()


class SimArm:
    """Simulated follower that behaves like LeRobot + the B601 plugin.

    command(): every goal is clamped to present +- max_relative_target (exactly what
    lerobot.robots.utils.ensure_safe_goal_position does; counted in self.clamped).
    read(): each joint follows its clamped goal with a first-order response, at most max_speed_dps. A 'weak' joint
    (dict index -> min error deg) only moves when its goal is more than that many degrees away - a PD joint that
    lifts against gravity (the 22:29 elbow_flex: with max_relative_target 1 deg it crept at ~1 deg/s and the old
    executor aborted on lag). dead_after: after that many reads the feedback freezes (power cut)."""
    def __init__(self, q0, max_step_deg=4.0, noise_deg=0.02, seed=0, max_speed_dps=60.0, weak=None, dead_after=None,
                 dt=1 / 30):
        self.q = np.array(q0, float); self.goal = self.q.copy()
        self.mrt = np.array([float(max_step_deg)] * 6 + [2.0])
        self.rng = np.random.default_rng(seed); self.noise = noise_deg; self.finished = False
        self.vmax, self.dt, self.weak, self.dead_after = max_speed_dps, dt, dict(weak or {}), dead_after
        self.clamped, self.n_read, self.n_cmd_after_dead, self.stop_calls, self._frozen = 0, 0, 0, 0, None

    def dead(self):
        return self.dead_after is not None and self.n_read >= self.dead_after

    def read(self):
        self.n_read += 1
        if self.dead():
            if self._frozen is None:
                self._frozen = (self.q.copy(), np.zeros(7))
            return self._frozen[0].copy(), self._frozen[1].copy()
        d = self.goal - self.q
        for j, e0 in self.weak.items():
            d[j] = np.sign(d[j]) * max(0.0, abs(d[j]) - e0)
        step = np.clip(0.7 * d, -self.vmax * self.dt, self.vmax * self.dt)
        self.q = self.q + step
        return self.q + self.rng.normal(0, self.noise, 7), step / self.dt

    def command(self, q7):
        if self.dead():
            self.n_cmd_after_dead += 1
            return
        q7 = np.array(q7, float)
        safe = self.q + np.clip(q7 - self.q, -self.mrt, self.mrt)
        if np.any(np.abs(safe - q7) > 1e-9):
            self.clamped += 1
        self.goal = safe

    def stop_now(self):
        self.stop_calls += 1; self.finished = True
        return not self.dead()

    def finish(self):
        if self.dead():
            raise ArmUnresponsive("no answer during the rest move")
        self.finished = True


class SimCams:
    """Renders the board into a virtual wrist camera from the SimArm's current joints."""
    def __init__(self, arm, truth, kin):
        from charuco import Charuco
        self.arm, self.truth, self.kin = arm, truth, kin
        self.board = Charuco()
        self.ppm, self.margin = 8, 15
        self.bimg = self.board.image(self.ppm, self.margin)
        self.t = 0.0

    def frame(self, key):
        from charuco import render_view
        from handeye_solve import inv
        tr = self.truth
        if key == "wrist":
            q = self.arm.q[:6] * np.array([tr["signs"][0], 1, 1, tr["signs"][1], tr["signs"][2], tr["signs"][3]])
            G = self.kin.T(q)
            T_b2c = inv(tr["X"]) @ inv(G) @ tr["Z"]
            K, dist, size = tr["K"], tr["dist"], tr["size"]
        else:
            T_b2c = inv(tr["T_ov2base"]) @ tr["Z"]
            K, dist, size = tr["K_ov"], tr["dist_ov"], tr["size_ov"]
        self.t += 1
        return render_view(self.bimg, self.ppm, self.margin, T_b2c, K, dist, size, blur=0.6, noise=2.0,
                           seed=int(self.t)), time.time()

    def source(self, key):
        cams = self
        class _S:
            def grab(self, after_ts=0.0, timeout=3.0):
                return cams.frame(key)
            def close(self):
                pass
        return _S()


# ---------------------------------------------------------------- executor
class Executor:
    def __init__(self, arm, cams, cfg, out_dir, sim=False, log=say):
        self.arm, self.cams, self.cfg, self.out, self.sim, self.log = arm, cams, cfg, Path(out_dir), sim, log
        self.stop_flag = False
        self.dt = 1 / 30
        self.t_start = time.time()
        self.sim_t = 0.0
        self.n_moves = 0
        self._last_q, self._same_t0 = None, None
        mrt = np.asarray(getattr(arm, "mrt", [cfg.get("follower_mrt_deg", cfg["max_step_deg"])] * 7), float)
        # v3.9.1: the setpoint never leads the measured position by more than lead_frac * max_relative_target, so
        # LeRobot never has to clamp (the clamp is relative to the position at send time; 25 % margin)
        self.lead = cfg.get("lead_frac", 0.75) * mrt[:6]

    def _sleep(self, s):
        if self.sim:
            self.sim_t += s
        else:
            time.sleep(s)

    def _now(self):
        return self.sim_t if self.sim else time.monotonic()

    def _check(self):
        if self.stop_flag:
            raise Stop()
        if not self.sim and time.time() - self.t_start > self.cfg["max_total_s"]:
            raise RuntimeError("time limit")

    def read(self):
        """arm.read() + communication checks: comm exceptions -> ArmUnresponsive; feedback bit-identical for
        frozen_s while we command motion -> ArmUnresponsive (checked in move_to)."""
        try:
            q, v = self.arm.read()
        except (ArmUnresponsive, Stop):
            raise
        except Exception as e:
            if _is_comm_error(e):
                raise ArmUnresponsive(str(e)) from None
            raise
        q = np.asarray(q, float)
        if not np.all(np.isfinite(q)):
            raise ArmUnresponsive("invalid joint reading")
        now = self._now()
        if self._last_q is not None and np.array_equal(q, self._last_q):
            self._same_t0 = self._same_t0 if self._same_t0 is not None else now
        else:
            self._same_t0 = None
        self._last_q = q.copy()
        return q, v

    def _frozen_for(self):
        return 0.0 if self._same_t0 is None else self._now() - self._same_t0

    def command(self, q7):
        try:
            self.arm.command(q7)
        except (ArmUnresponsive, Stop):
            raise
        except Exception as e:
            if _is_comm_error(e):
                raise ArmUnresponsive(str(e)) from None
            raise

    @staticmethod
    def _joint_table(q, sp, tgt):
        return "; ".join(f"{j}: present {q[k]:.1f}, sent {sp[k]:.1f}, target {tgt[k]:.1f}" for k, j in enumerate(ARM_JOINTS))

    def move_to(self, q6_target, grip, speed=None):
        """Joint-space move that tracks reality (v3.9.1).

        Every cycle the setpoint advances towards the target by at most speed*dt, but it is also kept within
        +- lead (= lead_frac * the follower's max_relative_target) of the MEASURED position, so the commanded goal can
        never run ahead of the arm and LeRobot never clamps. The lag test uses the goal that was actually sent.
        Aborts: MotionAbort when a joint is pinned at the lead without progress for stall_s (blocked / too weak /
        guard froze it) or deviates more than max_lag_deg from the sent goal (falling / pushed); ArmUnresponsive when
        the feedback is frozen while motion is commanded. Then waits until reached + settles; returns the median
        joint reading and the joint spread during the settle phase."""
        cfg = self.cfg
        v = float(speed or cfg["speed_dps"])
        step = v * self.dt
        tgt = np.asarray(q6_target, float)
        q, _ = self.read()
        sp = q[:6].copy()
        stall_s = cfg.get("stall_s", 2.5); min_prog = cfg.get("stall_min_progress_deg", 0.5)
        frozen_s = cfg.get("frozen_s", 1.0)
        pin_t0 = [None] * 6; pin_q0 = np.zeros(6)
        tol = np.array([cfg["reach_tol_compliant_deg"] if j in HP.COMPLIANT else cfg["reach_tol_deg"] for j in ARM_JOINTS])
        d0 = float(np.max(np.abs(tgt - sp)))
        t_limit = self._now() + 1.5 * d0 / v + cfg["reach_timeout_s"] + 5.0
        first = self.n_moves == 0
        self.n_moves += 1
        try:
            while True:
                self._check()
                ideal = sp + np.clip(tgt - sp, -step, step)
                sp = np.clip(ideal, q[:6] - self.lead, q[:6] + self.lead)
                self.command(np.append(sp, grip))
                q, _ = self.read()
                now = self._now()
                lag = np.abs(q[:6] - sp)
                if np.max(lag) > cfg["max_lag_deg"]:
                    j = int(np.argmax(lag))
                    raise MotionAbort(f"Gelenk {ARM_JOINTS[j]} weicht {lag[j]:.1f}° vom gesendeten Ziel ab (fällt / "
                                      f"wird gedrückt?) [{self._joint_table(q, sp, tgt)}]")
                moving = np.any(np.abs(tgt - q[:6]) > tol)
                if moving and self._frozen_for() >= frozen_s:
                    raise ArmUnresponsive(f"joint feedback frozen for {self._frozen_for():.1f} s while moving")
                # stall: joint pinned at the lead (wants to go further) without real progress
                for k in range(6):
                    pinned = abs(ideal[k] - q[k]) >= 0.9 * self.lead[k] and abs(tgt[k] - q[k]) > tol[k]
                    if not pinned:
                        pin_t0[k] = None
                    elif pin_t0[k] is None:
                        pin_t0[k], pin_q0[k] = now, q[k]
                    elif now - pin_t0[k] >= stall_s:
                        if abs(q[k] - pin_q0[k]) < min_prog:
                            raise MotionAbort(f"Gelenk {ARM_JOINTS[k]} kommt nicht voran ({abs(q[k] - pin_q0[k]):.1f}° in "
                                              f"{now - pin_t0[k]:.1f} s) – blockiert oder zu schwach? "
                                              f"[{self._joint_table(q, sp, tgt)}]")
                        pin_t0[k], pin_q0[k] = now, q[k]
                if np.all(sp == tgt) and np.all(np.abs(q[:6] - tgt) <= tol):
                    break
                if now > t_limit:
                    raise MotionAbort(f"Ziel nicht erreicht: error {np.round(q[:6] - tgt, 1).tolist()} deg "
                                      f"[{self._joint_table(q, sp, tgt)}]")
                self._sleep(self.dt)
        except MotionAbort:
            if first:
                self.log("first move failed - per joint: " + self._joint_table(q, sp, tgt))
            raise
        # settle while holding the target, then median joint reading
        samples = []
        for k in range(int(cfg["settle_s"] / self.dt) + 10):
            self._check()
            self.command(np.append(tgt, grip))
            q, _ = self.read()
            if k * self.dt >= cfg["settle_s"]:
                samples.append(q[:6])
            self._sleep(self.dt)
        qs = np.array(samples)
        return np.median(qs, 0), float(np.max(np.ptp(qs, 0)))

    def hold_while(self, fn, q6, grip):
        """Run fn() in a thread (e.g. the ~40 s planning) while holding q6 and watching the arm (Stop, comm loss)."""
        import threading
        box = {}
        def work():
            try:
                box["r"] = fn()
            except BaseException as e:      # noqa: BLE001 - re-raised in the caller's thread
                box["e"] = e
        th = threading.Thread(target=work, daemon=True); th.start()
        while th.is_alive():
            self._check()
            self.command(np.append(q6, grip))
            self.read()
            if self._frozen_for() >= self.cfg.get("frozen_s", 1.0) * 3:
                raise ArmUnresponsive("joint feedback frozen while holding")
            self._sleep(self.dt)
            if self.sim:
                time.sleep(0.01)
        if "e" in box:
            raise box["e"]
        return box["r"]

    def detect_wrist(self, board):
        """Fresh wrist frame + ChArUco detection. -> (det or None, frame, n_corners)."""
        img, _ = self.cams["wrist"].grab(after_ts=time.time() if not self.sim else 0.0)
        det = board.detect(img)
        return det, img, 0 if det is None else len(det["ids"])

    def auto_start(self, board, guess=None, cam_model=None, plan_after=True):
        """v3.9.1 automatic start (no taught pose): slow move (approach_speed_dps) from wherever the arm is to a high
        search pose looking down at the expected board point, detect the board in the wrist camera, estimate its centre
        in the robot frame, move to a pose aim_height_mm above it, re-measure (up to auto_iters, the estimates must
        agree within auto_tol_mm), then plan the sweep from that pose while holding it. -> plan
        (plan_after=False, tests: -> dict(q_teach, board_centre_mm) without planning)."""
        cfg, kin = self.cfg, Kin()
        v = cfg.get("approach_speed_dps", 6.0)
        q7, _ = self.read(); grip = float(q7[6])
        P0 = np.asarray(guess if guess is not None else board_guess(cfg)[0], float)
        sp = HP.search_pose(P0, cfg, kin)
        q_s = np.array(sp["q"])
        ok, clr, why = HP.path_safety(kin, q7[:6], q_s, cfg)
        if not ok:
            raise RuntimeError(f"path from the current arm pose to the search pose is not safe ({why}); "
                               f"move the arm to rest and retry")
        self.log(f"start: current joints {np.round(q7[:6], 1).tolist()}, moving slowly ({v:.0f} deg/s) to the search pose "
                 f"{sp['height_mm']:.0f} mm above {np.round(P0, 0).tolist()} mm (min clearance {clr:.0f} mm)")
        self.move_to(q_s, grip, speed=v)
        z_plane = float(cfg["table_z_mm"])
        q_cur, P, hist = q_s, None, []
        for it in range(int(cfg.get("auto_iters", 3)) + 1):
            self._check()
            self._sleep(0.3)
            det, img, n = self.detect_wrist(board)
            cv2.imwrite(str(self.out / f"auto_{it}.png"), img)
            if n < cfg["min_corners"]:
                raise BoardNotFound(f"{MSG_NO_BOARD} ({n} von {board.n_corners} Ecken erkannt, mindestens "
                                    f"{cfg['min_corners']} nötig)")
            model = cam_model or wrist_model((img.shape[1], img.shape[0]))
            P_new, info = estimate_board_centre(det, model, q_cur, kin, z_plane, board)
            hist.append({"iter": it, "q": [float(x) for x in q_cur], "corners": n, "P": P_new.tolist(), **info})
            self.log(f"  board seen: {n} corners, centre {np.round(P_new, 0).tolist()} mm (camera model {info['model']}, rotation {info['rotation']})")
            if P is not None and np.linalg.norm(P_new - P) <= cfg.get("auto_tol_mm", 25.0):
                P_fin = 0.5 * (P + P_new)
                if np.linalg.norm(P_new - P) > 10.0:     # small final correction
                    q_a, _, _ = HP.aim_pose(P_fin, q_cur, cfg, kin)
                    self._checked_move(kin, q_cur, q_a, grip, v); q_cur = q_a
                P = P_fin
                break
            if it == int(cfg.get("auto_iters", 3)):
                raise RuntimeError("board position estimates do not agree (wrist camera mounting unknown): teach the "
                                   "pose once with 📍 Ziel-Pose übernehmen")
            P = P_new
            q_a, tilt, _ = HP.aim_pose(P, q_cur, cfg, kin)
            self.log(f"  moving to {cfg.get('aim_height_mm', 200):.0f} mm above the board centre (tilt {tilt:.0f} deg)")
            self._checked_move(kin, q_cur, q_a, grip, v); q_cur = q_a
        (self.out / "auto_start.json").write_text(json.dumps({"guess_mm": P0.tolist(), "search_pose": sp,
                                                              "board_centre_mm": P.tolist(), "q_teach": [float(x) for x in q_cur],
                                                              "iterations": hist}, indent=1))
        if not plan_after:
            return {"q_teach": np.asarray(q_cur, float), "board_centre_mm": P}
        self.log(f"start pose found: board centre {np.round(P, 0).tolist()} mm; planning the poses (~40 s, arm holds still)")
        cfg_run = dict(cfg); cfg_run["table_z_mm"] = z_plane
        plan = self.hold_while(lambda: HP.plan_poses(q_cur, cfg_run, kin, log=lambda *a: None), q_cur, grip)
        plan["time"] = time.strftime("%Y-%m-%d %H:%M:%S"); plan["auto_start"] = True
        (self.out / "plan.json").write_text(json.dumps(plan, indent=1))
        try:
            HP.plot_plan(plan, self.out / "plan.png")
        except Exception:
            pass
        print(HP.summary_text(plan), flush=True)
        return plan

    def _checked_move(self, kin, qa, qb, grip, v):
        ok, clr, why = HP.path_safety(kin, qa, qb, self.cfg)
        if not ok:
            raise RuntimeError(f"path to the board pose not safe ({why})")
        self.move_to(qb, grip, speed=v)

    def capture(self, i, board, hold):
        """Two wrist frames 0.25 s apart (corner motion < 0.7 px = no blur / no shake) + one overview frame."""
        t0 = time.time()
        for attempt in range(3):
            self._check()
            a, ta = self.cams["wrist"].grab(after_ts=t0)
            hold()
            self._sleep(0.25)
            b, tb = self.cams["wrist"].grab(after_ts=ta + 1e-3 if not self.sim else 0)
            da, db = board.detect(a), board.detect(b)
            n = 0 if db is None else len(db["ids"])
            if da is not None and db is not None and n >= self.cfg["min_corners"]:
                common = np.intersect1d(da["ids"], db["ids"])
                ia = {k: p for k, p in zip(da["ids"], da["img"])}; ib = {k: p for k, p in zip(db["ids"], db["img"])}
                motion = float(np.mean([np.linalg.norm(ia[c] - ib[c]) for c in common])) if len(common) else 99
                if motion < 0.7:
                    ov, _ = self.cams["overview"].grab(after_ts=t0)
                    cv2.imwrite(str(self.out / f"wrist_{i:02d}.png"), b)
                    cv2.imwrite(str(self.out / f"overview_{i:02d}.png"), ov)
                    return {"corners": n, "motion_px": motion, "wrist": f"wrist_{i:02d}.png", "overview": f"overview_{i:02d}.png"}
                self.log(f"  pose {i}: image not steady ({motion:.2f} px), retry")
            else:
                self.log(f"  pose {i}: only {n} board corners in the wrist image" + (", retry" if attempt < 2 else ""))
            t0 = time.time()
        cv2.imwrite(str(self.out / f"rejected_wrist_{i:02d}.png"), b)
        return None

    def run(self, plan, board):
        self.out.mkdir(parents=True, exist_ok=True)
        kin = Kin()
        q_teach = np.array(plan["q_teach"], float)
        q7, _ = self.read()
        grip = float(q7[6])
        # start path: current -> taught pose (checked); if unsafe, refuse
        ok, clr, why = HP.path_safety(kin, q7[:6], q_teach, self.cfg)
        if not ok:
            raise RuntimeError(f"path from the current arm pose to the taught pose is not safe ({why}); "
                               f"move the arm to rest and retry")
        if np.max(np.abs(q7[:6] - q_teach)) > 1.0:
            self.log(f"start: current joints {np.round(q7[:6], 1).tolist()}, moving slowly "
                     f"({self.cfg.get('approach_speed_dps', 6.0):.0f} deg/s) to the taught pose (min clearance {clr:.0f} mm)")
        self.move_to(q_teach, grip, speed=self.cfg.get("approach_speed_dps", 6.0))
        samples, rows = [], []
        for i, p in enumerate(plan["poses"], 1):
            self._check()
            self.log(f"pose {i}/{len(plan['poses'])}: height {p['height_mm']:.0f} mm, tilt {p['tilt_deg']:.0f}, roll {p['roll_deg']:.0f}")
            q_meas, ptp = self.move_to(p["q"], grip)
            hold = lambda: self.command(np.append(np.array(p["q"]), grip))
            c = self.capture(i, board, hold)
            row = {"i": i, "q": q_meas.tolist(), "q_planned": p["q"], "joint_ptp_deg": ptp, "time": time.time()}
            if c:
                row.update(c); samples.append(row)
                self.log(f"  ok: {c['corners']} corners, joints steady within {ptp:.2f} deg")
            else:
                rows.append(row)
            if i == 2 and not samples:
                raise RuntimeError("the wrist camera does not see the board at the first poses: re-teach the aim pose")
            (self.out / "samples.json").write_text(json.dumps({"samples": samples, "rejected": rows, "plan": plan,
                                                               "board": board.spec}, indent=1))
        self.log(f"sweep done: {len(samples)} good samples, back to the taught pose")
        self.move_to(q_teach, grip, speed=self.cfg.get("approach_speed_dps", 6.0))
        return samples


# ---------------------------------------------------------------- v3.9.1 auto start: board estimate from one wrist image
def board_guess(cfg=None):
    """Expected board centre (robot base frame, mm) for the search pose: previous hand-eye result (handeye.json
    T_board2base) > taught pose look point > config board_guess_mm (default [463, -46, -70])."""
    cfg = cfg or HP.load_config()
    try:
        if RESULT_JSON.exists():
            from charuco import Charuco
            r = json.loads(RESULT_JSON.read_text())
            Z = np.array(r["T_board2base"], float)
            w, h = Charuco().size_mm
            return Z[:3, :3] @ np.array([w / 2, h / 2, 0.0]) + Z[:3, 3], "handeye.json"
    except Exception:
        pass
    try:
        if HP.TEACH.exists():
            return np.array(json.loads(HP.TEACH.read_text())["look_point_mm"], float), "taught pose"
    except Exception:
        pass
    return np.array(cfg.get("board_guess_mm", HP.DEFAULT_BOARD_POINT), float), "default"


def wrist_model(size, cfg=None):
    """Wrist camera model for the auto start. From handeye.json if a calibration exists (K, dist, camera->gripper,
    joint signs); otherwise a prior: pinhole with wrist_hfov_deg_prior, no distortion, camera at
    camera_prior.offset_mm, rotation estimated per image (see estimate_board_centre)."""
    cfg = cfg or HP.load_config()
    try:
        if RESULT_JSON.exists():
            from charuco import scale_K
            r = json.loads(RESULT_JSON.read_text())
            wi = r["wrist_intrinsics"]
            K = np.array(wi["K"], float)
            if tuple(wi["size"]) != tuple(size):
                K = scale_K(K, wi["size"], size)
            return {"K": K, "dist": np.array(wi["dist"], float), "X": np.array(r["T_cam2gripper"], float),
                    "signs": tuple(r.get("joint_signs", (1, 1, 1, 1))), "model": "handeye.json"}
    except Exception:
        pass
    w, h = size
    f = 0.5 * w / np.tan(np.radians(cfg.get("wrist_hfov_deg_prior", 90.0)) / 2)
    K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1.0]])
    return {"K": K, "dist": np.zeros(5), "X": None, "signs": None,
            "offset": np.array(cfg["camera_prior"]["offset_mm"], float), "model": "prior"}


def _unit(v):
    return v / np.linalg.norm(v)


def estimate_board_centre(det, model, q, kin, z_plane, board):
    """Board centre in the robot base frame (mm) from ONE wrist image, without trusting the image depth.

    The centre pixel comes from the homography of the detected corners (independent of the camera model). Its viewing
    ray is intersected with the board plane z = z_plane (table_z_mm, measured by touch), so a wrong focal length only
    tilts the ray slightly instead of scaling the distance. Camera rotation in the gripper frame: from handeye.json,
    else TRIAD from two direction pairs - board normal (PnP) <-> gravity (base -z seen from the gripper, FK) and
    optical axis <-> finger axis - which works for any roll of the camera mount as long as the view is tilted
    (search/aim poses are tilted 20-30 deg). -> (P, info)."""
    K, dist = model["K"], model["dist"]
    G = kin.T(q, model.get("signs"))
    w, h = board.size_mm
    Hm, _ = cv2.findHomography(det["obj"][:, :2], det["img"])
    if Hm is None:
        raise BoardNotFound(f"{MSG_NO_BOARD} (board geometry not solvable)")
    c = Hm @ np.array([w / 2, h / 2, 1.0]); c_px = c[:2] / c[2]
    T_b2c, err = board.pose(det, K, dist)
    info = {"model": model["model"], "centre_px": c_px.tolist()}
    if model.get("X") is not None:
        R_cg, t_cg = model["X"][:3, :3], model["X"][:3, 3]
        info["rotation"] = "handeye.json"
    else:
        t_cg = model["offset"]
        R_cg = np.column_stack([[0, 1.0, 0], [0, 0, 1.0], [1.0, 0, 0]])      # prior: optical axis = finger axis
        info["rotation"] = "prior"
        if T_b2c is not None:
            n_c = T_b2c[:3, 2] * (1 if T_b2c[2, 2] > 0 else -1)                # board normal, pointing away from the camera
            if np.degrees(np.arccos(np.clip(n_c[2], -1, 1))) > 8.0:            # tilted view -> roll observable
                n_g = G[:3, :3].T @ np.array([0, 0, -1.0])                     # 'down' in the gripper frame
                Tc = np.column_stack([n_c, _unit(np.cross(n_c, [0, 0, 1.0])), np.zeros(3)])
                Tc[:, 2] = np.cross(Tc[:, 0], Tc[:, 1])
                Tg = np.column_stack([n_g, _unit(np.cross(n_g, [1.0, 0, 0])), np.zeros(3)])
                Tg[:, 2] = np.cross(Tg[:, 0], Tg[:, 1])
                R_cg = Tg @ Tc.T
                info["rotation"] = "triad"
    und = cv2.undistortPoints(c_px.reshape(1, 1, 2), K, dist).reshape(2)
    ray = G[:3, :3] @ R_cg @ _unit(np.array([und[0], und[1], 1.0]))
    o = G[:3, :3] @ t_cg + G[:3, 3]
    if ray[2] > -0.2:
        raise BoardNotFound(f"{MSG_NO_BOARD} (view ray does not point down at the table)")
    P = o + (z_plane - o[2]) / ray[2] * ray
    if T_b2c is not None:
        Xc = np.eye(4); Xc[:3, :3] = R_cg; Xc[:3, 3] = t_cg
        info["z_from_depth_mm"] = float((G @ Xc @ T_b2c @ np.array([w / 2, h / 2, 0, 1.0]))[2])
        info["reproj_px"] = err
    return P, info


def new_out_dir(prefix=""):
    d = HE_DIR / (prefix + time.strftime("%Y%m%d_%H%M%S"))
    d.mkdir(parents=True, exist_ok=True)
    return d


def auto_preview(cfg=None, log=say):
    """`plan --auto`: offline preview of the automatic start (search pose + path check from rest). No hardware."""
    cfg = cfg or HP.load_config(); kin = Kin()
    P0, src = board_guess(cfg)
    sp = HP.search_pose(P0, cfg, kin)
    ok, clr, why = HP.path_safety(kin, [0.0] * 6, sp["q"], cfg)
    d = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "guess_mm": P0.tolist(), "guess_source": src, "search_pose": sp,
         "rest_path_ok": ok, "rest_path_clearance_mm": clr}
    AUTO_JSON.write_text(json.dumps(d, indent=1))
    log(f"auto start: search pose {sp['height_mm']:.0f} mm above {np.round(P0, 0).tolist()} mm ({src}), tilt "
        f"{sp['tilt_deg']:.0f} deg, joints {np.round(sp['q'], 1).tolist()}, clearance {sp['clearance_mm']:.0f} mm; "
        f"path from rest {'safe' if ok else 'NOT safe'} (min {clr:.0f} mm)")
    return d


def run(yes=False, sim=False, sim_truth=None, out_dir=None, apply_good=True, plan=None, log=say, auto=False,
        sim_guess=None, arm=None, cams=None, cam_model=None):
    """auto=True (v3.9.1): no taught pose / plan needed - the executor finds the board itself (Executor.auto_start).
    arm / cams / cam_model: test hooks (fake followers)."""
    from charuco import Charuco
    cfg = HP.load_config()
    if not auto:
        plan = plan or (json.loads(PLAN_JSON.read_text()) if PLAN_JSON.exists() else None)
        if plan is None:
            raise RuntimeError("no plan: run `handeye.py plan` first (or `run --auto`)")
    if not sim and arm is None:
        if not auto:
            teach_q = load_teach()["q"]
            if np.max(np.abs(np.array(teach_q) - np.array(plan["q_teach"]))) > 0.01:
                raise RuntimeError("the plan does not belong to the current taught pose: run `handeye.py plan` again")
        import play_policy as pp
        conf = pp.conflicts()
        if conf:
            raise RuntimeError("refused: " + "; ".join(conf))
    if auto:
        if not sim:
            auto_preview(cfg, log=log)
    else:
        print(HP.summary_text(plan), flush=True)
    if not yes and not sim:
        a = input("The ARM WILL MOVE. Hand on Stop (Ctrl+C) / power strip. Type 'yes' to start: ")
        if a.strip().lower() != "yes":
            log("cancelled"); return None
    board = Charuco()
    out = Path(out_dir) if out_dir else new_out_dir("sim_" if sim else "")
    out.mkdir(parents=True, exist_ok=True)
    kin = Kin()
    mrt_deg = cfg.get("follower_mrt_deg", cfg["max_step_deg"])
    if arm is not None:
        pass
    elif sim:
        arm = SimArm(list(sim_truth["q_start"]) + [0.0], mrt_deg)
        sc = SimCams(arm, sim_truth, kin)
        cams = {"wrist": sc.source("wrist"), "overview": sc.source("overview")}
    else:
        cams = open_cameras()
        log("connecting the follower (collision guard ON, max_relative_target %.1f deg, approach %.0f deg/s, sweep %.0f deg/s)"
            % (mrt_deg, cfg.get("approach_speed_dps", 6.0), cfg["speed_dps"]))
        try:
            arm = RealArm(mrt_deg, frozen_s=cfg.get("frozen_s", 1.0))
        except ArmUnresponsive as e:
            log(f"ERROR: {MSG_NO_ARM} ({e})")
            for c in cams.values():
                try: c.close()
                except Exception: pass
            return {"rc": 3, "folder": str(out)}
    ex = Executor(arm, cams or {}, cfg, out, sim=sim or isinstance(arm, SimArm), log=log)
    phase = {"p": "run"}

    def on_sig(signum, _f):
        ex.stop_flag = True
        if phase["p"] == "rest":
            raise Stop()
    real_signals = not sim and threading_main()
    old = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
    if real_signals:
        signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)
    rc, samples = 0, []
    off_only = False         # True: arm unresponsive or did not follow -> no further motion, just torque off once
    try:
        log("RUNNING: auto calibration - STOP = motors off immediately")
        if auto:
            plan = ex.auto_start(board, guess=sim_guess, cam_model=cam_model)
        samples = ex.run(plan, board)
    except Stop:
        log("STOP: motors off now"); arm.stop_now(); rc = 130
    except ArmUnresponsive as e:
        log(f"ERROR: {MSG_NO_ARM} ({e})"); rc = 3; off_only = True
    except MotionAbort as e:
        log(f"ABORT: {e}"); rc = 1; off_only = True
    except BoardNotFound as e:
        log(f"ERROR: {e}"); rc = 4
    except Exception as e:
        log(f"ABORT: {e}"); rc = 1
        try:
            if not ex.stop_flag and plan is not None:
                q_teach = np.array(plan["q_teach"]); q, _ = ex.read()
                if HP.path_safety(kin, q[:6], q_teach, cfg)[0]:
                    log("returning to the taught pose")
                    ex.move_to(q_teach, float(q[6]), speed=cfg.get("approach_speed_dps", 6.0))
        except Stop:
            log("STOP: motors off now"); arm.stop_now(); rc = 130
        except ArmUnresponsive as e2:
            log(f"ERROR: {MSG_NO_ARM} ({e2})"); rc = 3; off_only = True
        except Exception as e2:
            log(f"return failed ({e2})"); off_only = True
    finally:
        if rc != 130:
            if off_only:
                ok = arm.stop_now()
                log("motors off (no further motion)" if ok else f"motors off: no answer from the arm – {MSG_NO_ARM}")
            else:
                phase["p"] = "rest"
                try:
                    log("rest move (safe_zero), then motors off"); arm.finish(); log("rest reached, motors off")
                except Stop:
                    arm.stop_now(); log("STOP: motors off now")
                except Exception as e:
                    ok = arm.stop_now()
                    log(f"rest move failed ({str(e)[:120]}): motors off" + ("" if ok else f" – no answer, {MSG_NO_ARM}"))
        n_cl = getattr(getattr(arm, "clamps", None), "n", 0) or getattr(arm, "clamped", 0)
        if n_cl:
            log(f"note: LeRobot clamped {n_cl} commands (max_relative_target)")
        for c in (cams or {}).values():
            try: c.close()
            except Exception: pass
        if real_signals:
            signal.signal(signal.SIGINT, old[0]); signal.signal(signal.SIGTERM, old[1])
    log(f"samples saved in {out}")
    if len(samples) < 6:
        log(f"only {len(samples)} good samples: not enough to calibrate"); return {"rc": rc or 2, "folder": str(out)}
    r = replay(out, apply=apply_good and not sim, log=log)
    return {"rc": rc, "folder": str(out), "result": r}


# ---------------------------------------------------------------- solve / apply
def replay(folder, apply=False, force=False, intrinsics=None, log=say):
    import handeye_solve as HS
    cfg = HP.load_config()
    r = HS.solve_folder(folder, intrinsics=intrinsics, log=log, table_z_mm=cfg["table_z_mm"])
    (Path(folder) / "result.json").write_text(json.dumps(r, indent=1))
    print(HS.report_text(r), flush=True)
    if apply:
        if r["quality"]["grade"] == "poor" and not force:
            log("quality POOR: NOT applied (check the board is taped flat and visible; or `apply <folder> --force`)")
        else:
            apply_result(r)
    return r


def apply_result(r, log=say):
    import calibrate as cal
    RESULT_JSON.write_text(json.dumps(r, indent=1))
    hist = HE_DIR / "applied"; hist.mkdir(parents=True, exist_ok=True)
    (hist / (time.strftime("handeye_%Y%m%d_%H%M%S") + ".json")).write_text(json.dumps(r, indent=1))
    msg = cal.apply_handeye(r)
    log(f"applied: {RESULT_JSON.name} + table.json ({msg})")


def status():
    if not RESULT_JSON.exists():
        print("no hand-eye calibration yet"); return
    import handeye_solve as HS
    r = json.loads(RESULT_JSON.read_text())
    print(f"applied {r['time']} from {r['folder']}"); print(HS.report_text(r))


def main():
    ap = argparse.ArgumentParser(description="automatic hand-eye calibration")
    ap.add_argument("cmd", choices=["teach", "plan", "run", "replay", "apply", "status"])
    ap.add_argument("folder", nargs="?")
    ap.add_argument("--dry-run", action="store_true", help="plan: same as plan (never moves)")
    ap.add_argument("--yes", action="store_true", help="run: no terminal confirmation (the GUI asks instead)")
    ap.add_argument("--sim", action="store_true", help="run: simulated arm + rendered images")
    ap.add_argument("--auto", action="store_true", help="plan/run: automatic start (find the board, no taught pose)")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--no-apply", action="store_true")
    ap.add_argument("--port", type=int)
    a = ap.parse_args()
    if a.cmd in ("replay", "apply") and not a.folder:
        runs = sorted(d for d in HE_DIR.glob("2*") if (d / "samples.json").exists()) if HE_DIR.exists() else []
        if not runs:
            say("no sample folder yet"); sys.exit(2)
        a.folder = str(runs[-1]); say(f"newest sample folder: {a.folder}")
    try:
        if a.cmd == "teach":
            teach(a.port)
        elif a.cmd == "plan" and a.auto:
            auto_preview()
        elif a.cmd == "plan":
            plan = make_plan()
            print(HP.summary_text(plan)); say(f"plan: {PLAN_JSON}  plot: {PLAN_PNG}  (dry run, nothing moved)")
        elif a.cmd == "run":
            if a.sim:
                import handeye_synth as SY
                if a.auto:
                    truth = SY.make_truth()
                    r = run(yes=True, sim=True, sim_truth=truth, auto=True, sim_guess=SY.board_guess(truth),
                            cam_model=SY.prior_model(truth))
                else:
                    truth, plan = SY.make_truth_and_plan()
                    r = run(yes=True, sim=True, sim_truth=truth, plan=plan)
                if r and r.get("result"):
                    SY.compare(truth, r["result"])
                sys.exit(0 if r and r.get("rc", 1) == 0 else 1)
            else:
                r = run(yes=a.yes, apply_good=not a.no_apply, auto=a.auto)
                sys.exit(0 if r and r.get("rc", 1) == 0 else 1)
        elif a.cmd == "replay":
            replay(a.folder, apply=False)
        elif a.cmd == "apply":
            r = json.loads((Path(a.folder) / "result.json").read_text()) if (Path(a.folder) / "result.json").exists() else replay(a.folder)
            if r["quality"]["grade"] == "poor" and not a.force:
                say("quality POOR: use --force to apply anyway"); sys.exit(1)
            apply_result(r)
        elif a.cmd == "status":
            status()
    except (RuntimeError, ValueError) as e:
        say(f"ERROR: {e}"); sys.exit(2)


if __name__ == "__main__":
    main()
