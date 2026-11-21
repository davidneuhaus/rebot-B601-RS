"""Pure-software grasp sequence planner. Output = joint waypoints (raw follower degrees) + gripper targets.
NO execution code: nothing here imports a robot driver or talks to motors. A future executor must still apply its own
speed limits and safety checks (guard, stop button) to the waypoints.

Sequence: pre-grasp (above, jaws open) -> descend (Cartesian steps, height lock) -> close -> lift -> [carry above
place spot -> descend -> release] -> lift -> return to rest.

Height lock: the fingertip z is never planned below  table_z + min_tip_clearance_mm  (grasp depth is clamped, the
clamp is reported). Every waypoint must be reachable (ReachChecker, IK seeded from the previous waypoint so the
joints stay in one branch), and the joint-space path between waypoints must keep tip and wrist above their floors.
"""
import json
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from . import GRASP_DIR
from .locate import ReachChecker

PROFILES_JSON = GRASP_DIR / "profiles.json"


class PlanError(RuntimeError):
    pass


def load_profiles(path=None):
    return json.loads(Path(path or PROFILES_JSON).read_text())


def pick_profile(label, cfg):
    """-> (name, profile dict). First profile whose 'match' substring occurs in the label (case-insensitive)."""
    low = (label or "").lower()
    for name, p in cfg["profiles"].items():
        if name != "default" and any(m in low for m in p.get("match", [])):
            return name, p
    return "default", cfg["profiles"]["default"]


class GripperMap:
    def __init__(self, g):
        self.g = g
        self.calibrated = bool(g.get("calibrated"))

    def cmd(self, mm):
        g = self.g
        mm = float(np.clip(mm, g["closed_mm"], g["max_open_mm"]))
        f = (mm - g["closed_mm"]) / (g["max_open_mm"] - g["closed_mm"])
        return g["closed_cmd_deg"] + f * (g["max_open_cmd_deg"] - g["closed_cmd_deg"])

    @property
    def max_open_mm(self):
        return float(self.g["max_open_mm"])


@dataclass
class Waypoint:
    name: str
    kind: str                       # approach | descend | close | lift | carry | release | return
    q: list                         # 6 raw joint degrees
    tip_mm: Optional[list]
    gripper_mm: float
    gripper_cmd_deg: float
    speed_scale: float
    note: str = ""

    def to_json(self):
        return dict(name=self.name, kind=self.kind, q=[round(float(v), 2) for v in self.q],
                    tip_mm=None if self.tip_mm is None else [round(float(v), 1) for v in self.tip_mm],
                    gripper_mm=round(self.gripper_mm, 1), gripper_cmd_deg=round(self.gripper_cmd_deg, 1),
                    speed_scale=self.speed_scale, note=self.note)


@dataclass
class GraspPlan:
    ok: bool
    reason: str
    profile: str = ""
    waypoints: List[Waypoint] = field(default_factory=list)
    success_check: dict = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    info: dict = field(default_factory=dict)

    def to_json(self):
        return dict(ok=self.ok, reason=self.reason, profile=self.profile, warnings=self.warnings, info=self.info,
                    success_check=self.success_check, waypoints=[w.to_json() for w in self.waypoints],
                    note="DRY RUN: joint waypoints only, nothing was sent to the robot")


def success_check_spec(width_mm, gmap, check_cfg, closing_mm):
    """What an executor must verify after the close step (spec only, evaluated by a future executor)."""
    tol = check_cfg["width_tolerance_mm"]
    lo, hi = max(0.0, width_mm - tol), width_mm + tol
    return {
        "after_waypoint": "close", "settle_s": check_cfg["settle_s"], "signal": "gripper.pos (deg)",
        "held_if_opening_mm_between": [round(lo, 1), round(hi, 1)],
        "held_if_cmd_deg_between": [round(gmap.cmd(lo), 1), round(gmap.cmd(hi), 1)],
        "empty_if_cmd_deg_below": round(gmap.cmd(max(0.0, width_mm - tol) * 0.5 + closing_mm * 0.5), 1),
        "also_require": "gripper position stable (< 1 deg change over the last 0.2 s)",
        "on_fail": f"open to approach width, lift to pre-grasp, retry up to {check_cfg['retries']}x, then abort",
        "calibrated_gripper_map": gmap.calibrated,
    }


def _path_check(reach, qa, qb, table_z, tip_floor, wrist_floor, step_deg=1.0):
    """Joint-space interpolation: tip z >= tip_floor, wrist joints z >= wrist_floor. -> (ok, why)."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    n = max(2, int(np.ceil(np.max(np.abs(qb - qa)) / step_deg)) + 1)
    sg = reach.sg
    for s in np.linspace(0, 1, n):
        qu = (qa + s * (qb - qa)) * sg
        T = reach.kin.T(qu)
        tip = T[:3, 3] + T[:3, 0] * reach.tip_off
        if tip[2] < tip_floor - 2.0:
            return False, f"tip would pass {tip_floor - tip[2]:.0f} mm below the height lock at {s:.2f}"
        low = min(p[2] for nme, p in reach.kin.frames(qu) if nme in ("joint4", "joint5", "joint6"))
        if low < wrist_floor:
            return False, f"wrist would pass {wrist_floor - low:.0f} mm below its floor at {s:.2f}"
    return True, ""


def plan_grasp(target, label, reach: ReachChecker, table_z_mm, rest_q=None, place=None, seed_q=None,
               cfg=None, profile_name=None):
    """target: dict from locate.locate_object (xyz_mm = object top-face centre, jaw_yaw_deg, width_mm).
    place: optional dict(xyz_mm=[x, y, surface_z]) where the object is set down. rest_q: joints for the final return.
    Never raises for 'not reachable': returns GraspPlan(ok=False, reason=...)."""
    cfg = cfg or load_profiles()
    name, prof = (profile_name, cfg["profiles"][profile_name]) if profile_name else pick_profile(label, cfg)
    mo, gm = cfg["motion"], GripperMap(cfg["gripper"])
    plan = GraspPlan(False, "", name)
    if not gm.calibrated:
        plan.warnings.append("gripper mm->deg map is a placeholder (profiles.json gripper.calibrated=false): "
                             "gripper_cmd_deg values are NOT valid for the real gripper yet")
    width = prof.get("width_mm") or target.get("width_mm")
    if not width:
        plan.reason = "object width unknown (profile has none and the outline gave none)"
        return plan
    width = float(width)
    open_mm = width + prof["approach_open_margin_mm"]
    if open_mm > gm.max_open_mm:
        if width + 4.0 > gm.max_open_mm:
            plan.reason = f"object ({width:.0f} mm) does not fit into the gripper (max {gm.max_open_mm:.0f} mm)"
            return plan
        open_mm = gm.max_open_mm
        plan.warnings.append("approach opening limited by the gripper maximum")
    close_mm = max(0.0, width - prof.get("squeeze_mm", 1.0))
    release_mm = prof.get("release_width_mm") or open_mm

    top = np.asarray(target["xyz_mm"], float)
    yaw = float(target.get("jaw_yaw_deg") or 0.0)
    tip_floor = table_z_mm + mo["min_tip_clearance_mm"]
    wrist_floor = reach.table_z + reach.wrist_clear
    z_grasp = top[2] - prof["insert_depth_mm"]
    if z_grasp < tip_floor:
        plan.warnings.append(f"height lock: grasp z raised from {z_grasp - table_z_mm:.0f} to "
                             f"{mo['min_tip_clearance_mm']:.0f} mm above the table")
        z_grasp = tip_floor
    z_pre = top[2] + mo["approach_height_mm"]

    wps: List[Waypoint] = []
    state = {"q": None, "yaw": yaw}

    def add(kind, nm, xyz, gmm, note=""):
        r = reach.check(xyz, state["yaw"], seed_q=state["q"] if state["q"] is not None else seed_q)
        if not r.ok:
            raise PlanError(f"{nm}: tip {np.round(xyz, 0).tolist()} mm not reachable ({r.reason})")
        if state["q"] is None:
            state["yaw"] = r.jaw_yaw_deg            # keep the flip that worked for the whole sequence
        if state["q"] is not None:
            ok, why = _path_check(reach, state["q"], r.q, table_z_mm, tip_floor, wrist_floor)
            if not ok:
                raise PlanError(f"{nm}: path from previous waypoint unsafe ({why})")
            jump = float(np.max(np.abs(np.asarray(r.q) - np.asarray(state["q"]))))
            if jump > 60:
                raise PlanError(f"{nm}: needs a {jump:.0f} deg joint jump (IK branch change)")
        state["q"] = r.q
        wps.append(Waypoint(nm, kind, r.q, list(map(float, xyz)), float(gmm), gm.cmd(gmm),
                            float(mo["speed_scale"].get({"carry": "approach", "release": "place"}.get(kind, kind), 1.0)), note))

    def column(kind, prefix, xy, z_from, z_to, gmm):
        n = max(1, int(np.ceil(abs(z_to - z_from) / mo["cartesian_step_mm"])))
        for i in range(1, n + 1):
            add(kind, f"{prefix}_{i}", [xy[0], xy[1], z_from + (z_to - z_from) * i / n], gmm)

    try:
        pxy = top[:2]
        add("approach", "pre_grasp", [pxy[0], pxy[1], z_pre], open_mm, "above the object, jaws open")
        column("descend", "descend", pxy, z_pre, z_grasp, open_mm)
        add("close", "close", [pxy[0], pxy[1], z_grasp], close_mm, "close; then run success_check")
        column("lift", "lift", pxy, z_grasp, z_pre, close_mm)
        if place is not None:
            P = np.asarray(place["xyz_mm"], float)
            z_set = max(tip_floor, P[2] + prof["top_height_mm"] - prof["insert_depth_mm"] + 2.0)
            z_carry = max(z_pre, P[2] + prof["top_height_mm"] + mo["approach_height_mm"])
            if z_carry > z_pre:
                column("lift", "carry_up", pxy, z_pre, z_carry, close_mm)
            add("carry", "above_place", [P[0], P[1], z_carry], close_mm)
            column("descend", "place_down", P[:2], z_carry, z_set, close_mm)
            add("release", "release", [P[0], P[1], z_set], release_mm, "open to release width")
            column("lift", "place_up", P[:2], z_set, z_carry, release_mm)
        if rest_q is not None:
            rq = np.asarray(rest_q, float)
            ok, why = _path_check(reach, state["q"], rq, table_z_mm, -1e9, wrist_floor)
            if not ok:
                raise PlanError(f"return: path to rest unsafe ({why})")
            wps.append(Waypoint("return_rest", "return", list(map(float, rq)), None, release_mm if place else close_mm,
                                gm.cmd(release_mm if place else close_mm), float(mo["speed_scale"]["return"]),
                                "rest pose (keep the object only if no place target was given)"))
        else:
            plan.warnings.append("no rest pose given: sequence ends above the last position")
    except PlanError as e:
        plan.reason = str(e)
        plan.waypoints = wps
        return plan

    plan.ok, plan.reason, plan.waypoints = True, "ok", wps
    plan.success_check = success_check_spec(width, gm, cfg["check"], close_mm)
    if place is None:
        plan.warnings.append("no place target: the object stays in the gripper after the lift")
    plan.info = dict(width_mm=round(width, 1), approach_open_mm=round(open_mm, 1), close_mm=round(close_mm, 1),
                     release_mm=round(release_mm, 1), jaw_yaw_deg=round(state["yaw"], 1), z_grasp_mm=round(z_grasp, 1),
                     tip_floor_mm=round(tip_floor, 1), n_waypoints=len(wps))
    return plan
