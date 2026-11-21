"""Hand-eye calibration pose planner + offline safety checks (no hardware access in this module).

Joint convention: raw follower degrees as reported in get_observation()/telemetry (`<joint>.pos`), the same
convention robot_fk.B601FK uses (q_urdf = radians(raw)). The executor converts to action values itself.

Planning (MoveIt-calibration / easy_handeye style):
- The look-at point P is where the gripper's finger axis (gripper_end x) hits the board plane at the TAUGHT pose
  (David aims the wrist camera at the board once with teleop, see handeye.py teach).
- Candidate gripper poses sit on a spherical cap above P: vertical height above the board `heights_mm`, tilt from
  vertical `tilts_deg` in 6 azimuth directions, roll about the view axis `rolls_deg`. The finger axis points at P.
- Each candidate is solved with damped-least-squares IK (numerical Jacobian) seeded from the taught joints,
  checked against URDF and plugin joint limits (minus a margin), height floor, visibility (camera prior) and
  joint-sign robustness, then a diverse subset is picked (farthest-point in rotation) and ordered to minimise
  joint travel.
- Safety floor: gripper_end and every wrist joint origin must stay >= table_z + min_clearance_mm, for ALL sign
  conventions of shoulder_pan / wrist_yaw / wrist_roll (the URDF-vs-motor sign of those joints is not verified on
  hardware; wrist_flex/lift/elbow were confirmed by the step-c readings). Joint-space paths between consecutive
  poses are checked the same way (sampled every <= 1 deg).
"""
import itertools
import json
import os
from pathlib import Path

import numpy as np

from robot_fk import ARM_JOINTS, B601FK

HERE = Path(__file__).resolve().parent
CAL_DIR = HERE / "calibration"
CONFIG = CAL_DIR / "handeye_config.json"
WORK_DIR = Path(os.environ.get("HANDEYE_WORK_DIR", str(CAL_DIR)))   # test hook: tests redirect teach/plan files
TEACH = WORK_DIR / "handeye_teach.json"
# plugin raw joint limits (config_seeed_b601_rs_follower.joint_limits, applied after joint_directions)
PLUGIN_LIMITS = {"shoulder_pan": (-145, 145), "shoulder_lift": (0, 170), "elbow_flex": (0, 200),
                 "wrist_flex": (-80, 90), "wrist_yaw": (-90, 90), "wrist_roll": (-130, 130)}
COMPLIANT = ("shoulder_pan", "wrist_yaw")       # kp 20 in teleop.sh -> looser reach tolerance
SIGN_VARIANTS = [(a, 1, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)]  # (pan, flex, yaw, roll)


def load_config(path=None):
    return json.loads(Path(path or CONFIG).read_text())


# ---------------------------------------------------------------- kinematics
class Kin:
    def __init__(self, fk=None):
        self.fk = fk or B601FK()
        lo, hi = [], []
        i = 0
        for j in self.fk.joints:
            if j["type"] == "revolute":
                ul, uh = np.degrees(j["lim"])
                pl, ph = PLUGIN_LIMITS[ARM_JOINTS[i]]
                lo.append(max(ul, pl)); hi.append(min(uh, ph)); i += 1
        self.lo, self.hi = np.array(lo), np.array(hi)

    def T(self, q, signs=None):
        """gripper_end pose in base frame, 4x4, translation in MM."""
        q = np.asarray(q, float)
        if signs is not None:
            q = q * np.array([signs[0], 1, 1, signs[1], signs[2], signs[3]])
        T = self.fk.pose(list(q)).copy()
        T[:3, 3] *= 1000.0
        return T

    def frames(self, q, signs=None):
        """Origins (mm) of all chain frames (joint origins + gripper_end), base frame."""
        q = np.asarray(q, float)
        if signs is not None:
            q = q * np.array([signs[0], 1, 1, signs[1], signs[2], signs[3]])
        from robot_fk import _axis_rot
        qr = np.radians(q)
        T, i, out = np.eye(4), 0, []
        for j in self.fk.joints:
            A = np.eye(4); A[:3, :3] = j["R"]; A[:3, 3] = j["xyz"]
            if j["type"] == "revolute":
                B = np.eye(4); B[:3, :3] = _axis_rot(j["axis"], qr[i]); i += 1
                A = A @ B
            T = T @ A
            out.append((j["name"], T[:3, 3] * 1000.0))
        return out

    def in_limits(self, q, margin=0.0):
        q = np.asarray(q)
        return bool(np.all(q >= self.lo + margin) and np.all(q <= self.hi - margin))

    def ik(self, T_goal, q0, iters=80, rot_w=200.0, tol_mm=0.5, tol_deg=0.3, margin=0.0):
        """DLS IK to a full 6D pose (mm). Returns (q, pos_err_mm, rot_err_deg)."""
        q = np.array(q0, float)
        lo, hi = self.lo + margin, self.hi - margin
        lam = 1.0
        for _ in range(iters):
            e = _pose_err(self.T(q), T_goal, rot_w)
            pe, re = np.linalg.norm(e[:3]), np.degrees(np.linalg.norm(e[3:]) / rot_w)
            if pe < tol_mm and re < tol_deg:
                break
            J = np.zeros((6, 6)); h = 0.05
            for k in range(6):
                dq = np.zeros(6); dq[k] = h
                J[:, k] = (_pose_err(self.T(q + dq), T_goal, rot_w) - e) / -h
            # e(q+dq) ~ e - J dq ; solve J dq = e
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            step = np.max(np.abs(dq))
            if step > 10:
                dq *= 10 / step
            qn = np.clip(q + dq, lo, hi)
            en = _pose_err(self.T(qn), T_goal, rot_w)
            if np.linalg.norm(en) < np.linalg.norm(e):
                q, lam = qn, max(lam * 0.5, 0.01)
            else:
                lam *= 4
                if lam > 1e4:
                    break
        e = _pose_err(self.T(q), T_goal, rot_w)
        return q, float(np.linalg.norm(e[:3])), float(np.degrees(np.linalg.norm(e[3:]) / rot_w))


def _rotvec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = np.arccos(c)
    if a < 1e-9:
        return np.zeros(3)
    if np.pi - a < 1e-6:
        w, V = np.linalg.eigh((R + R.T) / 2)
        ax = V[:, np.argmax(w)]
        return ax * a
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def _pose_err(T, G, rot_w):
    dp = G[:3, 3] - T[:3, 3]
    dr = _rotvec(G[:3, :3] @ T[:3, :3].T) * rot_w
    return np.concatenate([dp, dr])


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def look_at(pos, target, z_ref, roll_deg=0.0):
    """Tool frame: x (finger axis) points from pos to target; z as close as possible to z_ref; then roll about x."""
    d = np.asarray(target, float) - np.asarray(pos, float); d /= np.linalg.norm(d)
    z = np.asarray(z_ref, float) - np.dot(z_ref, d) * d
    if np.linalg.norm(z) < 1e-6:
        z = np.cross(d, [0, 1, 0])
    z /= np.linalg.norm(z)
    y = np.cross(z, d)
    T = np.eye(4); T[:3, :3] = np.column_stack([d, y, z]) @ _rx(np.radians(roll_deg)); T[:3, 3] = pos
    return T


# ---------------------------------------------------------------- safety
def pose_safety(kin, q, cfg, signs_list=SIGN_VARIANTS):
    """-> (ok, min_clearance_mm, reason). Clearance = lowest of gripper_end / wrist origins / camera prior above
    table_z, over all sign variants."""
    tz = cfg["table_z_mm"]; floor = cfg["min_clearance_mm"]
    worst, why = 1e9, ""
    off = np.array(cfg["camera_prior"]["offset_mm"], float)
    for sg in signs_list:
        fr = kin.frames(q, sg)
        T = kin.T(q, sg)
        cam = T[:3, :3] @ off + T[:3, 3]
        tool_pts = [(n, p) for n, p in fr if n in ("joint4", "joint5", "joint6", "j_gripper_end")] + [("camera", cam)]
        for n, p in tool_pts:
            c = p[2] - tz
            if c < worst:
                worst, why = c, f"{n} at signs {sg}"
        for n, p in fr[:3]:
            if p[2] < cfg.get("min_link_z_mm", -1e9):
                return False, worst, f"{n} below min_link_z at signs {sg}"
    return worst >= floor, float(worst), why


def path_safety(kin, qa, qb, cfg, step_deg=1.0):
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    n = max(2, int(np.ceil(np.max(np.abs(qb - qa)) / step_deg)) + 1)
    worst = 1e9; why = ""
    for s in np.linspace(0, 1, n):
        ok, c, w = pose_safety(kin, qa + s * (qb - qa), cfg)
        if c < worst:
            worst, why = c, w
        if not ok:
            return False, worst, f"path at {s:.2f}: {w}"
    return True, worst, why


def visible_fraction(kin, q, P_corners, cfg):
    """Fraction of board inner corners inside the prior camera's view cone (camera looks along the finger axis)."""
    T = kin.T(q)
    off = np.array(cfg["camera_prior"]["offset_mm"], float)
    cam = T[:3, :3] @ off + T[:3, 3]
    ax = T[:3, 0]
    v = P_corners - cam
    cosang = (v @ ax) / np.linalg.norm(v, axis=1)
    return float(np.mean(cosang >= np.cos(np.radians(cfg["camera_prior"]["half_fov_deg"]))))


# ---------------------------------------------------------------- planner
def look_point(kin, q_teach, table_z):
    """Where the finger axis of the taught pose hits the plane z = table_z (mm). None if not pointing down."""
    T = kin.T(q_teach)
    x = T[:3, 0]
    if x[2] > -0.5:            # more than 60 deg from straight down
        return None
    s = (table_z - T[2, 3]) / x[2]
    return T[:3, 3] + s * x


def plan_poses(q_teach, cfg=None, kin=None, board_half=(105.0, 52.5), log=print):
    """-> dict(poses=[{q, height_mm, tilt_deg, azimuth_deg, roll_deg, clearance_mm, visible}], look_point,
    rejected counts, path checks)."""
    cfg = cfg or load_config()
    kin = kin or Kin()
    q_teach = np.asarray(q_teach, float)
    P = look_point(kin, q_teach, cfg["table_z_mm"])
    if P is None:
        raise ValueError("taught pose: the gripper must point down at the board (finger axis < 60 deg from vertical)")
    T0 = kin.T(q_teach)
    z_ref = T0[:3, 2]
    # board corner cloud for the visibility check (orientation unknown -> use a circle of the half-diagonal)
    hd = float(np.hypot(*board_half))
    ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    corners = np.column_stack([P[0] + hd * np.cos(ang), P[1] + hd * np.sin(ang), np.full(12, P[2])])
    corners = np.vstack([corners, P[None]])
    margin = cfg["joint_margin_deg"]
    cands, rej = [], {"ik": 0, "limits": 0, "floor": 0, "visible": 0, "roll": 0}
    azis = np.arange(0, 360, 45)
    solved = []                      # (pos, q) of solved candidates -> better IK seeds
    for h in cfg["heights_mm"]:
        for tilt in cfg["tilts_deg"]:
            for az in (azis if tilt > 0 else [0]):
                for roll in cfg["rolls_deg"] + ([0] if tilt == 0 else []):
                    if abs(roll) > cfg["max_roll_from_teach_deg"]:
                        rej["roll"] += 1; continue
                    t, a = np.radians(tilt), np.radians(az)
                    dist = h / np.cos(t)
                    pos = P + dist * np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])
                    G = look_at(pos, P, z_ref, roll)
                    seeds = [q_teach] + [sq for _, sq in sorted(solved, key=lambda x: np.linalg.norm(x[0] - pos))[:2]]
                    for seed in seeds:
                        q, pe, re = kin.ik(G, seed, margin=margin)
                        if pe <= 1.0 and re <= 0.5:
                            break
                    if pe > 1.0 or re > 0.5:
                        rej["ik"] += 1; continue
                    solved.append((pos, q))
                    if not kin.in_limits(q, margin - 1e-6) or abs(q[5] - q_teach[5]) > cfg.get("max_joint_roll_change_deg", 70):
                        rej["limits"] += 1; continue
                    ok, clr, why = pose_safety(kin, q, cfg)
                    if not ok:
                        rej["floor"] += 1; continue
                    vis = visible_fraction(kin, q, corners, cfg)
                    if vis < 0.75:
                        rej["visible"] += 1; continue
                    cands.append(dict(q=q, height_mm=float(h), tilt_deg=float(tilt), azimuth_deg=float(az),
                                      roll_deg=float(roll), clearance_mm=clr, visible=vis, R=G[:3, :3]))
    if len(cands) < 8:
        raise ValueError(f"only {len(cands)} reachable safe poses (rejected {rej}); teach a pose closer above the board")
    # diverse subset: farthest-point sampling on rotation distance (+ height term), start nearest to teach
    n = min(cfg["n_poses"], len(cands))
    Rs = [c["R"] for c in cands]
    def dist(i, j):
        return rot_angle_deg(Rs[i].T @ Rs[j]) + abs(cands[i]["height_mm"] - cands[j]["height_mm"]) / 10.0
    start = int(np.argmin([np.max(np.abs(c["q"] - q_teach)) for c in cands]))
    chosen = [start]
    dmin = np.array([dist(start, j) for j in range(len(cands))])
    while len(chosen) < n:
        k = int(np.argmax(dmin)); chosen.append(k)
        dmin = np.minimum(dmin, [dist(k, j) for j in range(len(cands))])
    sel = [cands[i] for i in chosen]
    # order: greedy nearest neighbour in joint space starting from the taught pose
    order, cur, left = [], q_teach, list(range(len(sel)))
    while left:
        k = min(left, key=lambda i: np.max(np.abs(sel[i]["q"] - cur)))
        order.append(k); cur = sel[k]["q"]; left.remove(k)
    poses = [sel[i] for i in order]
    # path checks teach -> p1 -> ... -> pn -> teach
    seq = [q_teach] + [p["q"] for p in poses] + [q_teach]
    paths = []
    for a, b in zip(seq[:-1], seq[1:]):
        ok, clr, why = path_safety(kin, a, b, cfg)
        paths.append(dict(ok=ok, clearance_mm=clr, why=why, max_joint_move_deg=float(np.max(np.abs(b - a)))))
        if not ok:
            raise ValueError(f"unsafe joint path in the plan: {why}")
    out = {"look_point_mm": P.tolist(), "q_teach": q_teach.tolist(), "table_z_mm": cfg["table_z_mm"],
           "poses": [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in p.items() if k != "R"}
                     for p in poses],
           "paths": paths, "rejected": rej, "n_candidates": len(cands), "config": cfg}
    out["summary"] = summarize(out)
    return out


def summarize(plan):
    P = plan["poses"]
    h = [p["height_mm"] for p in P]; c = [p["clearance_mm"] for p in P]
    moves = [x["max_joint_move_deg"] for x in plan["paths"]]
    cfg = plan["config"]
    return {"n_poses": len(P), "height_mm": [min(h), max(h)], "tilt_deg_max": max(p["tilt_deg"] for p in P),
            "roll_deg": [min(p["roll_deg"] for p in P), max(p["roll_deg"] for p in P)],
            "min_clearance_mm": min(min(c), min(x["clearance_mm"] for x in plan["paths"])),
            "floor_mm": cfg["min_clearance_mm"], "max_joint_move_deg": max(moves),
            "speed_dps": cfg["speed_dps"], "est_duration_s": round(sum(1.5 * m / cfg["speed_dps"] + cfg["settle_s"] + 1.0
                                                                     for m in moves) + 10, 0)}


def summary_text(plan):
    s = plan["summary"]
    lines = [f"{s['n_poses']} poses above board point {np.round(plan['look_point_mm'], 0).tolist()} mm (robot base frame)",
             f"height above board {s['height_mm'][0]:.0f}-{s['height_mm'][1]:.0f} mm, tilt up to {s['tilt_deg_max']:.0f} deg, "
             f"roll {s['roll_deg'][0]:.0f}..{s['roll_deg'][1]:.0f} deg",
             f"lowest point of wrist/camera/gripper over the board: {s['min_clearance_mm']:.0f} mm (floor {s['floor_mm']:.0f} mm, "
             f"checked for all pan/yaw/roll sign conventions)",
             f"speed max {s['speed_dps']:.0f} deg/s, largest joint move {s['max_joint_move_deg']:.0f} deg, about {s['est_duration_s']/60:.1f} min"]
    lines.append(" #  height tilt  azim  roll  clear   joints (deg)")
    for i, p in enumerate(plan["poses"], 1):
        lines.append(f"{i:2d}  {p['height_mm']:5.0f} {p['tilt_deg']:5.0f} {p['azimuth_deg']:5.0f} {p['roll_deg']:5.0f} "
                     f"{p['clearance_mm']:6.0f}   {np.round(p['q'], 1).tolist()}")
    return "\n".join(lines)


def plot_plan(plan, path, kin=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    kin = kin or Kin()
    fig = plt.figure(figsize=(11, 5.2))
    P = np.array(plan["look_point_mm"])
    for k, (view, title) in enumerate((((25, -60), "3D"), ((0, -90), "side (x-z)"))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        ax.view_init(*view)
        tz = plan["table_z_mm"]
        bx, by = 122.5, 87.5
        ax.plot([P[0]-bx, P[0]+bx, P[0]+bx, P[0]-bx, P[0]-bx], [P[1]-by, P[1]-by, P[1]+by, P[1]+by, P[1]-by], [tz]*5, "k-")
        floor = tz + plan["config"]["min_clearance_mm"]
        ax.plot([P[0]-bx, P[0]+bx], [P[1], P[1]], [floor, floor], "r--", lw=1)
        for i, p in enumerate(plan["poses"]):
            fr = np.array([f for _, f in kin.frames(p["q"])])
            fr = np.vstack([[0, 0, 0], fr])
            ax.plot(fr[:, 0], fr[:, 1], fr[:, 2], "-", color=plt.cm.viridis(i / max(1, len(plan["poses"]) - 1)), lw=0.8)
            T = kin.T(p["q"])
            e = T[:3, 3]; d = T[:3, 0] * 60
            ax.quiver(e[0], e[1], e[2], d[0], d[1], d[2], color="r", lw=0.8)
        fr = np.array([[0, 0, 0]] + [f for _, f in kin.frames(plan["q_teach"])])
        ax.plot(fr[:, 0], fr[:, 1], fr[:, 2], "k-", lw=2.0, label="taught pose")
        ax.scatter([P[0]], [P[1]], [P[2]], c="r", s=20)
        ax.set_xlabel("x mm"); ax.set_ylabel("y mm"); ax.set_zlabel("z mm"); ax.set_title(title)
        try:
            ax.set_box_aspect((1, 1, 0.8))
        except Exception:
            pass
    fig.suptitle("Hand-eye calibration plan (base frame). Red dashed = height floor. Black = taught pose.", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=80)
    plt.close(fig)
    return path


# ---------------------------------------------------------------- v3.9.1: automatic start (no taught pose needed)
# Real joint readings with the gripper pointing down at the board (IK seeds): David's taught pose of 2026-11-21 22:27
# and the step-c reading. Both keep the wrist in the "fingers down" branch of the IK.
DOWN_SEEDS = ([-4.9, 79.4, 63.4, -46.0, -11.3, -7.4], [28.5, 118.8, 63.2, -21.7, 9.9, 17.5])
DEFAULT_BOARD_POINT = (463.0, -46.0, -70.0)     # board centre seen at the 22:27 teach (robot base frame, mm)


def _view_pose(kin, P, height, tilt_deg, seeds, cfg, margin, min_clear, azimuth_deg=None):
    """Gripper `height` mm above point P, finger axis aimed at P, tilted `tilt_deg` from vertical towards the robot
    base (default azimuth = from P back to the base). -> (q, clearance) or None if not reachable / not safe."""
    P = np.asarray(P, float)
    az = np.degrees(np.arctan2(-P[1], -P[0])) if azimuth_deg is None else azimuth_deg
    t, a = np.radians(tilt_deg), np.radians(az)
    pos = P + height / np.cos(t) * np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])
    best = None
    for seed in seeds:
        z_ref = kin.T(seed)[:3, 2]
        G = look_at(pos, P, z_ref, 0.0)
        q, pe, re = kin.ik(G, seed, margin=margin)
        if pe > 1.0 or re > 0.5 or not kin.in_limits(q, margin - 1e-6):
            continue
        ok, clr, _ = pose_safety(kin, q, cfg)
        if not ok or clr < min_clear:
            continue
        cost = float(np.max(np.abs(q - np.asarray(seeds[0], float))))
        if best is None or cost < best[2]:
            best = (q, clr, cost)
    return None if best is None else (best[0], best[1])


def search_pose(P0, cfg=None, kin=None, seeds=DOWN_SEEDS):
    """High 'search pose' looking down at the expected board point P0: gripper search_heights_mm above it
    (default 250-280 mm, i.e. clearance well above 200 mm), tilted towards the base. -> dict(q, height_mm,
    tilt_deg, clearance_mm, look_point_mm). Raises ValueError if no safe reachable pose exists."""
    cfg = cfg or load_config(); kin = kin or Kin()
    heights = cfg.get("search_heights_mm", [260, 280, 240, 220])
    tilts = cfg.get("search_tilts_deg", [30, 25, 35, 20, 40])
    min_clear = float(cfg.get("search_min_clearance_mm", 200.0))
    for h in heights:
        for tilt in tilts:
            r = _view_pose(kin, P0, h, tilt, [np.asarray(s, float) for s in seeds], cfg, cfg["joint_margin_deg"], min_clear)
            if r is not None:
                return {"q": r[0].tolist(), "height_mm": float(h), "tilt_deg": float(tilt), "clearance_mm": float(r[1]),
                        "look_point_mm": [float(v) for v in P0]}
    raise ValueError(f"no safe search pose above {np.round(P0, 0).tolist()} mm (board out of reach?)")


def aim_pose(P, seed, cfg=None, kin=None):
    """'Taught pose' computed from the measured board centre P: gripper aim_height_mm (200) above P, finger axis
    through P, the smallest workable tilt towards the base. -> (q, tilt_deg, clearance). ValueError if none."""
    cfg = cfg or load_config(); kin = kin or Kin()
    h = float(cfg.get("aim_height_mm", 200.0))
    seeds = [np.asarray(seed, float)] + [np.asarray(s, float) for s in DOWN_SEEDS]
    for tilt in cfg.get("aim_tilts_deg", [20, 25, 30, 15, 35, 10, 40]):
        r = _view_pose(kin, P, h, tilt, seeds, cfg, cfg["joint_margin_deg"], cfg["min_clearance_mm"] + 50.0)
        if r is not None and look_point(kin, r[0], P[2]) is not None:
            return r[0], float(tilt), float(r[1])
    raise ValueError(f"no reachable pose {h:.0f} mm above the board centre {np.round(P, 0).tolist()} mm")
