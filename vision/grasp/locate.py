"""Pixel -> 3D on the table plane and reach filter. Read-only: loads calibration JSON, never writes it.

Calibration files (vision/calibration, override with env HUMANOID_CAL_DIR or the `cal_dir` argument):
  intrinsics.json   overview camera K, dist, size                       (calibrate.py)
  table.json        board pose in the overview camera (rvec/tvec) + table_to_robot.T_board2base [+ T_overview2base]
  handeye.json      wrist camera: wrist_intrinsics, T_cam2gripper, joint_signs   (optional, only for locate_wrist)
Missing / incomplete files raise CalibrationMissing with the action to take (nothing is guessed).

Frames: robot base frame, mm, z up (URDF base_link). The table plane is the board plane (T_board2base) shifted
along its normal by the object height, so a tilted / raised board is handled.

Reach filter: reuses handeye_plan.Kin (URDF FK + DLS IK + joint limits) without modifying it. The grasp pose is the tool
pointing straight down (gripper_end x axis = -z) with the jaws across the object's short axis.
"""
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from . import VISION_DIR

CAL_DIR = Path(os.environ.get("HUMANOID_CAL_DIR", VISION_DIR / "calibration"))
# Which gripper_end axis the jaws close along. NOT verified on hardware yet -> check once with a test grasp
# (the demo output prints the jaw axis it assumed). Env HUMANOID_JAW_AXIS=y|z.
JAW_TOOL_AXIS = os.environ.get("HUMANOID_JAW_AXIS", "y")


class CalibrationMissing(RuntimeError):
    pass


# ---------------------------------------------------------------- calibration loading
def _read(path, what, hint):
    if not Path(path).exists():
        raise CalibrationMissing(f"{what} not found: {path}. {hint}")
    return json.loads(Path(path).read_text())


def _T_from_table_to_robot(m):
    """T_board2base from table.json['table_to_robot'] (full 3D if present, else planar R,t,z)."""
    if m is None:
        raise CalibrationMissing("table.json has no table_to_robot: run the hand-eye calibration (Auto-Kalibrierung) "
                                 "and apply it (✓ Übernehmen), or the manual corner touch (calibrate.py robot).")
    if m.get("T_board2base") is not None:
        return np.array(m["T_board2base"], float)
    T = np.eye(4)
    T[:2, :2] = np.array(m["R"], float); T[:2, 3] = m["t"]; T[2, 3] = m.get("z_mm", 0.0)
    return T


@dataclass
class CameraModel:
    K: np.ndarray
    dist: np.ndarray
    T_cam2base: np.ndarray      # mm
    size: tuple                 # (w, h) the K belongs to
    name: str = "overview"

    def scaled(self, size):
        w, h = size
        if (w, h) == tuple(self.size):
            return self
        K = self.K.copy()
        K[0, :] *= w / self.size[0]; K[1, :] *= h / self.size[1]
        return CameraModel(K, self.dist, self.T_cam2base, (w, h), self.name)

    def rays_base(self, px):
        """Pixels (N,2) -> (origin (3,), directions (N,3)) in the base frame."""
        px = np.asarray(px, np.float32).reshape(-1, 1, 2)
        und = cv2.undistortPoints(px, self.K, self.dist).reshape(-1, 2)
        d_cam = np.hstack([und, np.ones((len(und), 1))])
        R, t = self.T_cam2base[:3, :3], self.T_cam2base[:3, 3]
        d = d_cam @ R.T
        return t, d / np.linalg.norm(d, axis=1, keepdims=True)


@dataclass
class Calibration:
    overview: Optional[CameraModel]
    T_board2base: np.ndarray
    handeye: Optional[dict]
    table_z_mm: float
    source: str = ""

    def plane(self, height_mm=0.0):
        """(point, unit normal pointing up) of the board plane lifted by height_mm."""
        n = self.T_board2base[:3, 2].copy()
        if n[2] < 0:
            n = -n
        n /= np.linalg.norm(n)
        return self.T_board2base[:3, 3] + n * height_mm, n


def load_calibration(cal_dir=None, require_handeye=False):
    cal_dir = Path(cal_dir or CAL_DIR)
    table = _read(cal_dir / "table.json", "table calibration",
                  "Run the overview calibration ('Neu kalibrieren' in the GUI) and the hand-eye calibration first.")
    intr = _read(cal_dir / "intrinsics.json", "overview intrinsics", "Run calibrate.py intrinsics.")
    Z = _T_from_table_to_robot(table.get("table_to_robot"))
    R = cv2.Rodrigues(np.array(table["rvec"], float))[0]
    Tb2c = np.eye(4); Tb2c[:3, :3] = R; Tb2c[:3, 3] = np.array(table["tvec"], float)
    T_cam2base = np.array(table["T_overview2base"], float) if table.get("T_overview2base") is not None \
        else Z @ np.linalg.inv(Tb2c)
    cam = CameraModel(np.array(intr["K"], float), np.array(intr["dist"], float), T_cam2base, tuple(intr["size"]))
    he = None
    hp = cal_dir / "handeye.json"
    if hp.exists():
        he = json.loads(hp.read_text())
    elif require_handeye:
        raise CalibrationMissing(f"hand-eye result not found: {hp}. Run the Auto-Kalibrierung and apply it.")
    return Calibration(cam, Z, he, float(Z[2, 3]), str(cal_dir))


# ---------------------------------------------------------------- pixel -> 3D
def _intersect(origin, dirs, p0, n):
    denom = dirs @ n
    if np.any(np.abs(denom) < 1e-6):
        raise ValueError("viewing ray is parallel to the table plane")
    s = ((p0 - origin) @ n) / denom
    if np.any(s <= 0):
        raise ValueError("table plane lies behind the camera (calibration wrong?)")
    return origin + dirs * s[:, None]


def wrist_camera(cal, q_deg, image_size):
    """CameraModel of the wrist camera for the joint reading q_deg (raw degrees, as in get_observation)."""
    if not cal.handeye:
        raise CalibrationMissing("wrist localisation needs handeye.json (Auto-Kalibrierung not applied yet)")
    from handeye_plan import Kin
    he = cal.handeye
    wi = he["wrist_intrinsics"]
    K = np.array(wi["K"], float)
    cam = CameraModel(K, np.array(wi["dist"], float), np.eye(4), tuple(wi["size"]), "wrist")
    signs = tuple(he.get("joint_signs") or (1, 1, 1, 1))
    Tg = Kin().T(np.asarray(q_deg, float), signs)          # gripper_end in base, mm
    cam.T_cam2base = Tg @ np.array(he["T_cam2gripper"], float)
    return cam.scaled(tuple(image_size))


def pixel_to_base(px, cal, image_size, height_mm=0.0, camera="overview", q_deg=None):
    """Pixels (N,2) in an image of image_size=(w,h) -> (N,3) base-frame mm on the plane `height_mm` above the table."""
    cam = cal.overview.scaled(tuple(image_size)) if camera == "overview" else wrist_camera(cal, q_deg, image_size)
    if cam is None:
        raise CalibrationMissing("no overview camera model")
    o, d = cam.rays_base(px)
    p0, n = cal.plane(height_mm)
    return _intersect(o, d, p0, n)


def locate_object(obj, cal, image_size, height_mm=0.0, camera="overview", q_deg=None):
    """DetectedObject -> dict(xyz_mm, jaw_yaw_deg (robot frame, (-90,90]), width_mm, length_mm).
    height_mm = height of the object's top face above the table (the grasp plane); 0 if unknown (table level)."""
    cx, cy = obj.center_px
    pts = [(cx, cy)]
    if obj.jaw_dir_px is not None:
        u = np.array(obj.jaw_dir_px); v = np.array([-u[1], u[0]])
        pts += [tuple(np.array([cx, cy]) + u * obj.short_px / 2), tuple(np.array([cx, cy]) - u * obj.short_px / 2),
                tuple(np.array([cx, cy]) + v * obj.long_px / 2), tuple(np.array([cx, cy]) - v * obj.long_px / 2)]
    P = pixel_to_base(pts, cal, image_size, height_mm, camera, q_deg)
    out = dict(xyz_mm=[float(x) for x in P[0]], jaw_yaw_deg=None, width_mm=None, length_mm=None)
    if len(P) == 5:
        j = P[1] - P[2]
        a = float(np.degrees(np.arctan2(j[1], j[0])))
        a = (a + 90) % 180 - 90 if a != 90 else 90.0         # fold into (-90, 90]
        if a == -90:
            a = 90.0
        out.update(jaw_yaw_deg=a, width_mm=float(np.linalg.norm(j)), length_mm=float(np.linalg.norm(P[3] - P[4])))
    return out


# ---------------------------------------------------------------- reach filter
def grasp_orientation(jaw_yaw_deg, jaw_axis=None):
    """3x3 tool rotation: finger axis (x) straight down, jaw axis horizontal at heading jaw_yaw_deg."""
    jaw_axis = jaw_axis or JAW_TOOL_AXIS
    a = np.radians(jaw_yaw_deg)
    x = np.array([0.0, 0.0, -1.0])
    j = np.array([np.cos(a), np.sin(a), 0.0])
    other = np.cross(x, j)                     # completes the right-handed frame
    if jaw_axis == "y":
        return np.column_stack([x, j, other])
    if jaw_axis == "z":
        return np.column_stack([x, np.cross(j, x), j])
    raise ValueError("jaw axis must be 'y' or 'z'")


@dataclass
class ReachResult:
    ok: bool
    q: Optional[list]           # raw joint degrees (6) or None
    pos_err_mm: float = 0.0
    rot_err_deg: float = 0.0
    jaw_yaw_deg: float = 0.0    # the (possibly flipped by 180 deg) yaw that was reachable
    reason: str = ""

    def to_json(self):
        return dict(ok=self.ok, q=None if self.q is None else [round(float(v), 2) for v in self.q],
                    pos_err_mm=round(self.pos_err_mm, 2), rot_err_deg=round(self.rot_err_deg, 2),
                    jaw_yaw_deg=round(self.jaw_yaw_deg, 1), reason=self.reason)


class ReachChecker:
    """IK-based reach filter. tip_mm is the fingertip target; gripper_end sits tip_offset_mm higher (tool axis down).
    Wrist joints must stay >= table_z + wrist_clear_mm above the table (protects the camera / wrist, not the fingertips)."""

    def __init__(self, table_z_mm=0.0, signs=(1, 1, 1, 1), kin=None, margin_deg=3.0, wrist_clear_mm=40.0,
                 tip_offset_mm=None):
        from handeye_plan import Kin, DOWN_SEEDS
        import robot_fk
        self.kin = kin or Kin()
        self.sg = np.array([signs[0], 1, 1, signs[1], signs[2], signs[3]], float)    # URDF q = raw q * sg
        self.table_z, self.margin, self.wrist_clear = table_z_mm, margin_deg, wrist_clear_mm
        self.tip_off = robot_fk.TIP_OFFSET_MM if tip_offset_mm is None else tip_offset_mm
        self.seeds = [np.asarray(s, float) for s in DOWN_SEEDS]

    def check(self, tip_mm, jaw_yaw_deg, seed_q=None, flips=(0.0, 180.0)):
        """Try the jaw yaw and yaw+180 (the jaws are symmetric); first reachable wins. seed_q: raw degrees."""
        tip = np.asarray(tip_mm, float)
        seeds = ([np.asarray(seed_q, float)] if seed_q is not None else []) + self.seeds
        worst = ReachResult(False, None, 1e9, 1e9, jaw_yaw_deg, "not reachable")
        for fl in flips:
            yaw = jaw_yaw_deg + fl
            R = grasp_orientation(yaw)
            G = np.eye(4); G[:3, :3] = R; G[:3, 3] = tip - R[:, 0] * self.tip_off
            for s in seeds:
                qu, pe, re = self.kin.ik(G, s * self.sg, margin=self.margin)
                q_raw = qu * self.sg
                if pe < worst.pos_err_mm:
                    worst = ReachResult(False, list(q_raw), pe, re, yaw, f"IK error {pe:.1f} mm / {re:.1f} deg")
                if pe > 1.5 or re > 1.0:
                    continue
                if not self.kin.in_limits(q_raw, self.margin - 1e-6):
                    worst.reason = "joint limit"
                    continue
                low = min(p[2] for n, p in self.kin.frames(qu) if n in ("joint4", "joint5", "joint6"))
                if low < self.table_z + self.wrist_clear:
                    worst.reason = f"wrist would be {low - self.table_z:.0f} mm above the table (< {self.wrist_clear:.0f})"
                    continue
                return ReachResult(True, list(q_raw), pe, re, yaw, "ok")
        return worst
