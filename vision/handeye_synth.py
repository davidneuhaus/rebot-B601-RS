"""Synthetic ground truth for testing the hand-eye pipeline without hardware.

A known wrist camera (wide angle, strong distortion, 640x360 like the GUI's shared frames), a known camera->gripper
transform, a known board pose on the table and the real overview camera geometry (from a real overview image of the
board) are used to RENDER the ChArUco board into the camera images at the planned joint poses (SimArm/SimCams in
handeye.py). The pipeline must recover the truth.
"""
import json
from pathlib import Path

import cv2
import numpy as np

import handeye_plan as HP
from charuco import Charuco, scale_K
from handeye_solve import inv, rot_angle_deg

HERE = Path(__file__).resolve().parent
OV_IMG = HERE / "tests" / "data" / "overview_charuco_640.png"


def _rot(axis, deg):
    a = np.radians(deg); ax = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def make_truth(look=(350.0, -30.0, -70.0), board_yaw_deg=20.0, board_tilt_deg=1.0, cam_tilt_deg=8.0,
               cam_offset=(-110.0, 6.0, 35.0), signs=(1, 1, 1, 1), cam_roll_deg=0.0):
    kin = HP.Kin()
    # wrist camera -> gripper_end (X): optical axis = finger axis tilted by cam_tilt_deg
    zc = _rot([0, 1, 0], cam_tilt_deg) @ np.array([1.0, 0, 0])
    yc = np.array([0, 0, 1.0]) - zc[2] * zc; yc /= np.linalg.norm(yc)
    xc = np.cross(yc, zc)
    X = np.eye(4); X[:3, :3] = np.column_stack([xc, yc, zc]) @ _rot([0, 0, 1], cam_roll_deg); X[:3, 3] = cam_offset
    # board -> base (Z): board z points down into the table (camera looks at its front from above)
    board = Charuco()
    R = _rot([0, 0, 1], board_yaw_deg) @ np.diag([1.0, -1.0, -1.0]) @ _rot([1, 0, 0], board_tilt_deg)
    centre = np.array(look, float)
    Z = np.eye(4); Z[:3, :3] = R
    Z[:3, 3] = centre - R @ np.array([board.size_mm[0] / 2, board.size_mm[1] / 2, 0])
    K = np.array([[300.0, 0, 318.0], [0, 300.0, 182.0], [0, 0, 1]])
    dist = np.array([-0.30, 0.10, 0.001, -0.0005, -0.015])
    # overview camera: real geometry relative to the board from the real image
    intr = json.loads((HERE / "calibration" / "intrinsics.json").read_text())
    im = cv2.imread(str(OV_IMG))
    size_ov = (im.shape[1], im.shape[0])
    K_ov = scale_K(intr["K"], intr["size"], size_ov); dist_ov = np.array(intr["dist"])
    T_b2ov, _ = board.pose(board.detect(im), K_ov, dist_ov)
    T_ov2base = Z @ inv(T_b2ov)
    return {"X": X, "Z": Z, "K": K, "dist": dist, "size": (640, 360), "K_ov": K_ov, "dist_ov": dist_ov,
            "size_ov": size_ov, "T_ov2base": T_ov2base, "signs": signs, "q_start": [0.0] * 6, "kin": kin}


def teach_pose(truth, height=200.0, tilt=15.0):
    kin = truth["kin"]
    P = truth["Z"][:3, :3] @ np.array([122.5, 87.5, 0]) + truth["Z"][:3, 3]
    t = np.radians(tilt)
    pos = P + height / np.cos(t) * np.array([-np.sin(t), 0, np.cos(t)])
    seed = [28.5, 118.8, 63.2, -21.7, 9.9, 17.5]           # real step-c reading (gripper down at the board)
    q, pe, re = kin.ik(HP.look_at(pos, P, kin.T(seed)[:3, 2]), seed)
    assert pe < 1 and re < 0.5, (pe, re)
    return q


def make_truth_and_plan(**kw):
    truth = make_truth(**kw)
    q_teach = teach_pose(truth)
    plan = HP.plan_poses(q_teach)
    return truth, plan


def compare(truth, result, log=print):
    X = np.array(result["T_cam2gripper"]); Z = np.array(result["T_board2base"])
    e = {"X_mm": float(np.linalg.norm(X[:3, 3] - truth["X"][:3, 3])),
         "X_deg": rot_angle_deg(X[:3, :3].T @ truth["X"][:3, :3]),
         "Z_mm": float(np.linalg.norm(Z[:3, 3] - truth["Z"][:3, 3])),
         "Z_deg": rot_angle_deg(Z[:3, :3].T @ truth["Z"][:3, :3])}
    board = Charuco()
    P = board.corners3d
    e["board_corners_mm_max"] = float(np.max(np.linalg.norm((P @ Z[:3, :3].T + Z[:3, 3]) - (P @ truth["Z"][:3, :3].T + truth["Z"][:3, 3]), axis=1)))
    K = np.array(result["wrist_intrinsics"]["K"])
    e["fx_err_px"] = float(abs(K[0, 0] - truth["K"][0, 0])); e["cx_err_px"] = float(abs(K[0, 2] - truth["K"][0, 2]))
    ov = result.get("overview") or {}
    if ov.get("ok"):
        T = np.array(ov["T_overview2base"])
        e["overview_mm"] = float(np.linalg.norm(T[:3, 3] - truth["T_ov2base"][:3, 3]))
        e["overview_deg"] = rot_angle_deg(T[:3, :3].T @ truth["T_ov2base"][:3, :3])
        # what matters for pixel -> table: a pixel ray hits the board plane at the right place (mm)
        e["overview_px2table_mm_max"] = px2table_err(T, truth)
    e["pass"] = bool(e["X_mm"] < 3 and e["X_deg"] < 0.75 and e["Z_mm"] < 2 and e["Z_deg"] < 0.5)
    log("SYNTHETIC CHECK: " + ", ".join(f"{k} {v:.3f}" if isinstance(v, float) else f"{k} {v}" for k, v in e.items()))
    return e


def px2table_err(T_ov2base_est, truth):
    """Max error (mm) on the board plane when overview pixels are mapped to the table with the estimated vs the
    true camera pose (grid over the board area)."""
    board = Charuco()
    Z = truth["Z"]
    pts = np.array([[x, y, 0.0] for x in np.linspace(0, board.size_mm[0], 6) for y in np.linspace(0, board.size_mm[1], 5)])
    Pw = pts @ Z[:3, :3].T + Z[:3, 3]                                  # base frame, true
    err = []
    Ce = inv(T_ov2base_est)
    n = Z[:3, 2]; p0 = Z[:3, 3]
    for P in Pw:
        # true pixel ray direction from the true camera; re-cast it from the estimated camera with the same pixel
        Pc = (inv(truth["T_ov2base"]) @ np.append(P, 1))[:3]
        ray_c = Pc / Pc[2]
        o = T_ov2base_est[:3, 3]; dvec = T_ov2base_est[:3, :3] @ ray_c
        s = (n @ (p0 - o)) / (n @ dvec)
        err.append(np.linalg.norm(o + s * dvec - P))
    return float(max(err))


# ---------------------------------------------------------------- v3.9.1 automatic start
def board_centre(truth):
    board = Charuco()
    return truth["Z"][:3, :3] @ np.array([board.size_mm[0] / 2, board.size_mm[1] / 2, 0]) + truth["Z"][:3, 3]


def board_guess(truth, off=(60.0, -40.0, 0.0)):
    """Deliberately wrong expected board point (like an old handeye.json after the board was moved)."""
    return board_centre(truth) + np.asarray(off, float)


def prior_model(truth, hfov_deg=90.0):
    """The camera model the auto start uses when no calibration exists (independent of the truth: no distortion,
    focal length from a 90 deg field of view, camera rotation estimated from the image)."""
    w, h = truth["size"]
    f = 0.5 * w / np.tan(np.radians(hfov_deg) / 2)
    K = np.array([[f, 0, (w - 1) / 2], [0, f, (h - 1) / 2], [0, 0, 1.0]])
    cfg = HP.load_config()
    return {"K": K, "dist": np.zeros(5), "X": None, "signs": None,
            "offset": np.array(cfg["camera_prior"]["offset_mm"], float), "model": "prior"}
