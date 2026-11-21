"""Camera calibration for the overview camera, with a fast "Recalibrate" path.

Steps:
  (a) intrinsics      : slow, once (checkerboard, ~15 views). Reused until lens/resolution change.
  (b) table plane     : board lying flat on the table, ONE frame, < 1 s of compute. Redo every session.
  (c) table -> robot  : 2D rigid transform. Needs >= 3 table points touched with the gripper.
                        On recalibrate it is REUSED if the board is put on its usual mark (the table
                        frame is defined by the board), so the quick path is just step (b).

Table frame: origin = first inner corner of the board, x along columns, y along rows, z up (mm).
v3.9: the ChArUco board (calibration/board_charuco.json, 7x5 squares, 35 mm) is the default board when that file
exists. Its corners carry ids, so the order never flips and partial views work. Table frame = OpenCV ChArUco board
frame (origin at the outer grid corner). Step (c) is now automatic: vision/handeye.py (hand-eye calibration) writes
table_to_robot with the full 3D T_board2base via apply_handeye(); the manual corner touch is the fallback.

Quick path (also the future GUI button):
    from calibrate import recalibrate_table
    r = recalibrate_table(source=1)    # index | name | image path | BGR frame
    r -> {"ok", "message", "shift_mm" (mean/max vs previous), "warnings", "path"}
CLI:
    calibrate.py intrinsics --source 1 [--views 15]
    calibrate.py table      --source 1 [--image f.jpg]       # = recalibrate_table
    calibrate.py robot-capture [--manual] [--tip-offset-mm N]   # touch corners A-D, FK, fit, residual
    calibrate.py robot --pairs '[[tx,ty,rx,ry],...]'          # table mm -> robot mm
    calibrate.py check      --source 1                        # load + verify, print warnings
    calibrate.py pixel2table --px 320 240
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

CAL_DIR = Path.home() / "humanoid" / "vision" / "calibration"
HIST_DIR = CAL_DIR / "history"
CAMERAS_JSON = Path.home() / "humanoid" / "cameras.json"
KEEP_N = 10
BOARD = {"cols": 9, "rows": 6, "square_mm": 25.0}  # inner corners; override in calibration/board.json


# ---------------------------------------------------------------- helpers
def _board():
    """Active board spec. board.json (explicit override) > board_charuco.json (v3.9 default) > 9x6 checkerboard."""
    p = CAL_DIR / "board.json"
    if p.exists():
        return {**BOARD, **json.loads(p.read_text())}
    pc = CAL_DIR / "board_charuco.json"
    if pc.exists():
        d = json.loads(pc.read_text())
        return {k: d[k] for k in ("type", "squares_x", "squares_y", "square_mm", "marker_mm", "dictionary")}
    return dict(BOARD)


def _is_charuco(b):
    return b.get("type") == "charuco"


_CHARUCO = {}


def _charuco(b):
    key = json.dumps(b, sort_keys=True)
    if key not in _CHARUCO:
        sys.path.insert(0, str(Path(__file__).parent))
        from charuco import Charuco
        _CHARUCO[key] = Charuco(dict(b))
    return _CHARUCO[key]


def detect_board(bgr, b=None, min_corners=8):
    """-> (obj (N,3) float32 mm, img (N,2) float32 px) or None. Checkerboard: all inner corners; ChArUco: the
    detected corners with their ids (partial views allowed, >= min_corners)."""
    b = b or _board()
    if _is_charuco(b):
        d = _charuco(b).detect(bgr)
        if d is None or len(d["ids"]) < min_corners:
            return None
        return d["obj"].astype(np.float32), d["img"].astype(np.float32)
    c = find_corners(bgr, b)
    return None if c is None else (_obj_points(b), c)


def _intr_for(intr, size):
    """Intrinsics K for a frame of `size` (scaled when only the resolution differs, same aspect)."""
    K = np.array(intr["K"], float)
    if list(size) == list(intr["size"]):
        return K
    sys.path.insert(0, str(Path(__file__).parent))
    from charuco import scale_K
    return scale_K(K, intr["size"], size)


def _obj_points(b):
    g = np.zeros((b["rows"] * b["cols"], 3), np.float32)
    g[:, :2] = np.mgrid[0:b["cols"], 0:b["rows"]].T.reshape(-1, 2) * b["square_mm"]
    return g


def find_corners(bgr, b=None):
    b = b or _board()
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, (b["cols"], b["rows"]),
                                        cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:  # fallback: classic detector + subpixel refine (SB can fail on glare / steep views)
        ok, c = cv2.findChessboardCorners(gray, (b["cols"], b["rows"]),
                                          cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            c = cv2.cornerSubPix(gray, c, (11, 11), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    return (c.reshape(-1, 2).astype(np.float32) if ok else None)


def grab_frame(source, n_try=30):
    """source: BGR ndarray | image path | camera index | camera name."""
    if isinstance(source, np.ndarray):
        return source
    s = str(source)
    if Path(s).is_file():
        return cv2.imread(s)
    if not s.isdigit():
        sys.path.insert(0, str(Path(__file__).parent))
        from detect_live import index_for_name
        idx = index_for_name(s)
        if idx is None:
            raise RuntimeError(f"camera {s!r} not found")
        s = str(idx)
    cap = cv2.VideoCapture(int(s))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {s}")
    frame = None
    for _ in range(n_try):  # let exposure settle, keep last frame
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        raise RuntimeError("no frame from camera")
    return frame


def _setup_fingerprint():
    """Identifies the current camera setup: cameras.json content hash + mtime."""
    import hashlib
    if not CAMERAS_JSON.exists():
        return {"hash": None, "mtime": 0}
    data = CAMERAS_JSON.read_bytes()
    return {"hash": hashlib.sha1(data).hexdigest()[:12], "mtime": CAMERAS_JSON.stat().st_mtime}


# ---------------------------------------------------------------- (a) intrinsics
def calibrate_intrinsics(source, views=15, min_gap_s=1.0, log=print):
    """Show the board at varied angles/distances; collects `views` detections."""
    b = _board()
    cap = cv2.VideoCapture(int(source)) if str(source).isdigit() else None
    if cap is None or not cap.isOpened():
        raise RuntimeError("intrinsics needs a camera index")
    obj, img, size, last = [], [], None, 0
    t_end = time.time() + 180
    while len(img) < views and time.time() < t_end:
        ok, f = cap.read()
        if not ok:
            continue
        size = (f.shape[1], f.shape[0])
        if time.time() - last < min_gap_s:
            continue
        c = find_corners(f, b)
        if c is not None:
            obj.append(_obj_points(b)); img.append(c); last = time.time()
            log(f"view {len(img)}/{views}")
    cap.release()
    if len(img) < 8:
        raise RuntimeError(f"only {len(img)} views, need >= 8")
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    data = {"K": K.tolist(), "dist": dist.ravel().tolist(), "size": list(size), "rms_px": rms,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "setup": _setup_fingerprint()}
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    (CAL_DIR / "intrinsics.json").write_text(json.dumps(data, indent=1))
    return data


def load_intrinsics():
    p = CAL_DIR / "intrinsics.json"
    return json.loads(p.read_text()) if p.exists() else None


# ---------------------------------------------------------------- (b) table plane
def _pixel_to_table(K, dist, rvec, tvec, px):
    """Pixels (N,2) -> table XY mm by intersecting the viewing ray with z=0."""
    R, _ = cv2.Rodrigues(rvec)
    und = cv2.undistortPoints(np.asarray(px, np.float32).reshape(-1, 1, 2), K, dist).reshape(-1, 2)
    rays = np.hstack([und, np.ones((len(und), 1))])          # camera frame
    cam_o = (-R.T @ tvec.reshape(3))                         # camera origin in table frame
    d = rays @ R                                             # R.T applied to each ray
    s = -cam_o[2] / d[:, 2]
    return (cam_o[None, :] + d * s[:, None])[:, :2]


def _shift_mm(K, dist, old, new, size):
    """Same pixel grid mapped to the table with old and new plane -> disagreement in mm."""
    w, h = size
    xs, ys = np.meshgrid(np.linspace(0.2 * w, 0.8 * w, 5), np.linspace(0.2 * h, 0.8 * h, 5))
    px = np.stack([xs.ravel(), ys.ravel()], 1)
    a = _pixel_to_table(K, dist, np.array(old["rvec"]), np.array(old["tvec"]), px)
    b = _pixel_to_table(K, dist, np.array(new["rvec"]), np.array(new["tvec"]), px)
    d = np.linalg.norm(a - b, axis=1)
    return {"mean": float(d.mean()), "max": float(d.max())}


def _prune_history():
    files = sorted(HIST_DIR.glob("cal_*.json"))
    for f in files[:-KEEP_N]:
        f.unlink()


def list_history():
    return sorted(HIST_DIR.glob("cal_*.json"))


def recalibrate_table(source, log=None):
    """Quick path: one frame with the board lying on the table -> new table plane (+ reused table->robot).
    Never raises for expected problems; check result["ok"]."""
    warnings = []
    res = {"ok": False, "message": "", "shift_mm": None, "warnings": warnings, "path": None}
    try:
        intr = load_intrinsics()
        if intr is None:
            res["message"] = "No intrinsics yet. Run: calibrate.py intrinsics --source <cam> (one time)."
            return res
        frame = grab_frame(source)
        size = [frame.shape[1], frame.shape[0]]
        try:
            K = _intr_for(intr, size)        # v3.9: a scaled frame (same aspect) is fine, the pose is metric
        except ValueError:
            res["message"] = f"Resolution {size} differs from intrinsics {intr['size']}: redo intrinsics."
            return res
        b = _board()
        det = detect_board(frame, b)
        if det is None:
            res["message"] = "Board not found. Lay it flat, fully visible, no glare, then retry."
            return res
        obj, c = det
        if _is_charuco(b) and len(c) < 12:
            warnings.append(f"Only {len(c)} of 24 board corners visible: move the arm/cables off the board for a better fit.")
        dist = np.array(intr["dist"])
        ok, rvec, tvec = cv2.solvePnP(obj, c, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - c) ** 2, 1))))
        if not ok or err > 1.5:
            res["message"] = f"Poor fit (reprojection {err:.2f} px). Flatten the board and retry."
            return res
        prev = load_table()
        same_board = bool(prev) and prev.get("board") == b
        K_full = np.array(intr["K"])
        new = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "rvec": rvec.ravel().tolist(),
               "tvec": tvec.ravel().tolist(), "reproj_px": err, "board": b, "size": list(intr["size"]),
               "frame_size": size, "n_corners": int(len(c)), "setup": _setup_fingerprint(), "table_to_robot": None}
        if same_board and prev.get("table_to_robot"):
            new["table_to_robot"] = prev["table_to_robot"]
            new["robot_reused"] = True
            T_b2r = prev["table_to_robot"].get("T_board2base")
            if T_b2r is not None:        # v3.9: overview camera -> robot follows from the reused board -> robot
                Tc = np.eye(4); Tc[:3, :3] = cv2.Rodrigues(rvec)[0]; Tc[:3, 3] = tvec.ravel()
                new["T_overview2base"] = (np.array(T_b2r) @ np.linalg.inv(Tc)).tolist()
        elif prev and prev.get("table_to_robot") and not same_board:
            warnings.append("Board type changed: the old table->robot transform belongs to the old board and is not reused. "
                            "Run the auto calibration (handeye).")
        else:
            warnings.append("No table->robot transform yet: run the auto calibration (vision/handeye.py) "
                            "or the manual corner fallback (`calibrate.py robot-capture`).")
        if prev and same_board:
            res["shift_mm"] = _shift_mm(K_full, dist, prev, new, intr["size"])
            if res["shift_mm"]["max"] > 20:
                warnings.append(f"Large shift ({res['shift_mm']['max']:.0f} mm): check the board is on its mark "
                                "before trusting the reused table->robot transform.")
        HIST_DIR.mkdir(parents=True, exist_ok=True)
        path = HIST_DIR / (time.strftime("cal_%Y%m%d_%H%M%S_") + f"{int(time.time()*1000)%1000:03d}.json")
        path.write_text(json.dumps(new, indent=1))
        (CAL_DIR / "table.json").write_text(json.dumps(new, indent=1))
        _prune_history()
        res.update(ok=True, path=str(path), reproj_px=err)
        s = res["shift_mm"]
        res["message"] = (f"Table calibrated (reprojection {err:.2f} px)."
                          + (f" Shift vs previous: mean {s['mean']:.1f} mm, max {s['max']:.1f} mm." if s else " First calibration."))
        return res
    except Exception as e:  # camera errors etc.
        res["message"] = f"Recalibration failed: {e}"
        return res


def load_table(verify_source=None):
    """Load latest table calibration or None. Warnings are printed to stderr and returned via
    `load_table_with_warnings`."""
    t, _ = load_table_with_warnings(verify_source)
    return t


def load_table_with_warnings(verify_source=None):
    p = CAL_DIR / "table.json"
    if not p.exists():
        return None, ["No table calibration."]
    t, w = json.loads(p.read_text()), []
    cur = _setup_fingerprint()
    if cur["hash"] != t.get("setup", {}).get("hash"):
        w.append("Camera setup (cameras.json) changed since this calibration: recalibrate.")
    elif cur["mtime"] > time.mktime(time.strptime(t["time"], "%Y-%m-%d %H:%M:%S")):
        w.append("cameras.json was modified after this calibration: recalibrate.")
    intr = load_intrinsics()
    if intr is None or intr["size"] != t["size"]:
        w.append("Intrinsics missing or resolution changed: redo intrinsics.")
    if verify_source is not None:
        try:
            f = grab_frame(verify_source)
            det = detect_board(f, t["board"])
            if det is None:
                w.append("Board not visible: calibration cannot be confirmed.")
            elif intr:
                obj, c = det
                K, dist = _intr_for(intr, [f.shape[1], f.shape[0]]), np.array(intr["dist"])
                pr, _ = cv2.projectPoints(obj, np.array(t["rvec"]), np.array(t["tvec"]), K, dist)
                px = float(np.mean(np.linalg.norm(pr.reshape(-1, 2) - c, axis=1)))
                if px > 3:
                    w.append(f"Board is {px:.1f} px off the saved calibration: scene moved, recalibrate.")
        except Exception as e:
            w.append(f"Could not verify: {e}")
    for m in w:
        print("[calibrate] WARNING:", m, file=sys.stderr)
    return t, w


def pixel_to_table(px, table=None):
    t = table or load_table()
    intr = load_intrinsics()
    return _pixel_to_table(np.array(intr["K"]), np.array(intr["dist"]),
                           np.array(t["rvec"]), np.array(t["tvec"]), np.atleast_2d(px))


# ---------------------------------------------------------------- (c) table -> robot
def _fit_orth2d(A, B, allow_mirror=True):
    """Best 2D orthogonal map B ~ A @ R.T + t. Returns (R, t, rms, mirrored). The table frame comes from the
    board (z points into the table), so seen from above it may be a mirror image of the robot frame."""
    ca, cb = A.mean(0), B.mean(0)
    best = None
    for mirror in ((False, True) if allow_mirror else (False,)):
        Am = A.copy()
        if mirror:
            Am[:, 1] *= -1
        U, _, Vt = np.linalg.svd((Am - Am.mean(0)).T @ (B - cb))
        d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
        R0 = Vt.T @ np.diag([1, d]) @ U.T
        R = R0 @ (np.diag([1, -1]) if mirror else np.eye(2))
        t = cb - R @ ca
        rms = float(np.sqrt(np.mean(np.sum((A @ R.T + t - B) ** 2, 1))))
        if best is None or rms < best[2]:
            best = (R, t, rms, mirror)
    return best


def _save_robot_transform(out):
    t = load_table()
    if t is None:
        raise RuntimeError("run table calibration first")
    t["table_to_robot"] = out
    (CAL_DIR / "table.json").write_text(json.dumps(t, indent=1))
    h = sorted(HIST_DIR.glob("cal_*.json"))
    if h:
        h[-1].write_text(json.dumps(t, indent=1))


def calibrate_robot(pairs, extra=None, save=True):
    """pairs: [[table_x, table_y, robot_x, robot_y], ...] mm, >= 3. Rigid (or mirrored) 2D fit."""
    P = np.asarray(pairs, float)
    if len(P) < 3:
        raise ValueError("need >= 3 point pairs")
    R, tr, rms, mirror = _fit_orth2d(P[:, :2], P[:, 2:])
    out = {"R": R.tolist(), "t": tr.tolist(), "rms_mm": rms, "n": len(P), "mirrored": bool(mirror),
           "time": time.strftime("%Y-%m-%d %H:%M:%S"), **(extra or {})}
    if save:
        _save_robot_transform(out)
    return out


SIGN_COMBOS = [(a, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)]  # pan, wrist_flex, wrist_yaw


def fit_robot_from_joints(table_pts, joint_sets, tip_offset_mm=None, save=True):
    """table_pts: [(x,y)] mm in board frame; joint_sets: matching 6-joint degree lists (ARM_JOINTS order).
    Computes FK tips, tries the sign conventions of pan/wrist_flex/wrist_yaw (the URDF-vs-motor sign is not
    verified on hardware), and picks the best by 2D residual + spread of tip height (the board is planar, so all
    tips must have the same z). The default (+,+,+) is kept unless another combo is clearly better."""
    from robot_fk import B601FK
    fk = B601FK()
    A = np.asarray(table_pts, float)
    rows = []
    for sg in SIGN_COMBOS:
        tips = []
        for q in joint_sets:
            q = list(q); q[0] *= sg[0]; q[3] *= sg[1]; q[4] *= sg[2]
            tips.append(fk.tip_mm(q, tip_offset_mm))
        tips = np.array(tips)
        R, tr, rms, mirror = _fit_orth2d(A, tips[:, :2])
        zs = float(np.std(tips[:, 2]))
        rows.append(dict(signs=sg, rms_mm=rms, z_std_mm=zs, score=rms + zs, R=R, t=tr, mirrored=mirror,
                         z_mm=float(np.mean(tips[:, 2])), tips=tips))
    rows.sort(key=lambda r: r["score"])
    default = next(r for r in rows if r["signs"] == (1, 1, 1))
    best = default if default["score"] <= max(2 * rows[0]["score"], rows[0]["score"] + 3) else rows[0]
    out = {"R": best["R"].tolist(), "t": best["t"].tolist(), "rms_mm": best["rms_mm"], "z_std_mm": best["z_std_mm"],
           "z_mm": best["z_mm"], "n": len(A), "mirrored": bool(best["mirrored"]), "signs": list(best["signs"]),
           "tip_offset_mm": tip_offset_mm, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
           "per_point_error_mm": np.linalg.norm(A @ best["R"].T + best["t"] - best["tips"][:, :2], axis=1).round(1).tolist(),
           "alternatives": [(r["signs"], round(r["score"], 1)) for r in rows[:3]]}
    if save:
        _save_robot_transform(out)
    return out


def corner_targets():
    """The four outer inner corners of the board in table mm: A, B, C, D (A = origin)."""
    b = _board()
    if _is_charuco(b):   # outermost inner corners (ids 0, 5, 23, 18 for 7x5) in the board frame
        s = b["square_mm"]; xm, ym = (b["squares_x"] - 1) * s, (b["squares_y"] - 1) * s
        return {"A": (s, s), "B": (xm, s), "C": (xm, ym), "D": (s, ym)}
    xm, ym = (b["cols"] - 1) * b["square_mm"], (b["rows"] - 1) * b["square_mm"]
    return {"A": (0.0, 0.0), "B": (xm, 0.0), "C": (xm, ym), "D": (0.0, ym)}


def read_tap_joints(port=8765, seconds=1.0, max_vel_dps=1e9, timeout=10.0):
    # Note: reported motor .vel is noisy (~6 dps while holding still), so steadiness is judged by position spread.
    """Median of the follower joint angles from teleop's UDP telemetry tap (no CAN access, no commands).
    Needs teleop running (the tap is in lerobot_teleoperate_tap.py). Returns 6 degrees (ARM_JOINTS order)."""
    import socket
    from robot_fk import ARM_JOINTS
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("127.0.0.1", port))
    except OSError as e:
        raise RuntimeError(f"UDP port {port} busy (recorder/GUI listening?): {e}")
    sock.settimeout(0.5)
    samples, t_end, t0 = [], time.time() + timeout + seconds, None
    while time.time() < t_end:
        try:
            msg = json.loads(sock.recv(65536))
        except socket.timeout:
            continue
        obs = msg.get("obs") or {}
        if not all(f"{j}.pos" in obs for j in ARM_JOINTS):
            continue
        moving = max(abs(obs.get(f"{j}.vel", 0.0)) for j in ARM_JOINTS) > max_vel_dps
        if moving:
            samples, t0 = [], None
            continue
        t0 = t0 or time.time()
        samples.append([obs[f"{j}.pos"] for j in ARM_JOINTS])
        if time.time() - t0 >= seconds:
            break
    sock.close()
    if len(samples) < 5:
        raise RuntimeError("no telemetry (is teleop running?)")
    spread = float(np.max(np.ptp(np.array(samples), 0)))
    if spread > 2.0:
        print(f"  WARNING: joints moved {spread:.1f} deg during the reading; hold still and redo if the fit is poor")
    return np.median(np.array(samples), 0).tolist()


def robot_capture(corners=("A", "B", "C", "D"), manual=False, tip_offset_mm=None, input_fn=input, reader=None):
    """Interactive: touch each board corner with the gripper tip, press Enter, joints are read from the telemetry
    tap (read-only). Then fits table->robot and prints the residual. Does not command any motion."""
    tg = corner_targets()
    reader = reader or read_tap_joints
    pts, js = [], []
    print("Touch the gripper TIP to each board corner (inner corner nearest the board edge), hold still, Enter.")
    for c in corners:
        x, y = tg[c]
        input_fn(f"Corner {c} = table ({x:.0f}, {y:.0f}) mm: place the tip, then press Enter ")
        if manual:
            q = [float(v) for v in input_fn("  6 joint degrees (pan,lift,elbow,flex,yaw,roll): ").split(",")]
        else:
            q = reader()
        print("  joints:", [round(v, 1) for v in q])
        from robot_fk import B601FK
        bad = B601FK().check_limits(q)
        if bad:
            print("  WARNING outside URDF limits:", bad)
        pts.append((x, y)); js.append(q)
    r = fit_robot_from_joints(pts, js, tip_offset_mm)
    print(f"Fit: residual {r['rms_mm']:.1f} mm RMS, tip height spread {r['z_std_mm']:.1f} mm, "
          f"per point {r['per_point_error_mm']} mm, mirrored={r['mirrored']}, signs={r['signs']}")
    if r["rms_mm"] > 10 or r["z_std_mm"] > 10:
        print("WARNING: residual > 10 mm. Check the tip offset (HUMANOID_TIP_OFFSET_MM), that the tip touched the "
              "corners, or the joint sign convention. Alternatives (signs, score):", r["alternatives"])
    return r


def table_to_robot(xy, table=None):
    t = table or load_table()
    m = t["table_to_robot"]
    return np.atleast_2d(xy) @ np.array(m["R"]).T + np.array(m["t"])


def table_to_robot_xyz(xy, table=None):
    """Table mm -> robot base-frame (x, y, z) mm; z = tip height measured when touching the board.
    v3.9: with the hand-eye result (T_board2base) the exact 3D transform of the board plane is used."""
    t = table or load_table()
    T = t["table_to_robot"].get("T_board2base")
    if T is not None:
        T = np.array(T); xy = np.atleast_2d(xy).astype(float)
        P = np.hstack([xy, np.zeros((len(xy), 1))])
        return P @ T[:3, :3].T + T[:3, 3]
    xy_r = table_to_robot(xy, t)
    z = t["table_to_robot"].get("z_mm", 0.0)
    return np.hstack([xy_r, np.full((len(xy_r), 1), z)])


def apply_handeye(r):
    """v3.9: store the hand-eye result as table->robot (replaces step c). r = handeye_solve result dict.
    The board must be the same board that table.json was calibrated with; otherwise the table plane is taken from the
    sweep's overview images (press "Neu kalibrieren" afterwards for the full-resolution plane)."""
    T = np.array(r["T_board2base"])
    R2 = T[:2, :2]
    U, _, Vt = np.linalg.svd(R2)
    R2o = U @ Vt
    out = {"R": R2o.tolist(), "t": T[:2, 3].tolist(), "z_mm": float(T[2, 3]), "mirrored": bool(np.linalg.det(R2o) < 0),
           "T_board2base": T.tolist(), "method": "handeye", "quality": r["quality"]["grade"],
           "rms_mm": r["quality"]["board_spread"]["corner_mm_rms"], "n": r["n_views_used"], "signs": r.get("joint_signs"),
           "time": time.strftime("%Y-%m-%d %H:%M:%S"), "source": r.get("folder")}
    b = {k: r["board"][k] for k in ("type", "squares_x", "squares_y", "square_mm", "marker_mm", "dictionary")}
    t = load_table()
    msg = "table->robot updated"
    ov = r.get("overview") or {}
    if t is None or t.get("board") != b:
        if not ov.get("ok"):
            raise RuntimeError("table.json uses another board and the sweep has no overview pose: press 'Neu kalibrieren' "
                               "with the ChArUco board first")
        intr = load_intrinsics()
        Tc = np.array(ov["T_board2overview"])
        t = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "rvec": cv2.Rodrigues(Tc[:3, :3])[0].ravel().tolist(),
             "tvec": Tc[:3, 3].tolist(), "reproj_px": ov["reproj_px"], "board": b,
             "size": list(intr["size"]) if intr else ov["size"], "frame_size": ov["size"], "setup": _setup_fingerprint(),
             "from_handeye_sweep": True}
        msg += "; table plane taken from the sweep's overview images - press 'Neu kalibrieren' for full resolution"
    t["table_to_robot"] = out
    if ov.get("ok"):
        t["T_overview2base"] = ov["T_overview2base"]
    HIST_DIR.mkdir(parents=True, exist_ok=True)
    path = HIST_DIR / (time.strftime("cal_%Y%m%d_%H%M%S_") + f"{int(time.time()*1000)%1000:03d}.json")
    path.write_text(json.dumps(t, indent=1))
    (CAL_DIR / "table.json").write_text(json.dumps(t, indent=1))
    _prune_history()
    return msg


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["intrinsics", "table", "robot", "robot-capture", "check", "pixel2table", "history"])
    ap.add_argument("--source", default="1")
    ap.add_argument("--image")
    ap.add_argument("--views", type=int, default=15)
    ap.add_argument("--pairs")
    ap.add_argument("--px", nargs=2, type=float)
    ap.add_argument("--manual", action="store_true", help="robot-capture: type joint angles instead of telemetry")
    ap.add_argument("--tip-offset-mm", type=float, default=None)
    a = ap.parse_args()
    src = a.image or a.source
    if a.cmd == "intrinsics":
        d = calibrate_intrinsics(a.source, a.views)
        print(f"intrinsics saved, rms {d['rms_px']:.2f} px")
    elif a.cmd == "table":
        r = recalibrate_table(src)
        print(r["message"]); [print("WARNING:", w) for w in r["warnings"]]
        sys.exit(0 if r["ok"] else 1)
    elif a.cmd == "robot":
        print(calibrate_robot(json.loads(a.pairs)))
    elif a.cmd == "robot-capture":
        robot_capture(manual=a.manual, tip_offset_mm=a.tip_offset_mm)
    elif a.cmd == "check":
        _, w = load_table_with_warnings(src)
        print("OK" if not w else "\n".join(w))
    elif a.cmd == "pixel2table":
        print(pixel_to_table([a.px]))
    elif a.cmd == "history":
        [print(f.name) for f in list_history()]


if __name__ == "__main__":
    main()
