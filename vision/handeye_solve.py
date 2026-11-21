"""Hand-eye calibration solver (eye-in-hand wrist camera + eye-to-hand overview camera), offline.

Input: a sample folder vision/calibration/handeye/<timestamp>/ with samples.json
  {"samples": [{"i", "q": [6 raw deg], "wrist": "wrist_XX.png", "overview": "overview_XX.png"}, ...]}
Steps (OpenCV best practice):
 1. ChArUco detection in every wrist image; views with < min_corners are dropped.
 2. Wrist intrinsics: cv2.calibrateCamera on ChArUco object/image points from the same sweep (or a fixed file).
    Outlier views (reprojection > 3x median and > 1 px) are dropped and the calibration repeated.
 3. board->camera per view: solvePnP (IPPE + LM refine).
 4. cv2.calibrateHandEye with TSAI, PARK, HORAUD, ANDREFF, DANIILIDIS and cv2.calibrateRobotWorldHandEye
    (SHAH, LI); each candidate gives T_cam2gripper (X) and T_board2base (Z). The candidate with the lowest
    chain reprojection error is refined jointly (X, Z) by Levenberg-Marquardt on the board-corner reprojection.
 5. Joint sign conventions of pan/flex/yaw/roll are tried (16 combos); the default (all +1) is kept unless another
    is clearly better (it would mean robot_fk's convention is wrong -> reported).
 6. Overview camera: T_board2overview from its ChArUco pose (intrinsics.json scaled to the frame size) ->
    T_overview2base = T_board2base @ inv(T_board2overview).
Quality: AX=XB residuals (rotation deg / translation mm over all view pairs), spread of the per-view board pose
in the base frame, chain reprojection RMS px, per-corner position spread in mm, board height vs touched height.
"""
import itertools
import json
import time
from pathlib import Path

import cv2
import numpy as np

from charuco import Charuco, rt_to_T, T_to_rt, scale_K
from handeye_plan import Kin, rot_angle_deg, _rotvec

HERE = Path(__file__).resolve().parent
CAL_DIR = HERE / "calibration"
HE_METHODS = {"TSAI": cv2.CALIB_HAND_EYE_TSAI, "PARK": cv2.CALIB_HAND_EYE_PARK, "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
              "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF, "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS}
RW_METHODS = {"RW_SHAH": cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH, "RW_LI": cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI}
ALL_SIGNS = list(itertools.product((1, -1), repeat=4))      # (pan, flex, yaw, roll)
# quality grades (what "good" means; see docs/OPERATING_PROCEDURE.md)
GOOD = {"spread_mm": 3.0, "spread_deg": 1.0, "reproj_px": 1.5}
OK_ = {"spread_mm": 6.0, "spread_deg": 2.0, "reproj_px": 3.0}


def inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def mean_T(Ts):
    """Rotation mean (chordal, SVD) + translation mean."""
    M = sum(T[:3, :3] for T in Ts)
    U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    out = np.eye(4); out[:3, :3] = R; out[:3, 3] = np.mean([T[:3, 3] for T in Ts], 0)
    return out


def _vec2T(v):
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(v[:3].reshape(3, 1))[0]; T[:3, 3] = v[3:]
    return T


def _T2vec(T):
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


# ---------------------------------------------------------------- detection / intrinsics
def detect_all(folder, samples, board, key="wrist", min_corners=10):
    out = []
    for s in samples:
        p = folder / s[key] if s.get(key) else None
        if p is None or not p.exists():
            continue
        im = cv2.imread(str(p))
        d = board.detect(im)
        if d is None or len(d["ids"]) < min_corners:
            continue
        out.append({"i": s["i"], "q": np.array(s["q"], float), "det": d, "size": (im.shape[1], im.shape[0])})
    return out


def calibrate_intrinsics(views, size, flags=0):
    obj = [v["det"]["obj"].astype(np.float32) for v in views]
    img = [v["det"]["img"].astype(np.float32) for v in views]
    rms, K, dist, rv, tv = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    per = []
    for o, im, r, t in zip(obj, img, rv, tv):
        pr, _ = cv2.projectPoints(o, r, t, K, dist)
        per.append(float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - im) ** 2, 1)))))
    return float(rms), K, dist.ravel(), per


# ---------------------------------------------------------------- hand-eye core
def handeye_candidates(G, C):
    """G: gripper->base (mm), C: board->cam (mm) lists. -> {name: (X cam->gripper, Z board->base)}"""
    Rg = [T[:3, :3] for T in G]; tg = [T[:3, 3] for T in G]
    Rc = [T[:3, :3] for T in C]; tc = [T[:3, 3] for T in C]
    out = {}
    for name, m in HE_METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rc, tc, method=m)
            X = np.eye(4); X[:3, :3] = R; X[:3, 3] = t.ravel()
            if not np.all(np.isfinite(X)):
                continue
            Z = mean_T([Gi @ X @ Ci for Gi, Ci in zip(G, C)])
            out[name] = (X, Z)
        except cv2.error:
            pass
    # robot-world-hand-eye: world=board, needs world->cam (=board->cam) and base->gripper
    Gi = [inv(T) for T in G]
    for name, m in RW_METHODS.items():
        try:
            Rbw, tbw, Rgc, tgc = cv2.calibrateRobotWorldHandEye([T[:3, :3] for T in C], [T[:3, 3] for T in C],
                                                                 [T[:3, :3] for T in Gi], [T[:3, 3] for T in Gi], method=m)
            T_g2c = np.eye(4); T_g2c[:3, :3] = Rgc; T_g2c[:3, 3] = tgc.ravel()
            T_b2w = np.eye(4); T_b2w[:3, :3] = Rbw; T_b2w[:3, 3] = tbw.ravel()
            X, Z = inv(T_g2c), inv(T_b2w)
            if np.all(np.isfinite(X)) and np.all(np.isfinite(Z)):
                out[name] = (X, Z)
        except cv2.error:
            pass
    return out


def chain_reproj(X, Z, G, views, K, dist):
    """Predict board->cam = inv(X) inv(G) Z and reproject the detected corners. -> (rms px, per-view rms)."""
    per, allr = [], []
    for Gi, v in zip(G, views):
        Tc = inv(X) @ inv(Gi) @ Z
        rv, tv = T_to_rt(Tc)
        pr, _ = cv2.projectPoints(v["det"]["obj"], rv, tv, K, dist)
        r = pr.reshape(-1, 2) - v["det"]["img"]
        allr.append(r); per.append(float(np.sqrt(np.mean(np.sum(r ** 2, 1)))))
    allr = np.vstack(allr)
    return float(np.sqrt(np.mean(np.sum(allr ** 2, 1)))), per


def refine_lm(X, Z, G, views, K, dist, iters=30):
    """Joint LM refinement of X (cam->gripper) and Z (board->base) on the corner reprojection (12 params)."""
    def resid(p):
        Xp, Zp = _vec2T(p[:6]), _vec2T(p[6:])
        rs = []
        for Gi, v in zip(G, views):
            rv, tv = T_to_rt(inv(Xp) @ inv(Gi) @ Zp)
            pr, _ = cv2.projectPoints(v["det"]["obj"], rv, tv, K, dist)
            rs.append((pr.reshape(-1, 2) - v["det"]["img"]).ravel())
        return np.concatenate(rs)
    p = np.concatenate([_T2vec(X), _T2vec(Z)])
    r = resid(p); cost = r @ r; lam = 1e-3
    steps = np.array([1e-5] * 3 + [1e-3] * 3 + [1e-5] * 3 + [1e-3] * 3)
    for _ in range(iters):
        J = np.empty((len(r), 12))
        for k in range(12):
            dp = np.zeros(12); dp[k] = steps[k]
            J[:, k] = (resid(p + dp) - r) / steps[k]
        H = J.T @ J; g = J.T @ r
        improved = False
        for _ in range(8):
            d = -np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), g)
            rn = resid(p + d); cn = rn @ rn
            if cn < cost:
                p, r, cost, lam = p + d, rn, cn, max(lam / 5, 1e-9); improved = True
                break
            lam *= 10
        if not improved or np.max(np.abs(d)) < 1e-9:
            break
    return _vec2T(p[:6]), _vec2T(p[6:])


def axxb_residuals(X, G, C):
    """For all view pairs: A = inv(G_i) G_j (gripper motion), B = C_i inv(C_j) (camera motion, board fixed).
    AX = XB -> residual rotation deg / translation mm."""
    rd, td = [], []
    n = len(G)
    for i in range(n):
        for j in range(i + 1, n):
            A = inv(G[i]) @ G[j]; B = C[i] @ inv(C[j])
            L, Rr = A @ X, X @ B
            D = inv(L) @ Rr
            rd.append(rot_angle_deg(D[:3, :3])); td.append(float(np.linalg.norm(L[:3, 3] - Rr[:3, 3])))
    return {"rot_deg_median": float(np.median(rd)), "rot_deg_max": float(np.max(rd)),
            "trans_mm_median": float(np.median(td)), "trans_mm_max": float(np.max(td)), "pairs": len(rd)}


def board_spread(X, Z, G, C, board):
    """Per-view board pose in the base frame vs the solution Z, and per-corner 3D spread."""
    Zs = [Gi @ X @ Ci for Gi, Ci in zip(G, C)]
    rot = [rot_angle_deg(Z[:3, :3].T @ Zi[:3, :3]) for Zi in Zs]
    P = board.corners3d
    corner_err = []
    for Zi in Zs:
        a = P @ Zi[:3, :3].T + Zi[:3, 3]; b = P @ Z[:3, :3].T + Z[:3, 3]
        corner_err.append(np.linalg.norm(a - b, axis=1))
    ce = np.array(corner_err)
    return {"rot_deg_rms": float(np.sqrt(np.mean(np.square(rot)))), "rot_deg_max": float(np.max(rot)),
            "corner_mm_rms": float(np.sqrt(np.mean(ce ** 2))), "corner_mm_max": float(ce.max()),
            "per_view_corner_mm": ce.mean(1).round(2).tolist()}


def solve_views(views, kin, K, dist, board, signs=(1, 1, 1, 1)):
    sg = np.array([signs[0], 1, 1, signs[1], signs[2], signs[3]])
    G = [kin.T(v["q"] * sg) for v in views]
    C = []
    keep = []
    for k, v in enumerate(views):
        T, e = board.pose(v["det"], K, dist)
        if T is not None:
            C.append(T); keep.append(k)
    G = [G[k] for k in keep]; views = [views[k] for k in keep]
    cands = handeye_candidates(G, C)
    scored = []
    for name, (X, Z) in cands.items():
        rms, _ = chain_reproj(X, Z, G, views, K, dist)
        if np.isfinite(rms):
            scored.append((rms, name, X, Z))
    if not scored:
        raise RuntimeError("no hand-eye method produced a result")
    scored.sort(key=lambda s: s[0])
    rms0, name, X, Z = scored[0]
    Xr, Zr = refine_lm(X, Z, G, views, K, dist)
    rms1, per = chain_reproj(Xr, Zr, G, views, K, dist)
    if rms1 <= rms0:
        X, Z, rms = Xr, Zr, rms1
        refined = True
    else:
        rms, per = chain_reproj(X, Z, G, views, K, dist); refined = False
    return {"X": X, "Z": Z, "method": name, "refined": refined, "reproj_px": rms, "per_view_px": per,
            "methods": {s[1]: round(s[0], 3) for s in scored}, "G": G, "C": C, "views": views}


def solve_folder(folder, intrinsics=None, min_corners=10, try_signs=True, log=print, table_z_mm=None):
    folder = Path(folder)
    meta = json.loads((folder / "samples.json").read_text())
    samples = meta["samples"]
    board = Charuco(meta.get("board"))
    kin = Kin()
    views = detect_all(folder, samples, board, "wrist", min_corners)
    log(f"wrist views with >= {min_corners} corners: {len(views)} of {len(samples)}")
    if len(views) < 6:
        raise RuntimeError(f"only {len(views)} usable wrist views (need >= 6, better >= 12)")
    size = views[0]["size"]
    # --- wrist intrinsics
    if intrinsics:
        K, dist = np.array(intrinsics["K"], float), np.array(intrinsics["dist"], float)
        if tuple(intrinsics["size"]) != tuple(size):
            K = scale_K(K, intrinsics["size"], size)
        intr = {"K": K.tolist(), "dist": dist.tolist(), "size": list(size), "rms_px": intrinsics.get("rms_px"),
                "source": "fixed"}
        dropped = []
    else:
        rms, K, dist, per = calibrate_intrinsics(views, size)
        med = float(np.median(per))
        bad = [k for k, e in enumerate(per) if e > max(3 * med, 1.0)]
        dropped = [views[k]["i"] for k in bad]
        if bad and len(views) - len(bad) >= 6:
            views = [v for k, v in enumerate(views) if k not in bad]
            rms, K, dist, per = calibrate_intrinsics(views, size)
        intr = {"K": K.tolist(), "dist": dist.tolist(), "size": list(size), "rms_px": rms, "views": len(views),
                "source": "this sweep"}
        log(f"wrist intrinsics: rms {rms:.3f} px, fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}"
            + (f", dropped outlier views {dropped}" if dropped else ""))
    # --- hand-eye, sign conventions
    res = solve_views(views, kin, K, dist, board)
    sign_table = {(1, 1, 1, 1): res["reproj_px"]}
    if try_signs:
        best = res
        for sg in ALL_SIGNS:
            if sg == (1, 1, 1, 1):
                continue
            try:
                r = solve_views(views, kin, K, dist, board, sg)
            except Exception:
                continue
            sign_table[sg] = r["reproj_px"]
            if r["reproj_px"] < best["reproj_px"]:
                best = r; best["signs"] = sg
        if best is not res and best["reproj_px"] < 0.5 * res["reproj_px"]:
            log(f"NOTE: joint sign convention {best['signs']} (pan, flex, yaw, roll) fits much better than the default "
                f"({best['reproj_px']:.2f} vs {res['reproj_px']:.2f} px) - robot_fk's convention is wrong for those joints")
            res = best
    signs = res.get("signs", (1, 1, 1, 1))
    X, Z, G, C, views = res["X"], res["Z"], res["G"], res["C"], res["views"]
    q_ax = axxb_residuals(X, G, C)
    q_sp = board_spread(X, Z, G, C, board)
    # board plane in the base frame
    n = Z[:3, 2]; centre = Z[:3, :3] @ np.array([board.size_mm[0] / 2, board.size_mm[1] / 2, 0]) + Z[:3, 3]
    tilt = float(np.degrees(np.arccos(abs(n[2]))))
    # --- overview camera
    ov = overview_extrinsics(folder, samples, board, Z, log)
    grade = "good" if (q_sp["corner_mm_rms"] <= GOOD["spread_mm"] and q_sp["rot_deg_rms"] <= GOOD["spread_deg"]
                       and res["reproj_px"] <= GOOD["reproj_px"]) else \
            "ok" if (q_sp["corner_mm_rms"] <= OK_["spread_mm"] and q_sp["rot_deg_rms"] <= OK_["spread_deg"]
                     and res["reproj_px"] <= OK_["reproj_px"]) else "poor"
    out = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "folder": str(folder), "n_samples": len(samples),
           "n_views_used": len(views), "views_used": [v["i"] for v in views], "dropped_intrinsics_outliers": dropped,
           "wrist_intrinsics": intr, "method": res["method"], "refined": res["refined"], "methods_reproj_px": res["methods"],
           "joint_signs": list(signs), "sign_table_px": {str(k): round(v, 3) for k, v in sorted(sign_table.items(), key=lambda x: x[1])[:4]},
           "T_cam2gripper": X.tolist(), "T_board2base": Z.tolist(),
           "quality": {"grade": grade, "reproj_px": res["reproj_px"], "axxb": q_ax, "board_spread": q_sp,
                       "board_centre_base_mm": centre.round(1).tolist(), "board_tilt_deg": tilt,
                       "board_height_vs_touch_mm": (None if table_z_mm is None else float(centre[2] - table_z_mm)),
                       "thresholds_good": GOOD, "thresholds_ok": OK_},
           "overview": ov, "board": board.spec}
    return out


def overview_extrinsics(folder, samples, board, Z, log=print):
    p = CAL_DIR / "intrinsics.json"
    if not p.exists():
        return {"ok": False, "message": "no overview intrinsics (calibration/intrinsics.json)"}
    intr = json.loads(p.read_text())
    dets = []
    for s in samples:
        f = s.get("overview")
        if not f or not (folder / f).exists():
            continue
        im = cv2.imread(str(folder / f))
        d = board.detect(im)
        if d is not None and len(d["ids"]) >= 8:
            dets.append((d, (im.shape[1], im.shape[0]), f))
    if not dets:
        return {"ok": False, "message": "board not found in any overview image"}
    size = dets[0][1]
    try:
        K = scale_K(intr["K"], intr["size"], size)
    except ValueError as e:
        return {"ok": False, "message": str(e)}
    dist = np.array(intr["dist"], float)
    # the board and the overview camera are fixed -> average the pose over all images (the arm may cover parts)
    Ts, errs = [], []
    for d, sz, f in dets:
        T, err = board.pose(d, K, dist)
        if T is not None and err < 2.0:
            Ts.append(T); errs.append(err)
    if not Ts:
        return {"ok": False, "message": "overview pose failed"}
    T = mean_T(Ts); err = float(np.median(errs)); f = f"{len(Ts)} images"
    d = max(dets, key=lambda x: len(x[0]["ids"]))[0]
    T_ov2base = Z @ inv(T)
    log(f"overview camera: {len(d['ids'])} corners in {f}, reprojection {err:.2f} px, camera at "
        f"{np.round(T_ov2base[:3, 3], 0).tolist()} mm (base frame)")
    return {"ok": True, "image": f, "corners": int(len(d["ids"])), "reproj_px": err, "T_board2overview": T.tolist(),
            "T_overview2base": T_ov2base.tolist(), "size": list(size)}


def report_text(r):
    q = r["quality"]; ax = q["axxb"]; sp = q["board_spread"]
    lines = [f"Hand-eye result ({r['n_views_used']} of {r['n_samples']} views, method {r['method']}"
             f"{' + LM refinement' if r['refined'] else ''}): {q['grade'].upper()}",
             f"  wrist intrinsics: rms {r['wrist_intrinsics']['rms_px'] if r['wrist_intrinsics']['rms_px'] is not None else float('nan'):.3f} px ({r['wrist_intrinsics']['source']})",
             f"  chain reprojection (FK + hand-eye + board): {q['reproj_px']:.2f} px RMS",
             f"  AX=XB residual: rotation median {ax['rot_deg_median']:.2f} deg (max {ax['rot_deg_max']:.2f}), "
             f"translation median {ax['trans_mm_median']:.1f} mm (max {ax['trans_mm_max']:.1f})",
             f"  board pose spread over views: corners {sp['corner_mm_rms']:.1f} mm RMS (max {sp['corner_mm_max']:.1f}), "
             f"rotation {sp['rot_deg_rms']:.2f} deg RMS",
             f"  board centre in robot frame: {q['board_centre_base_mm']} mm, tilt vs horizontal {q['board_tilt_deg']:.1f} deg"
             + (f", height vs step-c touch height {q['board_height_vs_touch_mm']:+.0f} mm" if q.get('board_height_vs_touch_mm') is not None else ""),
             f"  camera -> gripper: t = {np.round(np.array(r['T_cam2gripper'])[:3, 3], 1).tolist()} mm",
             f"  methods (chain reprojection px before refinement): {r['methods_reproj_px']}",
             f"  joint signs (pan, flex, yaw, roll): {r['joint_signs']}"]
    ov = r.get("overview") or {}
    lines.append(f"  overview camera: " + (f"reprojection {ov['reproj_px']:.2f} px, position "
                 f"{np.round(np.array(ov['T_overview2base'])[:3, 3], 0).tolist()} mm" if ov.get("ok") else ov.get("message", "-")))
    return "\n".join(lines)
