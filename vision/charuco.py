"""ChArUco board helpers (Humanoid Project): board definition, detection, pose, rendering.

The board is defined in vision/calibration/board_charuco.json (7x5 squares; printed 33 mm squares, 24.51 mm markers,
DICT_5X5_100). Every chessboard corner has a unique id, so the corner order never flips when the board is rotated
and partial views (arm or cable in front) still work - the two problems of the old 9x6 checkerboard.

Board frame (mm) = OpenCV's: origin at the outer grid corner, x along squares_x, y along squares_y, z = x cross y.
"""
import json
from pathlib import Path

import cv2
import numpy as np

CAL_DIR = Path(__file__).resolve().parent / "calibration"
BOARD_JSON = CAL_DIR / "board_charuco.json"
# Fallback only (board_charuco.json wins). v3.9.1: 33 mm / 24.51 mm = the board as actually printed on 2026-11-21
# (PDF designed for 35 / 26 mm, printer scaled to ~94 %; David measured 33 mm per square).
DEFAULT = {"type": "charuco", "squares_x": 7, "squares_y": 5, "square_mm": 33.0, "marker_mm": 24.51,
           "dictionary": "DICT_5X5_100"}


def load_spec(path=None):
    p = Path(path) if path else BOARD_JSON
    spec = dict(DEFAULT)
    if p.exists():
        spec.update(json.loads(p.read_text()))
    return spec


class Charuco:
    def __init__(self, spec=None):
        self.spec = spec or load_spec()
        s = self.spec
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s["dictionary"]))
        # board in mm, so poses / object points come out in mm
        self.board = cv2.aruco.CharucoBoard((int(s["squares_x"]), int(s["squares_y"])), float(s["square_mm"]),
                                            float(s["marker_mm"]), self.dictionary)
        dp = cv2.aruco.DetectorParameters()
        dp.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        cp = cv2.aruco.CharucoParameters()
        cp.tryRefineMarkers = True
        self.detector = cv2.aruco.CharucoDetector(self.board, cp, dp)
        self.n_corners = (int(s["squares_x"]) - 1) * (int(s["squares_y"]) - 1)
        self.corners3d = self.board.getChessboardCorners().astype(np.float64)   # (N,3) mm, index = id
        self.size_mm = (s["squares_x"] * s["square_mm"], s["squares_y"] * s["square_mm"])

    # ------------------------------------------------------------ detection
    def detect(self, bgr):
        """-> dict(ids (n,), img (n,2) px, obj (n,3) mm, n_markers) or None if < 4 corners."""
        gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        cc, ci, mc, mi = self.detector.detectBoard(gray)
        n1 = 0 if ci is None else len(ci)
        if n1 < self.n_corners and gray.shape[1] < 1200:
            # small frames (640x360 GUI frames, board far away): markers are ~10 px and detection depends on the
            # image orientation; a 2x upscale makes it reliable. Corners are mapped back to original pixels.
            up = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
            cc2, ci2, mc2, mi2 = self.detector.detectBoard(up)
            if ci2 is not None and len(ci2) > n1:
                cc, ci, mi = (cc2 + 0.5) / 2.0 - 0.5, ci2, mi2
        if ci is None or len(ci) < 4:
            return None
        ids = ci.reshape(-1).astype(int)
        img = cc.reshape(-1, 2).astype(np.float64)
        return {"ids": ids, "img": img, "obj": self.corners3d[ids], "n_markers": 0 if mi is None else len(mi)}

    def sharpness(self, bgr, det):
        """Variance of the Laplacian inside the detected corner hull (blur check)."""
        gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        x, y, w, h = cv2.boundingRect(det["img"].astype(np.float32))
        roi = gray[max(0, y):y + h, max(0, x):x + w]
        if roi.size < 100:
            return 0.0
        return float(cv2.Laplacian(roi, cv2.CV_64F).var())

    # ------------------------------------------------------------ pose
    def pose(self, det, K, dist):
        """Board -> camera pose (4x4, mm) from detected corners + reprojection RMS px. None if degenerate."""
        if det is None or len(det["ids"]) < 6:
            return None, None
        obj, img = det["obj"].astype(np.float64), det["img"].astype(np.float64)
        # collinear corners (one row) give no pose
        if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(0), tol=1e-6) < 2:
            return None, None
        ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            ok, rv, tv = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, None
        rv, tv = cv2.solvePnPRefineLM(obj, img, K, dist, rv, tv)
        pr, _ = cv2.projectPoints(obj, rv, tv, K, dist)
        err = float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - img) ** 2, 1))))
        return rt_to_T(rv, tv), err

    # ------------------------------------------------------------ rendering
    def image(self, px_per_mm=8, margin_mm=0):
        w = int(round(self.size_mm[0] * px_per_mm)); h = int(round(self.size_mm[1] * px_per_mm))
        m = int(round(margin_mm * px_per_mm))
        return self.board.generateImage((w + 2 * m, h + 2 * m), marginSize=m, borderBits=1)


def rt_to_T(rvec, tvec):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3))[0]
    T[:3, 3] = np.asarray(tvec, float).reshape(3)
    return T


def T_to_rt(T):
    return cv2.Rodrigues(T[:3, :3])[0].reshape(3), T[:3, 3].copy()


def scale_K(K, from_size, to_size):
    """Intrinsics for a scaled image (same aspect ratio). Raises if the aspect differs (crop, not scale)."""
    fw, fh = from_size; tw, th = to_size
    sx, sy = tw / fw, th / fh
    if abs(sx - sy) > 0.01 * sx:
        raise ValueError(f"aspect differs: {from_size} -> {to_size}")
    K = np.array(K, float).copy()
    K[0, :] *= sx; K[1, :] *= sy
    return K


def render_view(board_img, px_per_mm, margin_mm, T_board2cam, K, dist, size, bg=150, blur=0.0, noise=0.0, seed=0):
    """Render the flat board as seen by a pinhole camera with distortion (exact per-pixel ray casting).
    board_img: from Charuco.image(px_per_mm, margin_mm). Returns BGR uint8."""
    w, h = size
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    pts = np.stack([xs.ravel(), ys.ravel()], 1).reshape(-1, 1, 2)
    und = cv2.undistortPoints(pts, K, dist).reshape(-1, 2)              # normalized rays
    R, t = T_board2cam[:3, :3], T_board2cam[:3, 3]
    # ray: X_cam = s * [u, v, 1]; board plane in cam: n.(X - t) = 0 with n = R[:,2]
    n = R[:, 2]
    rays = np.hstack([und, np.ones((len(und), 1))])
    denom = rays @ n
    s = (n @ t) / np.where(np.abs(denom) < 1e-9, 1e-9, denom)
    Xc = rays * s[:, None]
    Xb = (Xc - t) @ R                                                     # board frame mm
    mx = (Xb[:, 0] + margin_mm) * px_per_mm - 0.5
    my = (Xb[:, 1] + margin_mm) * px_per_mm - 0.5
    valid = (s > 0)
    mx = np.where(valid, mx, -1e6).reshape(h, w).astype(np.float32)
    my = np.where(valid, my, -1e6).reshape(h, w).astype(np.float32)
    out = cv2.remap(board_img, mx, my, cv2.INTER_AREA if False else cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=bg)
    out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if blur > 0:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    if noise > 0:
        rng = np.random.default_rng(seed)
        out = np.clip(out.astype(np.float32) + rng.normal(0, noise, out.shape), 0, 255).astype(np.uint8)
    return out


if __name__ == "__main__":
    import sys
    b = Charuco()
    if len(sys.argv) > 1:
        im = cv2.imread(sys.argv[1])
        d = b.detect(im)
        print(f"{sys.argv[1]}: {0 if d is None else len(d['ids'])}/{b.n_corners} corners, "
              f"{0 if d is None else d['n_markers']} markers")
    else:
        print(b.spec, b.n_corners, "corners")
