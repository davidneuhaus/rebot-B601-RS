"""Open-vocabulary detection + outline + grasp yaw (image space only, no calibration needed).

    d = GraspDetector(segmenter="auto")          # YOLO-World (vision/models) + MobileSAM if present, else GrabCut
    objs = d.detect(bgr, ["cup", "blue block"])  # -> [DetectedObject]
    objs[0].outline_px, .center_px, .yaw_img_deg, .jaw_dir_px, .short_px, .long_px

Grasp yaw: the minimum-area rectangle of the outline is taken; the gripper jaws close along its SHORT axis
(`jaw_dir_px`, unit vector in image pixels, `yaw_img_deg` = its angle in (-90, 90], image x right / y down).
locate.py converts this image direction into a robot-frame yaw.

`detect_from_masks()` / `outline_to_grasp()` are pure functions used by the tests (no model needed).
"""
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from . import VISION_DIR

MODEL_DIR = VISION_DIR / "models"
SAM_WEIGHTS = ("mobile_sam.pt", "sam2_t.pt", "sam2.1_t.pt")


@dataclass
class DetectedObject:
    label: str
    conf: float
    box: tuple                                 # x1, y1, x2, y2 (px)
    center_px: tuple                           # grasp centre (mask centroid, else box centre)
    outline_px: Optional[np.ndarray] = None    # (N,2) float32 polygon, None if no outline could be found
    mask: Optional[np.ndarray] = field(default=None, repr=False)   # uint8 HxW, 255 = object
    rect: Optional[tuple] = None               # cv2 minAreaRect ((cx,cy),(w,h),angle)
    jaw_dir_px: Optional[tuple] = None         # unit vector of the short axis (jaw closing direction)
    yaw_img_deg: Optional[float] = None
    short_px: Optional[float] = None
    long_px: Optional[float] = None
    segmenter: str = "none"

    def to_json(self):
        d = dict(label=self.label, conf=round(self.conf, 3), box=[round(float(v), 1) for v in self.box],
                 center_px=[round(float(v), 1) for v in self.center_px], yaw_img_deg=self.yaw_img_deg,
                 jaw_dir_px=self.jaw_dir_px, short_px=self.short_px, long_px=self.long_px, segmenter=self.segmenter,
                 n_outline_points=0 if self.outline_px is None else len(self.outline_px))
        return d


# ---------------------------------------------------------------- pure geometry
def outline_to_grasp(mask):
    """Binary mask -> dict(outline, center, rect, jaw_dir, yaw_img_deg, short_px, long_px) from the largest blob, or None."""
    m = (np.asarray(mask) > 0).astype(np.uint8)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 4:
        return None
    rect = cv2.minAreaRect(c)
    box = cv2.boxPoints(rect)                          # 4 corners in order
    e1, e2 = box[1] - box[0], box[2] - box[1]
    n1, n2 = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
    short_v, short_n, long_n = (e1, n1, n2) if n1 <= n2 else (e2, n2, n1)
    u = short_v / max(short_n, 1e-9)
    if u[0] < 0 or (abs(u[0]) < 1e-9 and u[1] < 0):    # sign convention: angle in (-90, 90]
        u = -u
    M = cv2.moments(c)
    cen = (M["m10"] / M["m00"], M["m01"] / M["m00"]) if M["m00"] > 0 else rect[0]
    return dict(outline=c.reshape(-1, 2).astype(np.float32), center=(float(cen[0]), float(cen[1])), rect=rect,
                jaw_dir=(float(u[0]), float(u[1])), yaw_img_deg=float(np.degrees(np.arctan2(u[1], u[0]))),
                short_px=short_n, long_px=long_n)


def detect_from_masks(items, segmenter="given"):
    """items: [(label, conf, box, mask_or_None)] -> [DetectedObject]. Box centre / no yaw if mask is None."""
    out = []
    for label, conf, box, mask in items:
        x1, y1, x2, y2 = box
        g = outline_to_grasp(mask) if mask is not None else None
        o = DetectedObject(label, float(conf), tuple(float(v) for v in box), ((x1 + x2) / 2, (y1 + y2) / 2),
                           segmenter=segmenter if g else "none")
        if g:
            o.outline_px, o.center_px, o.rect, o.jaw_dir_px = g["outline"], g["center"], g["rect"], g["jaw_dir"]
            o.yaw_img_deg, o.short_px, o.long_px, o.mask = g["yaw_img_deg"], g["short_px"], g["long_px"], mask
        out.append(o)
    return out


def grabcut_mask(bgr, box, iters=4, pad=0.08):
    """Foreground mask inside `box` with OpenCV GrabCut (no model download). Falls back to the box on failure."""
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = box
    px, py = (x2 - x1) * pad, (y2 - y1) * pad
    X1, Y1, X2, Y2 = int(max(0, x1 - px)), int(max(0, y1 - py)), int(min(w, x2 + px)), int(min(h, y2 + py))
    roi = bgr[Y1:Y2, X1:X2]
    if roi.shape[0] < 8 or roi.shape[1] < 8:
        return None
    sc = min(1.0, 320.0 / max(roi.shape[:2]))
    small = cv2.resize(roi, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA) if sc < 1 else roi
    gm = np.full(small.shape[:2], cv2.GC_PR_BGD, np.uint8)
    bx1, by1, bx2, by2 = [int(round(v * sc)) for v in (x1 - X1, y1 - Y1, x2 - X1, y2 - Y1)]
    gm[by1:by2, bx1:bx2] = cv2.GC_PR_FGD
    gm[:max(1, by1 // 2), :] = cv2.GC_BGD; gm[:, :max(1, bx1 // 2)] = cv2.GC_BGD     # outer margin = background
    gm[min(small.shape[0] - 1, (by2 + small.shape[0]) // 2):, :] = cv2.GC_BGD
    gm[:, min(small.shape[1] - 1, (bx2 + small.shape[1]) // 2):] = cv2.GC_BGD
    try:
        bgd, fgd = np.zeros((1, 65)), np.zeros((1, 65))
        cv2.grabCut(small, gm, None, bgd, fgd, iters, cv2.GC_INIT_WITH_MASK)
    except cv2.error:
        return None
    fg = ((gm == cv2.GC_FGD) | (gm == cv2.GC_PR_FGD)).astype(np.uint8) * 255
    if sc < 1:
        fg = cv2.resize(fg, (roi.shape[1], roi.shape[0]), interpolation=cv2.INTER_NEAREST)
    full = np.zeros((h, w), np.uint8)
    full[Y1:Y2, X1:X2] = fg
    area, barea = (full > 0).sum(), max(1.0, (x2 - x1) * (y2 - y1))
    if area < 0.05 * barea:          # GrabCut lost the object -> let the caller use the box
        return None
    return full


def box_mask(shape, box):
    m = np.zeros(shape[:2], np.uint8)
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    m[max(0, y1):y2, max(0, x1):x2] = 255
    return m


# ---------------------------------------------------------------- detector
class GraspDetector:
    """segmenter: 'auto' (MobileSAM/SAM2 weights in vision/models if present, else GrabCut), 'sam', 'grabcut',
    'box' (box rectangle as the outline) or 'none' (boxes only)."""

    def __init__(self, segmenter="auto", device=None, conf=0.15, model="yolov8s-worldv2.pt", detector=None):
        if detector is None:
            from detector import Detector            # vision/detector.py (YOLO-World, MPS if available)
            detector = Detector(model=model, device=device, conf=conf)
        self.det = detector
        self.device = getattr(detector, "device", device)
        self.segmenter = self._resolve(segmenter)
        self._sam = None

    @staticmethod
    def sam_weights():
        for n in SAM_WEIGHTS:
            if (MODEL_DIR / n).exists():
                return MODEL_DIR / n
        return None

    def _resolve(self, s):
        if s == "auto":
            return "sam" if self.sam_weights() else "grabcut"
        if s == "sam" and not self.sam_weights():
            raise FileNotFoundError(f"no SAM weights in {MODEL_DIR} (expected one of {SAM_WEIGHTS}); "
                                    "download mobile_sam.pt from the ultralytics assets release")
        if s not in ("sam", "grabcut", "box", "none"):
            raise ValueError(f"unknown segmenter {s!r}")
        return s

    def _sam_masks(self, bgr, boxes):
        if self._sam is None:
            from ultralytics import SAM
            self._sam = SAM(str(self.sam_weights()))
        res = self._sam.predict(bgr, bboxes=[list(map(float, b)) for b in boxes], device=self.device, verbose=False)[0]
        if res.masks is None:
            return [None] * len(boxes)
        ms = res.masks.data.cpu().numpy()
        out = []
        for i in range(len(boxes)):
            out.append((ms[i] > 0.5).astype(np.uint8) * 255 if i < len(ms) else None)
        return out

    def detect(self, bgr, prompts, conf=None, max_objects=10):
        """-> [DetectedObject], best confidence first."""
        if list(prompts) != list(getattr(self.det, "classes", [])):
            self.det.set_classes(prompts)
        dets = self.det.detect(bgr, conf=conf)[:max_objects]
        boxes = [d[2:6] for d in dets]
        if self.segmenter == "sam" and boxes:
            masks = self._sam_masks(bgr, boxes)
        elif self.segmenter == "grabcut":
            masks = [grabcut_mask(bgr, b) for b in boxes]
        elif self.segmenter == "box":
            masks = [box_mask(bgr.shape, b) for b in boxes]
        else:
            masks = [None] * len(boxes)
        used = self.segmenter
        items = []
        for d, m in zip(dets, masks):
            if m is None and self.segmenter in ("sam", "grabcut"):    # segmenter failed on this object -> box fallback
                m, used_i = box_mask(bgr.shape, d[2:6]), "box"
            else:
                used_i = used
            items.append((d[0], d[1], d[2:6], m))
        objs = detect_from_masks(items, segmenter=used)
        for o, (_, _, _, m), d in zip(objs, items, dets):
            if m is not None and used in ("sam", "grabcut") and (m > 0).sum() == box_mask(bgr.shape, d[2:6]).astype(bool).sum():
                o.segmenter = "box"
        return objs


# ---------------------------------------------------------------- drawing
def draw(bgr, objs, plans=None):
    """Annotated copy: box, outline, min-area rectangle, jaw axis, centre, label. plans: optional {index: text}."""
    img = bgr.copy()
    for i, o in enumerate(objs):
        x1, y1, x2, y2 = [int(v) for v in o.box]
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 160, 0), 1)
        if o.outline_px is not None and o.segmenter != "box":
            cv2.polylines(img, [o.outline_px.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        if o.rect is not None:
            cv2.polylines(img, [cv2.boxPoints(o.rect).astype(np.int32).reshape(-1, 1, 2)], True, (0, 200, 255), 1)
        cx, cy = o.center_px
        if o.jaw_dir_px is not None:
            u = np.array(o.jaw_dir_px) * o.short_px / 2
            cv2.line(img, (int(cx - u[0]), int(cy - u[1])), (int(cx + u[0]), int(cy + u[1])), (0, 0, 255), 2)
        cv2.circle(img, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        txt = f"#{i} {o.label} {o.conf:.2f}" + (f" yaw {o.yaw_img_deg:.0f}" if o.yaw_img_deg is not None else "")
        if plans and i in plans:
            txt += " | " + plans[i]
        cv2.putText(img, txt, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img
