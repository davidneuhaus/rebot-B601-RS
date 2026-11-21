"""Open-vocabulary object detector (YOLO-World via ultralytics).

API:
    det = Detector()                         # loads yolov8s-worldv2, picks MPS/CPU
    det.set_classes(["blue ball", "Nivea tin"])
    dets = det.detect(bgr_frame)             # [(label, conf, x1, y1, x2, y2), ...] in original pixels
    out = det.draw(bgr_frame, dets)          # copy with boxes + labels
"""
import os
import time
from pathlib import Path

import cv2
import numpy as np

MODEL_DIR = Path(os.environ.get("HUMANOID_VISION_MODELS", Path.home() / "humanoid" / "vision" / "models"))
DEFAULT_MODEL = "yolov8s-worldv2.pt"


def pick_device(prefer=None):
    if prefer:
        return prefer
    try:
        import torch
        if torch.backends.mps.is_available():
            return "mps"
    except Exception:
        pass
    return "cpu"


class Detector:
    def __init__(self, model=DEFAULT_MODEL, device=None, imgsz=640, conf=0.15, min_interval=0.0):
        from ultralytics import YOLOWorld
        MODEL_DIR.mkdir(parents=True, exist_ok=True)
        path = Path(model)
        if not path.is_absolute():
            path = MODEL_DIR / model
        if not path.exists():  # ultralytics downloads into cwd; do it in MODEL_DIR
            cwd = os.getcwd()
            os.chdir(MODEL_DIR)
            try:
                self.model = YOLOWorld(model)
            finally:
                os.chdir(cwd)
        else:
            self.model = YOLOWorld(str(path))
        self.device = pick_device(device)
        self.imgsz = imgsz
        self.conf = conf
        self.min_interval = min_interval  # throttle: reuse last result if called faster
        self.classes = []
        self._last_t = 0.0
        self._last = []

    def set_classes(self, prompts):
        prompts = [p.strip() for p in prompts if p and p.strip()]
        if not prompts:
            raise ValueError("need at least one prompt")
        self.model.set_classes(prompts)  # encodes text once with CLIP
        self.classes = prompts
        self._last = []

    def detect(self, bgr, conf=None):
        if not self.classes:
            raise RuntimeError("call set_classes() first")
        now = time.time()
        if self.min_interval and now - self._last_t < self.min_interval:
            return list(self._last)
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=conf or self.conf,
                                 device=self.device, verbose=False)[0]
        out = []
        names = res.names
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            out.append((names[int(b.cls[0])], float(b.conf[0]), x1, y1, x2, y2))
        out.sort(key=lambda d: -d[1])
        self._last, self._last_t = out, now
        return list(out)

    @staticmethod
    def draw(bgr, detections):
        img = bgr.copy()
        for label, conf, x1, y1, x2, y2 in detections:
            h = abs(hash(label)) % 180
            col = tuple(int(c) for c in cv2.cvtColor(np.uint8([[[h, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0])
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(img, p1, p2, col, 2)
            txt = f"{label} {conf:.2f}"
            (tw, th), bl = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            y0 = max(p1[1], th + bl + 2)
            cv2.rectangle(img, (p1[0], y0 - th - bl - 2), (p1[0] + tw + 4, y0), col, -1)
            cv2.putText(img, txt, (p1[0] + 2, y0 - bl), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        return img
