#!/usr/bin/env python
"""Live or still-image open-vocabulary detection.

  detect_live.py --source 1 --prompts 'blue ball,Nivea tin'          # camera index
  detect_live.py --source C920 --prompts 'trash bin'                 # camera name (macOS AVFoundation)
  detect_live.py --image in.jpg --prompts 'person,bus' --out out.jpg # still
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detector import Detector  # noqa: E402


def index_for_name(name):
    """Resolve a camera name to an AVFoundation index via ffmpeg (if available)."""
    try:
        out = subprocess.run(["ffmpeg", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                             capture_output=True, text=True, timeout=15).stderr
    except Exception:
        return None
    for line in out.splitlines():
        if name.lower() in line.lower() and "] [" in line:
            return int(line.split("] [")[2].split("]")[0])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--image")
    ap.add_argument("--out", default=None)
    ap.add_argument("--prompts", required=True)
    ap.add_argument("--device", default=None, help="mps|cpu (default auto)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.15)
    ap.add_argument("--max-fps", type=float, default=10.0, help="throttle detection rate")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0=forever)")
    ap.add_argument("--no-window", action="store_true")
    a = ap.parse_args()

    det = Detector(device=a.device, imgsz=a.imgsz, conf=a.conf)
    det.set_classes([p for p in a.prompts.split(",")])
    print(f"device={det.device} prompts={det.classes}")

    if a.image:
        img = cv2.imread(a.image)
        det.detect(img)  # warm-up
        t = time.time()
        n = 5
        for _ in range(n):
            dets = det.detect(img)
        print(f"{(time.time()-t)/n*1000:.0f} ms/frame")
        for d in dets:
            print("%s %.2f (%d,%d,%d,%d)" % (d[0], d[1], *map(int, d[2:])))
        out = a.out or str(Path(a.image).with_suffix("")) + "_det.jpg"
        cv2.imwrite(out, det.draw(img, dets))
        print("saved", out)
        return

    src = a.source
    if src.isdigit():
        src = int(src)
    else:
        idx = index_for_name(src)
        if idx is None:
            sys.exit(f"camera named {src!r} not found")
        src = idx
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"cannot open source {src}")
    show = not a.no_window
    min_dt = 1.0 / a.max_fps
    n, t0, last_det, dets = 0, time.time(), 0.0, []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        now = time.time()
        if now - last_det >= min_dt:
            dets = det.detect(frame)
            last_det = now
        n += 1
        if n % 30 == 0:
            print(f"fps={n/(time.time()-t0):.1f} dets={[(d[0], round(d[1],2)) for d in dets]}", flush=True)
            n, t0 = 0, time.time()
        if show:
            try:
                cv2.imshow("detect", det.draw(frame, dets))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            except cv2.error:
                print("no GUI support in this OpenCV build; running headless "
                      "(saving /tmp/detect_last.jpg)")
                show = False
        else:
            cv2.imwrite("/tmp/detect_last.jpg", det.draw(frame, dets))
        if a.frames and n >= a.frames:
            break
    cap.release()


if __name__ == "__main__":
    main()
