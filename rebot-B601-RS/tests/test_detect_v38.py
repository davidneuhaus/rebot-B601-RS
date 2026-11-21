"""Detection overlay worker test (loads YOLO-World on MPS, ~1.2 GB RAM, ~20 s). No camera, no motors.
Usage: ~/humanoid/.venv/bin/python test_detect_v38.py [~/humanoid]"""
import os, sys, threading, time, tkinter as tk
HUM = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/humanoid")
sys.path.insert(0, HUM)
import cv2, numpy as np, camera_feeds as cf, gui_v38
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n, flush=True)
    if not ok: fails.append(n)
img = None
for p in ("vision/test_bus_cpu.jpg", "vision/test_bus_mps.jpg"):
    q = os.path.join(os.path.expanduser("~/humanoid"), p)
    if os.path.exists(q): img = cv2.imread(q); break
check("test image found", img is not None)
f = cf.Feed("Übersicht", "none"); f.role = "overview"
stop = threading.Event()
def feeder():
    while not stop.is_set():
        f._got_frame(bgr=img); time.sleep(1 / 30)
threading.Thread(target=feeder, daemon=True).start()
root = tk.Tk()
logs = []
v = gui_v38.VisionUI(root, root, {"feeds": [f], "log": logs.append, "teleop_active": lambda: False, "rec": {},
                                  "session_active": lambda: False})
v.prompts.set("bus, person")
v.det_on.set(True); v.toggle_detect()
t0 = time.time(); n = 0; seen = set()
while time.time() - t0 < 40:
    root.update(); time.sleep(0.02)
    ov = f.overlay
    if ov and ov["ts"] not in seen:
        seen.add(ov["ts"]); n += 1
        if n == 1: t1 = time.time()
    if n >= 25: break
dt = time.time() - t1 if n > 1 else 1
fps = (n - 1) / dt if n > 1 else 0
labels = sorted({d[0] for d in (f.overlay or {}).get("dets", [])})
check(f"detections arrive on the overview feed (labels {labels})", n > 1 and "bus" in labels)
check(f"rate limited to <= 8 fps ({fps:.1f} fps)", 0 < fps <= 8.6)
im, _ = f.preview(); check("preview re-rendered with boxes", im is not None)
v.det_on.set(False); v.toggle_detect(); time.sleep(0.5)
check("toggle off clears the overlay", f.overlay is None)
stop.set(); root.destroy()
print(f"\n{len(fails)} Fehler"); sys.exit(1 if fails else 0)
