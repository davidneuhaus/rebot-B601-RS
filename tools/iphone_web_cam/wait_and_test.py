"""Wait (default 15 min) for iPhone frames, then grab one via cv2.VideoCapture and save it."""
import json, os, sys, time, urllib.request
import cv2
timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 900
out = os.path.expanduser("~/humanoid/outputs/iphone_test.jpg")
url = "http://127.0.0.1:8090/stream.mjpg"
t_end = time.time() + timeout
while time.time() < t_end:
    try:
        st = json.load(urllib.request.urlopen("http://127.0.0.1:8090/status", timeout=2))
        if st["frames"] > 30 and (st["age_s"] or 99) < 1: break
    except Exception: pass
    time.sleep(2)
else:
    sys.exit("timeout: no frames from iPhone")
time.sleep(3)
cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
ok, frame = cap.read()
if not ok: sys.exit("cv2 could not read stream")
t0, n = time.time(), 0
while time.time() - t0 < 4:
    ok, f = cap.read()
    if ok: frame, n = f, n + 1
fps = n / (time.time() - t0)
os.makedirs(os.path.dirname(out), exist_ok=True)
cv2.imwrite(out, frame)
st = json.load(urllib.request.urlopen("http://127.0.0.1:8090/status", timeout=2))
print(f"OK saved {out} shape={frame.shape} cv2_fps={fps:.1f} server={json.dumps(st)}")
