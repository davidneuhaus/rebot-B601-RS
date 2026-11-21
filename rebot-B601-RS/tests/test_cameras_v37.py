"""Offline tests for camera_feeds v3.7 (name-based sources, no built-in camera, retry back-off). No camera needed.
Usage: ~/humanoid/.venv/bin/python test_cameras_v37.py [~/humanoid/camera_feeds.py] [~/humanoid/cameras.json]"""
import importlib.util, json, os, sys
P = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/humanoid/camera_feeds.py")
CJ = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser("~/humanoid/cameras.json")
spec = importlib.util.spec_from_file_location("cf37", P); c = importlib.util.module_from_spec(spec); spec.loader.exec_module(c)
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n)
    if not ok: fails.append(n)
D3 = [(0, "FaceTime HD-Kamera", "EAB7A68F"), (1, "HD Pro Webcam C920", "0x1100046d082d"), (2, "U20CAM-1080P", "0x1200...")]
check("name:C920 -> index 1 (C920)", c.resolve_source("name:C920", D3) == (1, "HD Pro Webcam C920"))
check("name:Innomaker|U20CAM -> index 2", c.resolve_source("name:Innomaker|U20CAM", D3)[0] == 2)
check("case-insensitive (name:c920)", c.resolve_source("name:c920", D3)[0] == 1)
check("only FaceTime attached -> name sources resolve to nothing (no fallback)",
      c.resolve_source("name:C920", D3[:1])[0] is None and c.resolve_source("name:Innomaker|U20CAM", D3[:1])[0] is None)
os.environ.pop("CAM_ALLOW_BUILTIN", None)
check("numeric index pointing at FaceTime is refused", c.resolve_source("0", D3)[0] is None)
check("numeric index of a USB camera still works", c.resolve_source("1", D3)[0] == 1)
check("FaceTime only when asked by name", c.resolve_source("name:FaceTime", D3)[0] == 0)
check("a pattern never matches the built-in camera by accident (name:HD matches C920, not 'FaceTime HD')",
      c.resolve_source("name:HD", D3)[0] == 1)
f = c.Feed("t", "name:C920")
check("back-off 2, 4, 8, 16, 30, 30 s", [f.retry_wait(k) for k in range(1, 7)] == [2, 4, 8, 16, 30, 30])
# OpenCV order = sorted by uniqueID (cap_avfoundation_mac.mm); list_cameras sorts the same way
c._CAM_LIST.update(t=1e18, list=None)
import subprocess as _sp
_orig = _sp.run
class R: stdout = json.dumps({"SPCameraDataType": [{"_name": "FaceTime HD-Kamera", "spcamera_unique-id": "EAB7A68F"},
                                                   {"_name": "HD Pro Webcam C920", "spcamera_unique-id": "0x1100046d082d"}]})
_sp.run = lambda *a, **k: R()
c._CAM_LIST.update(t=0.0, list=None)
L = c.list_cameras()
_sp.run = _orig
check(f"list_cameras sorts by uniqueID like OpenCV ({[n for _, n, _ in L]})", [n for _, n, _ in L] == ["HD Pro Webcam C920", "FaceTime HD-Kamera"])
if os.path.exists(CJ):
    cfg = json.load(open(CJ))
    srcs = [fd["source"] for fd in cfg["feeds"]]
    check(f"cameras.json: name sources, Kamera 1 = Greifer/Innomaker, Kamera 2 = Übersicht/C920 ({srcs})",
          srcs[0].startswith("name:Innomaker") and srcs[1] == "name:C920")
    _cf = c.CONFIG_FILE; c.CONFIG_FILE = CJ
    feeds, _ = c.make_feeds()
    c.CONFIG_FILE = _cf
    ov, gr = c.by_role(feeds)
    check(f"recorder/play_policy order by role: overview={ov.name!r}, gripper={gr.name!r}",
          "C920" in ov.name and "Innomaker" in gr.name)
check("role guessed from the name when 'role' is missing",
      c.guess_role({"name": "Kamera 1: Greifer"}, 0) == "gripper" and c.guess_role({"name": "Kamera 2: Übersicht"}, 1) == "overview")
check("no roles, neutral names -> old positional order",
      [f.name for f in c.by_role([type("F", (), {"name": "a", "role": ""})(), type("F", (), {"name": "b", "role": ""})()])] == ["a", "b"])
# enlarge window helper: frame_fit keeps aspect and caps at 1920x1080
import numpy as np
f = c.Feed("t", "name:C920"); f._got_frame(bgr=np.zeros((1080, 1920, 3), np.uint8))
im, _ = f.frame_fit((1000, 1000)); check(f"frame_fit 1920x1080 into 1000x1000 -> {im.size} (aspect kept)", im.size == (1000, 562))
im, _ = f.frame_fit((3000, 2000)); check(f"frame_fit never larger than 1920x1080 -> {im.size}", im.size == (1920, 1080))
f2 = c.Feed("t2", "name:C920"); f2._got_frame(bgr=np.zeros((480, 640, 3), np.uint8))
im, _ = f2.frame_fit((1600, 900)); check(f"frame_fit upscales a 640x480 frame to the window -> {im.size}", im.size == (1200, 900))
print(f"\n{len(fails)} Fehler"); sys.exit(1 if fails else 0)
