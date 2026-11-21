"""End-to-end test of recorder.py session mode (v3.8) without robot or cameras: synthetic shared-memory frames,
fake UDP telemetry, a temporary dataset root. Never touches CAN or motors.
Usage: ~/humanoid/.venv/bin/python test_session_v38.py [~/humanoid]"""
import json, os, socket, subprocess, sys, tempfile, threading, time
HUM = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/humanoid")
sys.path.insert(0, HUM)
PORT, FWD = 18765, 18766
os.environ.update(HUMANOID_CAM_SHM_PREFIX="t38_", TELEMETRY_PORT=str(PORT), TELEMETRY_FWD_PORT=str(FWD))
import numpy as np
import camera_feeds as cf
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n, flush=True)
    if not ok: fails.append(n)

tmp = tempfile.mkdtemp(prefix="rec38_")
stop = threading.Event()
pubs = {r: cf.FramePublisher(r, prefix="t38_") for r in ("overview", "gripper")}
def cams():
    i = 0
    while not stop.is_set():
        for k, (r, p) in enumerate(pubs.items()):
            img = np.full((720, 1280, 3), (i * 3 + 80 * k) % 255, np.uint8)
            p.publish(img)
        i += 1; time.sleep(1 / 30)
MOT = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
tel_on = threading.Event(); tel_on.set()
def tel():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t = 0
    while not stop.is_set():
        if tel_on.is_set():
            obs = {f"{m}.pos": float(np.sin(t / 30 + j)) for j, m in enumerate(MOT)}
            obs.update({f"{m}.vel": 0.0 for m in MOT}); obs.update({f"{m}.torque": 0.1 for m in MOT})
            act = {f"{m}.pos": float(np.sin(t / 30 + j)) for j, m in enumerate(MOT)}
            s.sendto(json.dumps({"obs": obs, "act": act}).encode(), ("127.0.0.1", PORT)); t += 1
        time.sleep(1 / 30)
fwd_n = [0]
def fwd_listen():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", FWD)); s.settimeout(0.5)
    while not stop.is_set():
        try: s.recv(65536); fwd_n[0] += 1
        except socket.timeout: pass
for f in (cams, tel, fwd_listen):
    threading.Thread(target=f, daemon=True).start()

env = dict(os.environ, HUMANOID_CAM_SHM="1", REC_ROOT=tmp)
p = subprocess.Popen([sys.executable, os.path.join(HUM, "recorder.py")], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                     stderr=open(os.path.join(tmp, "recorder.log"), "w"), text=True, bufsize=1, cwd=HUM, env=env)
last = {}; saved = []; errors = []
def reader():
    for line in p.stdout:
        try: m = json.loads(line)
        except Exception: continue
        if m.get("type") == "status": last.clear(); last.update(m)
        elif m.get("type") == "saved": saved.append(m)
        elif m.get("type") == "error": errors.append(m)
threading.Thread(target=reader, daemon=True).start()
def cmd(c): p.stdin.write(c + "\n"); p.stdin.flush()
def wait(cond, t=20):
    t0 = time.time()
    while time.time() - t0 < t:
        if cond(): return True
        time.sleep(0.05)
    return False

check("recorder ready, cameras from shared memory", wait(lambda: last.get("cams") == [True, True], 30))
check("telemetry forwarded to the robot-link port", wait(lambda: fwd_n[0] > 10, 5))
cmd('session_start {"dataset": "s38", "task": "pick the ball"}')
check("session starts", wait(lambda: last.get("session") and last.get("state") == "session"))
time.sleep(float(os.environ.get("T38_FIRST_TAKE_S", "2.0"))); cmd('mark success {"task": "pick the ball"}')
check("after ✓: a new take runs while the previous one is being saved (async)",
      wait(lambda: last.get("take") == 2 and last.get("take_frames", 0) > 10 and last.get("saving", 0) >= 1, 3)
      or (wait(lambda: last.get("take") == 2 and last.get("take_frames", 0) > 10, 3) and len(saved) <= 1))
time.sleep(1.5); cmd('mark fail {"task": "pick the ball"}')
time.sleep(1.5); cmd("mark discard")
time.sleep(0.4); cmd("mark play")      # < 1 s -> not saved
time.sleep(0.3)
tel_on.clear()                          # teleop stopped -> take pauses, frames kept
check("no joint data -> take pauses", wait(lambda: last.get("take_paused"), 3))
tel_on.set(); check("joint data back -> take continues", wait(lambda: not last.get("take_paused"), 3))
time.sleep(1.5); cmd('session_stop {"task": "free play"}')
check("session stops, background saves finish", wait(lambda: not last.get("session") and last.get("saving") == 0
                                                     and last.get("state") == "idle" and len(saved) >= 3, 120))
check(f"3 episodes saved with labels success, fail, play (got {[m.get('label') for m in saved]})",
      [m.get("label") for m in saved] == ["success", "fail", "play"])
info = json.load(open(os.path.join(tmp, "s38", "meta", "info.json")))
check(f"info.json total_episodes 3 (got {info['total_episodes']})", info["total_episodes"] == 3)
labs = [json.loads(l) for l in open(os.path.join(tmp, "s38", "meta", "episode_labels.jsonl"))]
check(f"labels file {[(l['episode_index'], l['label'], l['task']) for l in labs]}",
      [(l["episode_index"], l["label"]) for l in labs] == [(0, "success"), (1, "fail"), (2, "play")]
      and labs[2]["task"] == "free play")
fr = [m["frames"] for m in saved]
T1 = float(os.environ.get("T38_FIRST_TAKE_S", "2.0"))
check(f"frame rate: take 1 has {fr[0]} frames for {T1:.1f} s (>= 85% of 30 fps), takes 2/3 {fr[1:]}",
      fr[0] >= 0.85 * 30 * T1 and fr[1] >= 0.85 * 30 * 1.5 and fr[2] >= 30)
check("no save errors", not errors)
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id="local/s38", root=os.path.join(tmp, "s38"))
it = ds[0]
check(f"dataset loads: {ds.num_episodes} episodes, {ds.num_frames} frames, image {tuple(it['observation.images.overview'].shape)}",
      ds.num_episodes == 3 and tuple(it["observation.images.overview"].shape) == (3, 360, 640))
# classic mode still works and labels its episode 'play'
cmd('start {"dataset": "s38", "task": "classic"}'); time.sleep(1.5); cmd("save")
check("classic Episode starten/speichern still works (episode 4, label play)", wait(lambda: len(saved) >= 4, 60)
      and saved[-1].get("label") == "play")
cmd("quit"); p.wait(60)
stop.set()
# export
r = subprocess.run([sys.executable, os.path.join(HUM, "tools", "export_dataset.py"), "s38", "--root", tmp],
                   capture_output=True, text=True)
print(r.stdout.strip()); print(r.stderr.strip()[-800:])
ok = os.path.exists(os.path.join(tmp, "s38_success", "meta", "info.json"))
check("export success-only -> s38_success with 1 episode",
      ok and json.load(open(os.path.join(tmp, "s38_success", "meta", "info.json")))["total_episodes"] == 1)
if ok:
    l2 = [json.loads(l) for l in open(os.path.join(tmp, "s38_success", "meta", "episode_labels.jsonl"))]
    check(f"exported labels renumbered {l2}", len(l2) == 1 and l2[0]["episode_index"] == 0 and l2[0]["source_episode"] == 0)
    d2 = LeRobotDataset(repo_id="local/s38_success", root=os.path.join(tmp, "s38_success"))
    check(f"exported dataset loads ({d2.num_frames} frames = {fr[0]})", d2.num_frames == fr[0])
for pb in pubs.values():
    try: pb.shm.unlink()
    except Exception: pass
print(f"\n{len(fails)} Fehler  (tmp {tmp})"); sys.exit(1 if fails else 0)
