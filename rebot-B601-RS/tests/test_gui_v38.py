"""GUI smoke test for 3.8 (session buttons, shortcuts, vision tools) - no teleop, no cameras, no motors.
Runs teleop_gui.py with TELEOP_GUI_TEST=1 (no mainloop) and drives it. A window appears for a few seconds.
Usage: ~/humanoid/.venv/bin/python test_gui_v38.py [~/humanoid]"""
import os, runpy, sys, tempfile, time
HUM = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/humanoid")
os.environ.update(TELEOP_GUI_TEST="1", TELEOP_CAM1="none", TELEOP_CAM2="none", TELEMETRY_PORT="18775",
                  TELEMETRY_FWD_PORT="18776", REC_ROOT=tempfile.mkdtemp(prefix="gui38_"))
sys.path.insert(0, HUM)
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n, flush=True)
    if not ok: fails.append(n)
g = runpy.run_path(os.path.join(HUM, "teleop_gui.py"), run_name="gui_test")
root = g["root"]
def pump(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        root.update(); time.sleep(0.02)
pump(3.0)
sess, vis = g.get("sess"), g.get("vis")
check("3.8 add-ons built (session + vision)", sess is not None and vis is not None)
check("4 marker buttons, disabled without session", len(sess.bmark) == 4 and all(str(b["state"]) == "disabled" for b in sess.bmark.values()))
check("recorder process running", g["rec"]["proc"] is not None and g["rec"]["proc"].poll() is None)
_rp = g["rec"]["proc"]; _rp.stdin.close(); _rp.wait(30); g["rec"]["proc"] = None; pump(0.5)  # no live status from here on
sent = []
sess.ctx["rec_cmd"] = lambda c: sent.append(c)
sess.toggle()
check(f"toggle -> session_start with dataset/task ({sent[-1][:60] if sent else None})", sent and sent[-1].startswith("session_start "))
sess.on_status({"type": "status", "state": "session", "session": True, "take": 2, "take_seconds": 3.4,
                "counts": {"success": 1}, "saving": 1})
check("session status -> marker buttons enabled, toggle says 'Session beenden'",
      all(str(b["state"]) == "normal" for b in sess.bmark.values()) and "beenden" in sess.btog["text"])
check(f"status line shows take and counters ({sess.info['text']})", "Versuch 2" in sess.info["text"] and "✓1" in sess.info["text"])
check("app reload blocked while a session runs", g["teleop_running"]())
root.focus_force(); pump(0.2)
for ch in "sfnd":
    root.event_generate("<KeyPress>", keysym=ch); pump(0.05)
marks = [c.split()[1] for c in sent if c.startswith("mark ")]
check(f"shortcuts S/F/N/D -> {marks}", marks == ["success", "fail", "play", "discard"])
import tkinter as tk
ents = [w for w in root.winfo_children()] 
e = tk.Entry(root); e.pack(); e.focus_force(); pump(0.2)
n0 = len(sent); e.event_generate("<KeyPress>", keysym="s"); pump(0.1)
check("typing 's' in a text field does not mark", len(sent) == n0)
e.destroy()
sess.bmark["success"].invoke(); check("✓ button -> mark success", sent[-1].startswith("mark success"))
sess.on_status({"type": "status", "state": "idle", "session": False, "saving": 0, "counts": {}})
check("session off -> buttons disabled again", all(str(b["state"]) == "disabled" for b in sess.bmark.values()))
pump(1.2)
check("'Arm mit Tisch verbinden' disabled while teleop is not running", str(vis.blink["state"]) == "disabled")
vis.recalibrate(); pump(0.5)
check(f"'Neu kalibrieren' without overview camera -> clear message ({vis.msg['text'][:60]})",
      "Übersichtskamera" in vis.msg["text"] or "kein Bild" in vis.msg["text"])
vis.robot_link(); pump(0.3)
check(f"robot link refuses without teleop ({vis.msg['text'][:50]})", vis.link is None and "START" in vis.msg["text"])
# overlay drawing helper
import camera_feeds as cf, numpy as np
f = cf.Feed("t", "none"); f._got_frame(bgr=np.zeros((1080, 1920, 3), np.uint8))
f.overlay = {"dets": [("ball", 0.9, 800, 400, 1100, 700)], "size": (1920, 1080), "ts": time.time()}
im, _ = f.preview(); px = im.getpixel((int(800 * im.size[0] / 1920), int(550 * im.size[1] / 1080)))
check(f"detection box drawn on the preview (pixel {px})", px != (0, 0, 0))
# robot link flow with fake telemetry on the forward port (calibration is written to the vision dir next to this
# teleop_gui.py - run this test only on a staged copy, never on ~/humanoid)
if True:
    import socket, threading, json as _j, shutil, gui_v38
    print("   robot link test", flush=True)
    cal = gui_v38._vision_import("calibrate")
    from pathlib import Path as _P
    _tmpcal = _P(tempfile.mkdtemp(prefix="cal38_"))
    for _fn in ("intrinsics.json", "table.json"):
        if (cal.CAL_DIR / _fn).exists(): shutil.copy(cal.CAL_DIR / _fn, _tmpcal / _fn)
    cal.CAL_DIR, cal.HIST_DIR = _tmpcal, _tmpcal / "history"     # never write the real calibration
    poses = [[0, 40, 60, -10, 0, 0], [20, 45, 55, -10, 0, 0], [20, 60, 30, 0, 0, 0], [0, 55, 35, 0, 0, 0]]
    step = {"i": 0, "run": True}
    def sender():
        so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
        while step["run"]:
            q = poses[min(step["i"], 3)]
            obs = {f"{j}.pos": float(v) for j, v in zip(J, q)}; obs.update({f"{j}.vel": 0.0 for j in J})
            obs["gripper.pos"] = 0.0
            so.sendto(_j.dumps({"obs": obs, "act": obs}).encode(), ("127.0.0.1", 18776)); time.sleep(1 / 30)
    threading.Thread(target=sender, daemon=True).start()
    vis.ctx["teleop_active"] = lambda: True
    vis.robot_link(); pump(0.5)
    check("robot link dialog opens while teleop runs", vis.link is not None)
    win = vis.link["win"]
    def _text():
        return [w for w in win.winfo_children() if isinstance(w, tk.Text)][0].get("1.0", "end")
    def _bnext():
        for fr in win.winfo_children():
            for w in fr.winfo_children():
                if isinstance(w, tk.Button) and str(w["text"]).startswith("Weiter"):
                    return w
    for i, c in enumerate("ABCD"):
        step["i"] = i
        t0 = time.time()
        while str(_bnext()["state"]) != "normal" and time.time() - t0 < 10: pump(0.05)
        pump(0.3); win.event_generate("<Return>")
        t0 = time.time()
        while f"Ecke {c}: joints" not in _text() and time.time() - t0 < 15: pump(0.05)
    t0 = time.time()
    while "Fit:" not in _text() and "FEHLER" not in _text() and time.time() - t0 < 20: pump(0.1)
    txt = [w for w in win.winfo_children() if isinstance(w, tk.Text)][0].get("1.0", "end")
    print("   dialog text:", txt.strip().replace("\n", " | ")[:400])
    check("4 corners captured from the forwarded telemetry and fitted", txt.count(": joints") == 4 and "Fit:" in txt)
    step["run"] = False
    win.destroy(); vis.link = None
root.destroy()
print(f"\n{len(fails)} Fehler"); sys.exit(1 if fails else 0)
