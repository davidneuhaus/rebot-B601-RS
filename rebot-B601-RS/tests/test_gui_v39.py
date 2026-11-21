"""GUI test for 3.9 (Auto-Kalibrierung) - no teleop, no cameras, no motors. The run uses `handeye.sh run --yes --sim`
(simulated arm + rendered images). Teach/plan/output files go to a temp dir. A window appears for ~2 minutes.
Usage: ~/humanoid/.venv/bin/python test_gui_v39.py [~/humanoid]"""
import json, os, runpy, socket, sys, tempfile, threading, time
HUM = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/humanoid")
WORK = tempfile.mkdtemp(prefix="gui39_")
os.environ.update(TELEOP_GUI_TEST="1", TELEOP_CAM1="none", TELEOP_CAM2="none", TELEMETRY_PORT="18775",
                  TELEMETRY_FWD_PORT="18776", REC_ROOT=tempfile.mkdtemp(prefix="gui39rec_"),
                  HUMANOID_CAM_SHM_PREFIX="test39_", HANDEYE_WORK_DIR=WORK, HANDEYE_OUT_DIR=os.path.join(WORK, "out"),
                  HANDEYE_GUI_EXTRA_ARGS="--sim")
sys.path.insert(0, HUM); sys.path.insert(0, os.path.join(HUM, "vision"))
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n, flush=True)
    if not ok: fails.append(n)
g = runpy.run_path(os.path.join(HUM, "teleop_gui.py"), run_name="gui_test")
root = g["root"]; ac = g["autocal"]
G = g["start"].__globals__
G["SCRIPT"] = os.path.join(HUM, "fake_teleop.sh") if os.path.exists(os.path.join(HUM, "fake_teleop.sh")) else "/usr/bin/true"
def pump(sec, until=None):
    t0 = time.time()
    while time.time() - t0 < sec:
        root.update(); time.sleep(0.02)
        if until and until():
            return True
    return False
pump(2.0)
check("3.9 auto-calibration box built", ac is not None)
check("title shows the version", "Version" in root.title())
check("'Ziel-Pose übernehmen' disabled without teleop", str(ac.bteach["state"]) == "disabled")
check("'Auto-Kalibrierung' enabled when idle", str(ac.brun["state"]) == "normal")
check("STOPP disabled when idle", str(ac.bstop["state"]) == "disabled")
# --- v3.9.1: no taught pose -> automatic start (offline search-pose check, then confirmation; nothing moves)
check("'Gespeicherte Ziel-Pose verwenden' off by default", not ac.use_teach.get())
check("idle hint explains the automatic start", "Automatischer Start" in ac.msg["text"])
ac.plan()
ok = pump(60, lambda: ac.dlg is not None)
check(f"no taught pose -> automatic-start window ({ac.msg['text'][:60]})", ok and ac.mode_auto)
if ok:
    check("auto: 'Jetzt bewegen' disabled until all 3 checks are ticked", str(ac.bgo["state"]) == "disabled")
    check("auto: no process started before the click", ac.proc is None)
    ac.dlg.destroy(); ac.dlg = None
# --- teach from fake telemetry on the forward port
import handeye_synth as SY
truth = SY.make_truth(); qt = SY.teach_pose(truth)
stop_tx = {"v": False}
def tx():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
    while not stop_tx["v"]:
        obs = {f"{n}.pos": float(v) for n, v in zip(names, qt)}; obs.update({f"{n}.vel": 0.0 for n in names})
        s.sendto(json.dumps({"obs": obs}).encode(), ("127.0.0.1", 18776)); time.sleep(0.03)
threading.Thread(target=tx, daemon=True).start()
ac.ctx["teleop_active"] = lambda: True
pump(0.5)
check("'Ziel-Pose übernehmen' enabled while teleop runs", str(ac.bteach["state"]) == "normal")
check("'Auto-Kalibrierung' disabled while teleop runs", str(ac.brun["state"]) == "disabled")
ac.teach()
ok = pump(15, lambda: os.path.exists(os.path.join(WORK, "handeye_teach.json")) and "✓ Ziel-Pose übernommen" in ac.msg["text"])
check(f"teach -> pose saved from telemetry ({ac.msg['text'][:70]})", ok)
pump(0.3); check("teach ticks 'Gespeicherte Ziel-Pose verwenden' (manual override)", ac.use_teach.get())
stop_tx["v"] = True
ac.ctx["teleop_active"] = lambda: False
# --- plan -> confirm dialog (nothing moves before the click)
ac.plan(); pump(0.3)
check("reload blocked while planning", g["teleop_running"]())
ok = pump(240, lambda: ac.dlg is not None)
check("plan done -> confirmation window with pose list + plot", ok)
if ok:
    check("'Jetzt bewegen' disabled until all 3 checks are ticked", str(ac.bgo["state"]) == "disabled")
    ac.chk[0].set(True); ac.chk[1].set(True); ac._chk_changed()
    check("2 of 3 ticked -> still disabled", str(ac.bgo["state"]) == "disabled")
    ac.chk[2].set(True); ac._chk_changed()
    check("all ticked -> enabled", str(ac.bgo["state"]) == "normal")
    check("no process started before the click", ac.proc is None)
    ac.bgo.invoke(); pump(1.0)
    check("click -> handeye.sh run started (sim)", ac.proc is not None or ac.result_lines)
    check("STOPP enabled while running", str(ac.bstop["state"]) == "normal" or ac.proc is None)
    check("app reload blocked while running", g["teleop_running"]())
    G["start"](); pump(0.3)
    check("teleop START refused while the auto calibration runs", G["proc"] is None)
    ok = pump(300, lambda: ac.proc is None)
    check(f"sim run finished with a result ({' | '.join(ac.result_lines)[:120]})", ok and ac.result_lines and "GOOD" in ac.result_lines[0])
    # --- stop test
    ac.go(); pump(4.0)
    t0 = time.time(); ac.stop()
    ok = pump(5, lambda: ac.motors_off or ac.proc is None)
    check(f"STOPP: motors off within 2 s ({time.time() - t0:.1f} s)", ok and time.time() - t0 < 2.5)
    # v3.9.2: with >= 10 good samples the process solves the partial sweep after the stop, then ends by itself
    ok = pump(120, lambda: ac.proc is None)
    check(f"after STOPP the process ends (partial sweep solved if >= 10 samples) ({' | '.join(ac.result_lines)[:100]})", ok)
# --- v3.9.1: automatic start end-to-end (handeye.sh run --yes --auto --sim)
ac.use_teach.set(False)
ac.plan()
ok = pump(60, lambda: ac.dlg is not None and ac.mode_auto)
check("auto window again after unticking the taught pose", ok)
if ok:
    for v in ac.chk: v.set(True)
    ac._chk_changed(); ac.bgo.invoke(); pump(1.0)
    ok = pump(400, lambda: ac.proc is None)
    check(f"auto sim run finished GOOD ({' | '.join(ac.result_lines)[:120]})", ok and ac.result_lines and "GOOD" in ac.result_lines[0])
print("\n%d failures" % len(fails) if fails else "\nALL OK")
try: root.destroy()
except Exception: pass
rp = g["rec"]["proc"]
if rp is not None:
    try: rp.stdin.close(); rp.wait(10)
    except Exception: pass
sys.exit(1 if fails else 0)
