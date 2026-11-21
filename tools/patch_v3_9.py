#!/usr/bin/env python3
"""Patch teleop_gui.py 3.8 -> 3.9 (auto calibration button, gui_v39.py). Idempotent. Usage: patch_v3_9.py <teleop_gui.py>"""
import sys
p = sys.argv[1]
s = open(p).read()
if "gui_v39" in s and "shared=False" in s:
    print("already patched"); sys.exit(0)
if "gui_v39" in s:   # 3.9 without the camera fix -> only the camera fix
    s = s.replace("    cam_feeds, cam_cfg = camera_feeds.make_feeds(CAM_BOX)\n", "    cam_feeds, cam_cfg = camera_feeds.make_feeds(CAM_BOX, shared=False)   # v3.9: GUI owns the cameras (also after a reload)\n")
    open(p, "w").write(s); print("camera fix applied"); sys.exit(0)
def rep(a, b):
    global s
    assert s.count(a) == 1, (a[:70], s.count(a))
    s = s.replace(a, b)
rep('''             os.path.join(os.path.dirname(APP_FILE), "gui_v38.py"))   # v3.8''',
    '''             os.path.join(os.path.dirname(APP_FILE), "gui_v38.py"),   # v3.8
             os.path.join(os.path.dirname(APP_FILE), "gui_v39.py"))   # v3.9''')
rep('''def start():
    global proc
    if globals().get("play") and play_running():
        log("Policy läuft – erst Policy-STOPP\\n"); return
''', '''def start():
    global proc
    if globals().get("play") and play_running():
        log("Policy läuft – erst Policy-STOPP\\n"); return
    if globals().get("autocal") is not None and autocal.busy():
        log("Auto-Kalibrierung läuft – erst deren STOPP / Ende abwarten\\n"); return   # v3.9
''')
rep('''def teleop_running():
    if globals().get("play") and play_running():
        return True   # v3.4: no app reload while a policy runs
''', '''def teleop_running():
    if globals().get("play") and play_running():
        return True   # v3.4: no app reload while a policy runs
    if globals().get("autocal") is not None and autocal.busy():
        return True   # v3.9: no app reload while the auto calibration plans/runs
''')
rep('''def on_close():
    if globals().get("play") and play_running():
        play_stop(); root.after(2500, on_close); return   # v3.4
''', '''def on_close():
    if globals().get("play") and play_running():
        play_stop(); root.after(2500, on_close); return   # v3.4
    if globals().get("autocal") is not None and autocal.running():
        autocal.stop(); root.after(2500, on_close); return   # v3.9
''')
rep('''def play_start(confirm=True):
    if play_running():
        return
''', '''def play_start(confirm=True):
    if play_running():
        return
    if globals().get("autocal") is not None and autocal.busy():
        play_msg.config(text="Auto-Kalibrierung läuft – erst deren Ende abwarten", fg="#c0392b"); return   # v3.9
''')
rep('''root.after(1000, rec_ensure)
root.after(1200, rec_poll)
''', '''# --- v3.9: automatic hand-eye calibration (gui_v39.py) -------------------------------------------------------------
def _arm_busy_reason():
    if proc is not None and proc.poll() is None or str(bstart["state"]) == "disabled" or stopping["on"]:
        return "Teleop läuft – erst ■ STOPP"
    if globals().get("play") and play_running():
        return "Policy läuft – erst Policy-STOPP"
    if rec.get("state") in ("recording", "paused", "saving", "session") or (sess is not None and sess.busy()):
        return "Aufnahme läuft / wird gespeichert"
    return None
autocal = None
try:
    import gui_v39
    autocal = gui_v39.build(root, body, ctx={
        "log": log, "busy": _arm_busy_reason,
        "teleop_active": lambda: proc is not None and proc.poll() is None and not stopping["on"]})
except Exception as _e:
    tk.Label(body, text=f"3.9-Auto-Kalibrierung deaktiviert: {_e}", fg="#c0392b").pack()
root.after(1000, rec_ensure)
root.after(1200, rec_poll)
''')
# v3.9 fix: the GUI always owns the cameras. After an automatic app reload (os.execv) the environment still had
# HUMANOID_CAM_SHM=1 (meant for the GUI's children), so the reloaded GUI waited for shared frames: "Kein Signal".
rep('''    cam_feeds, cam_cfg = camera_feeds.make_feeds(CAM_BOX)''',
    '''    cam_feeds, cam_cfg = camera_feeds.make_feeds(CAM_BOX, shared=False)   # v3.9: GUI owns the cameras (also after a reload)''')
open(p, "w").write(s)
print("patched", p)
