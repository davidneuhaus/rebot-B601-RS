#!/usr/bin/env python3
"""Small window: start/stop for teleoperation (white reBot 102 leader drives the B601-RS). GUI labels are German."""
import json, os, queue, signal, subprocess, sys, threading, tkinter as tk
from tkinter import scrolledtext

# --- version + self-update (v3.0) -------------------------------------------------
APP_FILE = os.path.abspath(__file__)
VERSION_FILE = os.path.join(os.path.dirname(APP_FILE), "VERSION")
UPDATE_CHECK_MS = 5000
CAM_FILES = (os.path.join(os.path.dirname(APP_FILE), "camera_feeds.py"),
             os.path.join(os.path.dirname(APP_FILE), "cameras.json"),
             os.path.join(os.path.dirname(APP_FILE), "gui_v38.py"),   # v3.8
             os.path.join(os.path.dirname(APP_FILE), "gui_v39.py"))   # v3.9

def read_version():
    try:
        with open(VERSION_FILE) as f:
            return f.read().strip() or "?"
    except OSError:
        return "?"

def file_sig():
    """Fingerprint of teleop_gui.py, VERSION, camera_feeds.py and cameras.json (mtime + size)."""
    sig = []
    for p in (APP_FILE, VERSION_FILE) + CAM_FILES:
        try:
            st = os.stat(p); sig.append((st.st_mtime_ns, st.st_size))
        except OSError:
            sig.append(None)
    return tuple(sig)

def update_decision(sig_now, start_sig, last_sig, running, bad_sig=None):
    """Pure decision (testable): 'none' = nothing changed, 'wait' = file is changing right now
    (must be equal at the next check), 'pending' = update available but teleop runs, 'restart' = reload,
    'bad' = this version already failed to compile."""
    if sig_now == start_sig:
        return "none"
    if bad_sig is not None and sig_now == bad_sig:
        return "bad"
    if running:
        return "pending"
    if sig_now != last_sig:
        return "wait"
    return "restart"

APP_VERSION = read_version()
START_SIG = file_sig()
upd = {"last": START_SIG, "bad": None}

SCRIPT = os.path.expanduser("~/humanoid/teleop.sh")
proc = None
LOGDIR = os.path.expanduser("~/humanoid/logs"); os.makedirs(LOGDIR, exist_ok=True)
logfile = None
q = queue.Queue()
ending = {"on": False}

OUT_MAX_LINES = 1500   # v3.7: keep the log window small (a 20k+ line Tk text widget burned ~90% CPU)
_ANSI = None
_loop_n = {"n": 0}

def show_in_window(line):
    """v3.7: which teleop output lines go into the log WINDOW (the session log file still gets every line).
    The 30 Hz 'Teleop loop time' lines (only every 150th, ~5 s), the clamp warnings and their dict dumps are
    skipped - they made up >90% of the output and flooded the Tk text widget."""
    s = clean_line(line).strip()
    if s.startswith("Teleop loop time"):
        _loop_n["n"] += 1
        return _loop_n["n"] % 150 == 1
    if "had to be clamped to be safe" in s:
        return False
    if s.startswith(("{'", "{ ", "'", "}")) and ("goal_pos" in s or s.startswith(("'", "}"))):
        return False
    return True

def clean_line(line):
    global _ANSI
    if _ANSI is None:
        import re
        _ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
    return _ANSI.sub("", line)

def log(msg, window=True):
    if logfile and not logfile.closed:
        logfile.write(msg); logfile.flush()
    if not window or not msg:
        return
    out.configure(state="normal"); out.insert("end", clean_line(msg))
    n = int(out.index("end-1c").split(".")[0])
    if n > OUT_MAX_LINES:
        out.delete("1.0", f"{n - OUT_MAX_LINES}.0")
    out.see("end"); out.configure(state="disabled")

def reader(p):
    # background thread: only put lines into the queue (Tk is not thread-safe)
    for line in iter(p.stdout.readline, ""):
        q.put(line)

def poll():
    # runs in the main thread: show output and detect process end
    # v3.7: batched - all queued lines go to the log file at once, the window gets one filtered insert
    buf_file, buf_win = [], []
    def _flush():
        if buf_file:
            if logfile and not logfile.closed:
                logfile.write("".join(buf_file)); logfile.flush()
            if buf_win:
                _win("".join(buf_win))
            buf_file.clear(); buf_win.clear()
    for _ in range(5000):
        try: item = q.get_nowait()
        except queue.Empty: break
        if isinstance(item, tuple) and item[0] == "off":
            _flush(); finished_off(item[1])
        else:
            buf_file.append(item)
            if show_in_window(item):
                buf_win.append(item)
            collision_status(item)
    _flush()
    if proc is not None and proc.poll() is not None and str(bstart["state"]) == "disabled" and not ending["on"]:
        ending["on"] = True
        finished(proc.returncode)
    root.after(200, poll)

def _win(msg):
    """v3.7: window-only insert (the file was already written by poll())."""
    out.configure(state="normal"); out.insert("end", clean_line(msg))
    n = int(out.index("end-1c").split(".")[0])
    if n > OUT_MAX_LINES:
        out.delete("1.0", f"{n - OUT_MAX_LINES}.0")
    out.see("end"); out.configure(state="disabled")

def collision_status(line):
    # show collision guard messages from the log in the status (only while teleop runs, not while stopping)
    if stopping["on"] or ending["on"] or proc is None or proc.poll() is not None:
        return
    if "KOLLISION aktiv" in line and "Leader: " in line:
        import re
        m = re.search(r"Leader: (.+?) [\d.]+ deg zurückführen", line)
        name = m.group(1) if m else "Gelenk"
        extra = "oder kurz warten" if "Auto-Weiter in" in line else ""
        status.config(text=f"⚠ KOLLISION\n{name} zurückführen {extra}".rstrip(), fg="#e67e22", justify="left")
    elif "KOLLISION erkannt" in line or "KOLLISION aktiv" in line:
        status.config(text="⚠ KOLLISION – Leader zurückführen", fg="#e67e22")
    elif "Auto-Weiter: Arm faehrt langsam weiter" in line:
        status.config(text="⟳ LANGSAM WEITER …", fg="#d4ac0d")
    elif "Kollision aufgehoben" in line:
        status.config(text="● LÄUFT", fg="#1a9c3a")

def start():
    global proc
    if globals().get("play") and play_running():
        log("Policy läuft – erst Policy-STOPP\n"); return
    if globals().get("autocal") is not None and autocal.busy():
        log("Auto-Kalibrierung läuft – erst deren STOPP / Ende abwarten\n"); return   # v3.9
    if proc and proc.poll() is None:
        return
    global logfile
    import datetime, glob
    old = sorted(glob.glob(os.path.join(LOGDIR, "session_*.log")))
    for f in old[:max(0, len(old) - 99)]: os.remove(f)   # keep only the last 100
    logfile = open(os.path.join(LOGDIR, datetime.datetime.now().strftime("session_%Y-%m-%d_%H-%M-%S.log")), "w")
    g = "1" if guard.get() else "0"
    log(f"\n=== Start (Greifer: {stufe.get()}, Kollisionsschutz: {'AN' if g == '1' else 'AUS'}) ===\n")
    env = dict(os.environ, COLLISION_GUARD=g)
    proc = subprocess.Popen(["/bin/bash", SCRIPT, stufe.get()], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, start_new_session=True, env=env)
    threading.Thread(target=reader, args=(proc,), daemon=True).start()
    status.config(text="● LÄUFT", fg="#1a9c3a"); bstart.config(state="disabled"); bstop.config(state="normal"); set_radios("disabled")

stopping = {"on": False, "t0": 0.0}
STOP_TERM_S = 25.0   # v3.7: SIGINT (clean stop + rest move) -> after 25 s SIGTERM -> after 32 s SIGKILL
STOP_KILL_S = 32.0
def stop():
    if proc and proc.poll() is None and not stopping["on"]:
        import time as _t
        stopping["on"] = True; stopping["t0"] = _t.monotonic()
        # v3.7: immediate visible feedback first, then signal
        bstop.config(state="disabled", text="◐ stoppt…")
        status.config(text="◐ STOPPT… fährt in Ruheposition", fg="#d68910")
        try: root.update_idletasks()
        except Exception: pass
        log("\n=== Stopp angefordert – Arm fährt in Ruheposition ===\n")
        try:
            if rec.get("state") in ("recording", "session"):
                rec_cmd("pause")   # v3.3: rest move is not recorded (v3.8: session take pauses too)
        except Exception: pass
        _signal(proc, signal.SIGINT)          # einmal Ctrl+C: sauberes Beenden
        root.after(1000, stop_watch, proc)

def _signal(p, sig):
    try: os.killpg(p.pid, sig)
    except ProcessLookupError: pass
    except Exception:
        try: p.send_signal(sig)
        except Exception: pass

def stop_watch(p):
    """v3.7: escalate SIGINT -> SIGTERM -> SIGKILL and show the elapsed time while the arm drives to rest."""
    import time as _t
    if p.poll() is not None or not stopping["on"]:
        return
    el = _t.monotonic() - stopping["t0"]
    if el >= STOP_KILL_S and not stopping.get("killed"):
        stopping["killed"] = True
        log("\n=== Erzwinge Beenden (SIGKILL) ===\n"); _signal(p, signal.SIGKILL)
    elif el >= STOP_TERM_S and not stopping.get("termed"):
        stopping["termed"] = True
        log("\n=== Teleop reagiert nicht – sende SIGTERM ===\n"); _signal(p, signal.SIGTERM)
    status.config(text=f"◐ STOPPT… fährt in Ruheposition ({el:.0f} s)", fg="#d68910")
    root.after(1000, stop_watch, p)

def force_kill(p):
    if p.poll() is None:
        log("\n=== Erzwinge Beenden ===\n"); _signal(p, signal.SIGKILL)

def finished(code):
    stopping["on"] = False; stopping.pop("termed", None); stopping.pop("killed", None)
    bstop.config(text="■  STOPP")
    log(f"\n=== Beendet (Code {code}) – schalte alle Motoren aus ===\n")
    status.config(text="◐ MOTOREN AUS…", fg="#d68910"); bstop.config(state="disabled")
    def _off():
        try:
            r = subprocess.run([os.path.expanduser("~/humanoid/.venv/bin/python"),
                                os.path.expanduser("~/humanoid/disable_motors.py")],
                               capture_output=True, text=True, timeout=90)   # v3.7: 20 -> 90 s (laptop import ~35-55 s)
            msg = (r.stdout + r.stderr).strip().splitlines()[-1:] or ["?"]
        except Exception as e:
            msg = [f"Motoren-Aus fehlgeschlagen: {e}"]
        q.put(("off", msg[0]))
    threading.Thread(target=_off, daemon=True).start()

def finished_off(msg):
    ending["on"] = False
    log(f"{msg}\n=== Fertig ===\n")
    status.config(text="■ GESTOPPT", fg="#c0392b"); bstart.config(state="normal"); bstop.config(state="disabled"); set_radios("normal")
    if logfile: logfile.close()
    root.after(1000, check_update, False)   # load a pending update right after STOP

def teleop_running():
    if globals().get("play") and play_running():
        return True   # v3.4: no app reload while a policy runs
    if globals().get("autocal") is not None and autocal.busy():
        return True   # v3.9: no app reload while the auto calibration plans/runs
    if globals().get("rec", {}).get("state") in ("recording", "paused", "saving", "session"):
        return True   # v3.3: an open episode blocks the app reload
    if globals().get("sess") is not None and sess.busy():
        return True   # v3.8: session active or takes still being saved
    return (proc is not None and proc.poll() is None) or str(bstart["state"]) == "disabled" or stopping["on"] or ending["on"]

def restart_app(reason):
    """Restart the GUI (same Python interpreter, same script). Never while teleop runs."""
    if teleop_running():
        log("App neu laden geht nur im Stillstand (erst STOPP).\n"); return False
    try:
        with open(APP_FILE) as f:
            compile(f.read(), APP_FILE, "exec")   # check only, write nothing
    except Exception as e:
        upd["bad"] = file_sig()
        log(f"Update NICHT geladen – teleop_gui.py hat einen Fehler: {e}\n")
        upd_label.config(text="Update fehlerhaft – nicht geladen", fg="#c0392b")
        return False
    log(f"App wird neu geladen ({reason}) …\n")
    os.environ["TELEOP_GUI_UPDATED_FROM"] = APP_VERSION
    try:
        root.update_idletasks()
        os.execv(sys.executable, [sys.executable, APP_FILE] + sys.argv[1:])
    except Exception as e:
        log(f"Neu laden fehlgeschlagen: {e}\n"); return False

def check_update(reschedule=True):
    try:
        now_sig = file_sig()
        d = update_decision(now_sig, START_SIG, upd["last"], teleop_running(), upd["bad"])
        upd["last"] = now_sig
        if d == "pending":
            upd_label.config(text="Update verfügbar – wird nach STOPP geladen", fg="#d68910")
        elif d == "restart":
            restart_app("automatisch, Datei geändert")
    finally:
        if reschedule:
            root.after(UPDATE_CHECK_MS, check_update)

def on_close():
    if globals().get("play") and play_running():
        play_stop(); root.after(2500, on_close); return   # v3.4
    if globals().get("autocal") is not None and autocal.running():
        autocal.stop(); root.after(2500, on_close); return   # v3.9
    if proc and proc.poll() is None:
        stop(); log("Fenster schließt, sobald der Arm steht …\n"); root.after(500, close_when_done)
    else:
        root.destroy()
def close_when_done():
    if (proc and proc.poll() is None) or str(bstart["state"]) == "disabled":
        root.after(500, close_when_done)
    else:
        root.destroy()

root = tk.Tk(); root.title(f"Robot Teleop – Version {APP_VERSION}")
root.geometry(f"820x{min(1320, max(700, root.winfo_screenheight() - 70))}+0+0")   # v3.8: fit the screen (1080p: 1010)
# v3.8: everything lives in a scrollable body, so nothing is cut off on a 1080p screen (window may be shorter
# than the content). Mouse wheel scrolls the body (text fields keep their own scrolling).
_outer = tk.Frame(root); _outer.pack(fill="both", expand=True)
_cv = tk.Canvas(_outer, highlightthickness=0)
_sb = tk.Scrollbar(_outer, orient="vertical", command=_cv.yview)
_cv.configure(yscrollcommand=_sb.set); _sb.pack(side="right", fill="y"); _cv.pack(side="left", fill="both", expand=True)
body = tk.Frame(_cv); _bw = _cv.create_window(0, 0, window=body, anchor="nw")
body.bind("<Configure>", lambda e: _cv.configure(scrollregion=_cv.bbox("all")))
_cv.bind("<Configure>", lambda e: _cv.itemconfigure(_bw, width=e.width))
def _wheel(e):
    if isinstance(e.widget, tk.Text):
        return
    d = e.delta if abs(e.delta) < 10 else e.delta / 120
    _cv.yview_scroll(int(-d) or (-1 if d > 0 else 1), "units")
root.bind_all("<MouseWheel>", _wheel, add="+")
top = tk.Frame(body); top.pack(fill="x", padx=12, pady=12)
bstart = tk.Button(top, text="▶  START", font=("Helvetica", 20, "bold"), width=9, height=2, command=start)
bstop = tk.Button(top, text="■  STOPP", font=("Helvetica", 20, "bold"), width=9, height=2, command=stop, state="disabled")
bstart.pack(side="left", padx=6); bstop.pack(side="left", padx=6)
status = tk.Label(top, text="■ GESTOPPT", font=("Helvetica", 16, "bold"), fg="#c0392b"); status.pack(side="left", padx=12)
stufe = tk.StringVar(value="vorsichtig")
rf = tk.LabelFrame(body, text="Greifer-Kraft (vor dem Start wählen)"); rf.pack(fill="x", padx=12)
radios = []
for label, val in [("Vorsichtig (Pappe, Empfindliches)", "vorsichtig"), ("Mittel", "mittel"), ("Kräftig (Werkseinstellung)", "kraeftig")]:
    r = tk.Radiobutton(rf, text=label, variable=stufe, value=val); r.pack(side="left", padx=6); radios.append(r)
guard = tk.BooleanVar(value=True)
gf = tk.LabelFrame(body, text="Sicherheit (vor dem Start wählen)"); gf.pack(fill="x", padx=12, pady=(6, 0))
guard_cb = tk.Checkbutton(gf, text="Kollisionsschutz an (Arm stoppt, wenn er gegen etwas stößt)", variable=guard)
guard_cb.pack(side="left", padx=6)
def set_radios(state):
    for r in radios: r.config(state=state)
    guard_cb.config(state=state)
    breload.config(state=state)
vf = tk.Frame(body); vf.pack(fill="x", padx=12, pady=(4, 0))
tk.Label(vf, text=f"Version {APP_VERSION}", fg="gray").pack(side="left")
breload = tk.Button(vf, text="⟳ App neu laden", command=lambda: restart_app("Knopf"))
breload.pack(side="right")
upd_label = tk.Label(vf, text="", fg="#d68910"); upd_label.pack(side="right", padx=8)
tk.Label(body, text="Vor Start: beide Arme in Ruheposition. Notfall: Steckdosenleiste aus.", fg="gray").pack()
# --- live camera feeds (v3.1.1) ------------------------------------------------------
# Two preview panels (camera_feeds.py, config ~/humanoid/cameras.json). Reader threads live only in this GUI
# process; teleop itself runs in its own process (teleop.sh), so the previews cannot block the control loop.
# Any error here only disables the previews, never the START/STOP part of the app.
CAM_BOX = (360, 220)
cam_updates = []
try:
    import camera_feeds
    cam_feeds, cam_cfg = camera_feeds.make_feeds(CAM_BOX, shared=False)   # v3.9: GUI owns the cameras (also after a reload)
    camf = tk.Frame(body); camf.pack(fill="x", padx=8, pady=(4, 0))
    for _i, _f in enumerate(cam_feeds):
        _fr, _upd = camera_feeds.make_panel(camf, _f, CAM_BOX)
        _fr.grid(row=0, column=_i, padx=4, sticky="n")
        cam_updates.append(_upd)
        if getattr(_f, "role", "") and _f.configured:
            try:   # v3.8: share frames with recorder.py / play_policy (cameras open only once, here)
                _f.publisher = camera_feeds.FramePublisher(_f.role)
            except Exception as _pe:
                print(f"Kamera-Teilen aus ({_f.name}): {_pe}", file=sys.stderr)
        _f.start()
    if any(getattr(_f, "publisher", None) for _f in cam_feeds):
        os.environ["HUMANOID_CAM_SHM"] = "1"   # children (recorder, play_policy) read the shared frames
    CAM_MS = max(30, int(1000 / max(1, cam_cfg.get("display_fps", 15))))
    _cam_last = [0.0]
    def cam_tick():
        # v3.9.6: while the auto calibration runs, the previews refresh at ~2 fps (less CPU on the 8 GB laptop; the
        # cameras keep publishing full-rate frames for handeye.py through the shared memory)
        import time as _time
        try:
            _ac = globals().get("autocal"); _slow = bool(_ac is not None and _ac.busy())
        except Exception:
            _slow = False
        if not _slow or _time.time() - _cam_last[0] >= 0.5:
            _cam_last[0] = _time.time()
            for _u in cam_updates:
                try: _u()
                except Exception: pass
        root.after(CAM_MS, cam_tick)
    root.after(300, cam_tick)
except Exception as _e:
    cam_err = f"Kameras deaktiviert: {_e}"
    tk.Label(body, text=cam_err, fg="#c0392b").pack()
# --- dataset recording (v3.3) -----------------------------------------------------------
# recorder.py runs as its own process (LeRobot dataset writer, camera decoding, video encoding), fed with joint
# data by the telemetry tap in lerobot_teleoperate_tap.py. The GUI only sends commands (stdin) and shows the
# JSON status lines (stdout). The teleop process is never blocked by recording.
REC_PY = os.path.join(os.path.dirname(APP_FILE), "recorder.py")
rec = {"proc": None, "state": "idle", "episodes": 0, "err": None}
rq = queue.Queue()
def rec_reader(p):
    for line in iter(p.stdout.readline, ""):
        rq.put(line)
def rec_ensure():
    if rec["proc"] is None or rec["proc"].poll() is not None:
        try:
            errlog = open(os.path.join(LOGDIR, "recorder.log"), "a")
            rec["proc"] = subprocess.Popen([sys.executable, REC_PY], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=errlog, text=True, bufsize=1, cwd=os.path.dirname(APP_FILE))
            threading.Thread(target=rec_reader, args=(rec["proc"],), daemon=True).start()
        except Exception as e:
            rec["err"] = str(e); rec_msg.config(text=f"Recorder startet nicht: {e}", fg="#c0392b")
def rec_cmd(c):
    rec_ensure()
    try:
        rec["proc"].stdin.write(c + "\n"); rec["proc"].stdin.flush()
    except Exception as e:
        rec_msg.config(text=f"Recorder-Fehler: {e}", fg="#c0392b")
def rec_start():
    rec_cmd("start " + json.dumps({"dataset": ds_name.get().strip(), "task": task_text.get().strip()}))
def rec_buttons():
    st = rec["state"]
    if st == "session" or (sess is not None and sess.busy()):   # v3.8: classic buttons off during a session
        for _b in (brec, bsave, bdisc):
            _b.config(state="disabled")
        ds_entry.config(state="disabled"); return
    brec.config(state="normal" if st in ("idle", "paused") else "disabled",
                text="●  Episode weiter" if st == "paused" else "●  Episode starten")
    bsave.config(state="normal" if st in ("recording", "paused") else "disabled")
    bdisc.config(state="normal" if st in ("recording", "paused") else "disabled")
    ds_entry.config(state="normal" if st == "idle" else "disabled")
def rec_poll():
    while True:
        try: line = rq.get_nowait()
        except queue.Empty: break
        try:
            m = json.loads(line)
        except Exception:
            continue
        t = m.get("type")
        if t == "status":
            rec["state"] = m.get("state", "idle"); rec["episodes"] = m.get("episodes", 0)
            sec = m.get("seconds", 0.0)
            col = {"recording": "#c0392b", "paused": "#d68910", "saving": "#d68910", "session": "#c0392b"}.get(rec["state"], "gray")
            ico = {"recording": "● REC", "paused": "❚❚ PAUSE", "saving": "⏳ SPEICHERT", "session": "● SESSION"}.get(rec["state"], "")
            cams = "".join("✓" if c else "✗" for c in m.get("cams", []))
            rec_info.config(text=f"{ico}  Episoden: {rec['episodes']}   Zeit: {int(sec // 60):02d}:{sec % 60:04.1f}   "
                                 f"Gelenke {'✓' if m.get('joints') else '✗'}  Kameras {cams}", fg=col)
            if m.get("msg"): rec_msg.config(text=m["msg"], fg="#555")
            if sess is not None:
                try: sess.on_status(m)
                except Exception: pass
            rec_buttons()
        elif t == "saved":
            log(f"Datensatz {m.get('dataset')}: Episode {m.get('episodes')} gespeichert ({m.get('frames')} Bilder, "
                f"Label {m.get('label', 'play')}) -> {m.get('path')}\n")
        elif t == "error":
            rec_msg.config(text=m.get("msg", "Fehler"), fg="#c0392b"); log(m.get("msg", "") + "\n")
    if rec["proc"] is not None and rec["proc"].poll() is not None and rec["state"] != "dead":
        rec["state"] = "dead"; rec_msg.config(text="Recorder beendet – siehe logs/recorder.log (App neu laden)", fg="#c0392b")
        brec.config(state="normal"); rec["proc"] = None; rec["state"] = "idle"
    root.after(200, rec_poll)
rf2 = tk.LabelFrame(body, text="Datensatz aufnehmen (LeRobot, lokal)"); rf2.pack(fill="x", padx=12, pady=(4, 0))
r1 = tk.Frame(rf2); r1.pack(fill="x")
tk.Label(r1, text="Datensatz:").pack(side="left")
ds_name = tk.StringVar(value=os.environ.get("REC_DATASET", "pick_place_v1"))
ds_entry = tk.Entry(r1, textvariable=ds_name, width=16); ds_entry.pack(side="left", padx=(2, 8))
tk.Label(r1, text="Aufgabe:").pack(side="left")
task_text = tk.StringVar(value="Pick up the object and place it in the box")
tk.Entry(r1, textvariable=task_text, width=44).pack(side="left", padx=2, fill="x", expand=True)
r2 = tk.Frame(rf2); r2.pack(fill="x", pady=(2, 0))
brec = tk.Button(r2, text="●  Episode starten", fg="#c0392b", command=rec_start)
bsave = tk.Button(r2, text="✔  Episode speichern", command=lambda: rec_cmd("save"), state="disabled")
bdisc = tk.Button(r2, text="✖  Verwerfen", command=lambda: rec_cmd("discard"), state="disabled")
brec.pack(side="left", padx=2); bsave.pack(side="left", padx=2); bdisc.pack(side="left", padx=2)
rec_info = tk.Label(r2, text="Episoden: –", fg="gray", font=("Menlo", 11)); rec_info.pack(side="left", padx=8)
rec_msg = tk.Label(rf2, text="Erst ▶ START (Teleop), dann ● Episode starten oder ⏺ Session aufnehmen.", fg="#555", anchor="w")
rec_msg.pack(fill="x")
# --- v3.8: session recording with live markers + vision tools (gui_v38.py) ---------------------------------------
sess = vis = None
try:
    import gui_v38
    sess, vis = gui_v38.build(root, rf2, parent=body, ctx={
        "rec_cmd": rec_cmd, "rec": rec, "ds_name": ds_name, "task_text": task_text, "log": log,
        "feeds": globals().get("cam_feeds") or [],
        "teleop_active": lambda: proc is not None and proc.poll() is None and not stopping["on"]})
except Exception as _e:
    tk.Label(body, text=f"3.8-Erweiterungen deaktiviert: {_e}", fg="#c0392b").pack()
# --- v3.9: automatic hand-eye calibration (gui_v39.py) -------------------------------------------------------------
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
# --- policy playback (v3.4) -----------------------------------------------------------
# "▶ Policy abspielen" runs play_policy.sh (own process group): the newest ACT checkpoint drives the follower
# (no leader arm), collision guard on, max 2°/cycle, time limit. "■ STOPP" = SIGTERM -> play_policy.py cuts the
# torque at once; if the process is not gone after 1.5 s it is killed and disable_motors.py runs as a backup.
PLAY_SH = os.path.join(os.path.dirname(APP_FILE), "play_policy.sh")
PLAY_EXTRA = os.environ.get("PLAY_GUI_EXTRA_ARGS", "").split()   # test hook only, e.g. "--dry-run --fake-cams"
play = {"proc": None, "rc": None, "stopping": False}
pq = queue.Queue()
def play_running():
    return play["proc"] is not None and play["proc"].poll() is None
def play_reader(p):
    for line in iter(p.stdout.readline, ""):
        pq.put(line)
def play_ckpt_text():
    if globals().get("pol_var") is not None and pol_var.get() == "SmolVLA":
        return "Modell: lerobot/smolvla_base (zero-shot, nie auf diesem Arm trainiert; Normierung aus pick_place_all7)"
    try:
        import play_policy
        c = play_policy.find_checkpoint()
        return f"Checkpoint: Schritt {play_policy._step_of(c)} ({os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(c))))})" if c else "Noch kein Checkpoint (Training läuft?)"
    except Exception as e:
        return f"Checkpoint-Suche fehlgeschlagen: {e}"
def play_refresh():
    if not play_running():
        play_ck.config(text=play_ckpt_text())
    root.after(30000, play_refresh)
def play_start(confirm=True):
    if play_running():
        return
    if globals().get("autocal") is not None and autocal.busy():
        play_msg.config(text="Auto-Kalibrierung läuft – erst deren Ende abwarten", fg="#c0392b"); return   # v3.9
    if (proc is not None and proc.poll() is None) or str(bstart["state"]) == "disabled":
        play_msg.config(text="Erst Teleop STOPP – Policy und Teleop können nicht gleichzeitig laufen", fg="#c0392b"); return
    try:
        tr = subprocess.run(["pgrep", "-f", "lerobot-train"], capture_output=True, text=True).stdout.strip()
    except Exception:
        tr = ""
    _smol = pol_var.get() == "SmolVLA"
    _dry = "--dry-run" in PLAY_EXTRA or (_smol and play.get("mode") == "test")
    if tr and not _dry:
        play_msg.config(text="Training läuft noch – Policy erst danach abspielen", fg="#c0392b"); return
    _task = (smol_text() or SMOL_DEFAULT_TASK) if _smol else ""
    if _smol and not _dry and _task not in play["smol_dry_ok"]:
        play_msg.config(text="SmolVLA: erst 🧪 Testen mit genau diesem Befehl", fg="#c0392b"); return
    if confirm and not _dry:
        from tkinter import messagebox
        if not messagebox.askokcancel("Policy abspielen",
                (f"SmolVLA-Befehl: „{_task}“\n\n" if _smol else "") +
                "Der Arm bewegt sich jetzt selbständig (gelernte Bewegung).\n\n"
                "• Arbeitsbereich frei, Objekt an der Startstelle\n• Hand an der Steckdosenleiste (Not-Aus)\n"
                "• ■ STOPP schaltet die Motoren SOFORT aus – der Arm sackt dann ab\n\nStarten?"):
            return
    try:
        secs = max(5, min(300, int(float(play_secs.get()))))
    except Exception:
        secs = 60
    log(f"\n=== Policy abspielen: {'SmolVLA «' + _task + '»' if _smol else 'ACT'} ({secs} s, {'TROCKENLAUF ohne Motoren' if _dry else 'Kollisionsschutz AN'}) ===\n")
    play["last"] = (_smol, _dry, _task)
    play["stopping"] = False; play["rc"] = None
    play["proc"] = subprocess.Popen(["/bin/bash", PLAY_SH, "--seconds", str(secs)] + play_policy_args() + PLAY_EXTRA,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                    start_new_session=True, cwd=os.path.dirname(APP_FILE))
    threading.Thread(target=play_reader, args=(play["proc"],), daemon=True).start()
    bplay.config(state="disabled"); bpstop.config(state="normal")
    play_msg.config(text="◐ Policy startet (Modell laden, Kameras, Arm verbinden) …", fg="#d68910")
def play_stop():
    if not play_running() or play["stopping"]:
        return
    play["stopping"] = True
    log("\n=== Policy STOPP – Motoren sofort aus ===\n")
    play_msg.config(text="■ STOPP – Motoren aus …", fg="#c0392b")
    try:
        os.killpg(play["proc"].pid, signal.SIGTERM)
    except Exception:
        pass
    root.after(1500, play_force, play["proc"])
def play_force(p):
    if p.poll() is None:
        log("Policy reagiert nicht – erzwinge Beenden und schalte Motoren aus\n")
        try: os.killpg(p.pid, signal.SIGKILL)
        except Exception: pass
        def _off():
            try:
                r = subprocess.run([sys.executable, os.path.join(os.path.dirname(APP_FILE), "disable_motors.py")],
                                   capture_output=True, text=True, timeout=20)
                pq.put(((r.stdout + r.stderr).strip().splitlines()[-1:] or ["?"])[0] + "\n")
            except Exception as e:
                pq.put(f"Motoren-Aus fehlgeschlagen: {e}\n")
        threading.Thread(target=_off, daemon=True).start()
def play_poll():
    while True:
        try: line = pq.get_nowait()
        except queue.Empty: break
        log(line)
        if "LÄUFT:" in line: play_msg.config(text=("🧪 TEST LÄUFT – keine Motoren" if play.get("mode") == "test" else "● POLICY LÄUFT – ■ STOPP = Motoren sofort aus"), fg="#1a9c3a")
        elif "VERWEIGERT" in line or "Kein Checkpoint" in line or "nicht erreichbar" in line or "kein Bild" in line:
            play_msg.config(text=line.split("] ", 1)[-1].strip()[:110], fg="#c0392b")
        elif "Ruheposition" in line and "fährt" in line: play_msg.config(text="◐ Zeit um – Arm fährt in Ruheposition …", fg="#d68910")
    p = play["proc"]
    if p is not None and p.poll() is not None and play["rc"] is None:
        play["rc"] = p.returncode
        if p.returncode == 0 and play.get("last") and play["last"][0] and play["last"][1]:
            play["smol_dry_ok"].add(play["last"][2])
            pq.put("SmolVLA-Test ok – jetzt ▶ Ausführen (Motoren an) möglich\n")
        log(f"=== Policy beendet (Code {p.returncode}) ===\n")
        if p.returncode in (0, -15, -9) and "VERWEIGERT" not in str(play_msg.cget("text")):
            play_msg.config(text=f"■ Policy beendet (Code {p.returncode}) – Motoren aus", fg="#555")
        elif "VERWEIGERT" not in str(play_msg.cget("text")) and p.returncode != 0:
            play_msg.config(text=f"⚠ Policy-Fehler (Code {p.returncode}) – siehe Log unten / logs/play_*.log", fg="#c0392b")
        bplay.config(state="normal"); bpstop.config(state="disabled"); play["proc"] = None; play["mode"] = None
        smol_ui_refresh()
        play_ck.config(text=play_ckpt_text())
    root.after(200, play_poll)
pf = tk.LabelFrame(body, text="Gelernte Bewegung abspielen (Policy, ohne Leader-Arm)"); pf.pack(fill="x", padx=12, pady=(4, 0))
pr1 = tk.Frame(pf); pr1.pack(fill="x")
bplay = tk.Button(pr1, text="▶  Policy abspielen", font=("Helvetica", 15, "bold"), fg="#1a7a32", command=play_start)
bpstop = tk.Button(pr1, text="■  STOPP (Motoren sofort aus)", font=("Helvetica", 15, "bold"), fg="#c0392b",
                   command=play_stop, state="disabled")
bplay.pack(side="left", padx=2); bpstop.pack(side="left", padx=2)
tk.Label(pr1, text="Dauer (s):").pack(side="left", padx=(10, 2))
play_secs = tk.StringVar(value="60")
tk.Spinbox(pr1, from_=5, to=300, increment=5, textvariable=play_secs, width=4).pack(side="left")
# --- smolvla command bar (v3.6) ------------------------------------------------------------------------
# Policy selector (ACT / SmolVLA). ACT: the plain "▶ Policy abspielen" button as in v3.4.
# SmolVLA: a command bar: large entry + "🧪 Testen (ohne Motoren)" + "▶ Ausführen (Motoren an)",
# visible preset chips and a short history. Safety gate: "Ausführen" only after a successful
# dry run ("Testen") with exactly the same text in this session. Enter = Testen, or Ausführen
# once that text has been tested.
SMOL_DEFAULT_TASK = "pick up the box and put it in the bin"
SMOL_PLACEHOLDER = "Sag dem Roboter, was er tun soll …"
SMOL_PRESETS = [
    "pick up the box and put it in the bin",
    "pick up the silver box and put it in the bin",
    "open the gripper",
    "close the gripper",
    "move to the box",
    "go back to the rest position",
]
SMOL_HIST_FILE = os.path.join(os.path.dirname(APP_FILE), "logs", "smolvla_history.json")
SMOL_HIST_MAX = 8
pol_var = tk.StringVar(value="ACT")
smol_task = tk.StringVar(value=SMOL_DEFAULT_TASK)
play["smol_dry_ok"] = set(); play["last"] = None; play["mode"] = None
def smol_hist_load():
    try:
        h = json.load(open(SMOL_HIST_FILE))
        return [str(x) for x in h if str(x).strip()][:SMOL_HIST_MAX]
    except Exception:
        return []
def smol_hist_add(t):
    t = (t or "").strip()
    if not t or t == SMOL_PLACEHOLDER:
        return
    h = [t] + [x for x in smol_hist_load() if x != t]
    try:
        os.makedirs(os.path.dirname(SMOL_HIST_FILE), exist_ok=True)
        json.dump(h[:SMOL_HIST_MAX], open(SMOL_HIST_FILE, "w"), ensure_ascii=False, indent=0)
    except Exception:
        pass
    smol_menu_fill()
def smol_text():
    t = smol_task.get().strip()
    return "" if (t == SMOL_PLACEHOLDER and smol_entry.cget("fg") == "gray") else t
def smol_set(t):
    smol_entry.config(state="normal", fg="black")
    smol_task.set(t)
    smol_entry.icursor("end")
    smol_entry.focus_set()
    smol_ui_refresh()
pr2 = tk.Frame(pf); pr2.pack(fill="x", pady=(4, 0))
tk.Label(pr2, text="Policy:", font=("Helvetica", 13, "bold")).pack(side="left", padx=(2, 6))
def pol_changed(*_):
    if pol_var.get() == "SmolVLA":
        bplay.pack_forget()
        cbar.pack(fill="x", pady=(6, 2), before=play_ck)
        smol_entry.focus_set()
        play_msg.config(text="SmolVLA zero-shot (Spaß-Test): kennt unseren Arm nicht – Bewegungen können sinnlos sein. Erst Testen, dann Ausführen.", fg="#7d3c98")
    else:
        cbar.pack_forget()
        bplay.pack(side="left", padx=2, before=bpstop)
        play_msg.config(text="Nur bei gestopptem Teleop. Zeitlimit beendet mit Ruhefahrt; STOPP = sofort stromlos.", fg="#555")
    smol_ui_refresh()
    if not play_running():
        play_ck.config(text=play_ckpt_text())
for _lbl, _val in (("ACT (eigenes Training)", "ACT"), ("SmolVLA (Text-Befehl)", "SmolVLA")):
    tk.Radiobutton(pr2, text=_lbl, variable=pol_var, value=_val, command=pol_changed,
                   font=("Helvetica", 13)).pack(side="left", padx=(0, 8))

cbar = tk.LabelFrame(pf, text="Befehl an den Roboter (SmolVLA)", fg="#7d3c98",
                     font=("Helvetica", 13, "bold"))
tk.Label(cbar, text="Was soll der Roboter tun?  (Englisch funktioniert am besten)",
         fg="#555", anchor="w").pack(fill="x", padx=8, pady=(6, 0))
cb1 = tk.Frame(cbar); cb1.pack(fill="x", padx=8, pady=(2, 4))
smol_entry = tk.Entry(cb1, textvariable=smol_task, font=("Helvetica", 17), relief="solid", bd=1,
                      highlightthickness=2, highlightcolor="#7d3c98", highlightbackground="#bbb")
smol_entry.pack(fill="x", ipady=8)
def _ph_in(_e=None):
    if smol_entry.cget("fg") == "gray":
        smol_entry.config(fg="black"); smol_task.set("")
def _ph_out(_e=None):
    if not smol_task.get().strip():
        smol_entry.config(fg="gray"); smol_task.set(SMOL_PLACEHOLDER)
smol_entry.bind("<FocusIn>", _ph_in); smol_entry.bind("<FocusOut>", _ph_out)
cb2 = tk.Frame(cbar); cb2.pack(fill="x", padx=8, pady=(0, 2))
btest = tk.Button(cb2, text="🧪  Testen (ohne Motoren)", font=("Helvetica", 15, "bold"), fg="#1f618d",
                  padx=10, pady=4, command=lambda: smol_go("test"))
brun = tk.Button(cb2, text="▶  Ausführen (Motoren an)", font=("Helvetica", 15, "bold"), fg="#1a7a32",
                 padx=10, pady=4, command=lambda: smol_go("run"))
btest.pack(side="left", padx=(0, 8)); brun.pack(side="left", padx=(0, 8))
smol_state = tk.Label(cb2, text="", font=("Helvetica", 13, "bold"), anchor="w"); smol_state.pack(side="left", fill="x", expand=True)
cbp = tk.Frame(cbar); cbp.pack(fill="x", padx=8, pady=(4, 0))
tk.Label(cbp, text="Vorschläge:", fg="#555").pack(side="left", padx=(0, 4))
def _chip(parent, text):
    short = text if len(text) <= 44 else text[:42] + "…"
    tk.Button(parent, text=short, command=lambda t=text: smol_set(t), font=("Helvetica", 11),
              fg="#4a235a", pady=1, padx=4, relief="raised").pack(side="left", padx=2, pady=1)
for _t in SMOL_PRESETS[:2]:
    _chip(cbp, _t)
cbp2 = tk.Frame(cbar); cbp2.pack(fill="x", padx=8, pady=(0, 2))
tk.Label(cbp2, text="          ", fg="#555").pack(side="left", padx=(0, 4))  # indent under "Vorschläge"
for _t in SMOL_PRESETS[2:]:
    _chip(cbp2, _t)
cb3 = tk.Frame(cbar); cb3.pack(fill="x", padx=8, pady=(2, 4))
tk.Label(cb3, text="Verlauf:", fg="#555").pack(side="left")
smol_hist_btn = tk.Menubutton(cb3, text="letzte Befehle ▾", relief="raised")
smol_hist_menu = tk.Menu(smol_hist_btn, tearoff=0); smol_hist_btn.config(menu=smol_hist_menu)
smol_hist_btn.pack(side="left", padx=(4, 10))
def smol_menu_fill():
    smol_hist_menu.delete(0, "end")
    h = smol_hist_load()
    for t in h:
        smol_hist_menu.add_command(label=t, command=lambda t=t: smol_set(t))
    if not h:
        smol_hist_menu.add_command(label="(noch keine)", state="disabled")
    smol_hist_btn.config(text=("letzte Befehle ▾" if not h else f"letzte Befehle ({len(h)}) ▾"))
smol_menu_fill()
smol_live = tk.Label(cbar, text="", font=("Helvetica", 16, "bold"), fg="#1a9c3a", anchor="w",
                     wraplength=740, justify="left")
def smol_ui_refresh(*_):
    """Enable/disable the two buttons and set the state line from text, gate and running state."""
    if "smol_state" not in globals():
        return
    t = smol_text()
    if play_running():
        btest.config(state="disabled"); brun.config(state="disabled"); smol_entry.config(state="disabled")
        return
    smol_entry.config(state="normal")
    smol_live.pack_forget()
    if not t:
        btest.config(state="disabled"); brun.config(state="disabled")
        smol_state.config(text="Befehl eintippen oder Vorschlag wählen", fg="#555")
    elif t in play["smol_dry_ok"]:
        btest.config(state="normal"); brun.config(state="normal")
        smol_state.config(text="✓ Test OK – bereit zum Ausführen (Enter)", fg="#1a9c3a")
    else:
        btest.config(state="normal"); brun.config(state="disabled")
        smol_state.config(text="Erst Testen (ohne Motoren). Enter = Testen.", fg="#d68910")
smol_task.trace_add("write", smol_ui_refresh)
def smol_go(mode):
    t = smol_text()
    if not t or play_running():
        return
    if mode == "run" and t not in play["smol_dry_ok"]:
        smol_state.config(text="Erst Testen – Ausführen ist nach einem erfolgreichen Test freigegeben", fg="#c0392b")
        return
    play["mode"] = mode
    smol_hist_add(t)
    play_start()
    if play_running():
        smol_live.config(text=("🧪 Test läuft (ohne Motoren): " if mode == "test" else "● Läuft: ") + f"„{t}“",
                         fg="#1f618d" if mode == "test" else "#1a9c3a")
        smol_live.pack(fill="x", padx=8, pady=(0, 6))
    else:
        play["mode"] = None
    smol_ui_refresh()
smol_entry.bind("<Return>", lambda e: smol_go("run" if smol_text() in play["smol_dry_ok"] else "test"))
smol_entry.bind("<KP_Enter>", lambda e: smol_go("run" if smol_text() in play["smol_dry_ok"] else "test"))
def play_policy_args():
    a = []
    smol = pol_var.get() == "SmolVLA"
    if smol:
        a += ["--policy", "smolvla", "--task", smol_text() or SMOL_DEFAULT_TASK]
    if smol and play.get("mode") == "test" and "--dry-run" not in PLAY_EXTRA:
        a += ["--dry-run"]
    return a
play_ck = tk.Label(pf, text="", fg="gray", anchor="w"); play_ck.pack(fill="x")
play_msg = tk.Label(pf, text="Nur bei gestopptem Teleop. Zeitlimit beendet mit Ruhefahrt; STOPP = sofort stromlos.",
                    fg="#555", anchor="w"); play_msg.pack(fill="x")
root.bind_all("<Escape>", lambda e: play_stop())   # Esc = policy STOP as well
root.after(500, play_refresh)
root.after(600, play_poll)
root.after(700, smol_ui_refresh)
out = scrolledtext.ScrolledText(body, height=12, state="disabled", font=("Menlo", 10)); out.pack(fill="both", expand=True, padx=12, pady=8)
root.protocol("WM_DELETE_WINDOW", on_close)
root.after(200, poll)
root.after(UPDATE_CHECK_MS, check_update)
_prev = os.environ.pop("TELEOP_GUI_UPDATED_FROM", None)
if _prev is not None:
    log(f"Aktualisiert auf Version {APP_VERSION} (vorher {_prev})\n")
if not os.environ.get("TELEOP_GUI_TEST"):
    root.mainloop()
