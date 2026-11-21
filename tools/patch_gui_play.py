#!/usr/bin/env python3
# Based on lerobot-robot-seeed-b601 1.0.5 by Seeed Technology Co., Ltd. (MIT); modified by David Neuhaus, 2026.
"""Add the policy playback panel (v3.4) to teleop_gui.py.  Usage: patch_gui_play.py <in> <out>
Anchor-based and idempotent: the main panel is inserted right before the log window ("out = scrolledtext..."),
plus small optional hooks (teleop START refused / no app reload / window close while a policy runs).
Exit 0 = patched (or already patched), 1 = main anchor missing (nothing written)."""
import re, sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
MARK = "# --- policy playback (v3.4)"
if MARK in s:
    open(dst, "w").write(s); print("already patched"); sys.exit(0)

PANEL = r'''# --- policy playback (v3.4) -----------------------------------------------------------
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
    if (proc is not None and proc.poll() is None) or str(bstart["state"]) == "disabled":
        play_msg.config(text="Erst Teleop STOPP – Policy und Teleop können nicht gleichzeitig laufen", fg="#c0392b"); return
    try:
        tr = subprocess.run(["pgrep", "-f", "lerobot-train"], capture_output=True, text=True).stdout.strip()
    except Exception:
        tr = ""
    if tr and "--dry-run" not in PLAY_EXTRA:
        play_msg.config(text="Training läuft noch – Policy erst danach abspielen", fg="#c0392b"); return
    if confirm:
        from tkinter import messagebox
        if not messagebox.askokcancel("Policy abspielen",
                "Der Arm bewegt sich jetzt selbständig (gelernte Bewegung).\n\n"
                "• Arbeitsbereich frei, Objekt an der Startstelle\n• Hand an der Steckdosenleiste (Not-Aus)\n"
                "• ■ STOPP schaltet die Motoren SOFORT aus – der Arm sackt dann ab\n\nStarten?"):
            return
    try:
        secs = max(5, min(300, int(float(play_secs.get()))))
    except Exception:
        secs = 60
    log(f"\n=== Policy abspielen ({secs} s, Kollisionsschutz AN) ===\n")
    play["stopping"] = False; play["rc"] = None
    play["proc"] = subprocess.Popen(["/bin/bash", PLAY_SH, "--seconds", str(secs)] + PLAY_EXTRA,
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
        if "LÄUFT:" in line: play_msg.config(text="● POLICY LÄUFT – ■ STOPP = Motoren sofort aus", fg="#1a9c3a")
        elif "VERWEIGERT" in line or "Kein Checkpoint" in line or "nicht erreichbar" in line or "kein Bild" in line:
            play_msg.config(text=line.split("] ", 1)[-1].strip()[:110], fg="#c0392b")
        elif "Ruheposition" in line and "fährt" in line: play_msg.config(text="◐ Zeit um – Arm fährt in Ruheposition …", fg="#d68910")
    p = play["proc"]
    if p is not None and p.poll() is not None and play["rc"] is None:
        play["rc"] = p.returncode
        log(f"=== Policy beendet (Code {p.returncode}) ===\n")
        if p.returncode in (0, -15, -9) and "VERWEIGERT" not in str(play_msg.cget("text")):
            play_msg.config(text=f"■ Policy beendet (Code {p.returncode}) – Motoren aus", fg="#555")
        elif "VERWEIGERT" not in str(play_msg.cget("text")) and p.returncode != 0:
            play_msg.config(text=f"⚠ Policy-Fehler (Code {p.returncode}) – siehe Log unten / logs/play_*.log", fg="#c0392b")
        bplay.config(state="normal"); bpstop.config(state="disabled"); play["proc"] = None
        play_ck.config(text=play_ckpt_text())
    root.after(200, play_poll)
pf = tk.LabelFrame(root, text="Gelernte Bewegung abspielen (Policy, ohne Leader-Arm)"); pf.pack(fill="x", padx=12, pady=(4, 0))
pr1 = tk.Frame(pf); pr1.pack(fill="x")
bplay = tk.Button(pr1, text="▶  Policy abspielen", font=("Helvetica", 15, "bold"), fg="#1a7a32", command=play_start)
bpstop = tk.Button(pr1, text="■  STOPP (Motoren sofort aus)", font=("Helvetica", 15, "bold"), fg="#c0392b",
                   command=play_stop, state="disabled")
bplay.pack(side="left", padx=2); bpstop.pack(side="left", padx=2)
tk.Label(pr1, text="Dauer (s):").pack(side="left", padx=(10, 2))
play_secs = tk.StringVar(value="60")
tk.Spinbox(pr1, from_=5, to=300, increment=5, textvariable=play_secs, width=4).pack(side="left")
play_ck = tk.Label(pf, text="", fg="gray", anchor="w"); play_ck.pack(fill="x")
play_msg = tk.Label(pf, text="Nur bei gestopptem Teleop. Zeitlimit beendet mit Ruhefahrt; STOPP = sofort stromlos.",
                    fg="#555", anchor="w"); play_msg.pack(fill="x")
root.bind_all("<Escape>", lambda e: play_stop())   # Esc = policy STOP as well
root.after(500, play_refresh)
root.after(600, play_poll)
'''

m = re.search(r"^out = scrolledtext\.ScrolledText\(root,", s, re.M)
if not m:
    print("MAIN ANCHOR MISSING – teleop_gui.py not changed"); sys.exit(1)
s = s[:m.start()] + PANEL + s[m.start():]
notes = []
g = re.search(r'root\.geometry\("(\d+)x(\d+)"\)', s)
if g:
    s = s[:g.start()] + f'root.geometry("{g.group(1)}x{int(g.group(2)) + 100}")' + s[g.end():]; notes.append("geometry")
hooks = [
    (r"def teleop_running\(\):\n", '    if globals().get("play") and play_running():\n        return True   # v3.4: no app reload while a policy runs\n', "reload-block"),
    (r"def start\(\):\n    global proc\n", '    if globals().get("play") and play_running():\n        log("Policy läuft – erst Policy-STOPP\\n"); return\n', "start-block"),
    (r"def on_close\(\):\n", '    if globals().get("play") and play_running():\n        play_stop(); root.after(2500, on_close); return   # v3.4\n', "close-stop"),
]
for pat, ins, name in hooks:
    h = re.search(pat, s)
    if h:
        s = s[:h.end()] + ins + s[h.end():]; notes.append(name)
    else:
        notes.append(name + " MISSING")
open(dst, "w").write(s)
print("patched: panel + " + ", ".join(notes))
