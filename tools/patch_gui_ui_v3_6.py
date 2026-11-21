#!/usr/bin/env python3
# Based on lerobot-robot-seeed-b601 1.0.5 by Seeed Technology Co., Ltd. (MIT); modified by David Neuhaus, 2026.
"""v3.6: command bar in teleop_gui.py (replaces the cramped v3.5 selector row + dry-run checkbox).

Usage: patch_gui_ui_v3_6.py <in> <out>
Idempotent. Exit 0 = patched (or already patched). Exit 1 = an anchor is missing (nothing written).

Adds a clear SmolVLA command bar: large text field, Testen (dry run) and Ausführen (motors on)
buttons, ACT/SmolVLA selector, visible preset chips, and command history. A real SmolVLA run is
only allowed after a successful Testen with exactly the same text in this app session. ACT keeps
the v3.4 "Policy abspielen" button. play_policy.py is unchanged.
"""
import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
if "# --- smolvla command bar (v3.6)" in s:
    open(dst, "w").write(s)
    print("already patched")
    sys.exit(0)

A = "# --- smolvla zero-shot (v3.5): policy selector + text command"
B = "play_ck = tk.Label(pf, text=\"\", fg=\"gray\", anchor=\"w\"); play_ck.pack(fill=\"x\")\n"
if A not in s or B not in s:
    print("v3.5 block not found – teleop_gui.py not changed")
    sys.exit(1)
i, j = s.index(A), s.index(B)

NEW = r'''# --- smolvla command bar (v3.6) ------------------------------------------------------------------------
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
'''
s = s[:i] + NEW + s[j:]

edits = [
    ('    _dry = "--dry-run" in PLAY_EXTRA or (globals().get("play_dry") is not None and play_dry.get())\n',
     '    _smol = pol_var.get() == "SmolVLA"\n'
     '    _dry = "--dry-run" in PLAY_EXTRA or (_smol and play.get("mode") == "test")\n'),
    ('    _smol = pol_var.get() == "SmolVLA"\n    _task = smol_task.get().strip() or SMOL_DEFAULT_TASK\n',
     '    _task = (smol_text() or SMOL_DEFAULT_TASK) if _smol else ""\n'),
    ('"SmolVLA: erst ein erfolgreicher Trockenlauf mit diesem Befehl (Haken setzen)"',
     '"SmolVLA: erst 🧪 Testen mit genau diesem Befehl"'),
    ('        if not messagebox.askokcancel("Policy abspielen",\n                "Der Arm bewegt sich jetzt selbständig (gelernte Bewegung).\\n\\n"',
     '        if not messagebox.askokcancel("Policy abspielen",\n                (f"SmolVLA-Befehl: „{_task}“\\n\\n" if _smol else "") +\n                "Der Arm bewegt sich jetzt selbständig (gelernte Bewegung).\\n\\n"'),
    ("            pq.put(\"SmolVLA-Trockenlauf ok – Haken 'Trockenlauf' entfernen, dann echt abspielen\\n\")\n",
     '            pq.put("SmolVLA-Test ok – jetzt ▶ Ausführen (Motoren an) möglich\\n")\n'),
    ('        bplay.config(state="normal"); bpstop.config(state="disabled"); play["proc"] = None\n',
     '        bplay.config(state="normal"); bpstop.config(state="disabled"); play["proc"] = None; play["mode"] = None\n'
     '        smol_ui_refresh()\n'),
    ('        if "LÄUFT:" in line: play_msg.config(text="● POLICY LÄUFT – ■ STOPP = Motoren sofort aus", fg="#1a9c3a")\n',
     '        if "LÄUFT:" in line: play_msg.config(text=("🧪 TEST LÄUFT – keine Motoren" if play.get("mode") == "test" else "● POLICY LÄUFT – ■ STOPP = Motoren sofort aus"), fg="#1a9c3a")\n'),
    ('root.geometry("780x1090")', 'root.geometry("800x1180")'),
    ('root.after(600, play_poll)\n',
     'root.after(600, play_poll)\nroot.after(700, smol_ui_refresh)\n'),
    ('root.mainloop()\n',
     'if not os.environ.get("TELEOP_GUI_TEST"):\n    root.mainloop()\n'),
]
miss = [a[:80] for a, b in edits if s.count(a) != 1]
if miss:
    print("ANCHOR MISSING:", miss)
    sys.exit(1)
for a, b in edits:
    s = s.replace(a, b, 1)
open(dst, "w").write(s)
print("patched v3.6 command bar")
