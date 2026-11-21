#!/usr/bin/env python3
"""Add the SmolVLA zero-shot mode (v3.5) to the v3.4 policy playback panel of teleop_gui.py.
Usage: patch_gui_smolvla.py <in> <out>
Adds: policy selector (ACT / SmolVLA), a text entry for the SmolVLA command (only shown for SmolVLA), a
"Trockenlauf (ohne Motoren)" checkbox (auto-on for SmolVLA). A real SmolVLA run on the arm is only allowed after a
successful SmolVLA dry run with the same command in this app session. ACT behaviour is unchanged (selector default ACT).
Anchor-based and idempotent. Exit 0 = patched (or already patched), 1 = an anchor is missing (nothing written)."""
import sys

src, dst = sys.argv[1], sys.argv[2]
s = open(src).read()
MARK = "# --- smolvla zero-shot (v3.5)"
if MARK in s:
    open(dst, "w").write(s); print("already patched"); sys.exit(0)
if "# --- policy playback (v3.4)" not in s:
    print("v3.4 policy panel not found – teleop_gui.py not changed"); sys.exit(1)

edits = [
    # 1) selector row right after the duration spinbox
    ('tk.Spinbox(pr1, from_=5, to=300, increment=5, textvariable=play_secs, width=4).pack(side="left")\n',
     'tk.Spinbox(pr1, from_=5, to=300, increment=5, textvariable=play_secs, width=4).pack(side="left")\n' + r'''# --- smolvla zero-shot (v3.5): policy selector + text command -----------------------------------------
SMOL_DEFAULT_TASK = "pick up the box and put it in the bin"
pr2 = tk.Frame(pf); pr2.pack(fill="x", pady=(2, 0))
tk.Label(pr2, text="Policy:").pack(side="left", padx=(2, 2))
pol_var = tk.StringVar(value="ACT")
play_dry = tk.BooleanVar(value=False)
smol_task = tk.StringVar(value=SMOL_DEFAULT_TASK)
play["smol_dry_ok"] = set(); play["last"] = None
def pol_changed(*_):
    if pol_var.get() == "SmolVLA":
        smol_lbl.pack(side="left", padx=(10, 2)); smol_entry.pack(side="left", fill="x", expand=True, padx=(0, 4))
        play_dry.set(True)
        play_msg.config(text="SmolVLA zero-shot (Spaß-Test): kennt unseren Arm nicht. Erst Trockenlauf, dann echt.", fg="#7d3c98")
    else:
        smol_lbl.pack_forget(); smol_entry.pack_forget(); play_dry.set(False)
        play_msg.config(text="Nur bei gestopptem Teleop. Zeitlimit beendet mit Ruhefahrt; STOPP = sofort stromlos.", fg="#555")
    if not play_running():
        play_ck.config(text=play_ckpt_text())
for _lbl, _val in (("ACT (eigenes Training)", "ACT"), ("SmolVLA (Text-Befehl)", "SmolVLA")):
    tk.Radiobutton(pr2, text=_lbl, variable=pol_var, value=_val, command=pol_changed).pack(side="left")
tk.Checkbutton(pr2, text="Trockenlauf (ohne Motoren)", variable=play_dry).pack(side="left", padx=(8, 0))
smol_lbl = tk.Label(pr2, text="Befehl:")
smol_entry = tk.Entry(pr2, textvariable=smol_task, width=40)
def play_policy_args():
    a = []
    if pol_var.get() == "SmolVLA":
        a += ["--policy", "smolvla", "--task", smol_task.get().strip() or SMOL_DEFAULT_TASK]
    if play_dry.get() and "--dry-run" not in PLAY_EXTRA:
        a += ["--dry-run"]
    return a
'''),
    # 2) checkpoint label shows the base model when SmolVLA is selected
    ('def play_ckpt_text():\n',
     'def play_ckpt_text():\n    if globals().get("pol_var") is not None and pol_var.get() == "SmolVLA":\n'
     '        return "Modell: lerobot/smolvla_base (zero-shot, nie auf diesem Arm trainiert; Normierung aus pick_place_all7)"\n'),
    # 3) training check: a dry run may run during training (play_policy.py then uses the CPU)
    ('    if tr and "--dry-run" not in PLAY_EXTRA:\n',
     '    _dry = "--dry-run" in PLAY_EXTRA or (globals().get("play_dry") is not None and play_dry.get())\n'
     '    if tr and not _dry:\n'),
    # 4) SmolVLA gate + no confirm dialog for dry runs
    ('    if confirm:\n        from tkinter import messagebox\n',
     '    _smol = pol_var.get() == "SmolVLA"\n'
     '    _task = smol_task.get().strip() or SMOL_DEFAULT_TASK\n'
     '    if _smol and not _dry and _task not in play["smol_dry_ok"]:\n'
     '        play_msg.config(text="SmolVLA: erst ein erfolgreicher Trockenlauf mit diesem Befehl (Haken setzen)", fg="#c0392b"); return\n'
     '    if confirm and not _dry:\n        from tkinter import messagebox\n'),
    ('    log(f"\\n=== Policy abspielen ({secs} s, Kollisionsschutz AN) ===\\n")\n',
     '    log(f"\\n=== Policy abspielen: {\'SmolVLA «\' + _task + \'»\' if _smol else \'ACT\'} ({secs} s, '
     '{\'TROCKENLAUF ohne Motoren\' if _dry else \'Kollisionsschutz AN\'}) ===\\n")\n'
     '    play["last"] = (_smol, _dry, _task)\n'),
    # 5) command line
    ('["/bin/bash", PLAY_SH, "--seconds", str(secs)] + PLAY_EXTRA',
     '["/bin/bash", PLAY_SH, "--seconds", str(secs)] + play_policy_args() + PLAY_EXTRA'),
    # 6) remember a successful SmolVLA dry run
    ('        play["rc"] = p.returncode\n',
     '        play["rc"] = p.returncode\n'
     '        if p.returncode == 0 and play.get("last") and play["last"][0] and play["last"][1]:\n'
     '            play["smol_dry_ok"].add(play["last"][2])\n'
     '            pq.put("SmolVLA-Trockenlauf ok – Haken \'Trockenlauf\' entfernen, dann echt abspielen\\n")\n'),
]
missing = [a[:50] for a, b in edits if s.count(a) != 1]
if missing:
    print("ANCHOR MISSING/AMBIGUOUS – teleop_gui.py not changed:", missing); sys.exit(1)
for a, b in edits:
    s = s.replace(a, b, 1)
open(dst, "w").write(s)
print("patched: SmolVLA selector, text command, dry-run checkbox, SmolVLA dry-run gate")
