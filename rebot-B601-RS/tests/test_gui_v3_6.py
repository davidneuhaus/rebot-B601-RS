#!/usr/bin/env python3
"""Offline tests for the v3.6 command-bar patch. No motors, no network.

Usage (from a scratch dir or anywhere):
  TELEOP_GUI_TEST=1 python test_gui_v3_6.py --v35 <teleop_gui_v3.5.py> --patch patch_gui_ui_v3_6.py
Exit 0 = all passed.
"""
from __future__ import annotations

import json, os, shutil, subprocess, sys, tempfile, traceback
from pathlib import Path

def fail(msg):
    print("FAIL:", msg)
    raise SystemExit(1)

def ok(msg):
    print("  ok:", msg)

def run(cmd, **kw):
    return subprocess.run(cmd, text=True, capture_output=True, **kw)


def test_patch(patch: Path, v35: Path, tmp: Path):
    out = tmp / "teleop_gui_v36.py"
    r = run([sys.executable, str(patch), str(v35), str(out)])
    if r.returncode != 0:
        fail(f"patch failed: {r.stdout}{r.stderr}")
    if "patched v3.6 command bar" not in r.stdout:
        fail(f"unexpected patch stdout: {r.stdout!r}")
    ok("patch applied to v3.5 source")

    r2 = run([sys.executable, "-m", "py_compile", str(out)])
    if r2.returncode != 0:
        fail(f"py_compile failed: {r2.stderr}")
    ok("patched GUI compiles")

    s = out.read_text()
    for needle in ("smolvla command bar (v3.6)", "Testen (ohne Motoren)",
                   "Ausführen (Motoren an)", "pick up the box and put it in the bin",
                   "TELEOP_GUI_TEST", 'root.geometry("800x1180")'):
        if needle not in s:
            fail(f"missing in patched GUI: {needle}")
    if "play_dry" in s or 'text="Trockenlauf' in s:
        fail("old Trockenlauf checkbox / play_dry still present")
    if "# --- smolvla zero-shot (v3.5)" in s:
        fail("v3.5 block was not replaced")
    ok("markers present, old checkbox gone")

    out2 = tmp / "teleop_gui_v36_again.py"
    r = run([sys.executable, str(patch), str(out), str(out2)])
    if r.returncode != 0 or "already patched" not in r.stdout:
        fail(f"idempotent apply failed: {r.stdout}{r.stderr}")
    if out2.read_text() != out.read_text():
        fail("second apply changed an already-patched file")
    ok("idempotent (already patched)")

    bad_in, bad_out = tmp / "bad.py", tmp / "bad_out.py"
    bad_in.write_text("print('no anchors')\n")
    r = run([sys.executable, str(patch), str(bad_in), str(bad_out)])
    if r.returncode != 1:
        fail(f"missing-anchor should exit 1, got {r.returncode}")
    if bad_out.exists():
        fail("missing-anchor wrote an output file")
    ok("missing anchors: exit 1, nothing written")
    return out


def _stub_play_script(path: Path):
    path.write_text(
        "#!/bin/bash\n"
        "echo \"[policy] args: $*\"\n"
        "echo \"[policy] LÄUFT: stub\"\n"
        "sleep 0.4\n"
        "exit 0\n"
    )
    path.chmod(0o755)


def test_ui(gui: Path, tmp: Path):
    home = tmp / "home"
    hum = home / "humanoid"
    hum.mkdir(parents=True)
    (hum / "VERSION").write_text("3.6\n")
    (hum / "logs").mkdir()
    shutil.copy(gui, hum / "teleop_gui.py")
    _stub_play_script(hum / "play_policy.sh")
    (hum / "disable_motors.py").write_text("print('motors off')\n")
    # stub recorder so rec_ensure, if it ever fires, does nothing harmful
    (hum / "recorder.py").write_text(
        "import sys\n"
        "for line in sys.stdin:\n"
        "    pass\n"
    )

    env = os.environ.copy()
    env["HOME"] = str(home)
    env["TELEOP_GUI_TEST"] = "1"
    env["PLAY_GUI_EXTRA_ARGS"] = ""
    # run the UI assertions in a child so Tk state stays isolated
    probe = tmp / "probe.py"
    probe.write_text(r'''
import json, os, sys, time, runpy
from pathlib import Path
from tkinter import messagebox

hum = Path(os.path.expanduser("~/humanoid"))
os.chdir(hum)
sys.argv = [str(hum / "teleop_gui.py")]
ns = runpy.run_path(str(hum / "teleop_gui.py"), run_name="__teleop_gui_test__")

root = ns["root"]
root.update()

def step():
    root.update_idletasks(); root.update()

def mapped(w):
    return bool(w.winfo_ismapped())

errors = []
def check(cond, msg):
    if not cond:
        errors.append(msg)
        print("  FAIL:", msg)
    else:
        print("  ok:", msg)

# --- default is ACT: command bar hidden, play button visible
check(ns["pol_var"].get() == "ACT", "default policy is ACT")
check(mapped(ns["bplay"]), "ACT: Policy-abspielen button visible")
check(not mapped(ns["cbar"]), "ACT: command bar hidden")
check(ns["play_policy_args"]() == [], "ACT: play_policy_args is empty")

# --- switch to SmolVLA
ns["pol_var"].set("SmolVLA")
ns["pol_changed"]()
step()
check(mapped(ns["cbar"]), "SmolVLA: command bar visible")
check(not mapped(ns["bplay"]), "SmolVLA: Policy-abspielen button hidden")
check(mapped(ns["btest"]) and mapped(ns["brun"]), "SmolVLA: Testen and Ausführen visible")
task = ns["SMOL_DEFAULT_TASK"]
check(ns["smol_text"]() == task, f"default command is {task!r}")
check(str(ns["btest"]["state"]) == "normal", "Testen enabled with default text")
check(str(ns["brun"]["state"]) == "disabled", "Ausführen disabled before Testen")
check("Erst Testen" in ns["smol_state"].cget("text"), "state line asks to Testen first")

args = ns["play_policy_args"]()
check(args == ["--policy", "smolvla", "--task", task],
      f"args without mode=test have no --dry-run: {args}")

# --- gate: Ausführen without Testen does not start a process
ns["smol_go"]("run")
step()
check(ns["play"]["proc"] is None, "Ausführen without Testen does not start play")
check("Erst Testen" in ns["smol_state"].cget("text"), "gate message shown")

# --- preset chip sets the field (use a non-default preset)
ns["smol_set"]("open the gripper")
step()
check(ns["smol_text"]() == "open the gripper", "preset fills the entry")
check(str(ns["brun"]["state"]) == "disabled", "Ausführen still disabled for a new text")

# --- Testen starts a dry-run stub
ns["smol_set"](task)
step()
ns["smol_go"]("test")
step()
proc = ns["play"]["proc"]
check(proc is not None, "Testen started play_policy.sh")
check(ns["play"].get("mode") == "test", "mode is test")
# wait for stub to exit and for play_poll to harvest it
t0 = time.time()
while time.time() - t0 < 5:
    step()
    if ns["play"]["proc"] is None and task in ns["play"]["smol_dry_ok"]:
        break
    time.sleep(0.05)
check(task in ns["play"]["smol_dry_ok"], "successful Testen unlocks this command")
step()
check(str(ns["brun"]["state"]) == "normal", "Ausführen enabled after Testen")
check("Test OK" in ns["smol_state"].cget("text"), "state line shows Test OK")

# history file
hist = json.loads((hum / "logs" / "smolvla_history.json").read_text())
check(task in hist, f"history stored the command: {hist}")

# changing the text locks Ausführen again; restoring the tested text unlocks it
ns["smol_set"]("close the gripper")
step()
check(str(ns["brun"]["state"]) == "disabled", "new text disables Ausführen")
ns["smol_set"](task)
step()
check(str(ns["brun"]["state"]) == "normal", "restoring tested text re-enables Ausführen")

# --- Ausführen: mock the confirm dialog, expect no --dry-run
calls = {"n": 0, "cmds": []}
real_popen = ns["subprocess"].Popen
class Rec:
    def __init__(self, cmd, **kw):
        cmd = list(cmd)
        if any("play_policy.sh" in str(x) for x in cmd):
            calls["n"] += 1
            calls["cmds"].append(cmd)
        self._p = real_popen(cmd, **kw)
    def __getattr__(self, n):
        return getattr(self._p, n)
ns["subprocess"].Popen = Rec
messagebox.askokcancel = lambda *a, **k: True
ns["smol_go"]("run")
step()
ns["subprocess"].Popen = real_popen
check(calls["n"] == 1, f"Ausführen started one play_policy.sh (got {calls['n']}: {calls['cmds']})")
cmd = calls["cmds"][0] if calls["cmds"] else []
check(cmd and "--dry-run" not in cmd, f"real run has no --dry-run: {cmd}")
check("--policy" in cmd and "smolvla" in cmd, f"real run passes --policy smolvla: {cmd}")
check(task in cmd, f"real run passes the task: {cmd}")
# stop the stub so we don't leave a process
if ns["play"]["proc"] is not None:
    ns["play_stop"]()
    t0 = time.time()
    while time.time() - t0 < 5:
        step()
        if ns["play"]["proc"] is None:
            break
        time.sleep(0.05)

# --- switch back to ACT
ns["pol_var"].set("ACT")
ns["pol_changed"]()
step()
check(mapped(ns["bplay"]) and not mapped(ns["cbar"]), "back to ACT: play button shown, bar hidden")
check(ns["play_policy_args"]() == [], "ACT args empty again")

# --- empty field disables both buttons
ns["pol_var"].set("SmolVLA"); ns["pol_changed"](); step()
ns["smol_task"].set("")
# simulate placeholder on focus-out
ns["smol_entry"].config(fg="gray")
ns["smol_task"].set(ns["SMOL_PLACEHOLDER"])
step()
check(ns["smol_text"]() == "", "placeholder is not a real command")
check(str(ns["btest"]["state"]) == "disabled" and str(ns["brun"]["state"]) == "disabled",
      "empty/placeholder disables both buttons")

try:
    root.destroy()
except Exception:
    pass

if errors:
    print(f"{len(errors)} UI checks failed")
    sys.exit(1)
print("UI checks passed")
''')
    r = run([sys.executable, str(probe)], env=env, cwd=str(hum), timeout=30)
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        fail(f"UI test process exited {r.returncode}")
    ok("headless Tk command-bar behaviour")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--v35", required=True, type=Path)
    ap.add_argument("--patch", required=True, type=Path)
    args = ap.parse_args()
    if not args.v35.is_file() or not args.patch.is_file():
        fail("need --v35 and --patch files")
    print("== v3.6 offline tests")
    with tempfile.TemporaryDirectory(prefix="gui36_") as td:
        tmp = Path(td)
        gui = test_patch(args.patch.resolve(), args.v35.resolve(), tmp)
        test_ui(gui, tmp)
    print("ALL PASSED")


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
