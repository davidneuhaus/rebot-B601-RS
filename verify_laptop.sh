#!/bin/bash
# Offline self-check for the laptop install. Sends NO motor commands:
# - compiles all scripts, imports the plugins, runs the offline guard/GUI tests,
# - starts the GUI for a few seconds (window appears, START is not pressed),
# - runs preflight.py ONLY while the PCAN library (PCBUSB) is missing, so it stops before any CAN traffic.
cd "$HOME/humanoid" || exit 2
source .venv/bin/activate || exit 2
PASS=0; FAIL=0
ok(){ echo "  OK   $*"; PASS=$((PASS+1)); }
bad(){ echo "  FAIL $*"; FAIL=$((FAIL+1)); }
echo "== Python: $(command -v python) ($(python -V 2>&1)), prefix $(python -c 'import sys;print(sys.prefix)')"
echo "== compile"
for f in *.py tools/*.py tools/iphone_web_cam/*.py rebot-B601-RS/tests/*.py; do python -m py_compile "$f" 2>/dev/null && : || bad "py_compile $f"; done; ok "py_compile all"
for f in *.sh tools/iphone_web_cam/*.sh tools/train/*.sh rebot-B601-RS/rollback.sh; do bash -n "$f" || bad "bash -n $f"; done; ok "bash -n all"
echo "== imports / plugin registration"
python - <<'PY' && ok "imports + robot/teleop types registered" || bad "imports"
import importlib, sys
for m in ("lerobot", "motorbridge", "can", "cv2", "PIL", "numpy", "torch", "tkinter",
          "lerobot_robot_seeed_b601", "lerobot_teleoperator_rebot_arm_102", "aiohttp"):
    x = importlib.import_module(m); print(f"     {m} {getattr(x, '__version__', '')}")
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
r = RobotConfig.get_known_choices(); t = TeleoperatorConfig.get_known_choices()
assert "seeed_b601_rs_follower" in r, sorted(r)
assert "rebot_arm_102_leader" in t, sorted(t)
import lerobot_robot_seeed_b601 as P, os
src = open(os.path.join(os.path.dirname(P.__file__), "seeed_b601_follower.py")).read()
assert "guard v3.2" in src, "plugin follower not patched (guard v3.2 missing)"
print("     plugin:", os.path.dirname(P.__file__))
PY
echo "== offline tests"
OUT=$(env -u COLLISION_GUARD python rebot-B601-RS/tests/test_collision_guard.py 2>&1); echo "$OUT" | tail -1
echo "$OUT" | tail -1 | grep -q " 0 Fehler" && ok "test_collision_guard" || bad "test_collision_guard"
OUT=$(python rebot-B601-RS/tests/test_gravity_v32.py 2>&1); echo "$OUT" | tail -1
echo "$OUT" | tail -1 | grep -qE "(^| )0 Fehler|FAIL: 0|alle OK|0 failed" && ok "test_gravity_v32" || { echo "$OUT" | grep -E "FEHLER|FAIL|Error" | head -5; bad "test_gravity_v32 (see above)"; }
OUT=$(TELEOP_GUI_TEST=1 python rebot-B601-RS/tests/test_gui_v3_6.py --v35 rebot-B601-RS/versions/before_gui_v3_6/teleop_gui.py --patch tools/patch_gui_ui_v3_6.py 2>&1); RC=$?
echo "$OUT" | tail -3; [ $RC -eq 0 ] && ok "test_gui_v3_6" || bad "test_gui_v3_6"
python play_policy.py --help >/dev/null 2>&1 && ok "play_policy.py --help" || bad "play_policy.py --help"
echo "== GUI start (8 s, no START)"
python - <<'PY' && ok "GUI started and stayed up 8 s" || bad "GUI start"
import subprocess, sys, time, os
p = subprocess.Popen([sys.executable, os.path.expanduser("~/humanoid/teleop_gui.py")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(8)
alive = p.poll() is None
if alive: p.terminate()
try: out = p.communicate(timeout=5)[0]
except Exception: p.kill(); out = p.communicate()[0]
print("     GUI output:", (out or "").strip()[-400:] or "(none)")
sys.exit(0 if alive else 1)
PY
echo "== leader USB"
L=$(ls /dev/tty.usbserial-* 2>/dev/null | head -1); [ -n "$L" ] && ok "leader port $L" || bad "no /dev/tty.usbserial-* (leader USB/12 V?)"
echo "== PCAN library (user space: ~/humanoid/lib)"
echo "     DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH"
python - <<'PY' && ok "PCBUSB loadable by python-can and motorbridge (load only, no bus opened)" || bad "PCBUSB not loadable"
import ctypes, os
ctypes.CDLL("libPCBUSB.dylib")                       # same lookup as python-can's pcan backend
from motorbridge import platform_hints as h
assert h.can_load_pcbusb(), "motorbridge cannot load PCBUSB"
print("     ~/.local/lib/libPCBUSB.dylib ->", os.path.realpath(os.path.expanduser("~/.local/lib/libPCBUSB.dylib")))
PY
echo "     preflight.py is NOT run here: with PCBUSB present it talks to the motors (configure + feedback)."
echo "     Run it yourself with the arm powered: ~/humanoid/.venv/bin/python ~/humanoid/preflight.py"
echo "== result: $PASS ok, $FAIL failed"
[ $FAIL -eq 0 ]
