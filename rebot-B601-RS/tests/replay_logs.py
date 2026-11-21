"""Replay real session logs (clamp warnings) through CollisionGuard and count false alarms.
Usage: python replay_logs.py <follower.py> ~/humanoid/logs/session_*.log  (thresholds from env COLLISION_*)
Approximation: consecutive clamp warnings = consecutive cycles; only clamped joints are known; no torque."""
import re, sys, logging, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _load import load_follower
F, _ = load_follower(sys.argv[1])
files = sys.argv[2:]
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
class Cap(logging.Handler):
    def __init__(s): super().__init__(); s.n = []
    def emit(s, r): s.n.append(r.getMessage())
total = 0
for f in files:
    cap = Cap(); lg = logging.getLogger(f); lg.addHandler(cap); lg.propagate = False; lg.setLevel(logging.INFO)
    g = F.CollisionGuard.from_env(J, log=lg)  # thresholds from the environment (e.g. teleop.sh values)
    g.grace_s = 0
    last = {j: 0.0 for j in J}; t = 0.0
    for b in open(f).read().split("had to be clamped to be safe.")[1:]:
        b = b.split("WARNING")[0].split("INFO")[0]
        pres = dict(last); goal = dict(last)
        for m in re.finditer(r"'(\w+)': \{\s*'original goal_pos': ([-\d.e]+),\s*'safe goal_pos': ([-\d.e]+)\}", b):
            j, o, s = m.group(1), float(m.group(2)), float(m.group(3))
            if j == "gripper": continue
            pres[j] = s - (5.0 if o > s else -5.0); goal[j] = o
        g.update(goal, pres, {}, t); t += 1/30
        if g.frozen is not None: g.frozen = None  # count events only
        last = pres
    hits = [m for m in cap.n if m.startswith("KOLLISION erkannt")]
    total += len(hits)
    print(os.path.basename(f), "Fehlalarme:", len(hits), *hits[:3], sep="\n  " if hits else " ")
print("GESAMT", total)
