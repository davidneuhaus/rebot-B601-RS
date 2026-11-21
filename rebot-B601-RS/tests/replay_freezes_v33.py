"""Replay every collision freeze found in session logs through guard v3.2 and v3.3 settings, and check the
gravity model against the logged 5 s Guard-Stats. Usage:
  ~/humanoid/.venv/bin/python replay_freezes_v33.py <seeed_b601_follower.py> ~/humanoid/logs/session_*.log
Per event the logged deviation, torque, gravity and progress are held for 12 cycles (0.4 s)."""
import glob, logging, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _load import load_follower
F, _ = load_follower(sys.argv[1])
files = sys.argv[2:] or sorted(glob.glob(os.path.expanduser("~/humanoid/logs/session_*.log")))
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
KP = {"shoulder_pan": 20.0, "shoulder_lift": 150.0, "elbow_flex": 150.0, "wrist_flex": 40.0, "wrist_yaw": 20.0, "wrist_roll": 50.0}
ERR = {"shoulder_pan": 6.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": 10.0, "wrist_yaw": 8.0, "wrist_roll": 15.0}
CFG = {"3.2": dict(torq_limit={"shoulder_pan": 1.6, "wrist_yaw": 1.25}, grav_res_joints=()),
       "3.3": dict(torq_limit={"shoulder_pan": 1.6, "wrist_yaw": 2.4},
                   grav_res_nm={"shoulder_lift": 4.0, "elbow_flex": 4.0, "wrist_flex": 1.5},
                   grav_res_joints=("shoulder_lift", "elbow_flex", "wrist_flex"))}
class Cap(logging.Handler):
    def __init__(s): super().__init__(); s.m = []
    def emit(s, r): s.m.append(r.getMessage())
cap = Cap(); lg = logging.getLogger("replay33"); lg.addHandler(cap); lg.propagate = False; lg.setLevel(logging.INFO)
RX = re.compile(r"(\d\d:\d\d:\d\d) \S+ KOLLISION erkannt: (\w+) err=([-\d.]+) deg .*?torq=([-\d.na/]+) Nm fortschritt=([-\d.]+) deg/Zyklus \[([^\]]+)\]")
RG = re.compile(r"Schwerkraft beim Einfrieren: (\w+) g=([-\d.]+) Nm")
def run(v, j, err, tq, g, prog, creep):
    cap.m.clear()
    gd = F.CollisionGuard(J, err_deg=ERR, cycles=6, grace_s=0.0, torq_cycles=6, torq_min_lead_deg=3.0, torq_max_progress_deg=0.5,
                          kp=KP, clamp_deg=5.0, ff_scale=0.0, auto_resume=True, creep_cycles=10,
                          creep_lead_deg={k: (4.0 if KP[k] <= 40 else 2.5) for k in J},
                          creep_lag_deg={k: (3.0 if KP[k] <= 40 else 2.0) for k in J}, log=lg, **CFG[v])
    base = {k: 0.0 for k in J}; base.update(shoulder_lift=120.0, elbow_flex=90.0)
    gv = {k: 0.0 for k in J}; gv[j] = g; s = 1.0 if err >= 0 else -1.0
    if creep:   # freeze happened while creeping: start a creep towards a leader err deg away
        gd.frozen = dict(base); gd.hit_joint = j; gd._start_creep({**base, j: base[j] + err}, base, "replay")
    for i in range(12 if not creep else 15):
        p = dict(base); p[j] = base[j] + s * prog * i
        goal = dict(p); goal[j] = (base[j] + err) if creep else p[j] + err
        gd.update(goal, p, {**{k: 0.0 for k in J}, j: tq}, i * (1 / 30), grav=gv)
        if gd.frozen is not None:
            m = [x for x in cap.m if x.startswith("KOLLISION erkannt")][-1]
            return "torque" if "[torque" in m else "blockiert"
    return None
tot = {"3.2": 0, "3.3": 0}; n = 0
for f in files:
    lines = open(f, errors="ignore").read().splitlines()
    for i, ln in enumerate(lines):
        m = RX.search(ln)
        if not m: continue
        t, j, err, tq, prog, why = m.group(1), m.group(2), float(m.group(3)), m.group(4), float(m.group(5)), m.group(6)
        tq = 0.0 if tq == "n/a" else float(tq)
        g = 0.0
        for k in range(i + 1, min(i + 4, len(lines))):
            mg = RG.search(lines[k])
            if mg and mg.group(1) == j: g = float(mg.group(2)); break
        creep = "Weiterfahren" in why
        r = {v: run(v, j, err, tq, g, prog, creep) for v in ("3.2", "3.3")}
        n += 1; tot["3.2"] += r["3.2"] is not None; tot["3.3"] += r["3.3"] is not None
        print(f"{os.path.basename(f)[8:-4]} {t} {j:13s} err={err:6.1f} tq={tq:6.2f} g={g:6.2f} push={(tq - g) * (1 if err >= 0 else -1):6.2f} "
              f"prog={prog:5.2f} [{why}] -> v3.2 {r['3.2'] or '-':9s} v3.3 {r['3.3'] or '-'}")
print(f"EVENTS {n}: v3.2 freezes {tot['3.2']}, v3.3 freezes {tot['3.3']}")
# ---- gravity model vs Guard-Stats (windows where the joint holds still: lead < 4 deg) ----
RS = re.compile(r"(\w+) trk=[\d.]+ lead=([\d.]+) tq=([\d.]+) g=([\d.]+) res=([\d.]+)")
print("\nGravity model vs measured torque (5 s Guard-Stats maxima; 'hold' = lead < 4 deg and |tq| > 1 Nm):")
for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
    same, opp, rest, ratios = 0, 0, 0, []
    for f in files:
        for ln in open(f, errors="ignore"):
            if "Guard-Stats" not in ln: continue
            for m in RS.finditer(ln):
                if m.group(1) != j: continue
                lead, tq, g, res = (float(m.group(k)) for k in range(2, 6))
                if lead >= 4.0: continue
                if tq < 1.0: rest += 1; continue          # supported (rest pose on the stop)
                if res < 0.5 * (tq + g): same += 1; ratios.append(tq / g if g > 0.3 else float("nan"))
                else: opp += 1
    rr = sorted(x for x in ratios if x == x)
    med = rr[len(rr) // 2] if rr else float("nan")
    print(f"  {j:13s}: hold windows same sign {same}, opposite sign {opp}, resting (|tq|<1 Nm, arm on its stop) {rest}; "
          f"median measured/model {med:.2f}")
