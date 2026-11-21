"""Offline test of the collision guard and the app update logic (no hardware, no motor commands).
Usage: ~/humanoid/.venv/bin/python test_collision_guard.py [path/to/seeed_b601_follower.py]
Check labels are printed in German for David."""
import importlib.util, logging, math, os, sys, types
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/humanoid/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601/seeed_b601_follower.py")

sys.path.insert(0, HERE)
from _load import load_follower
F, stubbed = load_follower(PATH)

class Cap(logging.Handler):
    def __init__(self): super().__init__(); self.msgs = []
    def emit(self, r): self.msgs.append((r.levelname, r.getMessage()))
cap = Cap(); lg = logging.getLogger("guardtest"); lg.addHandler(cap); lg.setLevel(logging.DEBUG); lg.propagate = False

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
DT = 1 / 30
fails = []
def check(name, cond):
    print(("OK   " if cond else "FAIL ") + name)
    if not cond: fails.append(name)

def new_guard(**kw):
    cap.msgs.clear(); return F.CollisionGuard(J, log=lg, **kw)

base = {j: 0.0 for j in J}

# 1) normal tracking: leader 20 deg ahead, follower moves 1 deg/cycle (as in real logs) -> no trigger
g = new_guard(); t = 0.0; pres = dict(base); out = None; trig = False
for i in range(300):
    goal = {j: pres[j] + 20.0 for j in J}
    out = g.update(goal, pres, {j: 1.0 for j in J}, t); trig |= out is not None
    pres = {j: pres[j] + 1.0 for j in J}; t += DT
check("normale Bewegung (1 deg/Zyklus, Leader 20 deg voraus) loest nicht aus", not trig)

# 1b) static offset (gravity sag 3 deg, leader still) -> no trigger
g = new_guard(); t = 0.0; trig = False
for i in range(300):
    trig |= g.update({j: 3.0 for j in J}, base, {}, t) is not None; t += DT
check("stehender Arm mit 3 deg Durchhang loest nicht aus", not trig)

# 1c) slow start from rest like real shoulder_lift data (0.25 deg/cycle for 5 cycles, then faster)
g = new_guard(); t = 0.0; trig = False; p = 0.0
for i in range(200):
    if i > 80: p = min(25.0, p + (0.12 if i < 86 else 1.0))
    goal = dict(base); goal["shoulder_lift"] = (25.0 if i > 80 else 0.0); prs = dict(base); prs["shoulder_lift"] = p
    trig |= g.update(goal, prs, {}, t) is not None; t += DT
check("langsames Anfahren (5 Zyklen <0.15 deg) loest nicht aus", not trig)

# 2) grace period: blocked within first 2 s -> no trigger
g = new_guard(); t = 0.0; trig = False
for i in range(55):   # 1.8 s
    goal = dict(base); goal["elbow_flex"] = 30.0
    trig |= g.update(goal, base, {}, t) is not None; t += DT
check("Anlaufpause 2 s: Blockade in den ersten 1.8 s loest nicht aus", not trig)

# 3) sustained block -> triggers after 6 cycles, freezes at present
g = new_guard(); t = 0.0
for i in range(90):
    g.update(base, base, {}, t); t += DT
pres = dict(base); pres["elbow_flex"] = 42.0; pres["wrist_roll"] = -7.0
for i in range(3):   # approach
    g.update(pres, pres, {}, t); t += DT
results = []
for i in range(10):
    goal = dict(pres); goal["elbow_flex"] = 42.0 + 12.0 + i   # leader keeps pushing
    results.append(g.update(goal, pres, {"elbow_flex": 9.5}, t)); t += DT
first = next((k for k, r in enumerate(results) if r is not None), None)
check(f"anhaltende Blockade loest aus (nach {None if first is None else first+1} Zyklen, erwartet 6)", first == 5)
check("Freeze = Position bei Erkennung", results[first] == pres if first is not None else False)
check("Log 'KOLLISION erkannt: elbow_flex' mit torq", any("KOLLISION erkannt: elbow_flex" in m and "torq=9.50" in m for _, m in cap.msgs))

# 3b) short block (5 cycles) then free -> no trigger
g = new_guard(); t = 0.0
for i in range(90): g.update(base, base, {}, t); t += DT
trig = False
for i in range(5):
    goal = dict(base); goal["shoulder_pan"] = 15.0
    trig |= g.update(goal, base, {}, t) is not None; t += DT
prs = dict(base)
for i in range(20):
    prs["shoulder_pan"] += 1.0; trig |= g.update({**base, "shoulder_pan": 15.0}, prs, {}, t) is not None; t += DT
check("kurze Blockade (5 Zyklen) loest nicht aus", not trig)

# 4) freeze holds present while leader moves anywhere; rate-limited warning
g = new_guard(); t = 0.0
for i in range(90): g.update(base, base, {}, t); t += DT
for i in range(6): g.update({**base, "shoulder_lift": 20.0}, base, {}, t); t += DT
assert g.frozen is not None
cap.msgs.clear(); held = True
for i in range(90):   # 3 s, leader wandering far away
    out = g.update({j: 30.0 + i for j in J}, {j: 0.5 for j in J}, {}, t); held &= (out == base); t += DT
warns = [m for lv, m in cap.msgs if "KOLLISION aktiv" in m]
check("Freeze haelt eingefrorene Position, solange Leader weit weg", held)
check(f"Warnung hoechstens 1x/s ({len(warns)} in 3 s)", 2 <= len(warns) <= 4)

# 5) resume: leader within 6 deg for 10 cycles
res = []
for i in range(12):
    res.append(g.update({j: 4.0 for j in J}, base, {}, t)); t += DT
check("bleibt eingefroren bis 10 Zyklen nah", all(r is not None for r in res[:9]))
check("Freigabe nach 10 Zyklen innerhalb 6 deg", res[9] is None and g.frozen is None)
check("Log 'Kollision aufgehoben'", any("Kollision aufgehoben" in m for _, m in cap.msgs))
# leader jitter breaks resume counter
g2 = new_guard(); t2 = 0.0
for i in range(90): g2.update(base, base, {}, t2); t2 += DT
for i in range(6): g2.update({**base, "shoulder_lift": 20.0}, base, {}, t2); t2 += DT
seq = [4.0] * 8 + [9.0] + [4.0] * 9
outs = [g2.update({j: v for j in J}, base, {}, t2 + k * DT) for k, v in enumerate(seq)]
check("Ausreisser >6 deg setzt Freigabe-Zaehler zurueck", all(o is not None for o in outs))

# 6) torque trigger (optional, off by default)
g = new_guard(torq_limit={"shoulder_lift": 8.0}); t = 0.0
for i in range(90): g.update(base, base, {}, t); t += DT
outs = [g.update(base, base, {"shoulder_lift": 9.0}, t + k * DT) for k in range(4)]
check("optionaler Drehmoment-Trigger (8 Nm) loest nach 4 Zyklen aus (COLLISION_TORQ_CYCLES)", outs[-1] is not None and all(o is None for o in outs[:-1]))
g = new_guard(); t = 0.0; trig = False
for i in range(200): trig |= g.update(base, base, {"shoulder_lift": 30.0}, t) is not None; t += DT
check("ohne Drehmoment-Limit: hohes torq allein loest nicht aus", not trig)

# 7) stats line every 5 s
g = new_guard(); t = 0.0
for i in range(320): g.update(base, base, {"elbow_flex": 2.5}, t); t += DT
stats = [m for _, m in cap.msgs if m.startswith("Guard-Stats")]
check(f"Statistik-Zeile alle 5 s ({len(stats)} in 10.6 s)", len(stats) == 2 and "elbow_flex trk=" in stats[0] and "tq=2.50" in stats[0])

# 8) env config
os.environ.update({"COLLISION_GUARD": "1", "COLLISION_ERR_DEG": "12", "COLLISION_ERR_WRIST_ROLL": "20", "COLLISION_CYCLES": "4"})
ge = F.CollisionGuard.from_env(J, log=lg)
check("env: Schwellen/Override/Zyklen", ge.err_deg["shoulder_pan"] == 12 and ge.err_deg["wrist_flex"] == 15 and ge.err_deg["wrist_roll"] == 20 and ge.cycles == 4)
for k in ["COLLISION_ERR_DEG", "COLLISION_ERR_WRIST_ROLL", "COLLISION_CYCLES"]: os.environ.pop(k)
gd = F.CollisionGuard.from_env(J, log=lg)
check("env Defaults 10/15 deg, 6 Zyklen", gd.err_deg["elbow_flex"] == 10 and gd.err_deg["wrist_yaw"] == 15 and gd.cycles == 6)

# 9) integration through send_action with fake motors (no hardware)
class FakeMotor:
    def __init__(self, deg): self.st = NS(pos=math.radians(deg), vel=0.0, torq=0.5, t_mos=30.0); self.sent = []
    def get_state(self): return self.st
    def request_feedback(self): pass
    def send_mit(self, *a): self.sent.append(a)
class FakeBus:
    def poll_feedback_once(self): pass
names = J + ["gripper"]
cfg = NS(joint_directions={**{j: 1.0 for j in J}, "gripper": 6.0},
         joint_limits={**{j: (-200.0, 200.0) for j in J}, "gripper": (0.0, 270.0)},
         max_relative_target={**{j: 5 for j in J}, "gripper": 25}, gravity_compensation=False,
         mit_kp={j: 50.0 for j in J}, mit_kd={j: 3.0 for j in J}, pos_vel_velocity=[0] * 7,
         temp_alarm_threshold_c=115.0, temp_overheat_threshold_c=125.0)
R = object.__new__(F.SeeedB601FollowerBase)
R.config = cfg; R.bus = FakeBus(); R.cameras = {}; R.motor_type = "rs"; R.motor_names = names
R.motors = {n: FakeMotor(0.0) for n in names}; R._in_safe_zero = False
R.mit_output_torque_limit = lambda m, p: 0.25
os.environ["COLLISION_GUARD"] = "1"; os.environ["COLLISION_GRACE_S"] = "0"
import time as _t
act = {f"{n}.pos": 0.0 for n in names}
for i in range(3): R.send_action(act)
check("send_action: Guard aktiv via COLLISION_GUARD=1", R._collision_guard is not None)
R.motors["elbow_flex"].st.pos = math.radians(10.0)   # blocked at 10 deg
act2 = dict(act); act2["elbow_flex.pos"] = 30.0; act2["gripper.pos"] = 5.0
for i in range(8): R.send_action(act2)
check("send_action: eingefroren nach Blockade", R._collision_guard.frozen is not None)
for m in R.motors.values(): m.sent.clear()
ret = R.send_action(act2)
check("send_action: Arm-Ziele = eingefrorene Position (elbow 10 deg)",
      abs(math.degrees(R.motors["elbow_flex"].sent[-1][0]) - 10.0) < 1e-6 and abs(ret["shoulder_pan.pos"]) < 1e-6)
check("send_action: normale kp im Freeze (kein Absacken)", R.motors["elbow_flex"].sent[-1][2] == 50.0)
check("send_action: Greifer laeuft normal weiter (eigene MIT-Logik)", R.motors["gripper"].sent[-1] == (0, 0, 0, 1.5, 0.25))
R._in_safe_zero = True; R.send_action(act)
check("send_action: safe_zero setzt Guard zurueck (Heimfahrt nicht blockiert)", R._collision_guard.frozen is None)
os.environ["COLLISION_GUARD"] = "0"
R2 = object.__new__(F.SeeedB601FollowerBase); R2.__dict__.update({k: v for k, v in R.__dict__.items() if k != "_collision_guard"})
R2._in_safe_zero = False; R2.send_action(act)
check("ohne COLLISION_GUARD=1: Guard aus", R2._collision_guard is None)


# ---- 10) guard v2: compliant rotary joints (values as in teleop.sh) ----
V2 = dict(err_deg={"shoulder_pan": 6.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": 15.0, "wrist_yaw": 8.0, "wrist_roll": 15.0},
          torq_limit={"shoulder_pan": 1.3, "wrist_yaw": 1.0}, torq_cycles=4)
def warm(g, t=0.0):
    for i in range(90): g.update(base, base, {}, t); t += DT
    return t
# 10a) hand holds shoulder_pan: leader 7 deg ahead (previously below the 10 deg threshold), no progress
g = new_guard(**V2); t = warm(g); outs = []
for i in range(8):
    outs.append(g.update({**base, "shoulder_pan": 7.0 + 0.3 * i}, base, {"shoulder_pan": 1.0}, t)); t += DT
first = next((k for k, r in enumerate(outs) if r is not None), None)
check(f"v2: Hand blockiert Schulter-Drehung bei 7 deg Abweichung -> Ausloesung nach 6 Zyklen ({None if first is None else first+1})", first == 5)
# 10b) hand yields slowly (progress 0.3 deg/cycle) but torque 1.5 Nm -> torque trigger after 4 cycles
g = new_guard(**V2); t = warm(g); outs = []; prs = dict(base)
for i in range(8):
    outs.append(g.update({**prs, "shoulder_pan": prs["shoulder_pan"] + 12.0}, prs, {"shoulder_pan": 1.5}, t)); t += DT
    prs["shoulder_pan"] += 0.3
first = next((k for k, r in enumerate(outs) if r is not None), None)
check(f"v2: nachgebende Hand + 1.5 Nm -> Kraft-Trigger nach 4 Zyklen ({None if first is None else first+1})", first == 3)
check("v2: Log nennt [torque]", any("KOLLISION erkannt: shoulder_pan" in m and "[torque]" in m for _, m in cap.msgs))
# 10c) wrist_yaw blocked at 9 deg
g = new_guard(**V2); t = warm(g); trig = None
for i in range(8):
    o = g.update({**base, "wrist_yaw": 9.0}, base, {"wrist_yaw": 0.6}, t); t += DT
    if o is not None and trig is None: trig = i + 1
check(f"v2: wrist_yaw blockiert bei 9 deg -> Ausloesung ({trig})", trig == 6)
# 10d) torque spikes of 3 cycles (acceleration) -> no alarm; counter independent of the block counter
g = new_guard(**V2); t = warm(g); trig = False; prs = dict(base)
for k in range(20):
    tq = {"shoulder_pan": 1.6 if k % 5 < 3 else 0.9, "wrist_yaw": 1.2 if k % 5 < 3 else 0.5}
    trig |= g.update({j: prs[j] + 30.0 for j in J}, prs, tq, t) is not None; t += DT
    prs = {j: prs[j] + 1.0 for j in J}
check("v2: Kraft-Spitzen je 3 Zyklen loesen nicht aus", not trig)
# 10e) free fast motion (leader up to 42 deg ahead, 1 deg/cycle, torque as scaled ~1.1/0.8 Nm)
g = new_guard(**V2); t = warm(g); trig = False; prs = dict(base)
for i in range(300):
    lead = 42.0 * abs(math.sin(i / 20.0))
    trig |= g.update({j: prs[j] + lead for j in J}, prs, {"shoulder_pan": 1.1, "wrist_yaw": 0.8}, t) is not None; t += DT
    prs = {j: prs[j] + 1.0 for j in J}
check("v2: freie Bewegung (Leader bis 42 deg voraus, 1.1/0.8 Nm) ohne Fehlalarm", not trig)
# 10f) start from rest at kp 20: 4 cycles with almost no progress -> no alarm
g = new_guard(**V2); t = warm(g); trig = False; p = 0.0
for i in range(40):
    p += 0.05 if i < 4 else 0.8
    trig |= g.update({**base, "shoulder_pan": 20.0}, {**base, "shoulder_pan": p}, {"shoulder_pan": 1.2}, t) is not None; t += DT
check("v2: traeges Anfahren (4 Zyklen) loest nicht aus", not trig)
# 10g) env
for k, v in {"COLLISION_ERR_SHOULDER_PAN": "6", "COLLISION_ERR_WRIST_YAW": "8", "COLLISION_TORQ_SHOULDER_PAN": "1.3",
             "COLLISION_TORQ_WRIST_YAW": "1.0", "COLLISION_TORQ_CYCLES": "4"}.items(): os.environ[k] = v
ge = F.CollisionGuard.from_env(J, log=lg)
check("v2 env: pan 6 / yaw 8 deg, Kraft 1.3/1.0 Nm, 4 Zyklen, Rest unveraendert",
      ge.err_deg["shoulder_pan"] == 6 and ge.err_deg["wrist_yaw"] == 8 and ge.err_deg["shoulder_lift"] == 10
      and ge.err_deg["wrist_flex"] == 15 and ge.torq_limit["shoulder_pan"] == 1.3 and ge.torq_limit["wrist_yaw"] == 1.0
      and ge.torq_limit["elbow_flex"] == 0 and ge.torq_cycles == 4 and "fuer 4 Zyklen" in ge.describe())
for k in ["COLLISION_ERR_SHOULDER_PAN", "COLLISION_ERR_WRIST_YAW", "COLLISION_TORQ_SHOULDER_PAN", "COLLISION_TORQ_WRIST_YAW", "COLLISION_TORQ_CYCLES"]: os.environ.pop(k)
check("v2 env Default COLLISION_TORQ_CYCLES=4", F.CollisionGuard.from_env(J, log=lg).torq_cycles == 4)
# 10h) MIT gain override in send_action
for k in list(os.environ):
    if k.startswith("MIT_K"): os.environ.pop(k)
check("v2: ohne MIT_KP_/MIT_KD_ keine Overrides", F.mit_gain_overrides() == ({}, {}))
os.environ.update({"MIT_KP_SHOULDER_PAN": "20", "MIT_KD_SHOULDER_PAN": "1.9", "MIT_KP_WRIST_YAW": "20", "MIT_KD_WRIST_YAW": "2.5", "MIT_KP_ELBOW_FLEX": "abc"})
os.environ["COLLISION_GUARD"] = "0"
R3 = object.__new__(F.SeeedB601FollowerBase); R3.__dict__.update({k: v for k, v in R.__dict__.items() if k not in ("_collision_guard", "_mit_gain_overrides")})
R3._in_safe_zero = False
for m in R3.motors.values(): m.sent.clear()
R3.send_action(act)
sent = {n: R3.motors[n].sent[-1] for n in J}
check("v2: send_action nutzt kp 20/kd 1.9 (pan) und kp 20/kd 2.5 (wrist_yaw)",
      sent["shoulder_pan"][2:4] == (20.0, 1.9) and sent["wrist_yaw"][2:4] == (20.0, 2.5))
check("v2: andere Gelenke unveraendert (kp 50/kd 3), ungueltiger Wert ignoriert",
      all(sent[j][2:4] == (50.0, 3.0) for j in ["shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]))
check("v2: Greifer unveraendert", R3.motors["gripper"].sent[-1] == (0, 0, 0, 1.5, 0.25))
R3.config = NS(**{**R3.config.__dict__, "gravity_compensation": True, "gravity_mit_kp": {j: 30.0 for j in J}, "gravity_mit_kd": {j: 2.0 for j in J}})
R3._gravity_feedforward = lambda: {}
for m in R3.motors.values(): m.sent.clear()
R3.send_action(act)
check("v2: bei gravity_compensation keine Overrides (gravity-Gains)", R3.motors["shoulder_pan"].sent[-1][2:4] == (30.0, 2.0))
for k in list(os.environ):
    if k.startswith("MIT_K"): os.environ.pop(k)


# ---- 11) guard v3: wrist_flex + auto-resume ----
V3 = dict(err_deg={"shoulder_pan": 6.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": 8.0, "wrist_yaw": 8.0, "wrist_roll": 15.0},
          torq_limit={"shoulder_pan": 1.45, "wrist_yaw": 1.0, "wrist_flex": 3.2}, torq_cycles=4)
# 11a) wrist_flex blocked at 9 deg (previously 15 deg threshold)
g = new_guard(**V3); t = warm(g); trig = None
for i in range(8):
    o = g.update({**base, "wrist_flex": 9.0 + 0.5 * i}, base, {"wrist_flex": 2.4}, t); t += DT
    if o is not None and trig is None: trig = i + 1
check(f"v3: wrist_flex blockiert bei 9 deg -> Ausloesung ({trig})", trig == 6)
# 11b) wrist_flex, yielding hand (progress 0.4 deg/cycle) at 3.4 Nm -> torque trigger
g = new_guard(**V3); t = warm(g); trig = None; prs = dict(base)
for i in range(8):
    o = g.update({**prs, "wrist_flex": prs["wrist_flex"] + 15.0}, prs, {"wrist_flex": 3.4}, t); t += DT
    prs["wrist_flex"] += 0.4
    if o is not None and trig is None: trig = i + 1
check(f"v3: wrist_flex 3.4 Nm (Hand gibt nach) -> Kraft-Trigger nach 4 Zyklen ({trig})", trig == 4)
# 11c) wrist_flex gravity hold 2.3 Nm + free motion up to 3.0 Nm -> no alarm
g = new_guard(**V3); t = warm(g); trig = False; prs = dict(base)
for i in range(300):
    trig |= g.update({**prs, "wrist_flex": prs["wrist_flex"] + 30 * abs(math.sin(i / 15))}, prs,
                     {"wrist_flex": 2.3 + 0.7 * abs(math.sin(i / 7))}, t) is not None; t += DT
    prs["wrist_flex"] += 1.0
check("v3: wrist_flex Halten 2.3 Nm + Bewegung bis 3.0 Nm ohne Fehlalarm", not trig)

LEAD31 = {"shoulder_pan": 4.0, "shoulder_lift": 2.5, "elbow_flex": 2.5, "wrist_flex": 4.0, "wrist_yaw": 4.0, "wrist_roll": 2.5}
LAG31 = {"shoulder_pan": 3.0, "shoulder_lift": 2.0, "elbow_flex": 2.0, "wrist_flex": 3.0, "wrist_yaw": 3.0, "wrist_roll": 2.0}
AR = dict(V3, auto_resume=True, auto_resume_s=1.5, resume_step_deg=1.0, creep_cycles=10, creep_lead_deg=LEAD31, creep_lag_deg=LAG31)
def block_elbow(g, t, pres):
    for i in range(7):
        g.update({**pres, "elbow_flex": pres["elbow_flex"] + 12.0 + i}, pres, {"elbow_flex": 12.0}, t); t += DT
    return t
# 11d) David's case: elbow blocked, obstacle removed, leader 22 deg away -> after 1.5 s still, creep on at max 1 deg/cycle
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
check("v3: Ellbogen-Blockade friert ein", g.frozen is not None)
sag = dict(pres); sag["elbow_flex"] = 37.0   # elbow sags 3 deg while frozen
leader = {**pres, "elbow_flex": 62.0}
cur = dict(sag); started = None; steps = []; prev_t = None; done_at = None
for i in range(120):
    o = g.update(leader, cur, {"elbow_flex": 11.0}, t); t += DT
    if g.creep is not None and started is None:
        started = i * DT; prev_t = 40.0
        check(f"v3: Weiterfahr-Ziel startet am Freeze-Punkt, nicht abgesackt ({o['elbow_flex']:.1f})", 40.0 < o["elbow_flex"] <= 41.0)
    if started is not None:
        if o is None: done_at = i; break
        steps.append(o["elbow_flex"] - prev_t); prev_t = o["elbow_flex"]
        cur["elbow_flex"] = o["elbow_flex"] - 3.0 - (0.5 if len(steps) < 3 else 0.0)   # follows with sag, sluggish start
check(f"v3: Auto-Weiter nach ~1.5 s Ruhe (nach {started if started is None else round(started, 2)} s)", started is not None and 1.4 <= started <= 1.7)
check("v3: Log 'Auto-Weiter: Arm faehrt langsam weiter'", any("Auto-Weiter: Arm faehrt langsam weiter" in m for _, m in cap.msgs))
check(f"v3: Weiterfahren max 1 deg/Zyklus (max {max(steps) if steps else 0:.2f})", bool(steps) and max(steps) <= 1.0 + 1e-9)
check(f"v3: zurueck im Normalbetrieb, wenn Ziel < 6 deg am Leader (nach {done_at} Zyklen)", done_at is not None and g.creep is None and g.frozen is None)
check("v3: Log 'Kollision aufgehoben (Auto-Weiter)' (GUI -> LAEUFT)", any("Kollision aufgehoben (Auto-Weiter)" in m for _, m in cap.msgs))
check("v3: kein erneuter Alarm beim freien Weiterfahren", sum("KOLLISION erkannt" in m for _, m in cap.msgs) == 1)
# 11e) new block while creeping -> frozen again (v3.1: 10 cycles, lead max 2.5 deg at the elbow)
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
for i in range(60):
    g.update({**pres, "elbow_flex": 62.0}, pres, {}, t); t += DT
    if g.creep is not None: break
assert g.creep is not None, "creep"
k_ref = None
for i in range(20):
    o = g.update({**pres, "elbow_flex": 62.0}, pres, {}, t); t += DT   # arm does not move (obstacle)
    if g.frozen is not None and k_ref is None: k_ref = i + 1
check(f"v3.1: Hindernis beim Weiterfahren -> wieder eingefroren (nach {k_ref} Zyklen, <=14)", k_ref is not None and k_ref <= 14)
check("v3: Log 'blockiert beim Weiterfahren'", any("blockiert beim Weiterfahren" in m for _, m in cap.msgs))
check("v3: Freeze-Punkt = aktuelle Position", g.frozen["elbow_flex"] == 40.0)
blk = [m for _, m in cap.msgs if "blockiert beim Weiterfahren" in m][-1]
check(f"v3.1: Vorlauf gegen Hindernis am Ellbogen max 2.5 deg (Kraft begrenzt): {blk.split('track=')[1].split(' ')[0]} deg",
      float(blk.split("track=")[1].split(" ")[0]) <= 2.5 + 1e-6)
# 11f) v3.1: no dead end – growing wait 1.5/3/5/5 s, unlimited
starts = []; t_fr = t; seen = g.auto_attempts
for k in range(6):
    t0 = t
    while g.creep is None and t - t0 < 10:
        g.update({**pres, "elbow_flex": 62.0}, pres, {}, t); t += DT
    starts.append(round(t - t0, 1))
    while g.creep is not None:
        g.update({**pres, "elbow_flex": 62.0}, pres, {}, t); t += DT
check(f"v3.1: Wartezeiten bis zum naechsten Versuch {starts} (3, 5, 5, ...)", 2.6 <= starts[0] <= 3.2 and all(4.9 <= s <= 5.2 for s in starts[1:]))
check(f"v3.1: nach {g.auto_attempts} Fehlversuchen weiter Auto-Weiter (keine Sackgasse)", g.auto_attempts >= 7)
# obstacle removed -> next attempt goes through
cur = dict(pres); done = False
for i in range(400):
    o = g.update({**pres, "elbow_flex": 62.0}, cur, {}, t); t += DT
    if g.creep is not None and o is not None: cur["elbow_flex"] = o["elbow_flex"] - 0.8
    if g.creep is None and g.frozen is None: done = True; break
check("v3.1: Hindernis weg -> Arm faehrt beim naechsten Versuch durch, Normalbetrieb", done and g.auto_attempts == 0)
# 11f2) message names the joint in German
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
for i in range(45): g.update({**pres, "elbow_flex": 62.0}, {**pres, "elbow_flex": 40.0 + (i % 2)}, {}, t); t += DT
akt = [m for _, m in cap.msgs if "KOLLISION aktiv" in m]
check("v3.1: Meldung nennt Gelenk auf Deutsch ('Leader: Ellbogen ... zurückführen')", bool(akt) and "Leader: Ellbogen " in akt[-1] and "zurückführen" in akt[-1])
# 11f3) David's case 19:46: wrist_roll 12 deg off – only the collided joint must come back
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
leader = {**pres, "elbow_flex": 42.0, "wrist_roll": 12.0}
cur = dict(pres); n_frozen = 0; ok = False
for i in range(200):
    jig = dict(cur); jig["shoulder_pan"] += 1.0 if i % 2 else 0.0   # arm never fully still -> only manual release possible
    o = g.update(leader, jig, {}, t); t += DT
    if g.creep is not None and o is not None: cur = {j: o[j] for j in J}
    if g.creep is None and g.frozen is None: ok = True; break
check("v3.1: Ellbogen zurueckgefuehrt, wrist_roll 12 deg daneben -> langsam weiter, dann Normalbetrieb", ok)
check("v3.1: Freigabe von Hand startet langsames Weiterfahren", any("Leader zurückgeführt" in m for _, m in cap.msgs))
g = new_guard(**V3); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
for i in range(60): g.update(leader, pres, {}, t); t += DT
akt = [m for _, m in cap.msgs if "KOLLISION aktiv" in m]
check("v3.1: ohne Auto-Weiter bleibt die alte Regel (alle Gelenke) und nennt 'Handgelenk rollen'",
      g.frozen is not None and "Leader: Handgelenk rollen" in akt[-1])
# 11g) arm is moved/pushed while frozen -> no auto-resume
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
for i in range(90):
    jig = dict(pres); jig["elbow_flex"] = 40.0 + (1.0 if i % 2 else -1.0)
    g.update({**pres, "elbow_flex": 62.0}, jig, {}, t); t += DT
check("v3: solange der Arm bewegt/gedrueckt wird, kein Auto-Weiter", g.frozen is not None and g.creep is None)
# 11h) without auto-resume (code default) the v2 behaviour stays
g = new_guard(**V3); t = warm(g); pres = dict(base); pres["elbow_flex"] = 40.0
t = block_elbow(g, t, pres)
for i in range(120): g.update({**pres, "elbow_flex": 62.0}, pres, {}, t); t += DT
check("v3: COLLISION_AUTO_RESUME aus -> bleibt eingefroren", g.frozen is not None and g.creep is None)
# 11i) torque trigger while creeping
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["wrist_flex"] = 10.0
for i in range(6): g.update({**pres, "wrist_flex": 30.0}, pres, {}, t); t += DT
for i in range(50): g.update({**pres, "wrist_flex": 30.0}, pres, {}, t); t += DT
assert g.creep is not None
cur = dict(pres)
for i in range(6):
    o = g.update({**pres, "wrist_flex": 30.0}, cur, {"wrist_flex": 3.5}, t); t += DT
    if o is not None and g.creep is not None: cur["wrist_flex"] = o["wrist_flex"] - 0.5
check("v3: Kraft-Trigger auch beim Weiterfahren", g.frozen is not None and any("torque beim Weiterfahren" in m for _, m in cap.msgs))
# 11j) env
for k, v in {"COLLISION_AUTO_RESUME": "1", "COLLISION_AUTO_RESUME_S": "2", "COLLISION_RESUME_STEP_DEG": "0.5",
             "COLLISION_ERR_WRIST_FLEX": "8", "COLLISION_TORQ_WRIST_FLEX": "3.2"}.items(): os.environ[k] = v
ge = F.CollisionGuard.from_env(J, log=lg)
check("v3 env: Auto-Weiter an, 2 s, 0.5 deg/Zyklus, wrist_flex 8 deg / 3.2 Nm",
      ge.auto_resume and ge.auto_resume_s == 2 and ge.resume_step_deg == 0.5 and ge.err_deg["wrist_flex"] == 8
      and ge.torq_limit["wrist_flex"] == 3.2 and "Auto-Weiter nach 2 s" in ge.describe())
for k in ["COLLISION_AUTO_RESUME", "COLLISION_AUTO_RESUME_S", "COLLISION_RESUME_STEP_DEG", "COLLISION_ERR_WRIST_FLEX", "COLLISION_TORQ_WRIST_FLEX"]: os.environ.pop(k)
check("v3 env: ohne Variable Auto-Weiter aus", not F.CollisionGuard.from_env(J, log=lg).auto_resume)
os.environ["MIT_KP_WRIST_FLEX"] = "40"; os.environ["MIT_KD_WRIST_FLEX"] = "4.5"
check("v3: MIT-Override wrist_flex kp 40 / kd 4.5", F.mit_gain_overrides() == ({"wrist_flex": 40.0}, {"wrist_flex": 4.5}))
os.environ.pop("MIT_KP_WRIST_FLEX"); os.environ.pop("MIT_KD_WRIST_FLEX")

# ---- 12) GUI (without a window): version, reload button, auto-update decision ----
import ast
GUI = os.environ.get("TELEOP_GUI", os.path.expanduser("~/humanoid/teleop_gui.py"))
if os.path.exists(GUI):
    src = open(GUI).read()
    compile(src, GUI, "exec")
    tree = ast.parse(src)
    ns = {}
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "update_decision")
    exec(compile(ast.Module(body=[fn], type_ignores=[]), GUI, "exec"), ns)
    ud = ns["update_decision"]
    A, B2, C = ((1, 1), (1, 1)), ((2, 2), (1, 1)), ((3, 3), (1, 1))
    check("GUI: nichts geaendert -> kein Neustart", ud(A, A, A, False) == "none")
    check("GUI: Datei aendert sich gerade -> erst warten", ud(B2, A, A, False) == "wait")
    check("GUI: Aenderung stabil, Teleop aus -> Neustart", ud(B2, A, B2, False) == "restart")
    check("GUI: Teleop laeuft -> nie Neustart, nur Hinweis", ud(B2, A, B2, True) == "pending")
    check("GUI: fehlerhafte Fassung nicht erneut laden", ud(C, A, C, False, bad_sig=C) == "bad")
    check("GUI: Version im Titel + Label", 'title(f"Robot Teleop – Version {APP_VERSION}")' in src and 'text=f"Version {APP_VERSION}"' in src)
    check("GUI: Knopf 'App neu laden' im Lauf gesperrt", "⟳ App neu laden" in src and "breload.config(state=state)" in src)
    check("GUI: Neustart per os.execv, vorher Syntax-Pruefung, nie waehrend Teleop",
          "os.execv(sys.executable" in src and 'compile(f.read(), APP_FILE, "exec")' in src and "if teleop_running():" in src)
    check("GUI: Hinweis 'Update verfügbar – wird nach STOPP geladen' und Log 'Aktualisiert auf Version'",
          "Update verfügbar – wird nach STOPP geladen" in src and "Aktualisiert auf Version" in src)
    check("GUI: Status 'LANGSAM WEITER' beim Auto-Weiter", "Auto-Weiter: Arm faehrt langsam weiter" in src)
else:
    print("(GUI-Datei nicht gefunden, GUI-Tests uebersprungen)")


# ---- 13) guard v3.1: friction while creeping, torque trigger with extra conditions, env, GUI ----
def creep_sim(g, joint, kp, kd, fric, t, n=200):
    """Physics approximation: the joint moves only if kp*error > friction; velocity = (torque - friction)/kd."""
    pres = dict(base); pres[joint] = 0.0
    for i in range(7): g.update({**pres, joint: 30.0}, pres, {}, t); t += DT     # block -> freeze
    if g.frozen is None:
        for i in range(10): g.update({**pres, joint: 30.0}, pres, {}, t); t += DT
    refreeze = 0; done = False; tgt = None
    for i in range(n):
        was_creep = g.creep is not None
        o = g.update({**pres, joint: 30.0}, pres, {}, t); t += DT
        if was_creep and g.frozen is not None: refreeze += 1
        if g.creep is not None and o is not None:
            e = math.radians(o[joint] - pres[joint]); m = kp * abs(e)
            v = max(0.0, m - fric) / kd
            pres[joint] += math.copysign(math.degrees(v * DT), e)
        if was_creep and g.creep is None and g.frozen is None: done = True; break
    return refreeze, done
r, d = creep_sim(new_guard(**AR), "shoulder_pan", 20.0, 1.9, 0.6, warm(new_guard(**AR)))
g = new_guard(**AR); t = warm(g); r, d = creep_sim(g, "shoulder_pan", 20.0, 1.9, 0.6, t)
check(f"v3.1: Schulter-Drehung kp 20 mit Reibung 0.6 Nm: Weiterfahren ohne Fehl-Einfrieren (neu eingefroren {r}x, fertig {d})", r == 0 and d)
g = new_guard(**AR); t = warm(g); r, d = creep_sim(g, "wrist_yaw", 20.0, 2.5, 0.6, t)
check(f"v3.1: Handgelenk drehen kp 20 mit Reibung: ohne Fehl-Einfrieren ({r}x, fertig {d})", r == 0 and d)
g = new_guard(**AR); t = warm(g); r, d = creep_sim(g, "wrist_flex", 40.0, 4.5, 1.2, t)
check(f"v3.1: Handgelenk hoch/runter kp 40 mit Reibung 1.2 Nm: ohne Fehl-Einfrieren ({r}x, fertig {d})", r == 0 and d)
OLD30 = dict(V3, auto_resume=True, auto_resume_s=1.5, creep_cycles=4, creep_lead_deg=2.0, creep_lag_deg=0.5)
g = new_guard(**OLD30); t = warm(g); r, d = creep_sim(g, "shoulder_pan", 20.0, 1.9, 0.6, t, n=150)
check(f"v3.1: Gegenprobe – mit v3.0-Werten (2 deg Vorlauf, 4 Zyklen) friert dieselbe Reibung faelschlich ein ({r}x)", r >= 1)
# 13b) torque trigger with minimum offset and progress limit (teleop.sh values)
TQ31 = dict(V3, torq_limit={"shoulder_pan": 1.6, "wrist_yaw": 1.25}, torq_cycles=6, torq_min_lead_deg=3.0, torq_max_progress_deg=0.5)
def tq_case(j, tq, lead, prog, n=10):
    g = new_guard(**TQ31); t = warm(g); prs = dict(base); first = None
    for i in range(n):
        o = g.update({**prs, j: prs[j] + lead}, prs, {j: tq}, t); t += DT
        prs[j] += prog
        if o is not None and first is None: first = i + 1
    return first
check("v3.1: wrist_yaw 1.11 Nm / 5.4 deg (Fehlalarm 19:45:42) loest nicht mehr aus", tq_case("wrist_yaw", 1.11, 5.4, 0.4) is None)
check("v3.1: shoulder_pan 1.49 Nm / 4.5 deg (Fehlalarm 19:46:21) loest nicht mehr aus", tq_case("shoulder_pan", 1.49, 4.5, 0.04) is None)
check("v3.1: wrist_yaw 1.3 Nm, 3.4 deg, steht -> Kraft-Trigger nach 6 Zyklen", tq_case("wrist_yaw", 1.3, 3.4, 0.1) == 6)
check("v3.1: wrist_yaw 1.3 Nm aber nur 2.7 deg Abweichung -> kein Alarm", tq_case("wrist_yaw", 1.3, 2.7, 0.0) is None)
check("v3.1: shoulder_pan 1.7 Nm, 8 deg, Hand gibt langsam nach (0.4 deg/Zyklus) -> Alarm", tq_case("shoulder_pan", 1.7, 8.0, 0.4) == 6)
check("v3.1: shoulder_pan 1.7 Nm, Arm bewegt sich zuegig (0.8 deg/Zyklus) -> kein Kraft-Alarm", tq_case("shoulder_pan", 1.7, 4.0, 0.8) is None)
# 13c) env defaults v3.1
for k, v in {"COLLISION_AUTO_RESUME": "1", "MIT_KP_SHOULDER_PAN": "20", "MIT_KP_WRIST_FLEX": "40", "COLLISION_TORQ_MIN_LEAD_DEG": "3",
             "COLLISION_TORQ_MAX_PROGRESS_DEG": "0.5"}.items(): os.environ[k] = v
ge = F.CollisionGuard.from_env(J, log=lg)
check("v3.1 env: Vorlauf/Haenge-Schwelle weich 4/3 deg, steif 2.5/2 deg; 10 Zyklen; unbegrenzte Versuche",
      ge.creep_lead["shoulder_pan"] == 4 and ge.creep_lag["shoulder_pan"] == 3 and ge.creep_lead["wrist_flex"] == 4
      and ge.creep_lead["elbow_flex"] == 2.5 and ge.creep_lag["elbow_flex"] == 2 and ge.creep_cycles == 10
      and ge.auto_resume_max == 0 and ge.torq_min_lead_deg == 3 and ge.torq_max_progress_deg == 0.5 and "unbegrenzt" in ge.describe())
for k in ["COLLISION_AUTO_RESUME", "MIT_KP_SHOULDER_PAN", "MIT_KP_WRIST_FLEX", "COLLISION_TORQ_MIN_LEAD_DEG", "COLLISION_TORQ_MAX_PROGRESS_DEG"]: os.environ.pop(k)
# 13d) GUI status line matches the log message
import re as _re
g = new_guard(**AR); t = warm(g); pres = dict(base); pres["wrist_flex"] = 10.0
for i in range(7): g.update({**pres, "wrist_flex": 30.0}, pres, {}, t); t += DT
for i in range(40): g.update({**pres, "wrist_flex": 30.0}, {**pres, "wrist_flex": 10.0 + (i % 2)}, {}, t); t += DT
akt = [m for _, m in cap.msgs if "KOLLISION aktiv" in m][-1]
mm = _re.search(r"Leader: (.+?) [\d.]+ deg zurückführen", akt)
check(f"v3.1: GUI erkennt Gelenk in der Meldung ({mm.group(1) if mm else None})", mm is not None and mm.group(1) == "Handgelenk hoch/runter" and "Auto-Weiter in" in akt)
if os.path.exists(GUI):
    _src = open(GUI).read()
    _rx = _re.search(r're\.search\(r"(.+?)", line\)', _src)
    _m2 = _re.search(_rx.group(1), akt) if _rx else None
    check("v3.1 GUI: Status-Regex der App findet den Gelenk-Namen in der echten Meldung", _m2 is not None and _m2.group(1) == "Handgelenk hoch/runter")

print(f"\n{'stub-imports' if stubbed else 'echte lerobot/motorbridge-Imports'}; {len(fails)} Fehler")
sys.exit(1 if fails else 0)
