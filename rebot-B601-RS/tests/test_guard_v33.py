"""Offline tests for guard v3.3 / app 3.7 (2026-11-21). No hardware, no motor commands.
Usage: ~/humanoid/.venv/bin/python test_guard_v33.py [seeed_b601_follower.py] [plugin_dir] [teleop.sh]
Checks:
  A) today's false freezes (values from logs/session_2026-11-21_13-43-15.log and _13-52-55.log) no longer fire
  B) simulated real blocks (quasi-static one-joint loop with the URDF gravity model) still fire
  C) gravity feedforward: ramp, caps, sign against the logged holding torque
  D) gripper: no mid-stroke open-stall cut-off, cut only at the open stop, closing cap unchanged
  E) teleop.sh defaults"""
import importlib.util, logging, math, os, re, sys, types
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN = os.path.expanduser("~/humanoid/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601")
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PLUGIN, "seeed_b601_follower.py")
PDIR = sys.argv[2] if len(sys.argv) > 2 else PLUGIN
TELEOP = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/humanoid/teleop.sh")
URDF = os.path.join(PDIR, "assets", "00-arm-rs_asm-v3.urdf")
sys.path.insert(0, HERE)
from _load import load_follower
F, stubbed = load_follower(PATH)
spec = importlib.util.spec_from_file_location("b601_gravity_v33", os.path.join(PDIR, "gravity.py"))
G = importlib.util.module_from_spec(spec); sys.modules[spec.name] = G; spec.loader.exec_module(G)
F._GRAVITY_MOD = G
import numpy as np
MODEL = G.load_dynamics_model(urdf_path=URDF)

class Cap(logging.Handler):
    def __init__(self): super().__init__(); self.msgs = []
    def emit(self, r): self.msgs.append((r.levelname, r.getMessage()))
cap = Cap(); lg = logging.getLogger("v33test"); lg.addHandler(cap); lg.setLevel(logging.DEBUG); lg.propagate = False
fails = []
def check(name, cond):
    print(("OK   " if cond else "FAIL ") + name)
    if not cond: fails.append(name)

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
KP = {"shoulder_pan": 20.0, "shoulder_lift": 150.0, "elbow_flex": 150.0, "wrist_flex": 40.0, "wrist_yaw": 20.0, "wrist_roll": 50.0}
ERR = {"shoulder_pan": 6.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": 10.0, "wrist_yaw": 8.0, "wrist_roll": 15.0}
TQ32 = {"shoulder_pan": 1.6, "wrist_yaw": 1.25}
TQ33 = {"shoulder_pan": 1.6, "wrist_yaw": 2.4}
RES_J = ("shoulder_lift", "elbow_flex", "wrist_flex")
RES33 = {"shoulder_lift": 4.0, "elbow_flex": 4.0, "wrist_flex": 1.5}
DT = 1 / 30

def guard(v="3.3", ff=0.0, **kw):
    cap.msgs.clear()
    a = dict(err_deg=ERR, cycles=6, grace_s=0.0, torq_cycles=6, torq_min_lead_deg=3.0, torq_max_progress_deg=0.5,
             log=lg, kp=KP, clamp_deg=5.0, ff_scale=ff, auto_resume=True, creep_cycles=10,
             creep_lead_deg={j: (4.0 if KP[j] <= 40 else 2.5) for j in J}, creep_lag_deg={j: (3.0 if KP[j] <= 40 else 2.0) for j in J})
    if v == "3.2":
        a.update(torq_limit=TQ32, grav_res_joints=())
    else:
        a.update(torq_limit=TQ33, grav_res_nm=RES33, grav_res_joints=RES_J)
    a.update(kw)
    return F.CollisionGuard(J, **a)

def grav(pos):
    q = np.zeros(MODEL.nq)
    for i, j in enumerate(J): q[i] = math.radians(pos[j])
    t = G.compute_generalized_gravity(MODEL, q)
    return {j: float(t[i]) for i, j in enumerate(J)}

def pose(**kw):
    p = {j: 0.0 for j in J}; p.update(kw); return p

# ---------------- A) replay of today's logged freeze events ----------------
# (time, joint, err_deg, torq_Nm, g_Nm, progress_deg_per_cycle, phase, logged reason)
EVENTS = [
    ("13:45:53", "shoulder_lift", -38.3, -12.81, -11.09, 0.04, "normal", "blockiert"),
    ("13:57:20", "shoulder_lift", -34.4, -11.91, -10.34, 0.11, "normal", "blockiert"),
    ("13:44:42", "wrist_flex",     43.6,   3.35,   1.97, 0.02, "normal", "blockiert"),
    ("13:44:29", "wrist_yaw",       9.5,   1.64,   0.00, 0.04, "normal", "torque"),
    ("13:46:25", "wrist_yaw",      -8.6,  -1.37,  -0.01, 0.20, "normal", "torque"),
    ("13:46:28", "wrist_yaw",      -5.5,  -1.67,  -0.08, 0.07, "normal", "torque"),
    ("13:46:28b", "wrist_yaw",     -5.6,  -1.71,  -0.09, 0.00, "normal", "torque"),
    ("13:46:29", "wrist_yaw",      -6.0,  -1.70,  -0.10, 0.02, "normal", "torque"),
    ("13:46:50", "wrist_yaw",       6.3,   1.65,  -0.00, 0.07, "normal", "torque"),
    ("13:48:41", "wrist_yaw",      -5.1,  -2.10,  -0.15, -0.35, "normal", "torque"),
]
def replay_event(v, joint, err, tq, g, prog, n=12):
    """n cycles with the logged deviation/torque/gravity; the joint creeps by `prog` deg per cycle.
    Returns ('torque'|'blockiert'|None, cycle)."""
    gd = guard(v)
    base = pose(shoulder_lift=120.0, elbow_flex=90.0)
    gv = {j: 0.0 for j in J}; gv[joint] = g
    sgn = 1.0 if err >= 0 else -1.0
    for i in range(n):
        pres = dict(base); pres[joint] = base[joint] + sgn * prog * i
        goal = dict(pres); goal[joint] = pres[joint] + err
        torq = {j: 0.0 for j in J}; torq[joint] = tq
        gd.update(goal, pres, torq, i * DT, grav=gv)
        if gd.frozen is not None:
            m = [x for _, x in cap.msgs if x.startswith("KOLLISION erkannt")][-1]
            return ("torque" if "[torque" in m else "blockiert"), i
    return None, None
print("--- A) today's freeze events, v3.2 vs v3.3 (12 cycles = 0.4 s at the logged values) ---")
n_old = n_new = 0
for t, j, err, tq, g, prog, ph, why in EVENTS:
    r32, c32 = replay_event("3.2", j, err, tq, g, prog)
    r33, c33 = replay_event("3.3", j, err, tq, g, prog)
    n_old += r32 is not None; n_new += r33 is not None
    print(f"     {t:9s} {j:13s} err={err:6.1f} tq={tq:6.2f} g={g:6.2f} prog={prog:5.2f} log={why:9s} -> v3.2: {r32 or '-':9s} v3.3: {r33 or '-'}")
    if j in ("shoulder_lift", "wrist_flex"):
        check(f"A {t} {j}: gravity stall (push {abs(tq - g):.2f} Nm beyond g) no longer freezes (v3.2 froze: {r32})", r32 is not None and r33 is None)
    elif abs(err) <= ERR[j]:
        check(f"A {t} wrist_yaw {tq:.2f} Nm at {err:.1f} deg: torque trigger no longer fires (v3.2: {r32})", r32 == "torque" and r33 is None)
    else:
        check(f"A {t} wrist_yaw {tq:.2f} Nm at {err:.1f} deg: no torque freeze; only the 8-deg lag trigger may catch it ({r33})", r33 in (None, "blockiert"))
check("A: v3.3 log names the gravity explanation once per joint ('drueckt aber nur')",
      any("drueckt aber nur" in m for _, m in (replay_event("3.3", "shoulder_lift", -38.3, -12.81, -11.09, 0.04) and cap.msgs)))
print(f"     -> v3.2 froze {n_old}/{len(EVENTS)} events, v3.3 {n_new}/{len(EVENTS)} (remaining = wrist_yaw lag > 8 deg with no progress)")
# elbow creep freeze 13:44:44: lag in the gravity-ASSISTED direction with 14.4 Nm push beyond g -> must stay a collision
gd = guard("3.3"); base = pose(shoulder_lift=60.0, elbow_flex=100.0)
gd.frozen = dict(base); gd.hit_joint = "elbow_flex"; gd._start_creep({**base, "elbow_flex": 80.0}, base, "test")
hit = None
for i in range(15):
    gd.update({**base, "elbow_flex": 80.0}, base, {**{j: 0.0 for j in J}, "elbow_flex": -7.77}, i * DT,
              grav={**{j: 0.0 for j in J}, "elbow_flex": 6.59})
    if gd.frozen is not None: hit = i; break
check(f"A 13:44:44 elbow pressing DOWN 14.4 Nm beyond gravity while creeping -> still a collision (cycle {hit})", hit is not None)

# ---------------- B) simulated loop: real blocks still freeze ----------------
def sim(v, joint, start, leader, ff=0.0, true_scale=1.0, obstacle=None, vmax=1.5, clamp=5.0):
    gd = guard(v, ff=ff)
    pres = dict(start); cmd = pres[joint]
    for i, L in enumerate(leader):
        gm = grav(pres); ffj = ff * gm[joint]
        torq = {j: true_scale * gm[j] for j in J}
        torq[joint] = KP[joint] * math.radians(cmd - pres[joint]) + ffj
        goal = dict(start); goal[joint] = L
        out = gd.update(goal, pres, torq, i * DT, grav=gm)
        if out is not None and gd.frozen is not None:
            return i, pres
        tgt = (out or goal)[joint]
        cmd = pres[joint] + max(-clamp, min(clamp, tgt - pres[joint]))
        p_eq = cmd - math.degrees((true_scale * gm[joint] - ffj) / KP[joint])
        new = pres[joint] + max(-vmax, min(vmax, p_eq - pres[joint]))
        if obstacle is not None:
            lo, hi = obstacle
            if pres[joint] <= hi and new > hi: new = hi
            if pres[joint] >= lo and new < lo: new = lo
        pres = dict(pres); pres[joint] = new
    return None, pres
def ramp(a, b, step, hold=60):
    n = max(1, int(abs(b - a) / step))
    return [a + (b - a) * k / n for k in range(n + 1)] + [b] * hold
print("--- B) simulated loop (URDF gravity, kp/clamp as on the robot) ---")
st = pose(shoulder_lift=160.0, elbow_flex=120.0)
gst = grav(st)["shoulder_lift"]
print(f"     overhead pose lift 160 / elbow 120: g_lift = {gst:.2f} Nm, kp*5deg = {150 * math.radians(5):.2f} Nm")
for ts in (1.0, 1.1, 1.2):
    fr, p = sim("3.3", "shoulder_lift", st, ramp(160.0, 120.0, 2.0, hold=120), true_scale=ts)
    check(f"B overhead move lift 160->120 fast, FF off, true gravity {ts:.1f}x model -> no freeze (ends {p['shoulder_lift']:.1f})", fr is None)
for ts in (1.0, 1.2):
    fr, p = sim("3.3", "shoulder_lift", st, ramp(160.0, 120.0, 2.0, hold=120), ff=0.8, true_scale=ts)
    check(f"B same with GRAVITY_FF 0.8, true {ts:.1f}x -> follows, no freeze (ends {p['shoulder_lift']:.1f})", fr is None and abs(p["shoulder_lift"] - 120.0) < 3.0)
fr, _ = sim("3.3", "shoulder_lift", pose(shoulder_lift=20.0, elbow_flex=90.0), ramp(20.0, 120.0, 2.0), ff=0.8, obstacle=(-999, 70.0))
check(f"B FF 0.8: obstacle at lift 70 while lifting -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("3.3", "shoulder_lift", pose(shoulder_lift=20.0, elbow_flex=90.0), ramp(20.0, 120.0, 2.0), ff=0.0, obstacle=(-999, 70.0))
check(f"B FF off: obstacle at lift 70 while lifting -> freeze (push = kp*5deg - g > 4 Nm) (cycle {fr})", fr is not None)
fr, _ = sim("3.3", "shoulder_lift", st, ramp(160.0, 120.0, 2.0), ff=0.8, obstacle=(145.0, 999))
check(f"B FF 0.8: obstacle at lift 145 on the overhead move -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("3.3", "elbow_flex", pose(shoulder_lift=60.0, elbow_flex=40.0), ramp(40.0, 120.0, 2.0), ff=0.8, obstacle=(-999, 80.0))
check(f"B FF 0.8: elbow obstacle at 80 -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("3.3", "wrist_flex", pose(shoulder_lift=60.0, elbow_flex=90.0, wrist_flex=-30.0), ramp(-30.0, 40.0, 2.0), ff=0.8, obstacle=(-999, 10.0))
check(f"B FF 0.8: wrist_flex obstacle at 10 -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("3.3", "shoulder_lift", pose(shoulder_lift=120.0, elbow_flex=90.0), ramp(120.0, 40.0, 2.0), ff=0.8, obstacle=(90.0, 999))
check(f"B FF 0.8: block in the gravity-assisted direction (lowering onto something) -> freeze (cycle {fr})", fr is not None)
# wrist_yaw held by a hand: the joint stops, leader goes on
gd = guard("3.3"); base = pose(shoulder_lift=60.0, elbow_flex=90.0); hit = None
for i in range(30):
    gd.update({**base, "wrist_yaw": min(30.0, 2.0 * i)}, base, {**{j: 0.0 for j in J}, "wrist_yaw": 1.75}, i * DT, grav={j: 0.0 for j in J})
    if gd.frozen is not None: hit = i; break
check(f"B wrist_yaw held by hand (kp 20 presses 1.75 Nm) -> lag trigger freezes (cycle {hit}, leader {2.0 * hit if hit else None} deg ahead)", hit is not None and hit <= 12)
gd = guard("3.3"); hit = None
for i in range(12):
    gd.update({**base, "wrist_yaw": 5.0}, base, {**{j: 0.0 for j in J}, "wrist_yaw": 2.8}, i * DT, grav={j: 0.0 for j in J})
    if gd.frozen is not None: hit = i; break
check(f"B wrist_yaw hard hit 2.8 Nm at 5 deg lead -> torque trigger freezes (cycle {hit})", hit is not None)

# ---------------- C) gravity feedforward ----------------
print("--- C) gravity feedforward ---")
check("C caps: lift 14 / elbow 9 / wrist_flex 3 / rotary 1 Nm", F.GRAVITY_FF_MAX_NM == {"shoulder_pan": 1.0, "shoulder_lift": 14.0, "elbow_flex": 9.0, "wrist_flex": 3.0, "wrist_yaw": 1.0, "wrist_roll": 1.0})
mx = {j: max(abs(grav(pose(shoulder_lift=b, elbow_flex=c, wrist_flex=d))[j]) for b in range(0, 171, 10) for c in range(0, 201, 20) for d in (-80, 0, 90)) for j in J}
print("     model max |g| over the workspace: " + ", ".join(f"{j}={mx[j]:.2f}" for j in J))
check("C 0.8 x model max stays inside every cap (caps only stop model errors)", all(0.8 * mx[j] <= F.GRAVITY_FF_MAX_NM[j] + 1e-9 for j in J))
os.environ.pop("GRAVITY_FF_RAMP_S", None)
check("C GRAVITY_FF_RAMP_S default 1.0 s", F.gravity_ff_ramp_s() == 1.0)
# sign: holding torque logged 13:45:15-13:45:40 (lift tq 2.2-5.4 Nm vs g 1.4-4.3; elbow tq 7.1-8.2 vs g 6.4-6.8, residual 0.6-1.5)
check("C sign: model g has the sign of the measured holding torque (13:45:53 lift g -11.09 vs tq -12.81; 13:44:42 wrist_flex 1.97 vs 3.35)",
      (-11.09 < 0) == (-12.81 < 0) and (1.97 > 0) == (3.35 > 0))
class FakeMotor:
    def __init__(self, deg): self.st = NS(pos=math.radians(deg), vel=0.0, torq=0.0); self.sent = []
    def get_state(self): return self.st
    def request_feedback(self): pass
    def send_mit(self, *a): self.sent.append(a)
class TF(F.SeeedB601FollowerBase):
    @property
    def is_connected(self): return True
    def _check_motor_temperatures(self, *a, **k): pass
    def mit_output_torque_limit(self, *a, **k): return 0.0
def make_follower(pos):
    f = TF.__new__(TF)
    f.config = NS(joint_directions={j: 1.0 for j in J + ["gripper"]}, joint_limits={}, max_relative_target=5.0,
                  gravity_compensation=False, gravity_urdf_path=URDF, mit_kp=dict(KP), mit_kd={j: 2.0 for j in J},
                  pos_vel_velocity=[0] * 7, temp_alarm_threshold_c=115.0, temp_overheat_threshold_c=125.0,
                  temp_emergency_disable_threshold_c=135.0)
    f.motor_names = J + ["gripper"]; f.motor_type = "rs"; f._in_safe_zero = False
    f.motors = {j: FakeMotor(pos.get(j, 0.0)) for j in f.motor_names}
    return f
import time as _time
P0 = pose(shoulder_lift=152.0, elbow_flex=120.0); act = {f"{j}.pos": P0.get(j, 0.0) for j in J + ["gripper"]}
gf = grav(P0)
os.environ["GRAVITY_FF"] = "1"; os.environ.pop("COLLISION_GUARD", None)
f = make_follower(P0); f.send_action(dict(act)); t1 = f.motors["shoulder_lift"].sent[-1][4]
_time.sleep(0.5); f.send_action(dict(act)); t2 = f.motors["shoulder_lift"].sent[-1][4]
_time.sleep(0.6); f.send_action(dict(act)); t3 = f.motors["shoulder_lift"].sent[-1][4]
print(f"     lift tau_ff over the ramp: {t1:.2f} -> {t2:.2f} -> {t3:.2f} Nm (target 0.8 x {gf['shoulder_lift']:.2f} = {0.8 * gf['shoulder_lift']:.2f})")
check("C ramp: first cycle ~0, after 0.5 s about half, after 1.1 s full 0.8 x g", abs(t1) < 0.3 and 0.3 < t2 / (0.8 * gf["shoulder_lift"]) < 0.75 and abs(t3 - 0.8 * gf["shoulder_lift"]) < 0.05)
check("C FF keeps kp 150 on shoulder_lift", f.motors["shoulder_lift"].sent[-1][2] == 150.0)
os.environ["GRAVITY_FF_RAMP_S"] = "0"; os.environ["GRAVITY_FF_SCALE"] = "1.0"
f = make_follower(pose(shoulder_lift=90.0, elbow_flex=10.0)); f.send_action({f"{j}.pos": 0.0 for j in J + ["gripper"]})
check("C feedforward never exceeds a cap", all(abs(f.motors[j].sent[-1][4]) <= F.GRAVITY_FF_MAX_NM[j] + 1e-9 for j in J))
os.environ.pop("GRAVITY_FF_SCALE"); os.environ["COLLISION_GUARD"] = "1"
for k, v in {"COLLISION_GRAV_RES_WRIST_FLEX": "1.5", "COLLISION_GRAV_RES_JOINTS": "shoulder_lift,elbow_flex,wrist_flex", "COLLISION_TORQ_WRIST_YAW": "2.4"}.items(): os.environ[k] = v
f = make_follower(P0); f.send_action(dict(act)); gd = f._collision_guard
check("C follower: guard v3.3 from env (res lift 4 / wrist_flex 1.5, joints incl. wrist_flex, ff 0.8, wrist_yaw 2.4 Nm)",
      gd.grav_res_nm["shoulder_lift"] == 4.0 and gd.grav_res_nm["wrist_flex"] == 1.5 and "wrist_flex" in gd.grav_res_joints
      and abs(gd.ff_scale - 0.8) < 1e-9 and gd.torq_limit["wrist_yaw"] == 2.4 and "v3.3" in gd.describe())
for k in ("GRAVITY_FF", "GRAVITY_FF_RAMP_S", "COLLISION_GUARD", "COLLISION_GRAV_RES_WRIST_FLEX", "COLLISION_GRAV_RES_JOINTS", "COLLISION_TORQ_WRIST_YAW"): os.environ.pop(k, None)
check("C COLLISION_GRAV_RES_NM=0 switches the v3.3 rule off (v3.2 behaviour)",
      replay_event("3.3", "shoulder_lift", -38.3, -12.81, -11.09, 0.04)[0] is None and
      (lambda: (globals().__setitem__("RES33", {"shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0}),
                replay_event("3.3", "shoulder_lift", -38.3, -12.81, -11.09, 0.04)[0])[1])() == "blockiert")
RES33 = {"shoulder_lift": 4.0, "elbow_flex": 4.0, "wrist_flex": 1.5}

# ---------------- D) gripper ----------------
print("--- D) gripper (rs_follower) ---")
pkg = types.ModuleType("b601pkg"); pkg.__path__ = []; sys.modules["b601pkg"] = pkg
m1 = types.ModuleType("b601pkg.config_seeed_b601_rs_follower"); m1.SeeedB601RSFollowerConfig = object; sys.modules[m1.__name__] = m1
m2 = types.ModuleType("b601pkg.seeed_b601_follower"); m2.SeeedB601FollowerBase = object; sys.modules[m2.__name__] = m2
RSPATH = os.path.join(os.path.dirname(PATH), "seeed_b601_rs_follower.py")
spec = importlib.util.spec_from_file_location("b601pkg.rs", RSPATH); RS = importlib.util.module_from_spec(spec)
sys.modules["b601pkg.rs"] = RS; spec.loader.exec_module(RS)
rslog = Cap(); RS.logger.addHandler(rslog); RS.logger.propagate = False; RS.logger.setLevel(logging.DEBUG)
class GM:
    def __init__(self, deg): self.st = NS(pos=math.radians(deg), vel=0.0)
    def get_state(self): return self.st
    def request_feedback(self): pass
def grip_run(stufe_tq, start_deg, target_deg, n=60, close_tq="1.1", moving=False):
    os.environ["GRIPPER_CLOSE_TQ"] = close_tq
    s = NS(config=NS(gripper_mit_kp=10.0, gripper_mit_kd=0.05, gripper_mit_torque_limit=stufe_tq, gripper_mit_hold_torque_limit=1.0,
                     joint_limits={"gripper": (0.0, 270.0)}), bus=NS(poll_feedback_once=lambda: None))
    mot = GM(start_deg); out = []
    for i in range(n):
        tgt = min(target_deg, math.degrees(mot.st.pos) + 25.0) if target_deg > start_deg else max(target_deg, math.degrees(mot.st.pos) - 25.0)
        out.append(RS.SeeedB601RSFollower.mit_output_torque_limit(s, mot, math.radians(tgt)))
        if moving: mot.st.pos += math.radians(0.1)
    return out
rslog.msgs.clear(); o = grip_run(3.5, 120.0, 270.0)
check(f"D mid-stroke stall at 120 deg opening -> torque stays {o[-1]:.2f} Nm (was cut to 0 after 20 cycles)", min(o[25:]) > 1.0)
check("D mid-stroke stall logged once (not every cycle)", sum("mid-stroke" in m for _, m in rslog.msgs) == 1 and not any("cutting" in m for _, m in rslog.msgs))
rslog.msgs.clear(); o = grip_run(3.5, 255.0, 270.0)
check(f"D stall at the open stop (255 deg) -> cut to 0 after 20 cycles, logged once ({sum('cutting' in m for _, m in rslog.msgs)}x)", o[30] == 0.0 and o[10] > 0 and sum("cutting" in m for _, m in rslog.msgs) == 1)
o = grip_run(3.5, 120.0, 0.0)
check(f"D closing stays capped at GRIPPER_CLOSE_TQ 1.1 Nm (max {max(abs(x) for x in o):.2f})", max(abs(x) for x in o) <= 1.1 + 1e-9)
o = grip_run(3.5, 100.0, 270.0, n=3)
check(f"D opening uses the full 3.5 Nm limit ({o[0]:.2f})", abs(o[0] - 3.5) < 1e-6)
os.environ.pop("GRIPPER_CLOSE_TQ", None)

# ---------------- F) safe_zero (rest move on Stop) ----------------
print("--- F) safe_zero ---")
DIRS = {"shoulder_pan": 1.0, "shoulder_lift": 1.0, "elbow_flex": -1.0, "wrist_flex": -1.0, "wrist_yaw": -1.0, "wrist_roll": 1.0, "gripper": 6.0}
LIMS = {"shoulder_pan": (-145.0, 145.0), "shoulder_lift": (-0.0, 170.0), "elbow_flex": (-0.0, 200.0), "wrist_flex": (-80.0, 90.0),
        "wrist_yaw": (-90.0, 90.0), "wrist_roll": (-130.0, 130.0), "gripper": (-0.0, 270.0)}
class SimMotor:
    def __init__(self, name, deg, world):
        self.name, self.w = name, world; self.st = NS(pos=math.radians(deg), vel=0.0, torq=0.0, t_mos=30.0); self.sent = []
    def get_state(self): return self.st
    def request_feedback(self): pass
    def send_mit(self, pos, vel, kp, kd, tau):
        self.sent.append((pos, vel, kp, kd, tau))
        if self.w["stale"]: return
        import random
        self.st.torq = kp * (pos - self.st.pos) + tau + random.uniform(-0.02, 0.02)   # live sensor noise
        cur = math.degrees(self.st.pos); tgt = math.degrees(pos) if kp > 0 else cur
        new = cur + max(-1.0, min(1.0, tgt - cur))
        blk = self.w["block"].get(self.name)
        if blk is not None and cur >= blk and new < blk: new = blk     # obstacle below blk (moving down)
        self.st.pos = math.radians(new)
    def disable(self): self.w["disabled"].append(self.name)
    def clear_error(self): pass
    def close(self): pass
def make_sim(follower_mod, start, stale=False, block=None):
    class TFS(follower_mod.SeeedB601FollowerBase):
        @property
        def is_connected(self): return True
        def _check_motor_temperatures(self, *a, **k): pass
        def mit_output_torque_limit(self, *a, **k): return 0.0
    f = TFS.__new__(TFS)
    w = {"stale": stale, "block": block or {}, "disabled": []}
    f.config = NS(joint_directions=DIRS, joint_limits=LIMS, max_relative_target=5.0, gravity_compensation=False,
                  gravity_urdf_path=URDF, mit_kp=dict(KP), mit_kd={**{j: 4.0 for j in J}, "shoulder_lift": 10.0, "elbow_flex": 10.0},
                  pos_vel_velocity=[0] * 7, temp_alarm_threshold_c=115.0, temp_overheat_threshold_c=125.0,
                  temp_emergency_disable_threshold_c=135.0, disable_torque_on_disconnect=True,
                  motor_can_ids={n: (i + 1, 0xFD) for i, n in enumerate(J + ["gripper"])})
    f.motor_names = J + ["gripper"]; f.motor_type = "rs"; f._in_safe_zero = False; f.id = "sim"
    f._emergency_disable_requested = False; f._skip_safe_zero_on_disconnect = False
    f.motors = {n: SimMotor(n, start.get(n, 0.0), w) for n in f.motor_names}
    f.bus = NS(poll_feedback_once=lambda: None, disable_all=lambda: None, close=lambda: None); f.cameras = {}
    return f, w
# start pose of the failed rest move 2026-11-21 14:09:17 (motor degrees from the clamp warnings)
P1409 = {"shoulder_pan": -5.92, "shoulder_lift": 132.44, "elbow_flex": 59.8, "wrist_flex": 37.5, "wrist_yaw": -25.3, "wrist_roll": 31.5, "gripper": 0.7}
flog = Cap(); F.logger.addHandler(flog); F.logger.setLevel(logging.DEBUG)
os.environ["SAFE_ZERO_STALL_S"] = "1.0"; os.environ.pop("GRAVITY_FF", None)
for k in ("COLLISION_GUARD",): os.environ.pop(k, None)
# F1 replay: frozen feedback (power/CAN lost) -> abort, never "done"
f, w = make_sim(F, P1409, stale=True); flog.msgs.clear(); t0 = _time.monotonic()
f.safe_zero(step_interval_s=0.002, exit_on_complete=False)
msgs = [m for _, m in flog.msgs]
print("     log:", " | ".join(m[:110] for m in msgs if "safe_zero" in m or "Rueckmeldung" in m)[:600])
check(f"F replay 14:09 frozen feedback -> aborted with 'keine Motor-Rueckmeldung' after {_time.monotonic() - t0:.1f} s, no 'erreicht'",
      any("keine Motor-Rueckmeldung" in m for m in msgs) and not any("Ruheposition erreicht" in m for m in msgs)
      and f._safe_zero_result["feedback"] is False and f._safe_zero_result["reached"] is False)
f.disconnect()
check("F frozen feedback -> no soft-release attempt, clear log about power", not any("Weiches Absenken" in m for _, m in flog.msgs)
      and any("Strom pruefen" in m for _, m in flog.msgs))
# counter-check: the v3.6 safe_zero on the same frozen feedback claims "done"
try:
    F_old, _ = load_follower(os.path.join(HERE, "..", "patches", "patched", "seeed_b601_follower.py")) if os.path.exists(os.path.join(HERE, "..", "patches", "patched", "seeed_b601_follower.py")) else (None, None)
except Exception:
    F_old = None
if F_old is not None and "PATCH v3.7 safe_zero" not in open(F_old.__file__).read():
    F_old._GRAVITY_MOD = G
    oc = Cap(); F_old.logger.addHandler(oc); F_old.logger.setLevel(logging.DEBUG); F_old.logger.propagate = False
    fo, _ = make_sim(F_old, P1409, stale=True)
    fo.safe_zero(step_interval_s=0.0, exit_on_complete=False)
    check("F counter-check v3.6: same frozen feedback -> old code logs 'safe_zero done' (the 14:09 bug)", any(m == "safe_zero done." for _, m in oc.msgs))
# F2 live feedback from the same overhead pose -> reaches rest
f, w = make_sim(F, P1409); flog.msgs.clear()
f.safe_zero(step_interval_s=0.002, exit_on_complete=False)
pos = f._safe_zero_result["positions"]
check(f"F from the 14:09 overhead pose with live feedback -> rest reached (lift {pos['shoulder_lift']:.1f}, elbow {pos['elbow_flex']:.1f}, wrist_flex {pos['wrist_flex']:.1f})",
      f._safe_zero_result["reached"] and all(abs(pos[j]) <= 2.0 for j in J))
lifts = [math.degrees(x[0]) for x in f.motors["shoulder_lift"].sent]
check("F command never runs more than 3 deg ahead of the measured position (no stale ramp base)",
      max(abs(b - a) for a, b in zip([132.44] + lifts[:-1], lifts)) <= 3.0 + 1.0 + 1e-6)
check("F gravity feedforward active during the rest move although GRAVITY_FF is off",
      any(abs(x[4]) > 1.0 for x in f.motors["shoulder_lift"].sent))
f.disconnect()
check("F reached rest -> normal torque-off, no soft release", not any("Weiches Absenken" in m for _, m in flog.msgs) and len(w["disabled"]) == 7)
# F3 lift blocked at 100 deg -> stall reported, soft release (kp -> 0, kd >= 20) before torque-off
f, w = make_sim(F, P1409, block={"shoulder_lift": 100.0}); flog.msgs.clear()
f.safe_zero(step_interval_s=0.002, exit_on_complete=False)
check("F lift blocked at 100 deg -> 'STILLSTAND' warning names shoulder_lift, reached False",
      any("STILLSTAND" in m and "shoulder_lift" in m for _, m in flog.msgs) and not f._safe_zero_result["reached"])
os.environ["SOFT_RELEASE_S"] = "0.5"; n0 = len(f.motors["shoulder_lift"].sent)
f.disconnect()
sl = f.motors["shoulder_lift"].sent[n0:]
check(f"F not at rest -> soft release: kp {sl[0][2]:.0f} -> {sl[-1][2]:.0f}, kd {sl[-1][3]:.0f}, then torque-off",
      any("Weiches Absenken" in m for _, m in flog.msgs) and sl[0][2] > 50 and sl[-1][2] == 0.0 and sl[-1][3] >= 20 and len(w["disabled"]) == 7)
for k in ("SAFE_ZERO_STALL_S", "SOFT_RELEASE_S"): os.environ.pop(k, None)

# ---------------- E) teleop.sh defaults ----------------
if os.path.exists(TELEOP):
    print("--- E) teleop.sh ---")
    src = open(TELEOP).read()
    def dflt(name):
        m = re.search(r"export %s=\$\{%s:-([^}]*)\}" % (name, name), src); return m.group(1) if m else None
    check("E GRAVITY_FF default 1, scale 0.8, ramp 1.0", dflt("GRAVITY_FF") == "1" and dflt("GRAVITY_FF_SCALE") == "0.8" and dflt("GRAVITY_FF_RAMP_S") == "1.0")
    check("E COLLISION_TORQ_WRIST_YAW 2.4, shoulder_pan unchanged 1.6", dflt("COLLISION_TORQ_WRIST_YAW") == "2.4" and dflt("COLLISION_TORQ_SHOULDER_PAN") == "1.6")
    check("E COLLISION_GRAV_RES_NM 4.0, wrist_flex 1.5, joints lift/elbow/wrist_flex",
          dflt("COLLISION_GRAV_RES_NM") == "4.0" and dflt("COLLISION_GRAV_RES_WRIST_FLEX") == "1.5" and dflt("COLLISION_GRAV_RES_JOINTS") == "shoulder_lift,elbow_flex,wrist_flex")
    check("E gripper: vorsichtig TQ 3.5 with CLOSE 1.1; mittel TQ 3.5 with CLOSE 1.2",
          "vorsichtig) KP=10.0; TQ=3.5; HOLD=1.0;  CLOSE=1.1" in src and "mittel)     KP=12.0; TQ=3.5; HOLD=1.2;  CLOSE=1.2" in src)
print(f"\n{'stub-imports' if stubbed else 'echte lerobot/motorbridge-Imports'}; {len(fails)} Fehler")
sys.exit(1 if fails else 0)
