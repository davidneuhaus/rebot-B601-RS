"""Offline tests for guard v3.2 (gravity-aware collision guard + optional gravity feedforward).
No hardware, no motor commands. Usage:
  ~/humanoid/.venv/bin/python test_gravity_v3_2.py [path/to/seeed_b601_follower.py] [plugin_dir]
plugin_dir must contain gravity.py and assets/00-arm-rs_asm-v3.urdf (default: the installed plugin)."""
import importlib.util, logging, math, os, sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
PLUGIN = os.path.expanduser("~/humanoid/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601")
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PLUGIN, "seeed_b601_follower.py")
PDIR = sys.argv[2] if len(sys.argv) > 2 else PLUGIN
URDF = os.path.join(PDIR, "assets", "00-arm-rs_asm-v3.urdf")

sys.path.insert(0, HERE)
from _load import load_follower
F, stubbed = load_follower(PATH)
spec = importlib.util.spec_from_file_location("b601_gravity_test", os.path.join(PDIR, "gravity.py"))
G = importlib.util.module_from_spec(spec); spec.loader.exec_module(G)
F._GRAVITY_MOD = G
import numpy as np
MODEL = G.load_dynamics_model(urdf_path=URDF)

class Cap(logging.Handler):
    def __init__(self): super().__init__(); self.msgs = []
    def emit(self, r): self.msgs.append((r.levelname, r.getMessage()))
cap = Cap(); lg = logging.getLogger("gravtest"); lg.addHandler(cap); lg.setLevel(logging.DEBUG); lg.propagate = False

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
KP = {"shoulder_pan": 20.0, "shoulder_lift": 150.0, "elbow_flex": 150.0, "wrist_flex": 40.0, "wrist_yaw": 20.0, "wrist_roll": 50.0}
ERR = {"shoulder_pan": 6.0, "shoulder_lift": 10.0, "elbow_flex": 10.0, "wrist_flex": 10.0, "wrist_yaw": 8.0, "wrist_roll": 15.0}
TQ = {"shoulder_pan": 1.6, "wrist_yaw": 1.25}   # teleop.sh defaults
DT = 1 / 30
fails = []
def check(name, cond):
    print(("OK   " if cond else "FAIL ") + name)
    if not cond: fails.append(name)

def grav(pos_deg):
    q = np.zeros(MODEL.nq)
    for i, j in enumerate(J):
        q[i] = math.radians(pos_deg[j])
    tau = G.compute_generalized_gravity(MODEL, q)
    return {j: float(tau[i]) for i, j in enumerate(J)}

def pose(**kw):
    p = {j: 0.0 for j in J}; p.update(kw); return p

def guard(use_grav=True, ff=0.0, torq_limit=None, **kw):
    cap.msgs.clear()
    args = dict(err_deg=ERR, cycles=6, grace_s=0.0, torq_limit=TQ if torq_limit is None else torq_limit,
                torq_cycles=6, torq_min_lead_deg=3.0, torq_max_progress_deg=0.5, log=lg)
    args.update(kw)
    if use_grav:
        args.update(kp=KP, clamp_deg=5.0, ff_scale=ff)
    return F.CollisionGuard(J, **args)

def sim(joint, start, leader, use_grav=True, ff=0.0, true_scale=1.0, obstacle=None, vmax=1.5, clamp=5.0, **gkw):
    """Quasi-static one-joint simulation of the real loop: leader goal -> guard -> max_relative_target clamp ->
    MIT P-control with optional feedforward against the TRUE gravity (= true_scale * URDF model).
    Returns (frozen_cycle or None, final present)."""
    g = guard(use_grav=use_grav, ff=ff, **gkw)
    pres = dict(start); cmd = pres[joint]
    for i, L in enumerate(leader):
        gm = grav(pres)
        ffj = ff * gm[joint]
        torq = {j: true_scale * gm[j] for j in J}
        torq[joint] = KP[joint] * math.radians(cmd - pres[joint]) + ffj
        goal = dict(start); goal[joint] = L
        out = g.update(goal, pres, torq, i * DT, grav=gm if use_grav else None)
        if out is not None and g.frozen is not None:
            return i, pres
        tgt = (out or goal)[joint]
        cmd = pres[joint] + max(-clamp, min(clamp, tgt - pres[joint]))
        gtrue = true_scale * gm[joint]
        p_eq = cmd - math.degrees((gtrue - ffj) / KP[joint])
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

# --- 1) gravity model sanity ------------------------------------------------------------
gf = grav(pose(shoulder_lift=152.0, elbow_flex=120.0))
check(f"model: freeze pose 20:57:39 (lift 152, elbow 120) g_lift={gf['shoulder_lift']:.2f} Nm vs measured -14.89 Nm (within 1.5 Nm)",
      abs(gf["shoulder_lift"] - (-14.89)) < 1.5)
check("model: shoulder_pan (vertical axis) never carries gravity",
      all(abs(grav(pose(shoulder_pan=a, shoulder_lift=b, elbow_flex=c))["shoulder_pan"]) < 1e-6
          for a in (-90, 0, 60) for b in (0, 90, 150) for c in (0, 100)))
check("model: max |g| lift <= 16 Nm, elbow <= 8 Nm over the workspace (fits the FF caps)",
      max(abs(grav(pose(shoulder_lift=b, elbow_flex=c, wrist_flex=d))["shoulder_lift"]) for b in range(0, 171, 10) for c in range(0, 201, 20) for d in (-60, 0, 60)) <= 16.0
      and max(abs(grav(pose(shoulder_lift=b, elbow_flex=c, wrist_flex=d))["elbow_flex"]) for b in range(0, 171, 10) for c in range(0, 201, 20) for d in (-60, 0, 60)) <= 8.0)
check(f"model: v3.1 root cause – kp 150 x 5 deg clamp = {150 * math.radians(5):.1f} Nm < {abs(gf['shoulder_lift']):.1f} Nm gravity overhead",
      150 * math.radians(5) < abs(gf["shoulder_lift"]))

# --- 2) overhead / lifting must NOT freeze (FF off) ---------------------------------------
start = pose(shoulder_lift=160.0, elbow_flex=120.0)
leader = ramp(160.0, 130.0, 0.8, hold=150)
fr31, _ = sim("shoulder_lift", start, leader, use_grav=False, true_scale=0.9)
check(f"counter-check v3.1: overhead return (lift 160->130, elbow 120) freezes falsely (cycle {fr31})", fr31 is not None)
for ts in (0.9, 1.0, 1.1):
    fr, p = sim("shoulder_lift", start, leader, true_scale=ts)
    check(f"v3.2: overhead return, true gravity {ts:.1f}x model -> no freeze (lift ends at {p['shoulder_lift']:.1f})", fr is None)
check("v3.2: saturation warning names the joint once ('Schwerkraft: Schulter heben/senken')",
      sum("Schwerkraft: Schulter heben/senken" in m for _, m in cap.msgs) <= 1)

start = pose(shoulder_lift=20.0, elbow_flex=150.0)
for ts in (1.0, 1.15):
    fr31, _ = sim("shoulder_lift", start, ramp(20.0, 120.0, 1.0), use_grav=False, true_scale=ts, vmax=0.6)
    fr, p = sim("shoulder_lift", start, ramp(20.0, 120.0, 1.0), true_scale=ts, vmax=0.6)
    check(f"v3.2: slow lift 20->120 under load (true {ts:.2f}x) -> no freeze (v3.1: {'freeze' if fr31 is not None else 'ok'}; ends {p['shoulder_lift']:.1f})", fr is None)

start = pose(shoulder_lift=90.0, elbow_flex=90.0, wrist_flex=-60.0)
fr, p = sim("wrist_flex", start, ramp(-60.0, 60.0, 1.0), vmax=0.5, true_scale=1.1)
check(f"v3.2: wrist_flex kp 40 slow lift under load -> no freeze (ends {p['wrist_flex']:.1f})", fr is None)

for ff in (0.8,):
    fr, p = sim("shoulder_lift", pose(shoulder_lift=160.0, elbow_flex=120.0), ramp(160.0, 130.0, 0.8, hold=150), ff=ff, true_scale=1.1)
    check(f"v3.2 + GRAVITY_FF {ff}: overhead return follows the leader, no freeze (lift {p['shoulder_lift']:.1f})", fr is None and abs(p["shoulder_lift"] - 130.0) < 4.0)
fr, p = sim("shoulder_lift", pose(shoulder_lift=20.0, elbow_flex=150.0), ramp(20.0, 120.0, 1.0), ff=0.8, true_scale=0.85)
check(f"v3.2 + GRAVITY_FF 0.8, model 15% too heavy: lift still no freeze (ends {p['shoulder_lift']:.1f})", fr is None)

# --- 3) real blocks must STILL freeze ------------------------------------------------------
fr, _ = sim("shoulder_lift", pose(shoulder_lift=40.0, elbow_flex=90.0), ramp(40.0, 120.0, 1.0), obstacle=(-1e9, 70.0))
check(f"v3.2: obstacle at lift 70 while lifting (FF off) -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("shoulder_lift", pose(shoulder_lift=40.0, elbow_flex=90.0), ramp(40.0, 120.0, 1.0), obstacle=(-1e9, 70.0), ff=0.8)
check(f"v3.2 + FF: obstacle at lift 70 while lifting -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("shoulder_lift", pose(shoulder_lift=160.0, elbow_flex=120.0), ramp(160.0, 110.0, 0.8), obstacle=(145.0, 1e9), ff=0.8)
check(f"v3.2 + FF: obstacle at lift 145 on the overhead return -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("elbow_flex", pose(shoulder_lift=60.0, elbow_flex=40.0), ramp(40.0, 140.0, 1.0), obstacle=(-1e9, 80.0))
check(f"v3.2: elbow obstacle at 80 -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("wrist_flex", pose(shoulder_lift=90.0, elbow_flex=90.0), ramp(0.0, 60.0, 1.0), obstacle=(-1e9, 20.0))
check(f"v3.2: wrist_flex obstacle at 20 -> freeze (cycle {fr})", fr is not None)
fr, _ = sim("shoulder_lift", pose(shoulder_lift=120.0, elbow_flex=60.0), ramp(120.0, 60.0, 1.0), obstacle=(95.0, 1e9))
check(f"v3.2: block in the gravity-assisted direction -> freeze like v3.1 (cycle {fr})", fr is not None)

# --- 4) residual torque triggers -------------------------------------------------------------
def tq_case(j, tq, g, lead=5.0, prog=0.05, n=12, **kw):
    gd = guard(**kw); p = pose(); t = 0.0
    for i in range(n):
        goal = dict(p); goal[j] = p[j] + lead
        gr = {k: 0.0 for k in J}; gr[j] = g
        if gd.update(goal, p, {j: tq}, t, grav=gr) is not None: return i + 1
        p = dict(p); p[j] += prog; t += DT
    return None
check("v3.2: wrist_yaw 1.40 Nm measured but 0.50 Nm is gravity (residual 0.90) -> no alarm", tq_case("wrist_yaw", 1.40, 0.50) is None)
check("v3.2: wrist_yaw 1.40 Nm with -0.30 Nm gravity (residual 1.70) -> alarm after 6 cycles", tq_case("wrist_yaw", 1.40, -0.30) == 6)
check("v3.2: shoulder_pan 1.7 Nm, g=0 -> alarm as in v3.1", tq_case("shoulder_pan", 1.7, 0.0) == 6)
tl = dict(TQ, shoulder_lift=3.0)
check("v3.2 optional COLLISION_TORQ_SHOULDER_LIFT=3: -14.5 Nm overhead hold (g -14.2) -> no alarm",
      tq_case("shoulder_lift", -14.5, -14.2, lead=-5.0, torq_limit=tl) is None)
check("v3.2 optional COLLISION_TORQ_SHOULDER_LIFT=3: -14.5 Nm where g is only -9 (residual 5.5) -> alarm",
      tq_case("shoulder_lift", -14.5, -9.0, lead=-5.0, torq_limit=tl) == 6)
check("v3.1 compatibility: no grav dict -> raw torque (1.40 Nm wrist_yaw alarms)", tq_case("wrist_yaw", 1.40, 0.0, use_grav=False) == 6)

# --- 5) creep (auto-resume) with gravity --------------------------------------------------------
g = guard(auto_resume=True, creep_lead_deg=2.5, creep_lag_deg=2.0, creep_cycles=10)
p = pose(shoulder_lift=150.0, elbow_flex=120.0); t = 0.0
g.frozen = dict(p); g.hit_joint = "shoulder_lift"; g.prev_present = dict(p)
goal = dict(p); goal["shoulder_lift"] = 120.0
creep_frz = False
for i in range(200):
    gm = grav(p)
    out = g.update(goal, p, {}, t, grav=gm); t += DT
    if g.creep is not None and out is not None:
        # joint sags: cannot follow the creep target (saturated) – only moves 0.05 deg/cycle
        p = dict(p); p["shoulder_lift"] -= 0.05
    if g.n_collisions > 0: creep_frz = True; break
check("v3.2: auto-resume creep overhead (saturated, slow) -> no re-freeze", not creep_frz)

# --- 6) env switches + follower integration ---------------------------------------------------------
os.environ.pop("GRAVITY_FF", None); os.environ.pop("GRAVITY_FF_SCALE", None); os.environ.pop("COLLISION_GRAVITY", None)
check("env: GRAVITY_FF default off, scale 0.8, gravity-aware guard default on",
      not F.gravity_ff_enabled() and abs(F.gravity_ff_scale() - 0.8) < 1e-9 and F.guard_gravity_enabled())
os.environ["GRAVITY_FF_SCALE"] = "3"
check("env: GRAVITY_FF_SCALE clamped to 1.0", F.gravity_ff_scale() == 1.0)
os.environ.pop("GRAVITY_FF_SCALE")

class FakeMotor:
    def __init__(self, deg, torq=0.0):
        self.st = NS(pos=math.radians(deg), vel=0.0, torq=torq); self.sent = []
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
                  pos_vel_velocity=[0] * 7)
    f.motor_names = J + ["gripper"]; f.motor_type = "rs"; f._in_safe_zero = False
    f.motors = {j: FakeMotor(pos.get(j, 0.0)) for j in f.motor_names}
    return f

P0 = pose(shoulder_lift=152.0, elbow_flex=120.0)
act = {f"{j}.pos": P0.get(j, 0.0) for j in J + ["gripper"]}
for k in ("COLLISION_GUARD", "MIT_KP_SHOULDER_PAN"): os.environ.pop(k, None)
f = make_follower(P0); f.send_action(dict(act))
check("follower: GRAVITY_FF off -> tau_ff 0 on all arm joints", all(abs(f.motors[j].sent[-1][4]) < 1e-9 for j in J))
os.environ["GRAVITY_FF"] = "1"
f = make_follower(P0); f.send_action(dict(act))
tl_ = f.motors["shoulder_lift"].sent[-1][4]
check(f"follower: GRAVITY_FF=1 -> shoulder_lift tau_ff {tl_:.2f} = 0.8 x {gf['shoulder_lift']:.2f} Nm", abs(tl_ - 0.8 * gf["shoulder_lift"]) < 0.05)
check("follower: GRAVITY_FF keeps the tuned kp (150 for shoulder_lift)", f.motors["shoulder_lift"].sent[-1][2] == 150.0)
os.environ["GRAVITY_FF_SCALE"] = "1.0"
f = make_follower(pose(shoulder_lift=0.0, elbow_flex=160.0, wrist_flex=0.0)); f.send_action({f"{j}.pos": 0.0 for j in J + ["gripper"]})
check("follower: feedforward never exceeds the per-joint cap", all(abs(f.motors[j].sent[-1][4]) <= F.GRAVITY_FF_MAX_NM[j] + 1e-9 for j in J))
os.environ.pop("GRAVITY_FF_SCALE")
os.environ["COLLISION_GUARD"] = "1"
f = make_follower(P0); f.send_action(dict(act))
gd = f._collision_guard
check("follower: guard gets kp/clamp/ff from config+env (lift kp 150, clamp 5, ff 0.8)",
      gd.kp["shoulder_lift"] == 150.0 and gd.clamp_deg["shoulder_lift"] == 5.0 and abs(gd.ff_scale - 0.8) < 1e-9)
check("follower: guard sees gravity each cycle", abs(gd.grav.get("shoulder_lift", 0.0) - gf["shoulder_lift"]) < 1e-6)
os.environ["COLLISION_GRAVITY"] = "0"; os.environ.pop("GRAVITY_FF")
f = make_follower(P0); f.send_action(dict(act))
check("follower: COLLISION_GRAVITY=0 -> exact v3.1 guard (no kp, no gravity)",
      not any(f._collision_guard.kp.values()) and f._collision_guard.grav == {})
os.environ.pop("COLLISION_GRAVITY"); os.environ.pop("COLLISION_GUARD")
f = make_follower(P0); f.config.gravity_urdf_path = "/nonexistent.urdf"; os.environ["GRAVITY_FF"] = "1"; os.environ["COLLISION_GUARD"] = "1"
try:
    f.send_action(dict(act)); ok = all(abs(f.motors[j].sent[-1][4]) < 1e-9 for j in J)
except Exception as e:
    ok = False; print("   exception:", e)
check("follower: missing URDF -> no crash, tau_ff 0, guard falls back to v3.1", ok and not any(f._collision_guard.kp.values()))
os.environ.pop("GRAVITY_FF"); os.environ.pop("COLLISION_GUARD")

print()
print(f"{len(fails)} failures" if fails else "all v3.2 checks passed", "(stub imports)" if stubbed else "(real lerobot/motorbridge imports)")
sys.exit(1 if fails else 0)
