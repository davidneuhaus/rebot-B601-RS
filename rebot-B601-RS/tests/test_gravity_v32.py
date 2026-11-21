"""Offline tests for guard v3.2 (gravity-aware collision guard + optional gravity feedforward).
No hardware, no motor commands. Usage:
  ~/humanoid/.venv/bin/python test_gravity_v32.py [path/to/seeed_b601_follower.py]
Check labels are printed in German for David (like test_collision_guard.py)."""
import logging, math, os, sys
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/humanoid/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601/seeed_b601_follower.py")
sys.path.insert(0, HERE)
from _load import load_follower
F, stubbed = load_follower(PATH)
# the follower imports ".gravity" relatively -> resolve against the installed plugin package
if not getattr(F, "__package__", None):
    F.__package__ = "lerobot_robot_seeed_b601"
import lerobot_robot_seeed_b601.config_seeed_b601_rs_follower as RSC

class Cap(logging.Handler):
    def __init__(self): super().__init__(); self.msgs = []
    def emit(self, r): self.msgs.append((r.levelname, r.getMessage()))
cap = Cap(); lg = logging.getLogger("gravtest"); lg.addHandler(cap); lg.setLevel(logging.DEBUG); lg.propagate = False
fails = []
def check(name, cond):
    print(("OK   " if cond else "FAIL ") + name)
    if not cond: fails.append(name)

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
DT = 1 / 30
check("v3.2: Patch erkannt (guard v3.2 im Follower)", "guard v3.2" in open(PATH).read())

# ---- 1) gravity model through the follower helper (same code path as on the robot) ----
cfg_cls = [c for c in vars(RSC).values() if isinstance(c, type) and hasattr(c, "__dataclass_fields__")
           and "gravity_urdf_path" in c.__dataclass_fields__][0]
urdf = cfg_cls.__dataclass_fields__["gravity_urdf_path"].default
fake = NS(motor_type="rs", motor_names=J + [F.FOLLOWER_GRIPPER_MOTOR],
          config=NS(gravity_urdf_path=urdf, gravity_compensation=False,
                    mit_kp={"shoulder_pan": 20, "shoulder_lift": 150, "elbow_flex": 150, "wrist_flex": 40, "wrist_yaw": 20, "wrist_roll": 50}))
Base = F.SeeedB601FollowerBase
fake._gravity_model_get = lambda: Base._gravity_model_get(fake)
check(f"v3.2: URDF-Modell laedt ({os.path.basename(urdf)})", fake._gravity_model_get() is not None)
pose = {"shoulder_pan": 0.0, "shoulder_lift": 152.0, "elbow_flex": 120.0, "wrist_flex": 0.0, "wrist_yaw": 0.0, "wrist_roll": 0.0}
g = Base._gravity_torques_deg(fake, pose)
print("     g(q) an Pose [0,152,120,0,0,0]:", ", ".join(f"{j}={g.get(j, float('nan')):.2f}" for j in J))
check("v3.2: Schulter heben bei 152/120 = ca. -14.2 Nm (Log: -14.9)", abs(g["shoulder_lift"] - (-14.24)) < 0.3)
check("v3.2: Ellbogen bei 152/120 = ca. 6.9 Nm", abs(g["elbow_flex"] - 6.87) < 0.3)
g0 = Base._gravity_torques_deg(fake, {j: 0.0 for j in J})
check("v3.2: Ruheposition (alles 0) braucht kaum Haltemoment (<2 Nm Schulter)", abs(g0["shoulder_lift"]) < 2.0)
check("v3.2: fehlende Gelenke -> kein Modellwert (leer)", Base._gravity_torques_deg(fake, {"shoulder_lift": 10.0}) == {})
bad = NS(motor_type="dm", motor_names=fake.motor_names, config=fake.config)
check("v3.2: DM-Arm -> kein Modell, Schutz faellt auf v3.1 zurueck", Base._gravity_model_get(bad) is None)

# ---- 2) env switches ----
for k in ("GRAVITY_FF", "GRAVITY_FF_SCALE", "COLLISION_GRAVITY"): os.environ.pop(k, None)
check("v3.2: GRAVITY_FF ist standardmaessig AUS", F.gravity_ff_enabled() is False)
check("v3.2: GRAVITY_FF_SCALE Standard 0.8", abs(F.gravity_ff_scale() - 0.8) < 1e-9)
os.environ["GRAVITY_FF_SCALE"] = "1.7"; check("v3.2: GRAVITY_FF_SCALE wird auf max 1.0 begrenzt", F.gravity_ff_scale() == 1.0)
os.environ.pop("GRAVITY_FF_SCALE")
check("v3.2: Schwerkraft-Schutz standardmaessig AN", F.guard_gravity_enabled() is True)
os.environ["COLLISION_GRAVITY"] = "0"; check("v3.2: COLLISION_GRAVITY=0 schaltet ihn aus", F.guard_gravity_enabled() is False)
os.environ.pop("COLLISION_GRAVITY")

# ---- 3) the 20:57:39 false freeze, replayed with the model ----
KP = {"shoulder_pan": 20, "shoulder_lift": 150, "elbow_flex": 150, "wrist_flex": 40, "wrist_yaw": 20, "wrist_roll": 50}
TQ = {"shoulder_pan": 1.6, "wrist_yaw": 1.25}
def guard(grav_on=True, ff=0.0, **kw):
    cap.msgs.clear()
    extra = dict(kp=KP, clamp_deg=5.0, ff_scale=ff) if grav_on else {}
    gg = F.CollisionGuard(J, log=lg, torq_limit=TQ, torq_cycles=6, torq_min_lead_deg=3.0, torq_max_progress_deg=0.5,
                          err_deg={"shoulder_pan": 6, "shoulder_lift": 10, "elbow_flex": 10, "wrist_flex": 10, "wrist_yaw": 8, "wrist_roll": 15},
                          **extra, **kw)
    gg.grace_s = 0
    return gg
def run(gg, goal, pres, torq, n=40, use_grav=True):
    t = 0.0; outs = []
    gv = Base._gravity_torques_deg(fake, pres) if use_grav else None
    for i in range(n):
        outs.append(gg.update(goal, pres, torq, t, grav=gv)); t += DT
    return outs
pres = dict(pose); goal = dict(pose); goal["shoulder_lift"] = 152.0 - 11.3      # leader 11.3 deg "higher"
torq = {j: 0.0 for j in J}; torq["shoulder_lift"] = -14.89
o31 = run(guard(grav_on=False), goal, pres, torq, use_grav=False)
check("Gegenprobe v3.1: Durchhang bei 152/120 friert ein (der Fehlalarm von 20:57)", any(o is not None for o in o31))
o32 = run(guard(), goal, pres, torq)
check("v3.2: derselbe Durchhang friert NICHT ein (Schwerkraft > kp*5 deg, Arm kann nicht folgen)", all(o is None for o in o32))
check("v3.2: dabei einmal Hinweis 'Schwerkraft ... Tipp: GRAVITY_FF=1' im Log",
      sum("GRAVITY_FF=1" in m for _, m in cap.msgs) == 1)

# ---- 4) real obstacles are still detected ----
# 4a) feedforward 0.8 -> only 2.8 Nm left for the P-term: an obstacle (12 deg lag, no progress) still freezes
o = run(guard(ff=0.8), {**pose, "shoulder_lift": 140.0}, pres, torq)
check("v3.2 mit GRAVITY_FF 0.8: Hindernis (12 deg Rueckstand gegen Schwerkraft) friert ein", any(x is not None for x in o))
# 4b) lag in the direction gravity helps (no extra tolerance) -> freezes like v3.1
o = run(guard(), {**pose, "shoulder_lift": 164.0}, pres, {j: 0.0 for j in J})
check("v3.2: Hindernis in Schwerkraft-Richtung (12 deg) friert ein wie v3.1", any(x is not None for x in o))
# 4c) near rest pose (little gravity) behaviour is unchanged: elbow blocked 12 deg -> freeze
rest = {j: 0.0 for j in J}; rest["elbow_flex"] = 30.0
o = run(guard(), {**rest, "elbow_flex": 42.0}, rest, {j: 0.0 for j in J})
check("v3.2: Ellbogen-Blockade nahe Ruheposition friert ein (unveraendert)", any(x is not None for x in o))

# ---- 5) residual torque trigger ----
gg = guard(torq_limit_override=None) if False else guard()
gg.torq_limit["wrist_flex"] = 2.0
p = dict(rest); gl = {**p, "wrist_flex": 4.0}
t = 0.0; hit = False
for i in range(20):
    hit |= gg.update(gl, p, {**{j: 0.0 for j in J}, "wrist_flex": 2.3}, t, grav={**{j: 0.0 for j in J}, "wrist_flex": 2.3}) is not None; t += DT
check("v3.2: Kraft-Trigger nutzt Rest-Drehmoment: 2.3 Nm gemessen = 2.3 Nm Schwerkraft -> kein Alarm", not hit)
gg = guard(); gg.torq_limit["wrist_flex"] = 2.0; t = 0.0; hit = False
for i in range(20):
    hit |= gg.update(gl, p, {**{j: 0.0 for j in J}, "wrist_flex": 4.6}, t, grav={**{j: 0.0 for j in J}, "wrist_flex": 2.3}) is not None; t += DT
check("v3.2: 4.6 Nm gemessen bei 2.3 Nm Schwerkraft (Rest 2.3 > 2.0) -> Alarm", hit)
gg = guard(grav_on=False); gg.torq_limit["wrist_flex"] = 2.0; t = 0.0; hit = False
for i in range(20):
    hit |= gg.update(gl, p, {**{j: 0.0 for j in J}, "wrist_flex": 2.3}, t) is not None; t += DT
check("Gegenprobe ohne Modell: 2.3 Nm loest aus (v3.1-Verhalten)", hit)

# ---- 6) without grav the guard is exactly v3.1 ----
check("v3.2: from_env ohne kp -> kein Schwerkraft-Teil (v3.1-Verhalten)",
      all(v == 0 for v in F.CollisionGuard.from_env(J, log=lg).kp.values()))
check("v3.2: describe() nennt Schwerkraft-Modell wenn aktiv", "Schwerkraft-Modell" in guard(ff=0.8).describe())

print(f"\n{'stub-imports' if stubbed else 'echte lerobot/motorbridge-Imports'}; {len(fails)} Fehler")
sys.exit(1 if fails else 0)
