"""Switch all follower motors torque-free (safe final state)."""
import logging; logging.disable(logging.CRITICAL)
import lerobot_robot_seeed_b601 as P
cfgcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
robcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollower")][0]
r = robcls(cfgcls(port="can0", id="follower"))
try:
    r.connect(calibrate=False)
except Exception as e:
    print("Motoren nicht erreichbar (Strom aus?):", e); raise SystemExit(0)
r._skip_safe_zero_on_disconnect = True
r.bus.disable_all()
try: r.disconnect()
except Exception: pass
print("Alle Motoren aus.")
