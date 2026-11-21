import time, math, sys
import lerobot_robot_seeed_b601 as P
cfgcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
robcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollower")][0]
r = robcls(cfgcls(port="can0", id="follower"))
r.connect(calibrate=False)
r._skip_safe_zero_on_disconnect = True
m = r.motors["gripper"]
def rd():
    for _ in range(5):
        m.request_feedback(); time.sleep(0.05)
        try: r.bus.poll_feedback_once()
        except Exception: pass
    s = m.get_state(); return math.degrees(s.pos) if s else None
r.bus.disable_all(); time.sleep(0.2)
print("before:", rd())
if "--set" in sys.argv:
    m.set_zero_position(); time.sleep(0.5)
    try: m.store_parameters(); print("stored")
    except Exception as e: print("store:", e)
    time.sleep(0.5)
    print("after:", rd())
r.bus.disable_all()
r.disconnect()
