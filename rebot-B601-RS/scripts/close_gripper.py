import time, math
import lerobot_robot_seeed_b601 as P
cfgcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
robcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollower")][0]
r = robcls(cfgcls(port="can0", id="follower"))
r.connect(calibrate=False)
r._skip_safe_zero_on_disconnect = True
m = r.motors["gripper"]
def rd():
    m.request_feedback(); time.sleep(0.02)
    try: r.bus.poll_feedback_once()
    except Exception: pass
    s = m.get_state(); return math.degrees(s.pos) if s else None
print("start", rd())
for i in range(60):
    m.send_mit(0.0, 0.0, 8.0, 0.3, 0.0)
    time.sleep(0.03)
    p = rd()
    if i % 10 == 0: print(i, p)
print("end", rd())
m.send_mit(0.0,0.0,0.0,1.5,0.0)
r.bus.disable_all(); r.disconnect()
