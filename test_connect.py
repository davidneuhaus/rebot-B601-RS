import logging, math, time
logging.basicConfig(level=logging.INFO)
import lerobot_robot_seeed_b601 as P
cfgcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
robcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollower")][0]
r = robcls(cfgcls(port="can0", id="follower"))
try:
    r.connect()
    print("CONNECT OK")
finally:
    r._skip_safe_zero_on_disconnect = True
    try: r.bus.disable_all(); r.disconnect()
    except Exception as e: print("disc", e)
