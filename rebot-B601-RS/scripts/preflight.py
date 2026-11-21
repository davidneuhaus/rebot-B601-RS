"""Pre-flight check before teleop: leader USB, CAN adapter, all 7 follower motors."""
import glob, sys, time, math, logging
logging.disable(logging.CRITICAL)
ports = sorted(glob.glob("/dev/tty.usbserial-*"))
if not ports:
    print("FEHLER: Leader-Arm (weiß) nicht gefunden – USB-Kabel prüfen."); sys.exit(2)
try:
    import lerobot_robot_seeed_b601 as P
    cfgcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollowerConfig")][0]
    robcls = [getattr(P,n) for n in dir(P) if n.endswith("RSFollower")][0]
    r = robcls(cfgcls(port="can0", id="follower"))
    r.connect(calibrate=False)
    r._skip_safe_zero_on_disconnect = True
except Exception as e:
    if "timeout" in str(e).lower():
        print("FEHLER: Roboterarm antwortet nicht – Strom am Arm einschalten."); sys.exit(4)
    print(f"FEHLER: CAN-Adapter (PCAN) nicht erreichbar – USB prüfen. ({e})"); sys.exit(3)
missing = []
try:
    for name, m in r.motors.items():
        ok = False
        for _ in range(10):
            m.request_feedback(); time.sleep(0.02)
            try: r.bus.poll_feedback_once()
            except Exception: pass
            if m.get_state() is not None: ok = True; break
        if not ok: missing.append(name)
finally:
    try: r.bus.disable_all(); r.disconnect()
    except Exception: pass
if missing:
    print("FEHLER: Roboterarm antwortet nicht (" + ", ".join(missing) + ") – Strom am Arm einschalten und 5 Sek. warten."); sys.exit(4)
print(ports[0])
