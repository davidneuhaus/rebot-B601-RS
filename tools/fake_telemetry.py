#!/usr/bin/env python3
"""Offline test helper: sends fake teleop telemetry (same format as lerobot_teleoperate_tap.py) at 30 Hz.
No hardware involved. Usage: ~/humanoid/.venv/bin/python tools/fake_telemetry.py [seconds]"""
import json, math, socket, sys, time
M = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll", "gripper"]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10
t0 = time.time(); n = 0
while time.time() - t0 < dur:
    t = time.time() - t0; n += 1
    act = {f"{m}.pos": 20 * math.sin(0.5 * t + i) for i, m in enumerate(M)}
    obs = {f"{m}.pos": v - 1.0 for m, v in zip(M, act.values())}
    obs.update({f"{m}.vel": 10 * math.cos(0.5 * t + i) for i, m in enumerate(M)})
    obs.update({f"{m}.torque": 0.1 * i for i, m in enumerate(M)})
    s.sendto(json.dumps({"t": time.time(), "seq": n, "obs": obs, "act": act}).encode(), ("127.0.0.1", 8765))
    time.sleep(1 / 30)
