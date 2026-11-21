#!/usr/bin/env python3
"""lerobot-teleoperate with a telemetry tap for dataset recording (v3.3).

Runs LeRobot's own teleoperate main() unchanged (same CLI arguments as `lerobot-teleoperate`, so the plugin,
collision guard and all teleop.sh settings stay exactly as they are). Before that it wraps the follower's
get_observation() and send_action(): after every send_action() one small UDP datagram (JSON, ~0.5 kB) with
the latest follower observation and the commanded action is sent to 127.0.0.1:$TELEMETRY_PORT (default 8765).

Cost per cycle: one non-blocking sendto() on localhost (~20 µs). If nobody listens, packets are simply
dropped. Any error in the tap is swallowed – it can never stop or slow teleoperation.
TELEMETRY_TAP=0 disables the tap completely (pure lerobot-teleoperate).

Datagram: {"t": unix time, "seq": n, "obs": {"shoulder_pan.pos": deg, ..., "<m>.vel": .., "<m>.torque": ..},
           "act": {"shoulder_pan.pos": deg, ...}}   (act = action given to send_action, before the
           follower's direction mapping, step clamp and guard – the same value LeRobot's record script stores)
The file name contains "lerobot_teleoperate" on purpose: rollback.sh detects running teleop via pgrep.
"""
import json, os, socket, sys, time


def _install_tap():
    port = int(os.environ.get("TELEMETRY_PORT", "8765"))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    addr = ("127.0.0.1", port)
    state = {"obs": None, "seq": 0}

    from lerobot_robot_seeed_b601 import seeed_b601_follower as base_mod
    classes = [base_mod.SeeedB601FollowerBase]
    try:
        from lerobot_robot_seeed_b601 import seeed_b601_rs_follower as rs_mod
        classes.append(rs_mod.SeeedB601RSFollower)
    except Exception:
        pass

    def wrap_obs(orig):
        def get_observation(self, *a, **kw):
            obs = orig(self, *a, **kw)
            try:
                state["obs"] = {k: float(v) for k, v in obs.items() if isinstance(v, (int, float))}
            except Exception:
                pass
            return obs
        get_observation._tapped = True
        return get_observation

    def wrap_act(orig):
        def send_action(self, action, *a, **kw):
            res = orig(self, action, *a, **kw)
            try:
                state["seq"] += 1
                msg = {"t": time.time(), "seq": state["seq"], "obs": state["obs"],
                       "act": {k: float(v) for k, v in action.items() if isinstance(v, (int, float))}}
                sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), addr)
            except Exception:
                pass   # nobody listening, buffer full, ... -> ignore
            return res
        send_action._tapped = True
        return send_action

    n = 0
    for cls in classes:
        for name, wrapper in (("get_observation", wrap_obs), ("send_action", wrap_act)):
            f = cls.__dict__.get(name)
            if f is not None and not getattr(f, "_tapped", False):
                setattr(cls, name, wrapper(f)); n += 1
    print(f"Telemetrie-Tap aktiv: UDP 127.0.0.1:{port} ({n} Methoden)", flush=True)


def main():
    from lerobot.scripts import lerobot_teleoperate as lt
    if os.environ.get("TELEMETRY_TAP", "1") != "0":
        try:
            from lerobot.utils.import_utils import register_third_party_plugins
            register_third_party_plugins()
        except Exception:
            pass
        try:
            _install_tap()
        except Exception as e:   # never block teleop because of the tap
            print(f"Telemetrie-Tap NICHT aktiv: {e}", flush=True)
    sys.argv[0] = "lerobot-teleoperate"
    lt.main()


if __name__ == "__main__":
    main()
