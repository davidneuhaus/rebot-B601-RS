"""Offline tests for the v3.9.4 drop diagnostics (no hardware): MotorTrace with fake motors, drop_report verdicts."""
import json
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

HUM = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(HUM / "vision"), str(HUM / "tools"), str(HUM)]
import handeye  # noqa: E402
import drop_report  # noqa: E402

M = drop_report.ORDER


class FakeMotor:
    def __init__(self, cid):
        self.cid, self.seq, self.fc, self.dead = cid, 0, 0, False

    def tick(self):
        if not self.dead:
            self.seq += 1

    def get_state_snapshot(self):
        return NS(feedback_age_s=0.002 if not self.dead else 9.0, feedback_sequence=self.seq)

    def get_state(self):
        return NS(can_id=self.cid, arbitration_id=0x2000000 | self.cid, status_code=2, fault_code=self.fc,
                  pos=1.0, vel=0.0, torq=0.5, t_mos=35.0, t_rotor=33.0)


def make_run(tmp, kill, fault=None, usb=None, cam_stall=None):
    """Simulate a 3 s run at 50 Hz; at t0+2 s motors in `kill` stop answering; returns the handeye log path."""
    t0 = time.time() - 100
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(t0))
    log = tmp / f"handeye_{stamp}.log"
    os.environ["HANDEYE_MOTOR_TRACE"] = str(log)[:-4] + "_motors.jsonl"
    tr = handeye.MotorTrace()
    motors = {n: FakeMotor(i + 1) for i, n in enumerate(M)}
    real_time = time.time
    try:
        for k in range(150):
            t = t0 + 1 + k * 0.02
            handeye.time.time = lambda t=t: t
            if k == 100:
                for n in kill: motors[n].dead = True
                if fault: motors[fault].fc = 0b1          # undervoltage
            for m in motors.values(): m.tick()
            tr.sample(motors, err="read: frozen" if k == 140 else None)
    finally:
        handeye.time.time = real_time
    tr.close()
    hh = lambda dt: time.strftime("%H:%M:%S", time.localtime(t0 + dt))
    log.write_text(f"[handeye {hh(1)}] pose 14/18: height 150 mm\n[handeye {hh(4)}] ERROR: Roboterarm antwortet nicht\n")
    mon = tmp / f"dropmon_{time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(t0 - 10))}"
    mon.mkdir()
    ev = [{"t": t0 - 5, "src": "mon", "ev": "start"}, {"t": t0 + 20, "src": "cam", "ev": "fps"}]
    if usb:
        ev.append({"t": t0 + 1 + 2.0 + usb[1], "src": "usb", "ev": "detach", "role": usb[0], "name": usb[0], "loc": "1"})
    if cam_stall is not None:
        ev.append({"t": t0 + 3 + cam_stall + 0.4, "src": "cam", "ev": "stall", "role": "gripper", "since_s": 0.4})
    (mon / "events.jsonl").write_text("\n".join(json.dumps(e) for e in ev) + "\n")
    return str(log), str(mon)


def verdict(tmp, **kw):
    log, mon = make_run(tmp, **kw)
    txt, v = drop_report.analyse(log, 5.0, mon)
    return " ".join(v), txt


def test_trace_disabled_without_env(monkeypatch):
    monkeypatch.delenv("HANDEYE_MOTOR_TRACE", raising=False)
    tr = handeye.MotorTrace(); tr.sample({"a": FakeMotor(1)}); tr.event("x"); tr.close()
    assert tr.f is None


def test_trace_survives_broken_motor(tmp_path, monkeypatch):
    monkeypatch.setenv("HANDEYE_MOTOR_TRACE", str(tmp_path / "t.jsonl"))
    class Bad:
        def get_state_snapshot(self): raise RuntimeError("boom")
        def get_state(self): raise RuntimeError("boom")
    tr = handeye.MotorTrace(); tr.sample({"a": Bad()}); tr.close()
    rows = [json.loads(x) for x in (tmp_path / "t.jsonl").read_text().splitlines()]
    assert any("m" in r and "state_err" in r["m"]["a"] for r in rows)


def test_wrist_only(tmp_path):
    v, _ = verdict(tmp_path, kill=["wrist_roll", "gripper"])
    assert v.startswith("C") and "wrist_roll" in v and "wrist_yaw" in v


def test_whole_bus(tmp_path):
    v, _ = verdict(tmp_path, kill=M)
    assert v.startswith("B")


def test_usb_adapter_and_camera_first(tmp_path):
    (tmp_path / "a").mkdir()
    v, _ = verdict(tmp_path / "a", kill=M, usb=("can_adapter", 0.01), cam_stall=-0.5)
    assert "A  CAN adapter" in v and "FIRST to vanish: camera" in v


def test_fault_flag(tmp_path):
    v, _ = verdict(tmp_path, kill=[], fault="elbow_flex")
    assert "undervoltage" in v
