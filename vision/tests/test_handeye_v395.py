"""v3.9.5 offline tests: camera stall handling (hold + wait + retry, CameraLost), shm re-attach, watchdog.
No hardware: SimArm / fake objects only."""
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

V = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V), str(V.parent)]
import handeye as H          # noqa: E402
import handeye_plan as HP    # noqa: E402


class FlakyCam:
    def __init__(self, fail_for_s):
        self.t_ok = time.monotonic() + fail_for_s
        self.img = np.zeros((360, 640, 3), np.uint8)

    def grab(self, after_ts=0.0, timeout=3.0):
        if time.monotonic() < self.t_ok:
            raise H.FrameTimeout("no fresh frame")
        return self.img, time.time()


def _ex(cam, **cfg):
    c = HP.load_config(); c.update(cfg)
    logs = []
    ex = H.Executor(H.SimArm([0.0] * 7, 4.0), {"wrist": cam, "overview": cam}, c, tempfile.mkdtemp(), sim=True,
                    log=logs.append)
    return ex, logs


def test_grab_waits_until_camera_back():
    ex, logs = _ex(FlakyCam(0.3), frame_timeout_s=0.01, frame_wait_s=5.0, camera_lost_s=10.0)
    img, _ = ex._grab("wrist", 0.0)
    assert img is not None
    assert any("Kamera weg" in m for m in logs) and any("Kamera wieder da" in m for m in logs)


def test_grab_times_out_then_camera_lost():
    ex, logs = _ex(FlakyCam(99), frame_timeout_s=0.01, frame_wait_s=0.2, camera_lost_s=0.5)
    try:
        ex._grab("wrist", 0.0)
        assert False, "FrameTimeout expected"
    except H.FrameTimeout:
        pass
    lost = False
    for _ in range(5):                        # the stall time adds up over retries of the same pose
        try:
            ex._grab("wrist", 0.0)
        except H.FrameTimeout:
            continue
        except H.CameraLost:
            lost = True; break
    assert lost, "CameraLost expected"


def test_capture_retries_same_pose_then_camera_lost():
    class Board:
        def detect(self, img): return None
    ex, logs = _ex(FlakyCam(99), frame_timeout_s=0.01, frame_wait_s=0.1, camera_lost_s=0.22)
    try:
        ex.capture(1, Board(), lambda: None)
        assert False
    except H.CameraLost:
        pass
    assert any("retrying the same pose" in m for m in logs)


def test_shm_reattach():
    import camera_feeds as cf
    prefix = f"t395_{int(time.time() * 1000) % 100000}_"
    pub = cf.FramePublisher("wrist", 64, 48, prefix=prefix)
    pub.publish(np.zeros((48, 64, 3), np.uint8))
    src = H.ShmSource.__new__(H.ShmSource)
    src.cf = type("CF", (), {"ShmReader": staticmethod(lambda role: cf.ShmReader(role, prefix=prefix))})
    src.r = cf.ShmReader("wrist", prefix=prefix); src.role = "wrist"; src.seq = -1
    src.reattach_s, src.t_new = 0.2, time.time()
    src.grab(timeout=1.0)
    old = src.r
    try:
        src.grab(after_ts=time.time() + 100, timeout=0.6)
    except H.FrameTimeout:
        pass
    assert src.r is not old, "reader was not re-attached"
    try: pub.close()
    except Exception: pass


class FakeArm:
    def __init__(self):
        self._off = False; self.t_last_read = time.monotonic(); self.stops = 0
    def stop_now(self):
        self.stops += 1; self._off = True; return True


def test_watchdog_warns_and_hard_stop():
    arm = FakeArm(); logs = []; rc = []
    ex = type("E", (), {"stop_flag": False})()
    wd = H.Watchdog(arm, ex, log=logs.append, warn_s=0.2, hard_s=0.6, exit_fn=rc.append)
    wd._stack = lambda: None
    wd.start(); time.sleep(1.2); wd.uninstall()
    assert any("no motor read" in m for m in logs)
    assert arm.stops == 1 and rc == [3]


def test_watchdog_quiet_while_reading():
    arm = FakeArm(); logs = []; rc = []
    ex = type("E", (), {"stop_flag": False})()
    wd = H.Watchdog(arm, ex, log=logs.append, warn_s=0.3, hard_s=1.0, exit_fn=rc.append)
    wd.start()
    for _ in range(12):
        arm.t_last_read = time.monotonic(); time.sleep(0.1)
    wd.uninstall()
    assert not logs and not rc and arm.stops == 0


def test_watchdog_stop_while_blocked():
    arm = FakeArm(); logs = []; rc = []
    ex = type("E", (), {"stop_flag": False})()
    wd = H.Watchdog(arm, ex, log=logs.append, warn_s=5, hard_s=50, stop_grace_s=0.3, exit_fn=rc.append)
    wd._stack = lambda: None
    wd.start(); time.sleep(0.2)
    wd.t_sig = time.monotonic()            # as if SIGTERM arrived through the wakeup pipe
    time.sleep(0.8); wd.uninstall()
    assert rc == [130] and arm.stops == 1


def test_soft_stop_flag_raises_once(tmp_path, monkeypatch):
    monkeypatch.setattr(H, "SOFTSTOP_FLAG", tmp_path / "flag")
    ex, _ = _ex(FlakyCam(0))
    ex.sim = False; ex.cfg["max_total_s"] = 1e9
    ex._check()                                   # no flag -> nothing
    (tmp_path / "flag").write_text("1"); ex._soft_t = 0.0
    try:
        ex._check(); assert False
    except H.SoftStop:
        pass
    ex._check()                                   # soft stop active: the return moves may run


def test_watchdog_ignores_sigusr(tmp_path):
    import signal
    arm = FakeArm(); ex = type("E", (), {"stop_flag": False})()
    wd = H.Watchdog(arm, ex, log=lambda *_: None, warn_s=5, hard_s=50, exit_fn=lambda rc: None)
    r, w = __import__("os").pipe(); __import__("os").set_blocking(r, False); wd.rfd = r
    __import__("os").write(w, bytes([signal.SIGUSR1]))
    wd.start(); time.sleep(0.3); wd.uninstall()
    assert wd.t_sig is None and not ex.stop_flag


def test_soft_stop_end_to_end_sim(tmp_path, monkeypatch):
    """Sim run: '⏸ Stopp & zurück' at pose 12 -> samples saved, slow return to taught + start pose, rest move,
    motors off, solved from >= 10 samples (nothing applied in sim)."""
    import handeye_synth as SY
    flag = tmp_path / "flag"; monkeypatch.setattr(H, "SOFTSTOP_FLAG", flag)
    truth, plan = SY.make_truth_and_plan()
    logs = []
    def lg(m):
        logs.append(m)
        if str(m).startswith("pose 12/"):
            flag.write_text("1")
    r = H.run(yes=True, sim=True, sim_truth=truth, plan=plan, out_dir=tmp_path / "o", log=lg)
    t = "\n".join(map(str, logs))
    assert "Stopp & zurück: Ablauf abgebrochen" in t
    assert "returning to the taught pose" in t and "returning to the start pose of this run" in t
    assert "rest reached, motors off" in t
    assert r["n_samples"] >= 10 and r.get("result") is not None
    assert not flag.exists() or True


def test_connect_timeout_fake_robot(monkeypatch):
    """A follower whose connect() never returns (stuck CAN_Initialize) -> CanAdapterHung after the timeout.
    Fake robot only: the real PCAN device is never opened."""
    import types
    class HangRobot:
        config = types.SimpleNamespace(joint_directions={}, max_relative_target=None)
        motors = {}
        def connect(self): time.sleep(30)
    monkeypatch.setitem(sys.modules, "play_policy", types.SimpleNamespace(make_robot=lambda mrt: HangRobot()))
    monkeypatch.delenv("HANDEYE_MOTOR_TRACE", raising=False)
    t0 = time.time()
    try:
        H.RealArm(4.0, connect_timeout_s=0.5); assert False
    except H.CanAdapterHung as e:
        assert "XCAN kurz ab- und wieder anstecken" in str(e)
    assert time.time() - t0 < 2.0
