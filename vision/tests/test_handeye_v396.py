"""v3.9.6 offline tests: feedback-freeze ride-through (torque stays on, hold commands, re-sync, soft stop / off).
Simulated arm only - no CAN device is opened."""
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

V = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V), str(V.parent)]
import handeye as H          # noqa: E402
import handeye_plan as HP    # noqa: E402

# never react to a real GUI soft-stop flag (~/humanoid/tmp/handeye_softstop) during tests
H.SOFTSTOP_FLAG = Path(tempfile.mkdtemp()) / "no_softstop_flag"


class FreezingArm(H.SimArm):
    """SimArm whose feedback freezes (FeedbackFrozen like RealArm) for freeze_s real seconds after n_ok reads."""
    def __init__(self, freeze_s, n_ok=20, cmd_error=False):
        super().__init__([0.0] * 7, 4.0)
        self.freeze_s, self.n_ok, self.cmd_error = freeze_s, n_ok, cmd_error
        self.t_freeze, self.n_reads, self.cmd_in_freeze, self.stops = None, 0, 0, 0

    def _in_freeze(self):
        if self.t_freeze is None:
            return False
        return time.monotonic() - self.t_freeze < self.freeze_s

    def read(self):
        self.n_reads += 1
        if self.n_reads == self.n_ok:
            self.t_freeze = time.monotonic()
        if self._in_freeze():
            raise H.FeedbackFrozen("motor feedback frozen for 1.0 s")
        return super().read()

    def command(self, q7):
        if self._in_freeze():
            self.cmd_in_freeze += 1
            if self.cmd_error:
                raise H.ArmUnresponsive("command not acknowledged (send_mit failed: pcan reconnect failed)")
        return super().command(q7)

    def stop_now(self):
        self.stops += 1; return super().stop_now()


def _ex(arm, **cfg):
    c = HP.load_config(); c.update(cfg)
    logs = []
    ex = H.Executor(arm, {}, c, tempfile.mkdtemp(), sim=True, log=logs.append)
    ex.ride_enabled = True
    return ex, logs


TGT = [10.0, 5.0, -5.0, 3.0, 2.0, 8.0]


def test_short_freeze_is_ridden_through():
    arm = FreezingArm(1.0)
    ex, logs = _ex(arm)
    q, ptp = ex.move_to(TGT, 0.0)
    assert np.max(np.abs(q - TGT)) < 1.0
    assert any("Rückmeldung weg" in m for m in logs) and any("Rückmeldung wieder da" in m for m in logs)
    assert arm.cmd_in_freeze > 5, "hold commands must keep going during the freeze"
    assert arm.stops == 0


def test_command_errors_during_freeze_are_ridden_through():
    arm = FreezingArm(0.8, cmd_error=True)
    ex, logs = _ex(arm)
    ex.move_to(TGT, 0.0)
    assert any("Rückmeldung wieder da" in m for m in logs)


def test_long_freeze_soft_stop():
    arm = FreezingArm(1.5)
    ex, logs = _ex(arm, freeze_ride_s=0.5, freeze_giveup_s=5.0)
    try:
        ex.move_to(TGT, 0.0); assert False
    except H.FeedbackRecovered:
        pass


def test_no_feedback_gives_up():
    arm = FreezingArm(99)
    ex, logs = _ex(arm, freeze_ride_s=0.3, freeze_giveup_s=1.0)
    t0 = time.time()
    try:
        ex.move_to(TGT, 0.0); assert False
    except H.ArmUnresponsive:
        pass
    assert 0.9 < time.time() - t0 < 4.0


def test_ride_disabled_keeps_old_rule():
    arm = FreezingArm(5)
    ex, _ = _ex(arm); ex.ride_enabled = False
    try:
        ex.move_to(TGT, 0.0); assert False
    except H.ArmUnresponsive:
        pass


def test_stop_during_freeze():
    arm = FreezingArm(99)
    ex, _ = _ex(arm, freeze_ride_s=5, freeze_giveup_s=20)
    import threading
    threading.Timer(0.5, lambda: setattr(ex, "stop_flag", True)).start()
    try:
        ex.move_to(TGT, 0.0); assert False
    except H.Stop:
        pass
