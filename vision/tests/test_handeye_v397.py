"""v3.9.7 offline tests: motor fault detection, driver temperature watch / cooling pause.
Simulated / fake arms only - no CAN device is opened."""
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

V = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V), str(V.parent)]
import handeye as H          # noqa: E402
import handeye_plan as HP    # noqa: E402

H.SOFTSTOP_FLAG = Path(tempfile.mkdtemp()) / "no_softstop_flag"


def test_decode_faults_00_36_run():
    assert H.decode_faults(18) == "overcurrent, HALL encoder fault"
    assert H.decode_faults(4) == "overtemperature"


class _Motor:
    def __init__(self, **st):
        self.st = NS(pos=0.0, vel=0.0, torq=0.0, t_mos=40.0, fault_code=0, status_code=2, can_id=2)
        self.st.__dict__.update(st)

    def get_state(self):
        return self.st


def _fake_real(motors):
    a = H.RealArm.__new__(H.RealArm)
    a.robot = NS(motors=motors)
    a.trace = NS(event=lambda *x, **k: None, sample=lambda *x, **k: None)
    a._off, a._run_seen, a.temps = False, set(), {}
    return a


def test_motor_leaving_run_mode_is_a_fault():
    m = {"shoulder_lift": _Motor(), "elbow_flex": _Motor(can_id=3)}
    a = _fake_real(m)
    a._signature()
    assert a.temps == {"elbow_flex": 40.0, "shoulder_lift": 40.0}
    m["shoulder_lift"].st.status_code = 0
    with pytest.raises(H.MotorFault) as e:
        a._signature()
    assert "shoulder_lift" in str(e.value) and not H._is_comm_error(e.value)


def test_fault_code_is_a_fault_but_not_a_comm_error():
    a = _fake_real({"shoulder_lift": _Motor(fault_code=18, t_mos=76.0)})
    with pytest.raises(H.MotorFault) as e:
        a._signature()
    assert "overcurrent, HALL encoder fault" in str(e.value) and "76 C" in str(e.value)
    assert not H._is_comm_error(e.value)


def test_status_0_before_enable_is_not_a_fault():
    a = _fake_real({"shoulder_lift": _Motor(status_code=0)})
    a._signature()                      # never seen in run mode -> fine
    a._off = True
    a.robot.motors["shoulder_lift"].st.fault_code = 18
    a._signature()                      # motors already off -> no exception


class HotArm(H.SimArm):
    def __init__(self, t0, step):
        super().__init__([0.0] * 7, 4.0)
        self.t, self.step = t0, step
        self.temps = {}

    def read(self):
        self.t += self.step
        self.temps = {"shoulder_lift": self.t, "wrist_roll": 35.0}
        return super().read()


def _ex(arm, **cfg):
    c = HP.load_config(); c.update(cfg)
    logs = []
    return H.Executor(arm, {}, c, tempfile.mkdtemp(), sim=True, log=logs.append), logs


def test_hot_motor_raises_motorhot_after_warning():
    ex, logs = _ex(HotArm(55.0, 0.05))
    with pytest.raises(H.MotorHot):
        for _ in range(20):
            ex.move_to([20.0, 10, -10, 5, 5, 10], 0.0)
            ex.move_to([0.0] * 6, 0.0)
    assert any("note: shoulder_lift driver at" in m for m in logs)


def test_no_motorhot_during_soft_return():
    ex, _ = _ex(HotArm(75.0, 0.0))
    ex.soft_active = True
    ex.move_to([5.0, 0, 0, 0, 0, 0], 0.0)


def test_cool_down_pauses_at_start_pose_and_returns():
    arm = HotArm(66.0, -0.5)
    ex, logs = _ex(arm)
    ex.q_wait = np.zeros(7); ex.q_teach = np.array([10.0, 5, -5, 0, 0, 0])
    ex.move_to([15.0, 5, -5, 0, 0, 0], 0.0)
    ex.t_hot = ("shoulder_lift", 66.0)
    assert ex.cool_down(ex.q_wait, 0.0) is True
    q, _ = ex.read()
    assert np.max(np.abs(q[:6])) < 1.0          # paused at the start pose
    assert any("motor hot" in m for m in logs) and any("cooled down" in m for m in logs)


def test_cool_down_not_needed():
    ex, _ = _ex(HotArm(40.0, 0.0))
    ex.q_wait = np.zeros(7)
    ex.read()
    assert ex.cool_down(ex.q_wait, 0.0) is False
