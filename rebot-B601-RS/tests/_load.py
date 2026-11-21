"""Helper for the offline tests: loads the follower file, with stub imports if lerobot/motorbridge are missing."""
import importlib.util, sys, types
from types import SimpleNamespace as NS


def _stub_if_missing():
    try:
        import lerobot.robots.utils  # noqa: F401
        import motorbridge  # noqa: F401
        return False
    except Exception:
        pass

    def mod(name, **kw):
        m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m

    class Robot:
        def __init__(self, config): pass

    def ensure_safe_goal_position(gp, mrt):
        out = {}
        for k, (g, p) in gp.items():
            if isinstance(mrt, dict) and k not in mrt:
                out[k] = g; continue
            lim = mrt[k] if isinstance(mrt, dict) else mrt
            out[k] = p + max(-lim, min(lim, g - p))
        return out

    mod("lerobot"); mod("lerobot.cameras", CameraConfig=object)
    mod("lerobot.cameras.utils", make_cameras_from_configs=lambda c: {})
    mod("lerobot.motors", MotorCalibration=object)
    mod("motorbridge", Controller=object, Mode=NS(MIT=1, POS_VEL=2, FORCE_POS=3))
    mod("lerobot.processor", RobotAction=dict, RobotObservation=dict)
    mod("lerobot.utils"); mod("lerobot.utils.errors", DeviceAlreadyConnectedError=Exception, DeviceNotConnectedError=Exception)
    mod("lerobot.robots"); mod("lerobot.robots.robot", Robot=Robot)
    mod("lerobot.robots.utils", ensure_safe_goal_position=ensure_safe_goal_position)
    return True


def load_follower(path):
    stubbed = _stub_if_missing()
    spec = importlib.util.spec_from_file_location("guarded_follower", path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod, stubbed
