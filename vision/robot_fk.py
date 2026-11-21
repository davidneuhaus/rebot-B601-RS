"""Forward kinematics of the reBot B601-RS from the Seeed URDF (no extra dependencies).

Source: lerobot_robot_seeed_b601/assets/00-arm-rs_asm-v3.urdf (shipped with the Seeed plugin 1.0.5).
Joint angles: the 6 arm joints in DEGREES as reported by the follower (`<joint>.pos` in get_observation, i.e. raw
motor angle). The plugin's own gravity model feeds these raw angles straight into the URDF q vector, so the
mapping is q_urdf[i] = radians(raw_deg[i]) for shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_yaw, wrist_roll = joint1..joint6.  (Motor-side `joint_directions` only apply to commands.)

Robot frame = URDF base_link (x, y, z in mm after conversion, z up, origin at the base mounting plane).
Tip = origin of `gripper_end` plus TIP_OFFSET_MM along the gripper_end x axis (the finger direction,
see sanity_report()). Adjust TIP_OFFSET_MM (or env HUMANOID_TIP_OFFSET_MM) to the real fingertip.
"""
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]
URDF = Path("/opt/homebrew/Caskroom/miniforge/base/envs/lerobot/lib/python3.12/site-packages/"
            "lerobot_robot_seeed_b601/assets/00-arm-rs_asm-v3.urdf")
TIP_OFFSET_MM = float(os.environ.get("HUMANOID_TIP_OFFSET_MM", 0.0))  # along tool axis, see above


def _urdf_path():
    try:
        import lerobot_robot_seeed_b601 as P
        return Path(P.__file__).parent / "assets" / "00-arm-rs_asm-v3.urdf"
    except Exception:
        return URDF


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _axis_rot(axis, a):
    ax = np.asarray(axis, float); ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


class B601FK:
    def __init__(self, urdf=None):
        root = ET.parse(urdf or _urdf_path()).getroot()
        self.joints = []  # chain base -> gripper_end
        by_parent = {j.find("parent").get("link"): j for j in root.findall("joint")
                     if j.get("name") not in ("joint_left", "joint_right")}
        link = "base_link"
        while link in by_parent:
            j = by_parent[link]
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = [float(v) for v in o.get("rpy").split()]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else None
            lim = j.find("limit")
            self.joints.append(dict(name=j.get("name"), type=j.get("type"), xyz=xyz, R=_rpy(*rpy), axis=axis,
                                    lim=(float(lim.get("lower")), float(lim.get("upper"))) if lim is not None and j.get("type") == "revolute" else None))
            link = j.find("child").get("link")
        self.end_link = link
        assert link == "gripper_end", link
        assert sum(j["type"] == "revolute" for j in self.joints) == 6

    def pose(self, joints_deg):
        """4x4 pose of gripper_end in base frame (metres). joints_deg: 6 values (ARM_JOINTS order) or dict."""
        if isinstance(joints_deg, dict):
            joints_deg = [joints_deg[k] if k in joints_deg else joints_deg[k + ".pos"] for k in ARM_JOINTS]
        q = list(np.radians(joints_deg))
        T, i = np.eye(4), 0
        for j in self.joints:
            A = np.eye(4); A[:3, :3] = j["R"]; A[:3, 3] = j["xyz"]
            if j["type"] == "revolute":
                B = np.eye(4); B[:3, :3] = _axis_rot(j["axis"], q[i]); i += 1
                A = A @ B
            T = T @ A
        return T

    def tip_mm(self, joints_deg, tip_offset_mm=None):
        """Gripper tip position (x, y, z) in mm, base frame."""
        T = self.pose(joints_deg)
        off = TIP_OFFSET_MM if tip_offset_mm is None else tip_offset_mm
        return (T[:3, 3] + T[:3, 0] * off / 1000.0) * 1000.0

    def check_limits(self, joints_deg):
        """Names of joints outside the URDF limit (input is probably not URDF-convention)."""
        bad, i = [], 0
        for j in self.joints:
            if j["type"] == "revolute":
                lo, hi = j["lim"]
                if not (lo - 0.05 <= np.radians(joints_deg[i]) <= hi + 0.05):
                    bad.append(ARM_JOINTS[i])
                i += 1
        return bad


def sanity_report(fk=None):
    fk = fk or B601FK()
    T = fk.pose([0] * 6)
    lines = [f"zero pose gripper_end origin (mm): {np.round(T[:3,3]*1000,1)}",
             f"zero pose gripper_end x axis: {np.round(T[:3,0],2)}  z axis: {np.round(T[:3,2],2)}"]
    for name, q in [("shoulder_lift=90", [0, 90, 0, 0, 0, 0]), ("lift=90 elbow=90", [0, 90, 90, 0, 0, 0]),
                    ("pan=+45", [45, 90, 90, 0, 0, 0])]:
        lines.append(f"{name}: tip {np.round(fk.tip_mm(q),1)}")
    reach = sum(np.linalg.norm(j["xyz"]) for j in fk.joints) * 1000
    lines.append(f"sum of link lengths (max reach upper bound): {reach:.0f} mm")
    return "\n".join(lines)


if __name__ == "__main__":
    print(sanity_report())
