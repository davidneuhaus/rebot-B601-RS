"""Offline tests for vision/grasp (synthetic images, fake calibration, no models, no robot, no cameras).
Run: ~/humanoid/.venv/bin/python -m pytest -q ~/humanoid/vision/grasp/tests
Optional real-model smoke test: GRASP_TEST_MODELS=1 (loads YOLO-World, ~10 s)."""
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

V = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(V)); sys.path.insert(0, str(V.parent / "tools"))
from grasp import detect as D, locate as L, grasp_plan as G      # noqa: E402

W, H = 640, 360
TABLE_Z = -70.0


# ---------------------------------------------------------------- fake calibration (overview camera above the table)
def fake_cal(tmp_path, tilt_deg=0.0, with_table_to_robot=True):
    Tb2base = np.eye(4); Tb2base[:3, 3] = [300, 0, TABLE_Z]
    a = np.radians(tilt_deg)
    Tb2base[:3, :3] = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    Tc2base = np.eye(4)
    Tc2base[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)    # looks straight down
    Tc2base[:3, 3] = [300, 0, 600]
    Tb2c = np.linalg.inv(Tc2base) @ Tb2base
    rvec = cv2.Rodrigues(Tb2c[:3, :3])[0].ravel()
    K = [[500.0, 0, 319.5], [0, 500.0, 179.5], [0, 0, 1]]
    (tmp_path / "intrinsics.json").write_text(json.dumps({"K": K, "dist": [0, 0, 0, 0, 0], "size": [W, H]}))
    table = {"rvec": rvec.tolist(), "tvec": Tb2c[:3, 3].tolist(), "size": [W, H],
             "table_to_robot": {"T_board2base": Tb2base.tolist()} if with_table_to_robot else None}
    (tmp_path / "table.json").write_text(json.dumps(table))
    return Tb2base, Tc2base, np.array(K)


def project(P, Tc2base, K):
    Tb = np.linalg.inv(Tc2base)
    pc = (Tb[:3, :3] @ np.asarray(P, float).T).T + Tb[:3, 3]
    uv = (K @ pc.T).T
    return uv[:, :2] / uv[:, 2:3]


def rect_corners(cx, cy, z, w, l, yaw_deg):
    """Rectangle w (along yaw direction) x l on the plane z, robot frame."""
    a = np.radians(yaw_deg)
    u, v = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    return np.array([[cx, cy] + sx * u * w / 2 + sy * v * l / 2 for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]])


class FakeDet:
    def __init__(self, dets): self.dets, self.classes, self.device = dets, [], "cpu"
    def set_classes(self, p): self.classes = list(p)
    def detect(self, bgr, conf=None): return list(self.dets)


# ---------------------------------------------------------------- detect.py
@pytest.mark.parametrize("angle", [0, 30, 75, -40])
def test_outline_yaw_short_axis(angle):
    m = np.zeros((300, 300), np.uint8)
    box = cv2.boxPoints(((150, 150), (120, 40), angle)).astype(np.int32)    # long side along `angle`
    cv2.fillPoly(m, [box], 255)
    g = D.outline_to_grasp(m)
    assert abs(g["short_px"] - 40) < 3 and abs(g["long_px"] - 120) < 3
    assert abs(g["center"][0] - 150) < 1.5 and abs(g["center"][1] - 150) < 1.5
    want = ((angle + 90 + 90) % 180) - 90                    # short axis = long axis + 90, folded to (-90, 90]
    d = abs((g["yaw_img_deg"] - want + 90) % 180 - 90)
    assert d < 3, (g["yaw_img_deg"], want)
    assert -90 < g["yaw_img_deg"] <= 90


def test_empty_mask_gives_none():
    assert D.outline_to_grasp(np.zeros((50, 50), np.uint8)) is None


def synthetic_scene():
    rng = np.random.default_rng(0)
    img = (rng.normal(120, 6, (300, 400, 3))).clip(0, 255).astype(np.uint8)
    box = cv2.boxPoints(((200, 150), (110, 50), 20)).astype(np.int32)
    cv2.fillPoly(img, [box], (30, 60, 200))
    return img, box


def test_grabcut_finds_object():
    img, poly = synthetic_scene()
    truth = np.zeros(img.shape[:2], np.uint8); cv2.fillPoly(truth, [poly], 255)
    m = D.grabcut_mask(img, (130, 100, 270, 200))
    assert m is not None
    iou = ((m > 0) & (truth > 0)).sum() / ((m > 0) | (truth > 0)).sum()
    assert iou > 0.8, iou


def test_detector_with_fake_model_grabcut_and_box():
    img, _ = synthetic_scene()
    for seg in ("grabcut", "box", "none"):
        gd = D.GraspDetector(segmenter=seg, detector=FakeDet([("cup", 0.9, 130, 100, 270, 200)]))
        objs = gd.detect(img, ["cup"])
        assert len(objs) == 1 and objs[0].label == "cup"
        if seg == "none":
            assert objs[0].outline_px is None and objs[0].yaw_img_deg is None
        else:
            assert objs[0].yaw_img_deg is not None
    gd = D.GraspDetector(segmenter="grabcut", detector=FakeDet([("cup", 0.9, 130, 100, 270, 200)]))
    o = gd.detect(img, ["cup"])[0]
    assert o.segmenter == "grabcut" and abs(o.center_px[0] - 200) < 8
    out = D.draw(img, [o])
    assert out.shape == img.shape and (out != img).any()
    json.dumps(o.to_json())


def test_missing_sam_weights_raises(monkeypatch):
    monkeypatch.setattr(D.GraspDetector, "sam_weights", staticmethod(lambda: None))
    with pytest.raises(FileNotFoundError):
        D.GraspDetector(segmenter="sam", detector=FakeDet([]))
    assert D.GraspDetector(segmenter="auto", detector=FakeDet([])).segmenter == "grabcut"


# ---------------------------------------------------------------- locate.py
def test_missing_calibration_errors(tmp_path):
    with pytest.raises(L.CalibrationMissing, match="table calibration not found"):
        L.load_calibration(tmp_path)
    fake_cal(tmp_path, with_table_to_robot=False)
    with pytest.raises(L.CalibrationMissing, match="hand-eye"):
        L.load_calibration(tmp_path)
    fake_cal(tmp_path)
    with pytest.raises(L.CalibrationMissing, match="handeye.json"):
        L.load_calibration(tmp_path, require_handeye=True)
    cal = L.load_calibration(tmp_path)
    with pytest.raises(L.CalibrationMissing, match="handeye.json"):
        L.pixel_to_base([[10, 10]], cal, (W, H), camera="wrist", q_deg=[0] * 6)


@pytest.mark.parametrize("tilt", [0.0, 4.0])
@pytest.mark.parametrize("height", [0.0, 40.0])
def test_pixel_to_base_roundtrip(tmp_path, tilt, height):
    Tb, Tc, K = fake_cal(tmp_path, tilt_deg=tilt)
    cal = L.load_calibration(tmp_path)
    p0, n = cal.plane(height)
    truth = np.array([p0 + n * 0 + np.array([dx, dy, 0]) for dx, dy in [(0, 0), (60, -40), (-80, 90)]])
    # put points exactly on the plane: solve z from the plane equation
    for P in truth:
        P[2] += -((P - p0) @ n) / n[2]
    px = project(truth, Tc, K)
    got = L.pixel_to_base(px, cal, (W, H), height_mm=height)
    assert np.abs(got - truth).max() < 0.05
    got2 = L.pixel_to_base(px / 2, cal, (W // 2, H // 2), height_mm=height)      # smaller image, scaled K
    assert np.abs(got2 - truth).max() < 0.5


def test_locate_object_yaw_and_size(tmp_path):
    Tb, Tc, K = fake_cal(tmp_path)
    cal = L.load_calibration(tmp_path)
    for yaw_long in (0, 25, -50):        # long axis heading in the robot frame; jaws must close across it
        top_z = TABLE_Z + 30
        corners = rect_corners(320, 20, top_z, 90, 40, yaw_long)       # 90 (along yaw_long) x 40 mm
        poly = project(np.column_stack([corners, np.full(4, top_z)]), Tc, K)
        mask = np.zeros((H, W), np.uint8); cv2.fillPoly(mask, [poly.astype(np.int32)], 255)
        obj = D.detect_from_masks([("block", 0.9, (0, 0, W, H), mask)])[0]
        loc = L.locate_object(obj, cal, (W, H), height_mm=30.0)
        assert np.allclose(loc["xyz_mm"][:2], [320, 20], atol=1.5) and abs(loc["xyz_mm"][2] - top_z) < 0.5
        assert abs(loc["width_mm"] - 40) < 3 and abs(loc["length_mm"] - 90) < 4
        want = ((yaw_long + 90 + 90) % 180) - 90
        assert abs((loc["jaw_yaw_deg"] - want + 90) % 180 - 90) < 3, (loc, want)


def test_reach_filter(tmp_path):
    rc = L.ReachChecker(table_z_mm=TABLE_Z)
    ok = rc.check([420, 0, TABLE_Z + 20], 15.0)
    assert ok.ok and len(ok.q) == 6 and ok.pos_err_mm < 1.5
    far = rc.check([2000, 0, 0], 0.0)
    assert not far.ok
    R = L.grasp_orientation(30.0)
    assert np.allclose(R[:, 0], [0, 0, -1]) and np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.linalg.det(R) > 0
    assert abs(R[1, 1] * 0 + np.degrees(np.arctan2(R[1, 1], R[0, 1])) - 30) < 1e-6


# ---------------------------------------------------------------- grasp_plan.py
def target(x=420.0, y=0.0, z=TABLE_Z + 30, yaw=15.0, width=40.0):
    return dict(xyz_mm=[x, y, z], jaw_yaw_deg=yaw, width_mm=width, length_mm=90.0)


@pytest.fixture(scope="module")
def reach():
    return L.ReachChecker(table_z_mm=TABLE_Z)


def test_profile_selection():
    cfg = G.load_profiles()
    assert G.pick_profile("blue block", cfg)[0] == "block"
    assert G.pick_profile("Nivea tin", cfg)[0] == "cup"
    assert G.pick_profile("dragon", cfg)[0] == "default"


def test_plan_sequence(reach):
    rest = [0, 5, 5, 0, 0, 0]
    pl = G.plan_grasp(target(), "blue block", reach, TABLE_Z, rest_q=rest, place={"xyz_mm": [380, -90, TABLE_Z]})
    assert pl.ok, pl.reason
    kinds = [w.kind for w in pl.waypoints]
    names = [w.name for w in pl.waypoints]
    order = ["approach", "descend", "close", "lift", "carry", "descend", "release", "lift", "return"]
    seq = [k for i, k in enumerate(kinds) if i == 0 or k != kinds[i - 1]]
    assert seq == order, seq
    assert names[0] == "pre_grasp" and names[-1] == "return_rest" and pl.waypoints[-1].q == rest
    i_close = kinds.index("close")
    assert pl.waypoints[i_close].gripper_mm < pl.waypoints[0].gripper_mm            # closes
    assert pl.waypoints[kinds.index("release")].gripper_mm > pl.waypoints[i_close].gripper_mm
    assert all(len(w.q) == 6 for w in pl.waypoints)
    # consecutive joint steps stay small (one IK branch, Cartesian columns subdivided)
    qs = np.array([w.q for w in pl.waypoints[:-1]])
    assert np.abs(np.diff(qs, axis=0)).max() < 45
    sc = pl.success_check
    assert sc["after_waypoint"] == "close" and sc["held_if_opening_mm_between"][0] < 40 < sc["held_if_opening_mm_between"][1]
    assert any("placeholder" in w for w in pl.warnings)
    json.dumps(pl.to_json())


def test_height_lock(reach):
    cfg = G.load_profiles()
    cfg["profiles"]["block"]["insert_depth_mm"] = 500.0          # absurdly deep grasp
    pl = G.plan_grasp(target(), "block", reach, TABLE_Z, cfg=cfg)
    assert pl.ok, pl.reason
    floor = TABLE_Z + cfg["motion"]["min_tip_clearance_mm"]
    assert min(w.tip_mm[2] for w in pl.waypoints if w.tip_mm) >= floor - 1e-6
    assert any("height lock" in w for w in pl.warnings)


def test_plan_failures(reach):
    assert not G.plan_grasp(target(x=2000.0), "block", reach, TABLE_Z).ok
    huge = G.plan_grasp(target(width=300.0), "block", reach, TABLE_Z)
    assert not huge.ok and "does not fit" in huge.reason
    nowidth = G.plan_grasp(dict(xyz_mm=[420, 0, TABLE_Z + 30], jaw_yaw_deg=0), "block", reach, TABLE_Z)
    assert not nowidth.ok and "width" in nowidth.reason


def test_planner_has_no_motor_code():
    forbidden = ("lerobot", "serial", "socket", "pcan", "import can", "teleop", "send_action", "torque")
    for f in ("detect.py", "locate.py", "grasp_plan.py"):
        src = (Path(G.__file__).parent / f).read_text().lower()
        for bad in forbidden:
            assert f"import {bad}" not in src and f"from {bad}" not in src, (f, bad)
    assert "send_action" not in (Path(G.__file__).read_text())


# ---------------------------------------------------------------- CLI demo (fake detector + fake calibration)
def test_demo_end_to_end(tmp_path):
    import grasp_demo
    cal_dir = tmp_path / "cal"; cal_dir.mkdir()
    Tb, Tc, K = fake_cal(cal_dir)
    top_z = TABLE_Z + 30
    corners = rect_corners(420, 0, top_z, 90, 40, 15)
    poly = project(np.column_stack([corners, np.full(4, top_z)]), Tc, K)
    img = np.full((H, W, 3), 110, np.uint8); cv2.fillPoly(img, [poly.astype(np.int32)], (30, 60, 200))
    x1, y1 = poly.min(0) - 5; x2, y2 = poly.max(0) + 5
    args = SimpleNamespace(prompts="block", segmenter="grabcut", profiles=None, cal_dir=str(cal_dir), rest_q=None,
                           place=None, pick=None)
    img_out, res = grasp_demo.build(args, img, detector=D.GraspDetector("grabcut", detector=FakeDet([("block", 0.8, x1, y1, x2, y2)])))
    e = res["objects"][0]
    assert e["plan"]["ok"], e["plan"]["reason"]
    assert np.allclose(e["location"]["xyz_mm"][:2], [420, 0], atol=4)
    assert res["dry_run"] and img_out.shape == img.shape
    # without calibration: detection only, clear message
    args.cal_dir = str(tmp_path / "none")
    _, res2 = grasp_demo.build(args, img, detector=D.GraspDetector("box", detector=FakeDet([("block", 0.8, x1, y1, x2, y2)])))
    assert res2["calibration"].startswith("MISSING") and not res2["objects"][0]["plan"]["ok"]


@pytest.mark.skipif(not os.environ.get("GRASP_TEST_MODELS"), reason="set GRASP_TEST_MODELS=1 for the real-model smoke test")
def test_real_models_smoke():
    img = cv2.imread(str(V / "test_bus_cpu.jpg"))
    gd = D.GraspDetector(segmenter="auto")
    objs = gd.detect(img, ["bus", "person"])
    assert objs and objs[0].outline_px is not None
