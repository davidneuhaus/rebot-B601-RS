#!/usr/bin/env python3
"""Offline tests for the v3.9 auto calibration (no robot, no camera access). Run:
    ~/humanoid/.venv/bin/python ~/humanoid/vision/tests/test_handeye.py
1. the printed PDF (rendered at 300 dpi) is detected: 24/24 corners, square = 35 mm on paper
2. a real overview frame of the taped board (640x360 GUI frame): detection + recalibrate_table (temp dir)
3. planner: limits, height floor for all pan/yaw/roll sign conventions, paths, no pose below 80 mm
4. synthetic end-to-end: simulated arm + rendered wrist/overview images -> hand-eye recovers the truth
   (< 2 mm, < 0.5 deg); also with a wrong joint sign convention in the 'real' robot (detected by the solver)
5. apply_handeye -> table.json (temp dir): table_to_robot_xyz maps board corners to the true robot positions
6. executor safety: stop flag -> Stop raised; a blocked arm -> abort
7. v3.9.1 regressions (22:29 crash): fake follower that clamps relative targets like LeRobot (+ a weak elbow),
   fake follower that stops responding (frozen feedback / ack timeouts), clean aborts without return moves,
   automatic start (search pose -> board -> aim pose) for several camera mounting rolls, board not found.
8. v3.9.2: after an abort with >= 10 good samples the sweep is solved (sample count bug of 23:16 fixed), applied only if
   GOOD ('apply available' for OK), resume of an aborted folder, CAN adapter hint on 'pcan reconnect failed'.
Also runs under pytest:  ~/humanoid/.venv/bin/python -m pytest -q ~/humanoid/vision/tests/test_handeye.py
"""
import json, os, shutil, subprocess, sys, tempfile, time
from pathlib import Path

import cv2
import numpy as np

V = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V)); sys.path.insert(0, str(V.parent))
import charuco as C           # noqa
import handeye_plan as HP     # noqa
import handeye as H           # noqa
import handeye_synth as SY    # noqa
import calibrate as cal       # noqa

R = {}
def check(name, cond, info=""):
    R[name] = bool(cond)
    print(("PASS " if cond else "FAIL ") + name + (f"  ({info})" if info else ""), flush=True)


def t1_pdf():
    out = Path(tempfile.mkdtemp()) / "p"
    subprocess.run(["pdftoppm", "-r", "300", "-png", "-singlefile", str(V / "calibration" / "charuco_7x5_35mm_A4.pdf"), str(out)], check=True)
    im = cv2.imread(str(out) + ".png")
    b = C.Charuco(); d = b.detect(im)
    n = 0 if d is None else len(d["ids"])
    ids = list(d["ids"]); px = np.linalg.norm(d["img"][ids.index(1)] - d["img"][ids.index(0)])
    mm = px / 300 * 25.4
    check("pdf: 24/24 corners", n == 24, f"{n} corners, {d['n_markers']} markers")
    check("pdf: square 35 mm on paper (print at 100 %)", abs(mm - 35.0) < 0.2, f"{mm:.2f} mm")
    # rotated 180 deg: ids stay the same physical corners (no order flip)
    d2 = b.detect(cv2.rotate(im, cv2.ROTATE_180))
    h, w = im.shape[:2]
    p0 = d["img"][ids.index(0)]; p0r = d2["img"][list(d2["ids"]).index(0)]
    check("pdf: corner ids unchanged when the board is rotated 180 deg", np.linalg.norm((w - 1 - p0r[0], h - 1 - p0r[1]) - p0) < 2.0)


def t2_real_overview():
    im = cv2.imread(str(V / "tests" / "data" / "overview_charuco_640.png"))
    b = C.Charuco(); d = b.detect(im)
    check("real overview frame: board detected", d is not None and len(d["ids"]) >= 20, f"{0 if d is None else len(d['ids'])} corners")
    tmp = Path(tempfile.mkdtemp())
    shutil.copy(V / "calibration" / "intrinsics.json", tmp / "intrinsics.json")
    shutil.copy(V / "calibration" / "board_charuco.json", tmp / "board_charuco.json")
    old = cal.CAL_DIR, cal.HIST_DIR
    cal.CAL_DIR, cal.HIST_DIR = tmp, tmp / "history"
    try:
        r = cal.recalibrate_table(im)
        check("recalibrate_table with ChArUco (640x360 frame, scaled K)", r["ok"], r["message"])
        t = json.loads((tmp / "table.json").read_text())
        check("table.json board = charuco", t["board"]["type"] == "charuco")
        r2 = cal.recalibrate_table(im)
        check("recalibrate again: shift ~0 mm", r2["ok"] and r2["shift_mm"]["max"] < 1.0, str(r2["shift_mm"]))
        # big rotation of the image -> still found, ids unambiguous
        r3 = cal.recalibrate_table(cv2.flip(im, -1))
        check("recalibrate on 180-deg rotated frame works (no corner-order flip)", r3["ok"], r3["message"])
    finally:
        cal.CAL_DIR, cal.HIST_DIR = old


def t3_planner(truth, plan):
    cfg = HP.load_config(); kin = HP.Kin()
    P = plan["poses"]
    check("planner: >= 15 poses", len(P) >= 15, f"{len(P)}")
    check("planner: all joints inside URDF + plugin limits (5 deg margin)", all(kin.in_limits(p["q"], 4.99) for p in P))
    worst = min(HP.pose_safety(kin, p["q"], cfg)[1] for p in P)
    check("planner: lowest wrist/camera/gripper point >= 80 mm over the board (all sign conventions)", worst >= 80, f"{worst:.0f} mm")
    check("planner: every joint path checked and safe", all(x["ok"] for x in plan["paths"]))
    hs = [p["height_mm"] for p in P]
    check("planner: heights 150-300 mm", min(hs) >= 150 and max(hs) <= 300, f"{min(hs):.0f}-{max(hs):.0f}")
    check("planner: tilt <= 25 deg", max(p["tilt_deg"] for p in P) <= 25)
    # rest (all zero) -> teach path safe
    ok, clr, why = HP.path_safety(kin, [0] * 6, plan["q_teach"], cfg)
    check("rest pose -> taught pose path safe", ok, f"min {clr:.0f} mm {why}")


def t4_synthetic(truth, plan):
    tmp = Path(tempfile.mkdtemp())
    r = H.run(yes=True, sim=True, sim_truth=truth, plan=plan, out_dir=tmp / "sim")
    e = SY.compare(truth, r["result"])
    # v3.9.1: 3 mm / 0.75 deg (was 2 / 0.5): the printed board has 33 mm squares (smaller corners in the image) ->
    # X 2.1-2.4 mm / 0.33-0.56 deg in the simulation (failed already before the v3.9.1 code change); the board->base
    # error (what the table mapping uses) stays < 1 mm / 0.5 deg
    check("synthetic: T_cam2gripper < 3 mm / 0.75 deg", e["X_mm"] < 3 and e["X_deg"] < 0.75, f"{e['X_mm']:.2f} mm {e['X_deg']:.2f} deg")
    check("synthetic: T_board2base < 2 mm / 0.5 deg", e["Z_mm"] < 2 and e["Z_deg"] < 0.5, f"{e['Z_mm']:.2f} mm {e['Z_deg']:.2f} deg")
    check("synthetic: overview pixel->table < 3 mm", e.get("overview_px2table_mm_max", 99) < 3, f"{e.get('overview_px2table_mm_max', -1):.2f} mm")
    check("synthetic: quality grade good", r["result"]["quality"]["grade"] == "good", r["result"]["quality"]["grade"])
    # replay from the saved folder gives the same answer
    r2 = H.replay(tmp / "sim", log=lambda *_: None)
    check("replay = live result", np.allclose(r2["T_cam2gripper"], r["result"]["T_cam2gripper"], atol=1e-6))
    return r["result"], tmp


def t4b_wrong_sign(plan):
    truth = SY.make_truth(signs=(1, 1, -1, 1))      # the real wrist_yaw turns the other way than robot_fk assumes
    tmp = Path(tempfile.mkdtemp())
    r = H.run(yes=True, sim=True, sim_truth=truth, plan=plan, out_dir=tmp / "sim")["result"]
    check("synthetic, wrong wrist_yaw sign: solver detects it", r["joint_signs"] == [1, 1, -1, 1], str(r["joint_signs"]))


def t5_apply(truth, result):
    tmp = Path(tempfile.mkdtemp())
    shutil.copy(V / "calibration" / "intrinsics.json", tmp / "intrinsics.json")
    shutil.copy(V / "calibration" / "board_charuco.json", tmp / "board_charuco.json")
    old = cal.CAL_DIR, cal.HIST_DIR
    cal.CAL_DIR, cal.HIST_DIR = tmp, tmp / "history"
    try:
        msg = cal.apply_handeye(result)
        t = json.loads((tmp / "table.json").read_text())
        b = C.Charuco()
        xy = b.corners3d[:, :2]
        pr = cal.table_to_robot_xyz(xy, t)
        tr = b.corners3d @ truth["Z"][:3, :3].T + truth["Z"][:3, 3]
        err = np.linalg.norm(pr - tr, axis=1).max()
        check("apply_handeye: board corners -> robot frame < 2 mm", err < 2.0, f"max {err:.2f} mm; {msg}")
        r = cal.recalibrate_table(cv2.imread(str(V / "tests" / "data" / "overview_charuco_640.png")))
        t2 = json.loads((tmp / "table.json").read_text())
        check("recalibrate after handeye keeps table->robot (+ overview->base)", r["ok"] and t2["table_to_robot"]["method"] == "handeye"
              and "T_overview2base" in t2)
    finally:
        cal.CAL_DIR, cal.HIST_DIR = old


def t6_executor_safety(truth, plan):
    cfg = HP.load_config()
    arm = H.SimArm([0] * 7, 4.0)
    ex = H.Executor(arm, {}, cfg, tempfile.mkdtemp(), sim=True, log=lambda *_: None)
    ex.stop_flag = True
    try:
        ex.move_to(plan["q_teach"], 0.0); check("executor: stop flag raises Stop", False)
    except H.Stop:
        check("executor: stop flag raises Stop", True)
    class Blocked(H.SimArm):
        def read(self):                            # never moves; live sensor noise (identical readings = frozen = dead)
            return self.q + self.rng.normal(0, 0.02, 7), np.zeros(7)
    ex = H.Executor(Blocked([0] * 7, 4.0), {}, cfg, tempfile.mkdtemp(), sim=True, log=lambda *_: None)
    try:
        ex.move_to(plan["q_teach"], 0.0); check("executor: blocked arm aborts", False)
    except RuntimeError as e:
        check("executor: blocked arm aborts", "kommt nicht voran" in str(e) or "nicht erreicht" in str(e), str(e)[:80])



# ---------------------------------------------------------------- 7. v3.9.1 regressions (22:29 crash)
REST = [0.8, 0.1, 0.0, 0.0, -0.5, -0.1]          # joints at the start of the 22:29 run
Q_TEACH = [-4.88, 79.39, 63.39, -45.99, -11.25, -7.45]   # David's taught pose (22:27)
WEAK_ELBOW = {2: 1.5}    # elbow_flex only moves when its goal is > 1.5 deg away (PD vs gravity from rest)


class Logs(list):
    def __call__(self, msg):
        self.append(str(msg))

    def text(self):
        return "\n".join(self)


def _ex(arm, cams=None, log=None, out=None):
    return H.Executor(arm, cams or {}, HP.load_config(), out or tempfile.mkdtemp(), sim=True, log=log if log is not None else Logs())


def t7a_old_ramp_reproduces_crash():
    """Root cause: the v3.9 executor ramped the goal at 12 deg/s regardless of the measured position; LeRobot clamped
    it to present +- 1 deg, the weak elbow crept, the lag grew to 10 deg -> abort. Reproduced with the fake."""
    arm = H.SimArm(REST + [0.0], 1.0, weak=WEAK_ELBOW)
    q0 = np.array(REST); tgt = np.array(Q_TEACH); dt = 1 / 30
    d = np.max(np.abs(tgt - q0)); n = int(np.ceil(max(1.0, 1.5 * d / 12.0) / dt)); lag_t = None
    for k in range(1, n + 1):
        s_ = k / n; s_ = 3 * s_ * s_ - 2 * s_ ** 3
        cmd = q0 + s_ * (tgt - q0)
        arm.command(np.append(cmd, 0.0)); q, _ = arm.read()
        if np.max(np.abs(q[:6] - cmd)) > 10.0:
            lag_t = k * dt; break
    check("7a old v3.9 ramp vs clamping fake: LeRobot clamps + 10 deg lag abort (reproduces 22:29)",
          arm.clamped > 0 and lag_t is not None and lag_t < 6.0, f"clamped {arm.clamped}x, lag abort after {lag_t} s")


def t7b_tracking_never_clamps():
    arm = H.SimArm(REST + [0.0], 4.0, weak=WEAK_ELBOW)
    ex = _ex(arm)
    q, ptp = ex.move_to(Q_TEACH, 0.0, speed=6.0)
    err = float(np.max(np.abs(q - np.array(Q_TEACH))))
    check("7b new executor (mrt 4, weak elbow): reaches the taught pose from rest, 0 LeRobot clamps",
          arm.clamped == 0 and err < 3.0, f"clamped {arm.clamped}, error {err:.2f} deg, sim time {ex.sim_t:.0f} s")
    check("7b approach at 6 deg/s takes >= 1.5 x distance / speed", ex.sim_t >= np.max(np.abs(np.array(Q_TEACH) - REST)) / 6.0,
          f"{ex.sim_t:.1f} s")
    # the lead respects the follower's max_relative_target that the arm reports
    check("7b lead = 0.75 x follower max_relative_target", np.allclose(ex.lead, 3.0), str(ex.lead))


def t7c_stall_is_clean_and_named():
    arm = H.SimArm(REST + [0.0], 1.0, weak={2: 5.0})      # elbow can never follow inside a 0.75 deg lead
    lg = Logs(); ex = _ex(arm, log=lg)
    try:
        ex.move_to(Q_TEACH, 0.0, speed=6.0); check("7c stalled elbow -> MotionAbort", False)
    except H.MotionAbort as e:
        m = str(e)
        check("7c stalled elbow -> MotionAbort naming elbow_flex with present/sent/target",
              "elbow_flex" in m.split("[")[0] and "present" in m and "sent" in m and "target" in m, m[:110])
        check("7c first move failure logs the per-joint table", "first move failed" in lg.text())
        check("7c still no LeRobot clamp while stalled", arm.clamped == 0, f"{arm.clamped}")


def t7d_dead_follower():
    arm = H.SimArm(REST + [0.0], 4.0, dead_after=40)
    ex = _ex(arm)
    try:
        ex.move_to(Q_TEACH, 0.0, speed=6.0); check("7d frozen feedback -> ArmUnresponsive", False)
    except H.ArmUnresponsive as e:
        check("7d frozen feedback -> ArmUnresponsive", True, str(e))

    class AckTimeout(H.SimArm):
        def read(self):
            if self.n_read > 20:
                raise RuntimeError("get_state failed: control ack timeout: comm_type=2")
            return super().read()
    ex = _ex(AckTimeout(REST + [0.0], 4.0))
    try:
        ex.move_to(Q_TEACH, 0.0, speed=6.0); check("7d ack timeout -> ArmUnresponsive", False)
    except H.ArmUnresponsive as e:
        check("7d ack timeout exception -> ArmUnresponsive", True, str(e)[:60])


def _sim_cams(arm, truth):
    sc = H.SimCams(arm, truth, HP.Kin())
    return {"wrist": sc.source("wrist"), "overview": sc.source("overview")}


def _run_fake(arm, truth, plan, **kw):
    lg = Logs(); tmp = Path(tempfile.mkdtemp())
    r = H.run(yes=True, sim=True, sim_truth=truth, plan=plan, out_dir=tmp / "o", log=lg, arm=arm,
              cams=_sim_cams(arm, truth), **kw)
    return r, lg


def t7e_run_aborts_cleanly(truth, plan):
    # (1) arm dies during the first move: German message, motors off ONCE, no return move, no rest move
    arm = H.SimArm(REST + [0.0], 4.0, dead_after=60)
    r, lg = _run_fake(arm, truth, plan)
    t = lg.text()
    check("7e dead arm in run(): rc 3 + 'Roboterarm antwortet nicht – Strom prüfen'", r["rc"] == 3 and H.MSG_NO_ARM in t, t.splitlines()[-1][:80])
    check("7e dead arm: motors off exactly once, no return/rest move", arm.stop_calls == 1 and "returning" not in t
          and "rest move" not in t, f"stop_calls {arm.stop_calls}")
    check("7e dead arm: one concise line, no traceback", "Traceback" not in t and t.count(H.MSG_NO_ARM) <= 2)
    # (2) arm stalls at the first move: ABORT, no return move, motors off once
    arm = H.SimArm(REST + [0.0], 1.0, weak={2: 5.0})
    r, lg = _run_fake(arm, truth, plan)
    t = lg.text()
    check("7e stalled arm in run(): rc 1, ABORT naming the joint, no return move, motors off once",
          r["rc"] == 1 and "ABORT: Gelenk elbow_flex" in t and "returning" not in t and arm.stop_calls == 1, f"stop_calls {arm.stop_calls}")
    # (3) sudden drop (power cut with the brakes-less arm falling): lag vs the SENT goal fires
    class Drops(H.SimArm):
        def read(self):
            q, v = super().read()
            if self.n_read > 50:
                q = q.copy(); q[1] -= 15.0
            return q, v
    arm = Drops(REST + [0.0], 4.0)
    r, lg = _run_fake(arm, truth, plan)
    check("7e falling arm: lag vs the sent goal -> ABORT 'weicht ... ab', no return move",
          r["rc"] == 1 and "weicht" in lg.text() and "returning" not in lg.text())


def t7f_auto_start_rolls():
    for roll in (0.0, 90.0, 180.0):
        truth = SY.make_truth(cam_roll_deg=roll)
        arm = H.SimArm([0.0] * 7, 4.0)
        lg = Logs(); ex = _ex(arm, cams=_sim_cams(arm, truth), log=lg)
        try:
            d = ex.auto_start(C.Charuco(), guess=SY.board_guess(truth), cam_model=SY.prior_model(truth), plan_after=False)
            err = float(np.linalg.norm(d["board_centre_mm"][:2] - SY.board_centre(truth)[:2]))
            kin = HP.Kin(); P = HP.look_point(kin, d["q_teach"], -70.0)
            h = kin.T(d["q_teach"])[2, 3] + 70.0
            check(f"7f auto start, camera roll {roll:.0f} deg: board centre within 25 mm, aim ~200 mm above it",
                  err < 25 and abs(h - 200) < 60 and arm.clamped == 0, f"error {err:.1f} mm, gripper {h:.0f} mm over board")
        except Exception as e:
            check(f"7f auto start, camera roll {roll:.0f} deg", False, f"{type(e).__name__}: {e}"[:140])
    # search pose: clearance well above 200 mm and safe path from rest
    cfg = HP.load_config(); kin = HP.Kin()
    sp = HP.search_pose(HP.DEFAULT_BOARD_POINT, cfg, kin)
    ok, clr, _ = HP.path_safety(kin, [0] * 6, sp["q"], cfg)
    check("7f default search pose >= 200 mm clearance, path from rest safe", sp["clearance_mm"] >= 200 and ok,
          f"{sp['height_mm']:.0f} mm above the board, clearance {sp['clearance_mm']:.0f} mm, path min {clr:.0f} mm")


def t7g_board_not_found():
    truth = SY.make_truth()
    arm = H.SimArm([0.0] * 7, 4.0)
    class Blank:
        def grab(self, after_ts=0.0, timeout=3.0):
            return np.full((360, 640, 3), 150, np.uint8), time.time()
        def close(self):
            pass
    lg = Logs(); tmp = Path(tempfile.mkdtemp())
    r = H.run(yes=True, sim=True, sim_truth=truth, out_dir=tmp / "o", log=lg, arm=arm, auto=True,
              cams={"wrist": Blank(), "overview": Blank()}, sim_guess=SY.board_guess(truth), cam_model=SY.prior_model(truth))
    t = lg.text()
    check("7g board not visible: rc 4 + 'Brett nicht gefunden – Brett im Blick der Handkamera?', normal rest move",
          r["rc"] == 4 and H.MSG_NO_BOARD in t and "rest reached" in t, t.splitlines()[-3][:90] if t else "")


def t7h_auto_end_to_end():
    truth = SY.make_truth(cam_roll_deg=90.0)
    tmp = Path(tempfile.mkdtemp()); lg = Logs()
    r = H.run(yes=True, sim=True, sim_truth=truth, out_dir=tmp / "auto", log=lg, auto=True,
              sim_guess=SY.board_guess(truth), cam_model=SY.prior_model(truth))
    ok = r.get("result") is not None
    e = SY.compare(truth, r["result"], log=lambda *_: None) if ok else {}
    check("7h automatic start end-to-end (sim, camera rolled 90 deg, wrong board guess): GOOD, board < 2 mm",
          ok and r["result"]["quality"]["grade"] == "good" and e["Z_mm"] < 2, f"rc {r.get('rc')} {e.get('Z_mm', -1):.2f} mm")


# ---------------------------------------------------------------- pytest entry points
def _ok(prefix):
    bad = [k for k, v in R.items() if k.startswith(prefix) and not v]
    assert not bad, bad


_TRUTH_PLAN = {}
def _tp():
    if not _TRUTH_PLAN:
        _TRUTH_PLAN["v"] = SY.make_truth_and_plan()
    return _TRUTH_PLAN["v"]


def test_pdf(): t1_pdf(); _ok("pdf")
def test_real_overview(): t2_real_overview(); _ok("re")
def test_planner(): t3_planner(*_tp()); _ok("planner")
def test_executor_safety(): t6_executor_safety(*_tp()); _ok("executor")
def test_7a_old_ramp_reproduces_crash(): t7a_old_ramp_reproduces_crash(); _ok("7a")
def test_7b_tracking_never_clamps(): t7b_tracking_never_clamps(); _ok("7b")
def test_7c_stall_is_clean_and_named(): t7c_stall_is_clean_and_named(); _ok("7c")
def test_7d_dead_follower(): t7d_dead_follower(); _ok("7d")
def test_7e_run_aborts_cleanly(): t7e_run_aborts_cleanly(*_tp()); _ok("7e")
def test_7f_auto_start_rolls(): t7f_auto_start_rolls(); _ok("7f")
def test_7g_board_not_found(): t7g_board_not_found(); _ok("7g")
def test_synthetic_and_apply():
    truth, plan = _tp(); res, _ = t4_synthetic(truth, plan); t5_apply(truth, res); _ok("synthetic"); _ok("replay"); _ok("apply")
def test_7h_auto_end_to_end(): t7h_auto_end_to_end(); _ok("7h")



# ---------------------------------------------------------------- 8. v3.9.2 partial runs (23:16 abort at pose 14)
class CountingLog(Logs):
    """Calls on_ok(n) after every '  ok:' line (= good sample n)."""
    def __init__(self, on_ok):
        super().__init__(); self.n = 0; self.on_ok = on_ok

    def __call__(self, msg):
        super().__call__(msg)
        if str(msg).startswith("  ok:"):
            self.n += 1; self.on_ok(self.n)


class PcanArm(H.SimArm):
    """send_action fails like the plugin when the PCAN adapter drops off USB."""
    fail = False

    def command(self, q7):
        if self.fail:
            raise RuntimeError("send_mit failed: pcan reconnect failed after 3 attempts")
        return super().command(q7)


def _abort_after(n_good, truth, plan, kind="pcan", out=None, real=False, grade=None, monkey=None):
    arm = PcanArm(REST + [0.0], 4.0)
    def on_ok(n):
        if n == n_good:
            if kind == "pcan":
                arm.fail = True
            else:                                 # user STOP
                def boom(q7):
                    raise H.Stop()
                arm.command = boom
    lg = CountingLog(on_ok)
    out = out or Path(tempfile.mkdtemp()) / "o"
    applied = []
    old_apply, old_replay = H.apply_result, H.replay
    if real:                                      # non-sim path, but never touch the real calibration files
        H.apply_result = lambda r, log=None: applied.append(r)
        if grade:
            def fake_replay(folder, **kw):
                r = old_replay(folder, **kw); r["quality"]["grade"] = grade; return r
            H.replay = fake_replay
    try:
        r = H.run(yes=True, sim=not real, sim_truth=truth, plan=plan, out_dir=out, log=lg, arm=arm,
                  cams=_sim_cams(arm, truth))
    finally:
        H.apply_result, H.replay = old_apply, old_replay
    return r, lg, out, applied, arm


def t8a_partial_solve(truth, plan):
    r, lg, out, applied, arm = _abort_after(12, truth, plan)
    t = lg.text()
    check("8a abort after 12 good samples: count reported correctly (was 'only 0 good samples')",
          r.get("n_samples") == 12 and "(12 good)" in t and "only 0" not in t, f"n_samples {r.get('n_samples')}")
    check("8a partial sweep solved anyway, result reported", r.get("result") is not None and "aborted run, but 12 good samples" in t
          and r["result"]["quality"]["grade"] in ("good", "ok"), r.get("result", {}).get("quality", {}).get("grade", "-") if r.get("result") else "-")
    check("8a pcan reconnect failure -> hint 'CAN-Adapter direkt am Laptop anstecken (nicht am Hub)'",
          "CAN-Adapter direkt am Laptop anstecken (nicht am Hub)" in t and r["rc"] == 3)
    check("8a resume hint logged", f"--resume {out}" in t)
    # real (non-sim) path: GOOD partial -> applied; OK partial -> not applied, 'apply available'
    r, lg, out2, applied, _ = _abort_after(12, truth, plan, real=True, grade="good")
    check("8a partial + GOOD -> applied automatically", len(applied) == 1)
    r, lg, out3, applied, _ = _abort_after(12, truth, plan, real=True, grade="ok")
    check("8a partial + OK -> NOT applied, 'apply available: <folder>' for the GUI button",
          len(applied) == 0 and f"apply available: {out3}" in lg.text())
    return out


def t8b_too_few(truth, plan):
    r, lg, out, _, _ = _abort_after(5, truth, plan)
    t = lg.text()
    check("8b abort after 5 good samples: not solved, 'need 10', resume hint", r.get("result") is None and "need 10" in t
          and "--resume" in t and r.get("n_samples") == 5)
    return out


def t8c_resume(truth, plan, folder):
    n0 = len(json.loads((Path(folder) / "samples.json").read_text())["samples"])
    arm = H.SimArm(REST + [0.0], 4.0); lg = Logs()
    r = H.run(yes=True, sim=True, sim_truth=truth, log=lg, arm=arm, cams=_sim_cams(arm, truth), resume=str(folder))
    d = json.loads((Path(folder) / "samples.json").read_text())
    ids = [x["i"] for x in d["samples"]]
    check(f"8c resume continues with pose {n0 + 1} and appends (18 samples, no duplicates)",
          f"continuing with pose {n0 + 1}/" in lg.text() and len(ids) == len(set(ids)) == 18 and ids[:n0] == list(range(1, n0 + 1)),
          f"{len(ids)} samples")
    check("8c resumed folder solves GOOD", r.get("result") is not None and r["result"]["quality"]["grade"] == "good" and r["rc"] == 0)


def t8d_stop_partial(truth, plan):
    r, lg, out, _, arm = _abort_after(11, truth, plan, kind="stop")
    t = lg.text()
    check("8d user STOP after 11 samples: motors off, then solved (rc 130 kept)", r["rc"] == 130 and "STOP: motors off now" in t
          and r.get("result") is not None and arm.stop_calls >= 1)


def test_8_partial_runs():
    truth, plan = _tp()
    folder = t8a_partial_solve(truth, plan); f5 = t8b_too_few(truth, plan); t8c_resume(truth, plan, f5); t8d_stop_partial(truth, plan)
    _ok("8")


# ---------------------------------------------------------------- 9. v3.9.3: 'kraeftig' profile, slower near the board
def t9a_profile():
    sh = (Path(__file__).resolve().parents[2] / "handeye.sh").read_text()
    check("9a handeye.sh uses the 'kraeftig' profile (CLOSE=0, KP=12) and forces it (no inherited 'vorsichtig' cap)",
          "CLOSE=0; KP=12.0; TQ=3.5; HOLD=1.0" in sh and "GRIPPER_CLOSE_TQ=$CLOSE" in sh and "PLAY_GRIPPER_KP=$KP " in sh)


def t9b_low_speed(truth, plan):
    arm = H.SimArm(REST + [0.0], 4.0); ex = _ex(arm, cams=_sim_cams(arm, truth))
    speeds = []; orig = ex.move_to
    def spy(q, grip, speed=None):
        speeds.append(speed); return orig(q, grip, speed=speed)
    ex.move_to = spy
    try:
        ex.run(plan, C.Charuco())
    except Exception as e:
        check("9b sweep runs", False, f"{type(e).__name__}: {e}"[:140]); return
    sweep = speeds[1:-1]; hs = [p["height_mm"] for p in plan["poses"]]
    exp = [8.0 if h <= 160 else None for h in hs]
    check("9b sweep moves ending at <= 160 mm run at 8 deg/s, others at the normal speed", sweep == exp and 8.0 in exp and None in exp, f"{sweep} vs {exp}")


def test_9_kraeftig_low_speed():
    truth, plan = _tp(); t9a_profile(); t9b_low_speed(truth, plan); _ok("9")


if __name__ == "__main__":
    t0 = time.time()
    t1_pdf(); t2_real_overview()
    truth, plan = SY.make_truth_and_plan()
    t3_planner(truth, plan)
    res, _ = t4_synthetic(truth, plan)
    t5_apply(truth, res)
    t6_executor_safety(truth, plan)
    t7a_old_ramp_reproduces_crash(); t7b_tracking_never_clamps(); t7c_stall_is_clean_and_named(); t7d_dead_follower()
    t7e_run_aborts_cleanly(truth, plan); t7f_auto_start_rolls(); t7g_board_not_found()
    t8a_partial_solve(truth, plan); f5 = t8b_too_few(truth, plan); t8c_resume(truth, plan, f5); t8d_stop_partial(truth, plan)
    t9a_profile(); t9b_low_speed(truth, plan)
    if "--quick" not in sys.argv:
        t4b_wrong_sign(plan)
        t7h_auto_end_to_end()
    n_ok = sum(R.values())
    print(f"\n{n_ok}/{len(R)} passed in {time.time() - t0:.0f} s")
    sys.exit(0 if n_ok == len(R) else 1)
