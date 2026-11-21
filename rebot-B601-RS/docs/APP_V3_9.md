# App 3.9: automatic hand-eye calibration ("🤖 Auto-Kalibrierung")

Installed on the laptop (<laptop>) on 2026-11-21. Builds on 3.8. GUI labels German, docs English.
Replaces the manual corner-touch step (c), which failed in practice (the board slid, and the 9×6 checkerboard's corner
order flips when the board is rotated). Runbook: `~/humanoid/docs/OPERATING_PROCEDURE.md` section 8.
Snapshots: `versions/before_v3_9`, `versions/v3_9` (`./rollback.sh before_v3_9`).

## Files
| File | Purpose |
|---|---|
| `vision/calibration/board_charuco.json` | board: ChArUco 7×5 squares, 35 mm, markers 26 mm, DICT_5X5_100 |
| `vision/charuco.py` | board definition, detection (2× upscale fallback for 640×360 frames), pose, exact rendering |
| `vision/handeye_plan.py` | kinematics wrapper (robot_fk, URDF ∩ plugin joint limits), DLS IK, pose planner, safety checks, plot |
| `vision/handeye_solve.py` | intrinsics, `cv2.calibrateHandEye` (5 methods) + `calibrateRobotWorldHandEye` (2), LM refinement, quality |
| `vision/handeye.py` | CLI: teach / plan / run [--sim] / replay / apply / status; executor; camera sources |
| `vision/handeye_synth.py` | synthetic ground truth for tests (rendered board, known transforms) |
| `vision/calibration/handeye_config.json` | heights, tilts, rolls, floor, speed, tolerances, camera prior |
| `handeye.sh` | wrapper with the teleop.sh guard exports (like play_policy.sh), log `logs/handeye_*.log` |
| `gui_v39.py` | the GUI box (teach, plan + confirmation window, run, STOPP/Esc) |
| `vision/calibrate.py` | ChArUco support in recalibrate_table / check, `apply_handeye`, 3D `table_to_robot_xyz` |
| `vision/tests/test_handeye.py`, `tests/test_gui_v39.py` | offline tests |

## Method (best practice, no new math)
1. **Teach** (read-only): joints of a pose where the wrist camera sees the whole board, from the teleop telemetry
   (recorder forward port 8766). The aim point = where the finger axis hits the board height.
2. **Plan** (offline): candidates on a spherical cap above the aim point: height 150/190/230/270 mm, tilt 0/12/25° in
   8 directions, roll −35/−12/+12/+35° about the view axis. DLS IK seeded from the taught joints. Rejected if: IK error,
   outside URDF ∩ plugin limits minus 5°, wrist roll > 70° from the taught roll, wrist joints / gripper_end / camera
   prior lower than 80 mm over the board for ANY sign convention of pan/yaw/roll, board not in the prior camera cone.
   18 poses by farthest-point sampling in rotation, ordered by joint travel; every joint path (teach → poses → teach,
   and current → teach at run time) is sampled every 1° and checked against the same floor.
   With the step-c board position (≈ 350 mm in front, 70 mm below the base plane) straight-down views above ~200 mm
   need wrist_flex beyond −80°, so the higher poses are the tilted ones (the planner finds this by itself).
3. **Run**: refuses while teleop / policy / training runs. Follower built exactly like play_policy (`make_robot`,
   guard v3.3 env from teleop.sh, gripper held), plugin step cap 1°/cycle, own smoothstep profile max 12°/s.
   Commands are raw motor degrees divided by `joint_directions` (observation and action conventions differ for
   elbow, wrist_flex, wrist_yaw). Per pose: reach (2°, compliant joints 3°, timeout 8 s), abort if the arm lags > 10°
   behind (blocked/guard), settle 1 s, median joints, two wrist frames 0.25 s apart (corner motion < 0.7 px, ≥ 10
   corners) + one overview frame. No board at poses 1–2 → abort (aim wrong). SIGTERM/SIGINT = torque off at once;
   other ends = back to the taught pose, plugin rest move, torque off. Time limit 10 min.
4. **Solve**: wrist intrinsics from the sweep (outlier views > 3× median dropped), board→cam by solvePnP (IPPE + LM),
   all 7 hand-eye methods, best chain reprojection → joint LM refinement of X (cam→gripper) and Z (board→base) on
   the corner reprojection. 16 joint-sign conventions are tried; a non-default one is used only if < 0.5× the error.
   Overview: mean board pose over all overview images → `T_overview2base`.
5. **Apply** (GOOD/OK only): `handeye.json`, `table.json` → `table_to_robot` {2D R/t for old callers, `z_mm`,
   `T_board2base`}, `T_overview2base`. `recalibrate_table` keeps it as long as the board type is the same.

## Test results (2026-11-21, laptop, no hardware)
- `vision/tests/test_handeye.py`: 25/25 pass (≈ 70 s): PDF render 24/24 corners, square 34.98 mm on paper, ids stable
  under 180° rotation; real 640×360 overview frame of the taped board 24/24 corners, recalibrate 0.42 px; planner limits /
  floor / paths; synthetic end-to-end through the executor; apply; stop flag; blocked arm; wrong wrist_yaw sign detected.
- Synthetic truth recovery (wide-angle 640×360 wrist camera, k1 = −0.30, noise + blur, 18 poses):
  T_cam2gripper 0.62 mm / 0.29°, T_board2base 0.64 mm / 0.27°, board corners ≤ 0.56 mm, fx error 0.6 px,
  overview pixel→table ≤ 2.0 mm. Chain reprojection 0.27 px, AX=XB median 0.33° / 1.9 mm.
- `tests/test_gui_v39.py` (staging copy, sim run): 21/21 OK: buttons enabled/disabled by state, teach from fake
  telemetry, plan → confirmation window, "Jetzt bewegen" only after 3 ticks, nothing started before the click, reload
  and teleop START blocked while running, sim result GOOD, STOPP ends the run in 0.1 s.

## Not verified on the real arm yet
- The real wrist camera's view direction vs the finger axis (planner assumes roughly parallel, 30° cone).
- That the plugin follows `send_action` targets without a leader the same way at 12°/s (play_policy does the same).
- FK accuracy of the URDF on this arm (the hand-eye residuals will show it) and the sign of shoulder_pan / wrist_yaw /
  wrist_roll (the solver reports it). wrist_flex/lift/elbow signs match the step-c readings.
- Rest pose → taught pose path: checked with FK only.
