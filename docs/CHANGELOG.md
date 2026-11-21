## 3.9.3 – 2026-11-21 (laptop) – auto calibration: 'kraeftig' profile, 8 deg/s near the board
- Two real runs (23:15, 23:35) died on the move pose 13 -> 14 ("pcan reconnect failed", then "motor feedback frozen
  1.0 s"; arm went limp). David's hint: earlier teleop drops happened with the 'vorsichtig' profile.
- Findings: the profiles (teleop.sh vorsichtig/mittel/kraeftig) differ only in gripper settings (kp 10 vs 12, closing
  cap GRIPPER_CLOSE_TQ 1.1 Nm vs none). Arm kp/kd (MIT_*), collision guard and gravity feedforward (x0.8) come from the
  same teleop.sh exports for every profile. The guard did not fire in either run (no KOLLISION lines) and it only
  freezes (holds position), it never switches torque off. The 15:37 teleop drop ('vorsichtig', safe_zero rest move)
  was the same "pcan reconnect failed" + "control ack timeout" pattern. Gravity load at pose 14 is moderate
  (shoulder_lift 8.6 Nm vs 11.6 Nm at other 150 mm poses); pose 5 has the same joints (only wrist_roll differs) and was
  fine in both runs. Pose 14 is the first move of wrist_roll into +20..40 deg.
- Change: handeye.sh now always uses the 'kraeftig' (factory) profile: gripper kp 12, no extra closing cap (the
  gripper is held where it is; inherited GRIPPER_CLOSE_TQ / PLAY_GRIPPER_* values are overridden). It prints
  "Arm-Profil: kraeftig ...". Guard and its safety aborts are unchanged.
- Sweep moves that end at <= 160 mm (`low_height_mm`) run at 8 deg/s (`low_speed_dps`); poses unchanged.
- Sim: a GOOD simulated run no longer prints "quality POOR: NOT applied" (now "simulation: quality GOOD, not applied").
- Tests: vision/tests/test_handeye.py section 9 (profile + low speed); pytest 15/15; `run --sim --auto` GOOD.
  GUI test not run (no GUI change, no visible windows).
- Snapshots: versions/before_v3_9_3, versions/v3_9_3.

## 3.9.2 – 2026-11-21 (laptop) – auto calibration: partial runs, resume, CAN hint
- Run 23:15 aborted at pose 14 ("pcan reconnect failed": CAN adapter dropped off USB) with 13 good samples saved,
  but printed "only 0 good samples" (the count was only taken from a completed sweep). Fixed: samples live on the
  executor and survive any abort.
- After any abort (comm loss, stuck joint, ■ STOPP) with >= 10 good samples (`min_samples_after_abort`) the sweep is
  solved and reported like a normal run; applied automatically only if GOOD. OK → "apply available: <folder>" and the
  GUI button "✓ Übernehmen" (or `./handeye.sh apply <folder>`). POOR → not applied. Exit code 5 = aborted but solved.
- STOP: the motors still go off at once; handeye.sh keeps bash/tee alive so the partial solve can finish and reach the
  GUI (a second STOP ends it). The GUI no longer force-kills once "motors off" was logged.
- Resume: `./handeye.sh run --resume <folder>` continues with the pose after the last good one and appends samples
  (the resume command is logged after an abort).
- "pcan reconnect failed" → treated as comm loss + hint "CAN-Adapter direkt am Laptop anstecken (nicht am Hub)".
- Tests: vision/tests/test_handeye.py section 8 (55/55 script, pytest 14/14); `run --sim` and `run --sim --auto` GOOD.
- Sim runs (`--sim`) log to $TMPDIR/handeye_sim_logs, never into logs/.
- Snapshots: versions/before_v3_9_2, versions/v3_9_2.

## 3.9.1 – 2026-11-21 (laptop) – auto calibration: crash fix + automatic start
- Crash 22:29 (logs/handeye_2026-11-21_22-29-39.log), root cause: the executor sent a precomputed 12°/s ramp regardless
  of the measured position. LeRobot's max_relative_target (1°) clamped every goal to present ± 1° (130 "had to be
  clamped" warnings, elbow_flex), so the elbow got at most 1° of PD lead – too little torque to lift from rest – and
  crept at ~1°/s while the ramp ran ahead. After ~3 s the lag reached 10° → ABORT; the return move hit the same lag
  error; then the arm stopped answering (disable: "control ack timeout comm_type=4", power supply most likely cut out)
  and the plugin printed one traceback chain per motor. 0 samples.
- Motion now tracks reality: each cycle goal = measured position + a step limited by the speed (approach 6°/s, sweep
  12°/s) AND by 0.75 × the follower's own max_relative_target (read from the robot config; now 4°, teleop uses 5), so
  LeRobot never clamps. Lag is measured against the goal actually sent. A joint pinned at its lead without progress for
  2.5 s → "Gelenk <name> kommt nicht voran … [present/sent/target per joint]"; the first move logs the per-joint table.
- Motor communication loss (comm exceptions / ack timeouts, motor without state, full motor state frozen ≥ 1 s) →
  "Roboterarm antwortet nicht – Strom prüfen". On that and on stall/lag aborts: no return or rest move, motors off once,
  plugin tracebacks silenced, one line. Clamp warnings are counted (first 3 shown).
- Automatic start (taught pose optional, GUI default): slow move from wherever the arm is to a high search pose
  (~260 mm above the expected board point: handeye.json > taught pose > [463, -46, -70] mm; path checked from the start
  pose), ChArUco detection in the wrist camera (< 10 corners → "Brett nicht gefunden – Brett im Blick der Handkamera?"),
  board centre = viewing ray of the board centre pixel ∩ plane table_z (camera rotation from handeye.json or estimated
  per image from board normal ↔ gravity, so any camera roll works), move 200 mm above it, re-measure (agree ≤ 25 mm),
  plan the 18 poses while holding (~40 s), sweep. CLI: `handeye.sh plan --auto | run --auto | run --sim --auto`.
  "📍 Ziel-Pose übernehmen" stays as override (ticks "Gespeicherte Ziel-Pose verwenden").
- Board: printed board measured 33 mm per square → board_charuco.json 33 / 24.51 mm, charuco.py fallback the same.
- Tests: vision/tests/test_handeye.py section 7 (clamping fake follower with a weak elbow reproduces the crash with the
  old ramp and passes with the new executor; dead follower; ack-timeout follower; clean run() aborts; automatic start
  with camera rolls 0/90/180°; board not found; automatic end-to-end). Runs under pytest (installed in the venv).
  Synthetic camera→gripper tolerance 3 mm / 0.75° (33 mm board; already failed at 2 mm / 0.5° before this change).
  rebot-B601-RS/tests/test_gui_v39.py: automatic-start window + sim run.
- Snapshots: versions/before_v3_9_1, versions/v3_9_1.

## 3.9 – 2026-11-21 (laptop) – automatic hand-eye calibration
- 🤖 Auto-Kalibrierung: the arm moves the wrist camera to 18 planned poses above the taped ChArUco board (150–230 mm,
  tilt ≤ 25°, roll ±35°, floor 80 mm checked for all joint-sign conventions, max 12°/s, guard v3.3 on). OpenCV hand-eye
  (7 methods + LM refinement) gives wrist intrinsics, camera→gripper, board→robot (= table→robot, replaces step c)
  and overview camera→robot. Confirmation window with plot + pose list; motion only after 3 ticks + click;
  ■ STOPP Auto-Kalibrierung / Esc = motors off. CLI `handeye.sh teach|plan|run|replay|apply|status|run --sim`.
- ChArUco board (7×5, 35 mm, DICT_5X5_100) is the default board: 📐 Neu kalibrieren works with partial views and the
  corner order can't flip. Manual corner touch (🤝) stays as fallback.
- Fix: after an automatic app reload the GUI waited for shared frames instead of opening the cameras ("Kein Signal").
- play_policy refuses while the auto calibration runs. Docs: docs/APP_V3_9.md, OPERATING_PROCEDURE.md section 8.
- Snapshots: versions/before_v3_9, versions/v3_9.

## 3.8 – 2026-11-21 (laptop)
- Session recording with live markers: ⏺ Session aufnehmen, ✓ geklappt (S) / ✗ fehlgeschlagen (F) /
  ↻ Neuer Versuch (N) / 🗑 Verwerfen (D). Asynchronous saving, per-episode label + task in
  meta/episode_labels.jsonl, export tool tools/export_dataset.py (success-only copies).
- Cameras open once: the GUI shares 640x360 frames via shared memory with recorder.py and play_policy.py.
- Vision tools: 📐 Neu kalibrieren (recalibrate_table on the overview frame), 🔍 detection overlay (YOLO-World,
  MPS, max 8 fps), 🤝 Arm mit Tisch verbinden (step c, telemetry forwarded to port 8766).
- Faster saves (in-process video encoding, ~2 s instead of 13-17 s per short episode); scrollable window that fits 1080p.
- Installed lerobot[dataset] (datasets was missing, so recording could not work on the laptop); numpy/opencv pins kept.
- Docs: docs/APP_V3_8.md, ~/humanoid/docs/OPERATING_PROCEDURE.md.

## 3.7 – 2026-11-21 (laptop) – guard v3.3
- Guard v3.3: no false 'blockiert' freezes when lifting overhead (a stall that gravity explains is not a collision,
  COLLISION_GRAV_RES_NM 4.0 / wrist_flex 1.5). Gravity feedforward on (GRAVITY_FF=1, x0.8, 1 s ramp, tighter caps),
  wrist_yaw torque trigger 1.25 -> 2.4 Nm. Replay of all logged freezes: tests/replay_freezes_v33.py.
- Gripper opens fully again: opening limit 3.5 Nm (vorsichtig/mittel, also play_policy). Closing stays capped
  (1.1/1.2 Nm). The open-stall cut-off now only acts near the open stop.
- safe_zero (rest move on Stop) is closed-loop: it follows the measured position (never >3 deg ahead), uses gravity
  feedforward, logs stalls/timeouts, and aborts clearly on frozen motor feedback. If rest isn't reached it releases
  softly (gains ramp to 0, extra damping) instead of switching off hard.
- App: Stop shows feedback right away (◐ stoppt…) and escalates SIGINT -> SIGTERM 25 s -> SIGKILL 32 s. The log
  window is filtered and capped (CPU). Camera reconnects back off (2…30 s).
- Cameras by name (`name:C920`, `name:Innomaker|U20CAM`), never the FaceTime camera. Roles in cameras.json; the
  recorder and play_policy map the cameras by role. ⤢ enlarge window per camera (double-click, ✕/Esc).
- Snapshots: versions/before_v3_7, versions/v3_7 (rollback.sh). Docs: docs/GUARD_V3_3.md.


## 3.6.1 (2026-11-21)
- Camera names fixed: Kamera 1 = gripper (Innomaker, index 0), Kamera 2 = overview (C920, index 1).

## vision 1.0 (2026-11-21)
- Added `vision/detector.py` (YOLO-World yolov8s-worldv2 via ultralytics, MPS/CPU), `vision/detect_live.py`, `vision/README.md`.
- Installed ultralytics 8.4.216 + ultralytics CLIP in the lerobot env; numpy 2.2.6 and opencv-headless 4.13.0 re-pinned.
- No changes to teleop_gui.py, camera_feeds.py, recorder.py or the robot plugins.
- vision/calibrate.py: intrinsics, fast table-plane recalibration (`recalibrate_table(source)`), table->robot fit, timestamped history (last 10) with mm shift report, load-time warnings.
- vision/robot_fk.py (URDF forward kinematics) and `calibrate.py robot-capture` (touch board corners, read joints from the teleop UDP telemetry tap, fit table->robot incl. mirror and joint-sign check, residual in mm). Synthetic tests only.
