# CHANGELOG – reBot B601-RS on the Mac mini

All times 2026-11-15, Europe/Berlin (CET), taken from file times and session logs in `~/humanoid`.
Snapshot names in parentheses = folders in `versions/` for `./rollback.sh <name>`.

## 12:15 – Installation
- venv `~/humanoid/.venv` with LeRobot 0.7.0, plugin lerobot-robot-seeed-b601 1.0.5, motorbridge 0.6.0.
- Original plugin files: `patches/original/` (hash-checked against the package RECORD) – snapshot `original`.

## 12:58 – first teleop script (`teleop_alt_langsam`)
- `lerobot-teleoperate` with `max_relative_target=5` (all joints max 5°/cycle), leader port hard-coded.

## 13:02 – teleop v2 (`teleop_v2`)
- `max_relative_target` as a dict: arm joints 5°, gripper 25° (the gripper is too sluggish otherwise, scaling ×6).
- Gripper torque limited: 2.0 Nm moving, 0.6 Nm holding.

## 13:04 – GUI v1 (`gui_v1`)
- Tk window with START/STOP; `Robot Teleop.app` starts it with a double click.

## until 14:47 – gripper levels (`teleop_v3`)
- `teleop.sh vorsichtig|mittel|kraeftig`; "vorsichtig" initially kp 4, max 1.5 Nm, hold 0.25 Nm.

## 14:47–14:51 – crash on exit: safe_zero / max_relative_target
- Log: `ValueError: max_relative_target keys must match those of goal_present_pos.`
- Cause: `safe_zero()` sends only subsets of the joints, but the dict from `teleop.sh` contains all 7 keys.
- Fix (plugin `seeed_b601_follower.py`, `send_action`): filter the dict to the keys of the current action.
  From 14:50 on, "safe_zero done" appears in the log.

## 14:47–14:51 – gripper zero (part 1)
- Log: `gripper zero-detect: gripper not homed, offset 13.3 deg` → start aborts.
- Fix: if the gripper is blocked at the closed stop with an offset < 15°, the zero is re-set there
  (`set_zero_position`, MIT mode, enable) instead of aborting. Helpers `rezero_gripper.py` (read-only,
  `--set` sets + saves) and `close_gripper.py`.

## 16:12 – pre-flight check + automatic leader port (teleop v3 → current structure)
- `preflight.py`: finds `/dev/tty.usbserial-*`, connects the follower without calibration, pings all 7 motors.
  Exit 2 = no leader, 3 = no PCAN, 4 = arm not answering (power).
- `teleop.sh`: up to 6 tries of 5 s on exit 4 (arm just switched on), then `--teleop.port=$LEADER_PORT`.

## 16:14–16:17 – STOP: humming fixed (`gui_v2` = before, `gui_v3` = after)
- Problem (log 16:14): STOP sent Ctrl+C, then SIGKILL after 5 s in the middle of the homing move (code −9) → motors
  stayed powered and hummed.
- Fix: STOP sends one SIGINT to the process group, status "◐ FÄHRT IN RUHEPOSITION…", hard kill only after 25 s.
  Afterwards always `disable_motors.py` (all motors torque-free) → "■ GESTOPPT". Protection against double handling
  (`stopping`/`ending`); the window closes only once the arm stands still.
- `test_connect.py` (16:19): connect/disconnect without homing, for testing.

## 16:17–16:24 – gripper zero (part 2): encoder wrap-around
- Log 16:17: `offset -38.34 deg`. If the arm is switched off with the gripper open, the encoder is on the wrong turn
  after power-up (open stop approx. 327° motor away from the closed stop).
- Fix (plugin `detect_gripper_zero`): before the check the gripper always moves gently in the closing direction
  (3°/step, kp 6, kd 0.3, max approx. 1 Nm) until it is blocked for 15 cycles, and sets the zero there.
  Log: "gripper homed at closed stop (was X deg)". Successful at every start from 16:24.
- Snapshot `follower_before_guard` = this plugin state.

## 16:21–16:35 – gripper opens again, holding torque (`rs_follower_patched`)
- Log: repeated `gripper open-stall detected, cutting torque` → the gripper lost its force when opening.
- Fix (plugin `seeed_b601_rs_follower.py`): open-stall cut-off only after 20 instead of 5 cycles; the low hold limit
  applies only when closing, opening always has the full limit (gets free from a grip).

## 16:35–17:56 – gripper "vorsichtig": closing force (`teleop_v4` → `teleop_v5`)
- Problem: with max 1.5 Nm cardboard got crushed – the stall detection (|v| < 0.25 rad/s) does not notice slowly
  yielding objects, so the full limit acts while closing.
- Fix: separate closing limit `GRIPPER_CLOSE_TQ` (environment variable from `teleop.sh`, closing direction only).
- History "vorsichtig": 16:36 close 0.6 / hold 0.2 Nm (kp 4) → 16:38 0.8 / 0.8 Nm (`teleop_v4`): cardboard intact,
  but the gripper no longer closed fully and did not grip properly (kp 4 ≈ friction ~0.35 Nm, stops a few degrees
  before "closed"; 0.8 Nm minus friction = hardly any grip force).
- 17:56: kp 10, close 1.1 Nm, hold 1.0 Nm (`teleop_v5`) – confirmed by David: "works great now".
- mittel: kp 12 / 2.0 / hold 1.2 / close 1.2; kraeftig: kp 12 / 3.5 / 1.0 / no extra limit.

## 18:xx – collision guard for the 6 arm joints (`follower_guard`, `teleop_v6_guard`, `gui_v4_guard`; = `guard_v1`)
- Goal: the arm must not keep pushing when it hits something/someone.
- Why not simply "tracking error > 10°": because of `max_relative_target=5` the sent target is always ≤ 5° from the
  present position; that error can never exceed ~5°. Real logs already show ~3.5–4.5° in normal fast motion.
  Detection instead: leader offset > 10° (shoulder/elbow) or 15° (wrist) AND progress < 0.15°/cycle for 6 cycles.
  Tested against all 18 session logs: 0 false alarms (approximation).
- Reaction: all 6 arm joints hold the present position at detection (normal kp, no sagging, no retreat); the gripper
  keeps working. Release when the leader is within 6° of the position on all joints for 10 cycles.
  Not active during the first 2 s and never during safe_zero (homing on STOP unchanged).
- Log: `KOLLISION erkannt: …`, `KOLLISION aktiv …` (max 1×/s), `Kollision aufgehoben, Teleop läuft weiter`,
  every 5 s `Guard-Stats` (max tracking error / leader offset / |torque| per joint).
- GUI: status turns orange "⚠ KOLLISION – Leader zurückführen", then "● LÄUFT" again.
- `teleop.sh`: `COLLISION_GUARD=1` + thresholds as overridable variables.
- Offline tests: `tests/test_collision_guard.py` (no hardware), `tests/replay_logs.py`.
- 18:40: collision guard switchable in the app (checkbox "Kollisionsschutz an", before start; sets
  COLLISION_GUARD=1/0). Snapshot: `gui_v5_guard_toggle`.

## 19:xx – collision guard v2: compliant rotary joints (`guard_v2_compliant`; before: `guard_v1`)
- Problem (David, log `session_2026-11-15_18-39-13.log`): a hand against the left-right rotation → the arm simply
  pushed the hand away, no alarm. Measured: shoulder_pan offset max 9.8° (threshold 10°), 3.59 Nm; wrist_yaw 7.7°,
  2.66 Nm. Free fast motion without contact (`session_2026-11-15_18-42-05.log`): offset up to 42° / 32°, torque up to
  2.69 / 1.97 Nm → the gap between normal and contact is too small.
- Cause: kp 50 + 5°/step produces enough force (up to ~4.4 Nm) to push a yielding hand away; the joint never stalls,
  so the stall detection never fires.
- Fix plugin `seeed_b601_follower.py`:
  - New: `MIT_KP_<JOINT>` / `MIT_KD_<JOINT>` from the environment override the MIT gains of single arm joints (only
    without gravity compensation; gripper untouched). Logged at the first command: `MIT-Gain-Override …`.
  - The torque trigger has its own counter `COLLISION_TORQ_CYCLES` (default 4 cycles = 133 ms), independent of the
    block counter (6 cycles). Log reason `[torque]` or `[blockiert]`.
- `teleop.sh`: shoulder_pan and wrist_yaw kp 50→20, kd 3→1.9 and 4→2.5 (kd ∝ √kp, same damping); offset threshold
  pan 6°, yaw 8°; torque trigger 1.3 Nm / 1.0 Nm.
- Torque values: at kp 20 the joint can press only ~1.75 Nm statically with the 5° limit (20 × 0.087 rad); the old
  3.2 Nm would never fire. Scaled (×0.4): free 1.08 / 0.79 Nm, contact 1.44 / 1.06 Nm.
- Tests: 40/40 OK (14 new v2 tests); `tests/replay_logs.py` now reads thresholds from the environment; replay of all
  21 logs with v2 thresholds: 0 false alarms (approximation, without torque).
- Backups: `~/humanoid/teleop_v6_backup.sh`, `~/humanoid/seeed_b601_follower.py.patched.v_before_guard2`,
  snapshot `guard_v1`. Way back: `./rollback.sh guard_v1`.

## 19:40 – version 3.0: collision guard v3 + app self-update (`guard_v3`; before: `guard_v2_compliant`)
David's test with v2 (log `session_2026-11-15_19-23-50.log`): shoulder rotation "SUPER", motions much softer.
- wrist_yaw fired `[torque]` 5× briefly (1.00–1.09 Nm at only 2.7–3.6° offset, ~1 s each, released by itself) –
  probably his test according to David; threshold 1.0 Nm kept (free motion at kp 20: max 0.85 Nm).
- shoulder_pan free at kp 20: max 1.19 Nm → torque threshold 1.3 → 1.45 Nm (margin was too small).
- **wrist_flex (wrist up/down) did not stop**: offset 15.3° at threshold 15°, 4.45 Nm, no torque trigger.
  - Gravity at the gripper: ~2.3 Nm at rest (3–3.5° sag at kp 50, several logs).
  - **kp 50 → 40 (not 20–25)**: because of the 5° step limit a joint can press at most kp × 5°. At kp 25 that is
    2.2 Nm < 2.3 Nm gravity → the wrist could no longer lift and would sag away. kp 40: max 3.5 Nm, ~1.2 Nm reserve,
    sag ~3–4°. kd 5 → 4.5 (∝ √kp).
  - `COLLISION_ERR_WRIST_FLEX=10` (8 gave false alarms in the 17:50 cardboard log: the wrist rested 8.3–8.5° off the
    leader; 10 detects David's 19:35 block at ~11.3° instead of 15°), `COLLISION_TORQ_WRIST_FLEX=3.2` Nm.
- **Auto-resume after a collision** (David: elbow blocked `[blockiert]` 12.4 Nm, the arm stopped and never moved again
  because the leader stayed 22° away):
  - Once the frozen arm has been still for 1.5 s (all joints < 0.3°/cycle) it creeps towards the leader: max 1°/cycle
    (instead of 5°), targets start at the frozen point (no sagging), target max 2° ahead of the joint (beyond the
    starting sag) → limited force if the obstacle is still there.
  - Guard stays active: a joint commanded forward that does not move for 4 cycles, or the torque trigger → frozen
    again (`[blockiert beim Weiterfahren]`). Max 3 attempts, then only "bring the leader back".
  - Why not "torque back to normal" as the stillness criterion: when frozen the elbow holds up to ~13 Nm of gravity,
    as much as at detection – torque cannot tell obstacle from gravity there.
  - Log: `Auto-Weiter: Arm faehrt langsam weiter …`, `Kollision aufgehoben (Auto-Weiter) …`. GUI: "⟳ LANGSAM WEITER …".
- **App (teleop_gui.py)**: version number from `~/humanoid/VERSION` (3.0) in the window title and bottom left; button
  "⟳ App neu laden" (os.execv, disabled while running); auto-update every 5 s when `teleop_gui.py` or `VERSION`
  changed – restart while stopped (only once the file is unchanged for one check and compiles), log "Aktualisiert auf
  Version X"; never while teleop runs, then hint "Update verfügbar – wird nach STOPP geladen" and reload right after
  STOP. A broken file is not loaded ("Update fehlerhaft – nicht geladen").
- `VERSION` in `~/humanoid` and in the bundle (snapshots `guard_v1`=1.0, `guard_v2_compliant`=2.0, `guard_v3`=3.0).
- Tests: 74/74 OK. Log replay of all 24 sessions with v3 values: only trigger = David's real wrist_flex block 19:35.
- Backups: `teleop_v7_backup.sh`, `seeed_b601_follower.py.patched.v_before_guard3`, `teleop_gui_v5_before_guard3.py`.
  Way back: `./rollback.sh guard_v2_compliant`.

## 19:55 – version 3.1: auto-resume without dead ends (`guard_v3_1`; before: `guard_v3`)
David's test with 3.0 (log `session_2026-11-15_19-45-18.log`): "80 % top", but after a block the arm stopped following
the leader.
- Cause 1 – false re-freezes while creeping: from 19:46:24 repeated `[blockiert beim Weiterfahren]` without any
  obstacle (wrist_yaw track 1.9–2.0°, 0.53–0.56 Nm; shoulder_pan 0.53 Nm; wrist_flex 1.3 Nm; progress
  0.02–0.13°/cycle). With max 2° lead a kp-20 joint presses only ~0.7 Nm – hardly more than friction – and
  "no progress for 4 cycles" fired.
- Cause 2 – dead end: after 3 failed attempts only "leader within 6° on ALL joints". wrist_roll stood 12.2° off (elbow
  up to 35°) and the message did not name the joint understandably → stuck until STOP.
- Cause 3 – torque-trigger false alarms: wrist_yaw 1.00–1.14 Nm at 2.7–5.4° offset, shoulder_pan 1.37–1.55 Nm at
  3.4–8°, wrist_flex 3.26–3.49 Nm (= kp 40 × 5°, i.e. normal full step).
- Fix plugin:
  - Creeping re-freezes only on a real obstacle: the joint lags its target by more than 3° (compliant joints, kp ≤ 40)
    or 2° (stiff joints), beyond the starting sag, AND does not move for 10 cycles – or the torque trigger fires.
  - Lead per joint: 4° for kp ≤ 40 (enough force against friction, ~1.4 Nm at kp 20), 2.5° for stiff joints (elbow
    ~6.5 Nm extra force against an obstacle).
  - No dead end: unlimited attempts, waits 1.5 s → 3 s → 5 s → every 5 s.
  - Manual release: only the joint that collided must be within 6° of the leader – the rest then follows slowly.
    (Without auto-resume the old all-joints rule applies.)
  - Messages name the joint in German: "Leader: Ellbogen 12.2 deg zurückführen", plus "Auto-Weiter in x s".
  - Torque trigger optionally requires offset ≥ `COLLISION_TORQ_MIN_LEAD_DEG` and progress <
    `COLLISION_TORQ_MAX_PROGRESS_DEG`.
- `teleop.sh`: torque shoulder_pan 1.45 → 1.6 Nm, wrist_yaw 1.0 → 1.25 Nm, wrist_flex 3.2 → off (0),
  `COLLISION_TORQ_CYCLES` 4 → 6, `COLLISION_TORQ_MIN_LEAD_DEG=3`, `COLLISION_TORQ_MAX_PROGRESS_DEG=0.5`. Of today's 22
  torque triggers only 5 remain (shoulder_pan at 5.5–13.8° offset and 1.8–1.9 Nm, nearly standing) – single-line
  samples; the 6-cycle condition comes on top.
- GUI: status "⚠ KOLLISION / <joint> zurückführen oder kurz warten". `VERSION` 3.1 → the open app (3.0) reloads itself
  while stopped.
- Tests: 91/91 OK (new: creeping with friction does not re-freeze – counter-check with 3.0 values does; an obstacle
  while creeping still re-freezes; 7+ failed attempts without dead end with 3/5/5 s waits; the wrist_roll 12° case;
  torque-trigger false alarms from the log). Log replay of all sessions: only David's real wrist_flex block 19:35.
- Backups: `teleop_v8_backup.sh`, `seeed_b601_follower.py.patched.v_before_guard3_1`, `teleop_gui_v6_before_guard3_1.py`.
  Way back: `./rollback.sh guard_v3`.

## 20:00 – documentation in English, TODO list
- README, CHANGELOG, code comments in the bundle scripts, `rollback.sh` and tests translated to English (GUI labels and
  the log lines the GUI matches stay German). New `docs/TODO.md` (ideas and next steps), linked from the README.
- Snapshot `guard_v3_1` contains the translated files; older snapshots keep the files as they were.


## 22:05 – v3.1.1: two live camera feeds in the GUI (`cameras_v3_1_1`)
- New `camera_feeds.py` (next to `teleop_gui.py`) + config `~/humanoid/cameras.json`. The app window (780×830) now
  shows two preview panels between the settings and the log:
  - Kamera 1 = iPhone via the browser bridge (`tools/iphone_web_cam`, MJPEG `http://127.0.0.1:8090/stream.mjpg`).
  - Kamera 2 = placeholder (empty source) – shows "Kein Signal" until a camera is configured.
- Sources per feed: MJPEG/HTTP URL, single-image URL (polled), OpenCV index ("0", "1", … via AVFoundation) or
  empty/"none". Env `TELEOP_CAM1` / `TELEOP_CAM2` override `cameras.json`. `display_fps` (default 15) caps preview work.
- Cannot slow teleop: teleop runs in its own process; each feed is a daemon thread in the GUI process that keeps only
  the newest frame; only shown frames are decoded (PIL draft mode, preview size). Auto-reconnect every 2 s, 3 s stall
  timeout; status line shows incoming fps + resolution or "Kein Signal (reason)". A camera error only disables the
  previews; START/STOP is unaffected.
- Self-update also watches `camera_feeds.py` and `cameras.json` (editing the config reloads the app while stopped).
- Measured: iPhone feed 30 fps in, 1280×720; both feeds together ~17 % of one core in the GUI process.
- Self-test without GUI: `~/humanoid/.venv/bin/python ~/humanoid/camera_feeds.py 5` (saves `outputs/feed1_preview.jpg`).
- Backups: `teleop_gui_v7_before_cameras.py`, `VERSION.bak_before_cameras`. Way back: `./rollback.sh guard_v3_1`
  (GUI without cameras). `rollback.sh` backups now include the camera files.
- 23:05: iPhone bridge phone page: camera picker, resolution and fps selection, choice remembered per channel.


## 22:50 – v3.2: gravity-aware collision guard + optional gravity feedforward (`guard_v3_2`)
- Root cause of the false freezes when lifting (20:48–20:57 logs, e.g. `shoulder_lift err=-11.3 torq=-14.89 Nm
  [blockiert]` at lift 152°/elbow 120°): gravity needs −14.2 Nm there (URDF model; measured −14.9 Nm), but with
  kp 150 and the 5°/cycle step limit the P-term can deliver at most 150 × 5° = 13.1 Nm → the joint sags and cannot
  follow; v3.1 read the lag as "blocked".
- Plugin `seeed_b601_follower.py` (patch script `scripts/dev/patch_v3_2_follower.py`):
  - Gravity model = Seeed `gravity.py` + `assets/00-arm-rs_asm-v3.urdf` (config `gravity_urdf_path`), loaded once,
    g(q) computed once per cycle from the measured motor angles (same convention as Seeed's own feedforward).
  - Torque triggers use the residual torque |measured − g(q)|.
  - Lag triggers ("blockiert", also while creeping) get an extra tolerance of `COLLISION_GRAV_LAG_GAIN` (1.5) × g/kp
    when the joint lags against gravity (≤ ~3° at kp 150), and are skipped when the uncompensated gravity exceeds
    `COLLISION_GRAV_SAT_FRAC` (0.85) of kp × 5° (joint physically cannot follow; one log hint "Schwerkraft: …
    Tipp: GRAVITY_FF=1"). Lag in the direction gravity helps → no tolerance (unchanged).
  - Freeze messages unchanged (GUI regex), plus an info line with g and the residual; stats lines show g/res.
  - Optional feedforward `GRAVITY_FF=1`: adds `GRAVITY_FF_SCALE` (0.8) × g(q) as MIT torque, capped per joint
    (18/18/12/4/4/4 Nm); the guard then only expects the remaining 20 % from the P-term (so real obstacles still
    freeze). Default OFF.
  - `COLLISION_GRAVITY=0` → exact v3.1 behaviour. No model (DM arm, URDF missing) → v3.1 behaviour + warning.
- `teleop.sh`: `COLLISION_GRAVITY=1`, `COLLISION_GRAV_LAG_GAIN=1.5`, `COLLISION_GRAV_SAT_FRAC=0.85`, `GRAVITY_FF=0`,
  `GRAVITY_FF_SCALE=0.8`.
- GUI: new checkbox "Schwerkraft-Ausgleich (hilft beim Heben, Test)" (default off) → `GRAVITY_FF=1`; start log line
  shows it. `VERSION` 3.2.
- Tests: `tests/test_collision_guard.py` 91/91 OK (run with a clean environment – exported `COLLISION_*` variables
  change the env tests); new `tests/test_gravity_v32.py` 24/24 OK (model −14.24/6.87 Nm at [0,152,120,0,0,0];
  the 20:57 freeze no longer triggers while v3.1 does; obstacles with FF, in gravity direction and near the rest pose
  still freeze; residual torque trigger; env switches).
- Log replay (`tests/replay_logs.py`, new option `REPLAY_GRAVITY=1`, teleop.sh thresholds): v3.1 834 events →
  v3.2 464. 21:00 session (lift sag, guard was off) 345 → 6; 19:35 real wrist_flex block still detected (3);
  20:48 session: the shoulder_lift sag freezes are gone; remaining 455 replay cycles are shoulder_pan (vertical axis,
  no gravity; real log had 5 pan freezes at 1.6–2.3 Nm while the lift sagged) → open item, see TODO.
  Replay with `GRAVITY_FF=1` is not meaningful (the logged arm sagged because it had no feedforward).
- Backups: `seeed_b601_follower.py.patched.v_before_guard3_2`, `teleop_v9_backup.sh`,
  `teleop_gui_v8_before_guard3_2.py`. `patches/patched` + `.diff` updated (diff verified on the original).
  Way back: `./rollback.sh cameras_v3_1_1` (v3.1 guard, keeps the camera panels) or `./rollback.sh guard_v3_1`.

## 2026-11-16 00:00–01:00 – v3.4: dataset cleanup, overnight training with timing, policy playback (`play_v3_4`)
- **Recording (v3.3, used tonight):**
  - `recorder.py` (own process, LeRobot 0.7 dataset, 640×360 @ 30 fps, joint data from the telemetry tap) and the GUI
    panel "Datensatz aufnehmen".
  - "✔ Episode speichern" encodes both videos (~9 s for ~1000 frames); the button does not react during that time.
    Wait for "Episode N gespeichert" before pressing STOP.
  - Datasets recorded: `pick_place_v1` (3 episodes; ep1 = failed demo, box missed), `pick_place_v2`, `pick_place_v3`,
    `pick_place_v4` (1 episode each).
- **Camera picker:** the iPhone bridge page (`tools/iphone_web_cam/`) has a camera picker to choose which iPhone camera
  Safari streams. Safari only exposes the 1× main camera (no 0.5× ultra-wide).
- **Episode cleanup:**
  - The failed demo `pick_place_v1` ep1 (1045 frames) is excluded from training. The untouched copy is in
    `datasets/_backup_pick_place_v1_with_fail`.
  - LeRobot 0.7.0 bug: `dataset_tools.delete_episodes` on a dataset merged with `aggregate_datasets` from sources that
    keep one episodes-metadata file per episode fails with `FileNotFoundError …/meta/episodes/chunk-000/file-002.parquet`.
  - The rebuild script `tools/clean_merge.py` (re-encodes the kept frames) crashed on the Mac with `BrokenProcessPool`
    during video encoding.
  - **Workaround in use:**
    - `tools/merge_datasets.py` → `datasets/pick_place_all7` (6 episodes, 8079 frames, ep1 = failed demo).
    - `train_act.sh` takes `EPISODES='[0,2,3,4,5]'` (→ `--dataset.episodes`). Backup: `train_act_v2_before_episodes.sh`.
    - That trains on the 5 clean episodes: 8079 − 1045 = 7034 frames, if the merge order was v1, v2, v3, v4.
- **Training (`train_act.sh`, timing version):**
  - Writes `outputs/train/<run>/timing.json` (live copy `<run>.timing.json`): start/end Europe/Berlin, wall clock,
    steps reached, steps/sec, last loss, ETA. One line goes under "Training runs" below at the end of every run.
  - `DEADLINE=HH:MM` stops gracefully at that time and keeps the newest checkpoint. ACT uses a constant learning rate,
    so an early stop is just a shorter run.
  - `SAVE_FREQ` 2000. `KEEP_CKPT` 3 keeps only the newest checkpoints (~0.6 GB each).
  - Previous version: `train_act_v1_before_timing.sh`.
  - Overnight run: `EPISODES='[0,2,3,4,5]' DEADLINE=05:10 RUN_NAME=act_pick_place_clean_night1 ./train_act.sh pick_place_all7 60000`,
    started ~00:30 CET, ~1.12 steps/s on MPS → about 18–19k steps by 05:10.
- **Policy playback:**
  - `play_policy.py` + `play_policy.sh`, plus the GUI panel "Gelernte Bewegung abspielen" with
    **▶ Policy abspielen** / **■ STOPP (Motoren sofort aus)** (Esc too).
  - Newest checkpoint, guard v3.2 on, 2°/cycle, 60 s limit, refuses while teleop/training runs. See
    `docs/POLICY_PLAYBACK.md`.
  - GUI patch: `tools/patch_gui_play.py` (anchor-based, idempotent).
- **Rollback:** `./rollback.sh before_play_v3_4` (GUI + VERSION as before this change); `play_v3_4` = this version.

## v3.5 (2026-11-16): SmolVLA zero-shot mode with text commands
- `play_policy.py`: new `--policy act|smolvla`, `--task "<text>"`, `--smolvla-model`, `--smolvla-stats`, `--smolvla-map so100|pad7`.
  SmolVLA runs `lerobot/smolvla_base` zero-shot: overview->camera1, gripper->camera2, 6 SO-100-like joints (wrist_yaw held),
  normalization from our `pick_place_all7` stats, actions clipped to our dataset joint range, chunked execution at 30 Hz,
  per-chunk inference latency logged. ACT path unchanged.
- GUI: policy selector (ACT / SmolVLA), text entry for the command (only for SmolVLA), "Trockenlauf (ohne Motoren)" checkbox;
  a real SmolVLA run needs a successful SmolVLA dry run with the same command first.
- venv: `lerobot[smolvla]==0.7.0` extras installed (transformers, num2words, tokenizers ...); pip freeze backup in `backups/`.
- Public reBot B601 datasets downloaded to `~/humanoid/datasets/public/` (not converted yet).
- Docs: `docs/SMOLVLA_ZERO_SHOT.md`. Rollback: `./rollback.sh before_smolvla_v3_5` (snapshots `before_smolvla_v3_5`, `smolvla_v3_5`).

## v3.6 (2026-11-16): SmolVLA command bar in the teleop GUI
- Replaces the cramped v3.5 row (radio + "Trockenlauf" checkbox + small "Befehl" field + one play button)
  with a clear command bar when **SmolVLA (Text-Befehl)** is selected:
  - large text field ("Was soll der Roboter tun?")
  - **🧪 Testen (ohne Motoren)** (dry run) and **▶ Ausführen (Motoren an)** (real run) as two separate buttons
  - visible preset chips, including `pick up the box and put it in the bin`
  - command history (last 8, stored in `logs/smolvla_history.json`)
- Safety gate unchanged in spirit: **Ausführen** stays disabled until a successful **Testen** with exactly
  the same text in this app session. Enter runs Testen, or Ausführen once that text has been tested.
- ACT keeps the v3.4 "▶ Policy abspielen" button; the command bar is hidden for ACT.
- `play_policy.py` / `.sh` unchanged (`--policy act|smolvla --task --dry-run --seconds` as in v3.5).
- Rollback: `./rollback.sh before_gui_v3_6` (snapshots `before_gui_v3_6`, `gui_v3_6`).
  Docs: `docs/GUI_COMMAND_BAR.md`. Reload the app after install ("App neu laden").

## 2026-11-21 – laptop copy (MacBook Pro M1 8 GB, `laptop_v3_6`)
- `~/humanoid` rebuilt from the Mac mini bundles: GUI 3.6 (cameras, recording, policy playback, SmolVLA command bar),
  guard v3.2 follower patch, rs_follower patch, teleop.sh (v3.3 telemetry tap + v3.2 gravity settings), preflight,
  disable/close/rezero scripts, recorder, play_policy v3.5, iPhone bridge, train_act.sh (+ `EPISODES`), docs, tests.
- Python: conda env `lerobot` (LeRobot 0.7.1 editable from `~/lerobot`); `~/humanoid/.venv` is a shim to it.
- Plugin wheels `lerobot-robot-seeed-b601` 1.0.5 and `lerobot-teleoperator-rebot-arm-102` 1.0.1 installed with
  `--no-deps`; original plugin files hash-checked against `patches/original/` before patching.
- PCBUSB in user space (`~/humanoid/lib`, no sudo): `.venv/bin/activate` exports `DYLD_LIBRARY_PATH`, symlink
  `~/.local/lib/libPCBUSB.dylib` (motorbridge search path).
- Not copied: datasets, public datasets, checkpoints, calibration files (Mac mini only). SmolVLA extras not installed.
- Rollback snapshots: `original`, `guard_v3_1`, `guard_v3_2`, `before_play_v3_4`, `play_v3_4`, `before_smolvla_v3_5`,
  `smolvla_v3_5`, `before_gui_v3_6`, `gui_v3_6`, `laptop_v3_6`.

## Training runs (automatic, written by train_act.sh)

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

## 3.9.4 – 2026-11-21 23:58 (laptop) – passive drop diagnostics (no motion change)
- Goal: find out exactly what breaks at the pose 13 -> 14 move (two drops: "pcan reconnect failed", "feedback frozen").
- `vision/handeye.py` `MotorTrace`: with env `HANDEYE_MOTOR_TRACE` (set by handeye.sh to `logs/handeye_<ts>_motors.jsonl`)
  every RealArm.read() writes per motor can_id, pos/vel/torque, t_mos/t_rotor, status_code, RobStride fault_code,
  arbitration id and the driver's feedback age / sequence (cached driver state, no extra bus traffic), plus
  command_error / frozen / stop_now events with ms timestamps. All trace errors are swallowed; control flow unchanged.
- `tools/drop_monitor.sh start|stop|status` + `tools/drop_monitor.py`: passive background monitor -> `logs/dropmon_<ts>/`:
  USB attach/detach/re-enumerate every 0.25 s via `ioreg -p IOUSB` (role tags: CAN adapter, leader serial, wrist/overview
  camera, hubs; `log stream` needs admin, not used), GUI shared-frame stalls per camera (header read only, no device
  access), and timestamped tails of handeye/recorder/GUI logs.
- `tools/drop_report.py [log]`: timeline +-5 s around the failure, last frame per motor, verdict A (CAN adapter lost on
  USB) / B (whole bus silent, adapter present: power or CAN cable to motor 1) / C (only distal motors silent: cable
  between the last answering and first silent motor) / D (motor fault flag), and which vanished first (camera vs CAN).
- Finding while setting up: XCAN-USB, leader USB Serial, C920 and Innomaker all share one USB 2.0 hub 3 levels deep;
  the Innomaker's registry id is much newer than its neighbours' (it has re-enumerated since boot).
- Tests: tools/tests/test_drop_diag.py 6/6 (fake motors, all verdicts); vision/tests/test_handeye.py 15/15.
- Snapshots: versions/before_v3_9_4, versions/v3_9_4.

