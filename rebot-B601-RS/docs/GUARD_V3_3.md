# Guard v3.3 / app 3.7 (2026-11-21, laptop)

Fixes for false collision freezes, the half-opening gripper, the slow Stop button and the failed rest move
seen in today's laptop sessions (`logs/session_2026-11-21_13-43-15.log`, `_13-52-55.log`, `_13-58-23.log`).

## 1. Gravity feedforward on by default (`GRAVITY_FF=1`)
- Without feedforward, lifting overhead needs the whole gravity torque from the P-term: up to 11.4 Nm logged
  (model max 15.4 Nm) versus kp 150 x 5 deg step limit = 13.1 Nm. The shoulder stalled and the guard froze it.
- Now `0.8 x g(q)` (URDF model) is sent as MIT torque feedforward, ramped in over `GRAVITY_FF_RAMP_S` (1 s) after
  connect, capped per joint: shoulder_lift 14, elbow_flex 9, wrist_flex 3, shoulder_pan/wrist_yaw/wrist_roll 1 Nm.
- Model check against the logs (5 s Guard-Stats, joint holding still): the model has the sign of the measured
  holding torque on shoulder_lift / elbow_flex / wrist_flex; measured is about 1.15-1.18x the model (friction,
  dynamics), so 0.8x never overcompensates.
- Elbow: the big residuals (up to 10.7 Nm) come from the rest pose. The arm lies on its mechanical stop there,
  so the motor torque is ~0.1 Nm while the model says 6.8 Nm. When the arm is held up, the elbow model fits
  (residual 0.6-1.5 Nm with g 6.4-6.8 Nm). **No sign or magnitude change was needed.**

## 2. Gravity-explained stall (guard v3.3)
- For `COLLISION_GRAV_RES_JOINTS` (teleop.sh: shoulder_lift, elbow_flex, wrist_flex), when a joint lags AGAINST
  gravity, the lag trigger ("blockiert") only counts if the joint presses more than `COLLISION_GRAV_RES_NM` (4 Nm;
  wrist_flex `COLLISION_GRAV_RES_WRIST_FLEX` = 1.5 Nm) beyond the expected gravity torque in the direction of motion.
- That residual push is the most a stalled joint can put on an obstacle. Today's false freezes pushed
  1.57 / 1.72 Nm (shoulder_lift) and 1.38 Nm (wrist_flex). With feedforward on, a real block pushes
  ~kp*5deg - 0.2*g (lift/elbow >= 10 Nm, wrist_flex ~3 Nm), so it is still caught. A lag in the gravity-assisted
  direction (pressing down onto something) is unchanged.
- `COLLISION_GRAV_RES_NM=0` = v3.2 behaviour.

## 3. wrist_yaw torque trigger 1.25 -> 2.4 Nm
- 11 false freezes today at 1.37-2.10 Nm (max 2.16 Nm in the stats) during normal moves.
- At kp 20 a static block can only press kp x 5 deg = 1.75 Nm. So a wrist held by hand is now caught by the lag
  trigger (8 deg, 6 cycles; simulated: freeze after 10 cycles, leader 20 deg ahead), and the torque trigger only
  catches hard dynamic hits (2.8 Nm -> freeze after 6 cycles).
- Not changed: shoulder_pan stays at 1.6 Nm (one borderline event at 1.61 Nm at 13:54:46; same physics, max
  static 1.75 Nm. Raise `COLLISION_TORQ_SHOULDER_PAN` if it repeats).

## 4. Gripper opens fully again
- Cause: in "vorsichtig" the opening torque limit was 1.5 Nm. The gripper stalled mid-stroke (75-100 deg and
  150-175 deg; the open stop is ~270 deg). After 20 stalled cycles the open-stall protection cut the torque to 0
  and **latched** while the leader stayed open: 1276 `gripper open-stall detected` lines in session 13:43.
  In "kraeftig" (3.5 Nm, session 13:52) the same gripper opened to ~245-270 deg. The calibration files are not the
  cause: the leader plugin clamps with `config.joint_ranges` (gripper 0-270), not the calibration range.
- Fix: the open-stall cut-off only applies near the open stop (`GRIPPER_OPEN_STALL_ZONE_DEG`, 35 deg). Mid-stroke
  stalls keep the opening torque and are logged once. In teleop.sh, vorsichtig/mittel TQ (opening) is now 3.5 Nm.
  The closing caps are unchanged (1.1 / 1.2 Nm via `GRIPPER_CLOSE_TQ`, hold 1.0 / 1.2 Nm).

## 5. Rest move on Stop (`safe_zero`)
- 14:09: the leader lost its USB/power (`servo 0 lost 45 consecutive responses`). The follower's feedback froze:
  every joint's position was identical to 14 decimals for 9 s, including the gravity-free wrist joints. The old
  open-loop ramp ran anyway, logged `safe_zero done` and left the arm overhead (lift 132, elbow 60).
- New closed-loop rest move:
  - The motors are re-read every frame. The command advances 0.5 deg per frame and never more than 3 deg ahead of
    the **measured** position.
  - Frozen feedback (all motor states bit-identical for 0.6 s while moving) aborts with
    `safe_zero ... ABGEBROCHEN: keine Motor-Rueckmeldung`.
  - Stall (3 s without progress) and timeout (20 s per stage) are logged with joint, position and gravity torque.
  - Gravity feedforward is used during the rest move (`SAFE_ZERO_GRAVITY_FF=1`).
- Torque-off: if rest was reached (all arm joints within 8 deg of 0), the motors are switched off normally.
  If not, there is a soft release: for 3 s (`SOFT_RELEASE_S`) kp ramps to 0 and the feedforward fades out, with
  kd >= 20 on shoulder_lift/elbow, then 1.5 s damping only, then torque-off.
- Env: `SAFE_ZERO_STEP_DEG`, `SAFE_ZERO_LEAD_DEG`, `SAFE_ZERO_TIMEOUT_S`, `SAFE_ZERO_STALE_S`, `SAFE_ZERO_STALL_S`,
  `SAFE_ZERO_TOL_DEG`, `SAFE_ZERO_REST_TOL_DEG`, `SOFT_RELEASE_S`.

## 6. App (teleop_gui.py / camera_feeds.py)
- 92% CPU: the log window was a Tk text widget filled with the 30 Hz `Teleop loop time` lines and clamp dumps
  (23k+ lines per session). Measured on the laptop: inserts at 30 Hz cost 20% CPU with 100 lines and 61% with
  60k lines. Now the window only shows useful lines (loop time every ~5 s, no clamp dumps), keeps at most 1500
  lines and inserts in one batch per 200 ms. The session log file still gets every line.
- Stop: the button immediately shows "◐ stoppt…" and is disabled, the status shows the elapsed seconds.
  SIGINT (clean stop + rest move) -> SIGTERM after 25 s -> SIGKILL after 32 s.
- `disable_motors.py` timeout 20 -> 90 s (on this laptop the import alone takes 35-55 s; it timed out at 13:51).
- Cameras: failing sources (no camera permission) back off 2, 4, 8, 16, 30 s instead of retrying every 2 s.

## Tests
- `tests/test_guard_v33.py`: today's freeze events (v3.2 vs v3.3), simulated real blocks, feedforward ramp/caps,
  gripper, safe_zero (14:09 replay with frozen feedback, overhead pose with live feedback, blocked lift + soft
  release), teleop.sh defaults.
- `tests/replay_freezes_v33.py <follower.py> logs/session_*.log`: every logged freeze through v3.2 and v3.3
  settings, plus the model-vs-measured check.

## Rollback
`~/humanoid/rebot-B601-RS/rollback.sh before_v3_7` (press STOP first). `v3_7` = this version.

## 7. Cameras by name (camera_feeds.py, cameras.json)
- USB camera indices shift when cameras are re-plugged, and the built-in FaceTime camera takes an index too.
  New source syntax in `cameras.json`: `"source": "name:C920"` or `"name:Innomaker|U20CAM"`
  (|-separated alternatives, case-insensitive substring of the device name).
- The name is resolved to an OpenCV index on every (re)connect. OpenCV's AVFoundation backend sorts the video
  devices by uniqueID (`cap_avfoundation_mac.mm`), so `camera_feeds.list_cameras()` sorts
  `system_profiler SPCameraDataType` the same way (USB IDs `0x…` sort before the FaceTime UUID, which matches
  the 3.6.1 check: index 0 = Innomaker, index 1 = C920). `~/humanoid/.venv/bin/python ~/humanoid/camera_feeds.py --list`
  shows the list and what each feed resolves to.
- The built-in camera (FaceTime) is never used unless a feed asks for it by name. A numeric index that
  points at it is refused (`CAM_ALLOW_BUILTIN=1` to override). If a named camera is missing, the panel shows
  "nicht angeschlossen (gefunden: …)" and retries with back-off (2, 4, 8 … 30 s).
- `cameras.json` keeps the 3.6.1 display order (Kamera 1 = Greifer/Innomaker, Kamera 2 = Übersicht/C920) and
  adds `"role": "gripper"/"overview"`. recorder.py and play_policy.py now pick the cameras by role
  (`camera_feeds.by_role`), not by position. Before, they stored feed 1 as `observation.images.overview`, so
  with the 3.6.1 order the wrist view would have been recorded as "overview". Without a role the name is used
  (Greifer/Übersicht), and if that doesn't work either, the order is used.

## 8. Enlarge a camera (⤢)
- Each camera panel has a ⤢ button (or double-click the picture). It opens a large resizable window with that feed
  live: fit to the window, aspect ratio kept, up to 1920×1080. Close it with ✕ or Esc; the feed keeps running.
- It uses the same newest-frame buffer and decodes only when a new frame arrived or the window was resized,
  at display_fps (15 fps). Clicking ⤢ again brings the open window to the front instead of opening a second one.

## 9. Policy play gripper (play_policy.py)
- Same half-open cause as in teleop: the opening limit was 1.5 Nm. Now 3.5 Nm (PLAY_GRIPPER_TQ), and closing is
  capped at 1.1 Nm through GRIPPER_CLOSE_TQ (PLAY_GRIPPER_CLOSE_TQ), the same as teleop 'vorsichtig'.
