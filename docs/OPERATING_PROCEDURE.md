# Operating procedure: humanoid arm on the laptop (app 3.9)

Runbook for David. Laptop: MacBook Air M1 (SH-0000-0052), app "Robot Teleop" (desktop icon), code in `~/humanoid`.
Emergency stop at any time: **switch off the power strip** of the arm.

---------------------------------------------------------------------------------------------------------
## 1. Startup
1. Both arms in the rest pose (follower folded down, leader in the same pose).
2. Plug in (USB hub is fine, but see section 9): leader arm (white, USB serial), PCAN CAN adapter (follower),
   cameras Innomaker (wrist) and Logitech C920 (overview).
3. Switch on the follower power and wait 5 s.
4. Open **Robot Teleop** on the desktop. The title shows the version (3.9).
5. Check the two camera panels: Kamera 1 = Greifer, Kamera 2 = Übersicht. The status line shows fps, resolution and the
   resolved device (`#0 Innomaker-U20CAM…`, `#1 HD Pro Webcam C920`). "nicht angeschlossen (gefunden: …)" means the
   camera is missing (section 9). The built-in FaceTime camera is never used.

## 2. Teleoperation
1. Choose the gripper level (Vorsichtig is the default: closes gently, max 1.1 Nm; opens fully, 3.5 Nm).
2. Leave "Kollisionsschutz an" checked.
3. Press **▶ START**. The motors switch on audibly. The leader now drives the follower.
4. Press **■ STOPP** to end. The button changes to "◐ stoppt…" at once and the arm drives to its rest pose (up to
   ~20 s per stage). If it doesn't finish, the app escalates: SIGTERM after 25 s, SIGKILL after 32 s, then all motors off.
   Don't press Stop twice.

## 3. First real-arm checks after 3.7 (do these once, carefully)
Watch the arm with a hand on the power strip. Check the session log afterwards (`~/humanoid/logs/session_*.log`).
1. **Overhead lift**: slowly lift the arm overhead with the leader and hold it there for 5 s, then lower it.
   Expected: no "blockiert" freeze. The log may show a "Schwerkraft: <joint> haengt … hinterher, drueckt aber nur … Nm" line (a lag explained by gravity, not a collision).
2. **Real block still detected**: gently push against the forearm or block it with a soft object while moving.
   Expected: the guard freezes the arm ("blockiert"/"torque"). Moving the leader back releases it.
3. **Wrist yaw**: turn the wrist quickly both ways. Expected: no false "torque" freezes (threshold now 2.4 Nm).
4. **Gripper**: open the leader gripper fully. Expected: the follower opens fully (not halfway). Closing on a soft
   object stays gentle.
5. **Stop from overhead**: with the arm overhead, press ■ STOPP. Expected: the arm comes down smoothly to rest. The log shows
   `safe_zero stage… erreicht` and `safe_zero done (Ruheposition erreicht)`. If the rest pose isn't reached, the log says why:
   `STILLSTAND` (stall), `TIMEOUT` or `ABGEBROCHEN: keine Motor-Rueckmeldung` (no motor feedback, e.g. power off). The arm then releases
   softly instead of dropping.
6. If any check fails, press STOPP, note the time, and roll back if needed (section 10).

## 4. Recording with live markers (session mode)
1. Start teleop (▶ START). Enter the **Datensatz** name (e.g. `pick_place_v2`) and the **Aufgabe** text.
2. Press **⏺ Session aufnehmen**. The status shows "● Versuch 1 00:03.2".
3. Do one attempt, then mark it (button or key; click outside the text fields first so the keys work):
   - **✓ geklappt (S)**: success. **✗ fehlgeschlagen (F)**: fail.
   - **↻ Neuer Versuch (N)**: keep it as `play` and start over. **🗑 Verwerfen (D)**: drop it.
   The next attempt starts immediately. Saving runs in the background ("⏳ speichert 1").
4. Press **■ Session beenden** when done (the running attempt is kept as `play` if ≥ 1 s).
   Wait until "speichert" disappears before closing the app.
5. Stopping teleop pauses the session (frames are kept). Start again and continue, or end the session.
6. Data: `~/humanoid/datasets/<name>/`, labels in `meta/episode_labels.jsonl`.
7. Export the successful attempts for training:
   `~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py <name>` gives `<name>_success`.
   (`--list` shows all episodes with labels.)
Classic mode ("● Episode starten" / "✔ speichern" / "✖ Verwerfen") still works, but not during a session.

## 5. Camera enlarge
Click **⤢** next to a camera status line, or double-click the picture. This opens a large live window (resizable, up to
1920×1080). Close it with ✕ or Esc. The feed keeps running.

## 6. Recalibrate the table (each session, or after the overview camera moved)
1. The **ChArUco board** (7×5 squares, 35 mm, printed from `vision/calibration/charuco_7x5_35mm_A4.pdf` at 100 %)
   stays taped at its fixed spot (section 8). Make sure the C920 sees it. Arm or cables may cover a few squares: the
   corners carry ids, so partial views work and the corner order can't flip.
2. Press **📐 Neu kalibrieren**.
3. Read the result: "Table calibrated (reprojection x px). Shift vs previous: mean … mm, max … mm".
   A shift above 20 mm gives a warning: the camera moved (fine, that's what this fixes) or the board moved (then
   the robot link is wrong: redo the auto calibration, section 8). "Board not found" means: avoid glare, retry.
4. The board can stay on the table while working, as long as nothing pushes it.
The old 9×6 checkerboard still works if `vision/calibration/board.json` names it, but don't mix boards: after
changing the board type the robot link is not reused.

## 7. Detection overlay
1. Type the objects into the field next to **🔍 Erkennung**, comma-separated, e.g. `blue ball, Nivea tin, trash bin`.
2. Check **🔍 Erkennung**. The first start loads the model (~10–20 s). Yellow boxes then appear on the overview camera
   (and in its ⤢ window) at up to 8 fps. Changing the text takes effect within a second.
3. Uncheck it to stop and free memory (~1.2 GB). On the 8 GB laptop, keep it off while recording long sessions.

## 8. Auto calibration (arm ↔ cameras ↔ table, "🤖 Auto-Kalibrierung", new in 3.9)
What it does: the arm moves the wrist camera by itself to ~18 poses above the board (150–230 mm high, tilted up to
25°, rolled ±35°). At each pose it stores the joint angles and both camera images. From that it computes (OpenCV
hand-eye calibration, the standard method of easy_handeye / MoveIt): the wrist camera's lens model, where the wrist
camera sits on the gripper, where the board lies in robot coordinates (= table → robot) and where the overview
camera is. It replaces the manual corner touching (now the fallback, section 8b). About 2 minutes.

**Once: tape the board**
1. Print `vision/calibration/charuco_7x5_35mm_A4.pdf` at 100 % (no "fit to page"). Measure one square: it should be
   35.0 mm (the board taped on 2026-11-21 measures 33 mm – printer scaled to ~94 %; board_charuco.json says 33 / 24.51). If not, put the measured value into `vision/calibration/board_charuco.json` (`square_mm`, and scale
   `marker_mm` = 26 × measured / 35).
2. Tape it **flat** on all four sides at its fixed spot in front of the arm, where the arm reached it in the step-c
   attempt (about 35 cm in front of the base). Glue it on cardboard if the paper waves. It must not move afterwards:
   if it moves, the robot link is wrong until the next auto calibration.

**Each auto calibration (first time: with David at the arm, hand on Stop)** – v3.9.1: no taught pose needed
1. Arm power on (check the power supply LED), app open, both camera panels show fps (wrist camera = Kamera 1 must show
   a picture). Teleop stopped. The arm may be at rest; it is even better if it already hangs roughly above the board.
2. Press **🤖 Auto-Kalibrierung …**. With "Gespeicherte Ziel-Pose verwenden" unticked (default) it checks the search
   pose offline (~1 s, nothing moves) and opens the confirmation window.
3. Tick the three checks (board taped, area free, hand on Stop) and click **▶ Jetzt bewegen**. Only now does the arm move:
   - slowly (6°/s) from wherever it is to a high **search pose** (~260 mm above the expected board point; expected point =
     last hand-eye result, else the last taught pose, else [463, -46, -70] mm),
   - it looks for the board in Kamera 1 (≥ 10 corners; otherwise "Brett nicht gefunden – Brett im Blick der
     Handkamera?", rest move, motors off),
   - moves 200 mm above the board centre, measures again (the estimates must agree within 25 mm),
   - plans the 18 poses there (~40 s, the arm holds still), then moves pose by pose (max 12°/s, ~1.5 s per photo).
4. **Stop** at any time: **■ STOPP Auto-Kalibrierung** (or Esc) = motors off immediately (the arm sags; it is ≥ 80 mm
   above the board). The power strip is still the emergency stop. Teleop START, policy and app reload are blocked.
   **⏸ Stopp & zurück** (v3.9.5) = soft stop: the sweep ends, the samples are saved (solved if ≥ 10, otherwise
   `--resume` is offered), the arm drives slowly (6 °/s, same clamp and collision guard) back to the taught pose and
   the start pose of the run, then the rest move, then motors off. Every other abort while the motors still answer
   (camera outage, time limit) ends the same way. If the arm cannot move, it falls back to motors off.
   **Camera outage** (v3.9.5): the arm holds the pose and the motors keep being read; the log shows "Kamera weg –
   warte…" and "Kamera wieder da – weiter". After 60 s the same pose is retried; after 90 s in total the run ends with
   the soft stop. **Watchdog**: no motor read for 1 s -> "WATCHDOG: …" with a stack dump in the log; after 10 s (or
   2 s after STOP while the control loop is blocked) the watchdog switches the motors off and ends the run.
   **"CAN-Adapter hängt"** (v3.9.5): connecting took > 10 s (driver stuck after a hard stop): unplug the XCAN adapter
   for a few seconds, plug it in (directly into the laptop if possible), start again.
5. At the end the arm returns to the start pose, then to rest, motors off. The result appears in the box.

Clean aborts (v3.9.1, one line in the box, no tracebacks; the motors are switched off once and NOTHING else moves):
- "Roboterarm antwortet nicht – Strom prüfen": no motor feedback (power supply off/cut out, CAN cable). Check power, then
  press 🤖 Auto-Kalibrierung again.
- "Gelenk <name> kommt nicht voran … [present/sent/target per joint]": that joint is pinned at its lead without progress
  for 2.5 s (blocked, too weak, collision guard froze it). Look at what is in the way.
- "Gelenk <name> weicht … vom gesendeten Ziel ab": the arm is falling or being pushed.

How the motion stays safe (v3.9.1): every cycle the goal sent to the arm is the MEASURED position plus at most 3° (0.75 ×
LeRobot's max_relative_target of 4°) and advances at most 6°/s (approach) or 12°/s (sweep), so LeRobot never has to clamp
and the goal can never run ahead of the arm. (3.9 sent a precomputed ramp; LeRobot clamped it to 1° around the arm, the
elbow could not lift from rest with only 1° of lead, and the run aborted on "lags 10 deg" – crash of 22:29.)

Aborted run with saved poses (v3.9.2): with ≥ 10 good samples the result is computed anyway and shown in the box.
GOOD is applied automatically; OK shows "✓ Übernehmen" (check the numbers first). To add the missing poses:
`./handeye.sh run --resume vision/calibration/handeye/<folder>` (arm moves; continues after the last good pose).
"pcan reconnect failed" = the CAN adapter dropped off USB: plug it directly into the laptop, not into the hub.

**Manual override (old flow)**: ▶ START, aim the wrist camera at the board with the leader (gripper ~20 cm above the
board centre, whole board in Kamera 1), **📍 Ziel-Pose übernehmen**, ■ STOPP. This ticks "Gespeicherte Ziel-Pose
verwenden": 🤖 Auto-Kalibrierung … then plans from the taught pose (~40 s) and shows the plot + pose list before moving.
The arm first moves slowly (6°/s) to the taught pose – best start the run with the arm already near it.

**What good numbers look like** (shown in the box, details in the log)
| Number | good | ok | poor (not applied) |
|---|---|---|---|
| board pose spread over all views ("corners … mm RMS") | ≤ 3 mm | ≤ 6 mm | > 6 mm |
| board rotation spread | ≤ 1° | ≤ 2° | > 2° |
| chain reprojection (FK + hand-eye + board) | ≤ 1.5 px | ≤ 3 px | > 3 px |
| AX=XB residual (median) | ≤ 0.5°, ≤ 3 mm | ≤ 1°, ≤ 6 mm | higher |
| board tilt vs horizontal | < 3° (table level) | | large = board not flat or FK wrong |
GOOD and OK results are written automatically to `vision/calibration/handeye.json` and `table.json`
(`table_to_robot` with the full 3D `T_board2base`, plus `T_overview2base`). Then press **📐 Neu kalibrieren** once,
so the table plane uses the full-resolution overview image. A POOR result is saved but not applied.
If the log says "joint sign convention … fits much better", robot_fk's sign for that joint is wrong: tell the bot.

**Afterwards / next sessions**: the board stays taped, so only **📐 Neu kalibrieren** is needed when the overview
camera moved. Redo the auto calibration when the board, the arm base or the wrist camera mount moved.

**Terminal** (app may stay open; teleop must be stopped for `run`):
```
cd ~/humanoid
./handeye.sh plan --auto    # dry run of the automatic start: search pose + path from rest, nothing moves
./handeye.sh run --auto     # ARM MOVES after typing 'yes': automatic start (finds the board); Ctrl+C = motors off
./handeye.sh teach          # optional, while teleop runs: save the aim pose (read-only)
./handeye.sh plan           # dry run from the taught pose: pose list + vision/calibration/handeye_plan.png
./handeye.sh run            # ARM MOVES after typing 'yes', from the taught pose; Ctrl+C = motors off
./handeye.sh replay         # recompute from the newest sample folder (no hardware)
./handeye.sh apply <folder> [--force]
./handeye.sh status
./handeye.sh run --sim      # simulated arm + rendered images (test, no hardware)
./handeye.sh run --sim --auto   # the same with the automatic start
```
Samples: `vision/calibration/handeye/<time>/` (wrist_NN.png, overview_NN.png, samples.json, result.json).
Log: `logs/handeye_*.log`. Offline tests: `~/humanoid/.venv/bin/python ~/humanoid/vision/tests/test_handeye.py`.

## 8b. Fallback: manual robot link (touch corners A–D)
Use only if the auto calibration can't run (e.g. wrist camera broken). Prerequisite: table calibration done
(section 6), board taped. With the ChArUco board the corners A–D are the outermost inner corners (ids 0, 5, 23, 18).
1. Start teleop (▶ START). No session or recording active.
2. Press **🤝 Arm mit Tisch verbinden**. A dialog opens. It only reads joint angles and sends no motor commands.
3. For each corner A, B, C, D (shown with its table mm): guide the follower's gripper tip with the leader exactly onto
   the board corner, hold still ~1 s, press **Weiter** (or Enter).
4. Result: "Restfehler x mm RMS, Höhenstreuung y mm". Under 5 mm is good. Over 10 mm gives a warning: check the tip offset
   (`HUMANOID_TIP_OFFSET_MM`), that the tip really touched the corners, or the joint signs (alternatives listed).
5. It is saved in `vision/calibration/table.json` (+ history).
Terminal alternative (app closed, teleop started from Terminal): `~/humanoid/.venv/bin/python ~/humanoid/vision/calibrate.py robot-capture`
(`--manual` to type joint angles). With the app open, the Terminal tool fails with "UDP port 8765 busy". Use the button instead.

---------------------------------------------------------------------------------------------------------
## 9. Troubleshooting
**Cameras**
- "nicht angeschlossen (gefunden: …)": the named camera isn't in the macOS camera list. Re-plug it. List them:
  `~/humanoid/.venv/bin/python ~/humanoid/camera_feeds.py --list` (OpenCV index order, which feed resolves to what).
- Kein Signal right after plugging in: the app retries with back-off (2, 4, 8 … 30 s). Wait up to 30 s, or reload the app
  (⟳ App neu laden, only when stopped).
- Black or no camera at all: System Settings → Privacy & Security → Camera → allow "Robot Teleop" / Terminal.
- Names live in `~/humanoid/cameras.json` (`"source": "name:C920"`, `"name:Innomaker|U20CAM"`, `"role"`).
  Don't use index numbers (they shift when cameras are re-plugged).
- Recording says "kein Bild": the recorder reads the GUI's shared frames. Check the camera panels show fps.

**USB hub**
- Power-hungry devices (two cameras + CAN adapter + leader) on one unpowered hub can drop out. Use a powered hub, or
  put the PCAN adapter directly on the laptop.
- After re-plugging the CAN adapter or leader: STOPP, wait, then START (preflight checks them:
  "Leader-Arm nicht gefunden", "CAN-Adapter nicht erreichbar", "Roboterarm antwortet nicht – Strom einschalten").

**Stop / rest pose**
- Stop seems slow: the rest move runs (status "◐ STOPPT… fährt in Ruheposition" with elapsed time). Wait. Don't press again.
- Arm didn't reach rest: see the log line after `safe_zero`. "keine Motor-Rueckmeldung" usually means arm power was
  off or CAN dropped. Switch power off and on, then fold the arm by hand.
- Motors still powered after the app ended: `~/humanoid/.venv/bin/python ~/humanoid/disable_motors.py` (up to 90 s).
- False collision freezes: note the time and send `logs/session_*.log`. The replay tool is
  `rebot-B601-RS/tests/replay_freezes_v33.py <follower.py> ~/humanoid/logs/session_*.log`.

**Auto calibration**
- "the wrist camera does not see the board at the first poses": re-teach (section 8 step 2) with the whole board in Kamera 1.
- "only N board corners": glare or the gripper covers the board; tilt the lamp, teach a slightly higher pose.
- "arm lags … behind the command" / "target not reached": something blocked the arm or the collision guard froze it.
  The run stops, returns to the taught pose and rests. Check the area, then retry.
- "refused: Teleop läuft …": press ■ STOPP first.
- "only N reachable safe poses": the taught pose is too far out or too high; teach closer above the board.

**App**
- The app reloads itself when its files change, but only while stopped and not recording.
- 3.9 fix: after such a reload the camera panels of 3.8 could stay "Kein Signal" (the reloaded app waited for shared
  frames instead of opening the cameras). If a camera still shows "Kein Signal" for > 30 s, re-plug it.
- Log window is quiet: by design (3.7 filters the 30 Hz loop lines). The full log is in `logs/session_*.log`,
  recorder events in `logs/recorder.log`.

## 10. Rollback
In Terminal (app stopped):
```
cd ~/humanoid/rebot-B601-RS
./rollback.sh list                 # snapshots
./rollback.sh -n before_v3_9       # show what would change
./rollback.sh before_v3_9          # back to 3.8 (no auto calibration; vision/calibrate.py without ChArUco)
./rollback.sh before_v3_8          # back to 3.7 (guard v3.3, no sessions)
./rollback.sh before_v3_7          # back to 3.6.1 (guard v3.2)
./rollback.sh status               # which snapshot matches now
```
The current state is backed up to `backups/<time>/` first. The app reloads itself afterwards.
