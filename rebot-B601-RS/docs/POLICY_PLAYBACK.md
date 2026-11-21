# Policy playback ("▶ Policy abspielen") – v3.4

Plays a trained ACT policy on the B601-RS follower. The leader arm is not needed. The same code runs from the GUI button
and from the command line (`./play_policy.sh`), so it can later be triggered as a skill.

## Morning checklist
1. Check the training: `cat ~/humanoid/outputs/train/act_pick_place_clean_night1/timing.json`
   (`status` = `finished` or `stopped_at_deadline`, plus start/end, steps, steps/sec). The GUI shows the newest
   checkpoint ("Checkpoint: Schritt N").
2. The training must be finished (play refuses while `lerobot-train` runs), and teleop must be stopped (■ STOPP).
3. 48 V on, arm at rest. Both iPhones stream (both camera panels show a picture), from the same positions as during
   recording. Put the object where it was at the start of the demos.
4. Press **▶ Policy abspielen**, read the dialog and confirm. Keep a hand on the power strip.
5. **■ STOPP (Motoren sofort aus)** or the **Esc** key cuts the torque immediately. The arm sags/falls onto the
   table, so keep the workspace clear under it. When the time limit (default 60 s, "Dauer (s)") ends, the arm drives to
   its rest position and then becomes torque-free.

## Command line
```
./play_policy.sh --check                     # checkpoint/features/cameras/conflicts only, loads nothing heavy
./play_policy.sh --dry-run --seconds 10      # policy + live cameras, simulated joints, NO motors
./play_policy.sh                             # newest checkpoint on the arm, 60 s
./play_policy.sh --seconds 30 outputs/train/act_pick_place_clean_night1/checkpoints/018000
```
Log: `~/humanoid/logs/play_*.log` (the last 50 are kept). Exit codes: 0 ok, 2 checkpoint, 3 refused (teleop/training
running), 5 cameras, 6 robot not reachable, 7 policy error.

## What it does
- **Checkpoint:** the newest one (highest step) in `outputs/train/act_pick_place_clean_night1*/checkpoints/<step>/pretrained_model`
  (override with `PLAY_RUN_GLOB` or an explicit path).
- **Observation:** exactly as recorded for training.
  - `observation.state` = follower `<motor>.pos` in degrees, in the motor order shoulder_pan…gripper.
  - `observation.images.overview` = iPhone 1 (cameras.json feed 1), `observation.images.gripper` = iPhone 2 (feed 2),
    resized by the recorder's own `Cam` class to 640×360.
  - Keys and shapes are checked against the checkpoint's `config.json` before anything moves.
- **Control:** a 30 Hz loop of policy (ACT action chunks of 100 steps) → `send_action`.
  - Actions are clipped to the joint range of the training data ±5° (`PLAY_RANGE_MARGIN_DEG`).
- **Safety:**
  - The collision guard v3.2 is forced ON, with all `COLLISION_*`/`MIT_*`/`GRAVITY_*` settings taken from `teleop.sh`.
  - Step limit 2°/cycle for the arm joints (≤ 60°/s) and 10° for the gripper (`PLAY_MAX_STEP_DEG`, `PLAY_MAX_STEP_GRIPPER_DEG`).
  - Gripper force "vorsichtig".
  - It refuses to start while teleop (CAN bus) or a training (MPS/memory) runs.
  - If a camera drops out for more than 2 s, the arm drives to rest.
  - STOP = SIGTERM → immediate `disable_all`. If the process is still alive after 1.5 s, the GUI kills it and runs
    `disable_motors.py`.
  - The GUI blocks teleop START and the app auto-reload while a policy runs. Closing the window stops the policy first.
- **Not certified.** The emergency stop is still the power strip. The guard thresholds were tuned for teleop. With a
  policy the "leader" is the policy target, so a freeze ("KOLLISION") can also happen when the policy pushes against
  the table.

## Tested offline (bot computer, 2026-11-16, LeRobot 0.7.0, fake robot + fake MJPEG bridge, tiny ACT checkpoint)
- `--check`, dry run with live MJPEG frames via `recorder.Cam` (~29 Hz, inference ~3 ms per step on CPU, ~150 ms for a
  new chunk).
- The time limit ends with a rest move and then torque off.
- SIGTERM while running → `disable_all` without a rest move. SIGTERM during the rest move → immediate `disable_all`.
- It refuses while a `lerobot-train` process exists.
- GUI under Xvfb: start/stop, teleop START is refused while playing, window close stops the policy, and the force-kill
  plus `disable_motors.py` backup work when the process hangs.
