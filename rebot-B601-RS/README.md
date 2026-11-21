# reBot B601-RS – Teleoperation & Learned Control (Mac mini + laptop)

Documentation and rollback bundle for David Neuhaus's robot arm project.
As of 2026-11-15. Plain files; versioned with git by David (push: https://github.com/davidneuhaus/rebot-B601-RS).
Current app/guard version: see `VERSION` (3.6 GUI, guard v3.2). Laptop copy: see [docs/LAPTOP_SETUP.md](docs/LAPTOP_SETUP.md). Ideas and next steps: [docs/TODO.md](docs/TODO.md).

## Goal

Learned robot control (imitation learning with LeRobot): the small white **reBot 102** (leader) is moved by hand,
the large **Seeed reBot Arm B601-RS** (follower) mirrors the motion. From these demonstrations (later with a camera)
a policy will be trained that controls the arm on its own.

Operating goal: power on → press **START** in the app → teleop runs. **STOP** → the arm drives to its rest position
and becomes torque-free (quiet, safe).

## Hardware

| Part | Details |
|---|---|
| Follower | Seeed reBot Arm B601-RS, 6 joints + gripper, RobStride motors (rs-06: shoulder pan/lift, elbow; rs-00: 3 wrist axes + gripper), approx. 6.5 kg, payload approx. 2.5 kg |
| Power | 48 V supply on a power strip (= emergency stop) |
| Bus | CAN via PCAN-USB adapter, channel `can0` (socketcan), motor IDs 1–7 |
| Leader | reBot 102 (white), USB serial `/dev/tty.usbserial-*` (port is detected automatically) |
| Computer | Mac mini (macOS), project folder `~/humanoid` |

## Software

- Python 3.12 venv: `~/humanoid/.venv`
- **LeRobot 0.7.0** (`lerobot-teleoperate`)
- Plugin **lerobot-robot-seeed-b601 1.0.5** (robot type `seeed_b601_rs_follower`), patched locally in
  `~/humanoid/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601/` (see `patches/` and `docs/CHANGELOG.md`)
- motorbridge 0.6.0 (CAN driver, MIT impedance control)
- Leader type `rebot_arm_102_leader`

## Operation

### App / GUI
`Robot Teleop.app` starts `~/humanoid/teleop_gui.py` (log: `/tmp/teleop_gui.log`). GUI labels are German.

1. Both arms in rest position, switch on 48 V.
2. Choose gripper force (Vorsichtig / Mittel / Kräftig = gentle / medium / strong), optionally untick the collision
   guard ("Kollisionsschutz an"), press **START**.
3. **STOP**: one Ctrl+C (SIGINT) to the process group → the arm drives to rest (safe_zero). Hard kill after 25 s at the
   latest. Then `disable_motors.py` makes all motors torque-free → status "■ GESTOPPT".
4. Closing the window while teleop runs = STOP first; the window closes once the arm stands still.

Session logs: `~/humanoid/logs/session_*.log` (the last 100 are kept).

### Command line
```
~/humanoid/teleop.sh [vorsichtig|mittel|kraeftig]     # default: vorsichtig, stop with Ctrl+C
```

Gripper levels (current):

| Level | KP | TQ (open max, Nm) | HOLD (Nm) | CLOSE (close max, Nm) |
|---|---|---|---|---|
| vorsichtig (gentle) | 10.0 | 1.5 | 1.0 | 1.1 |
| mittel (medium) | 12.0 | 2.0 | 1.2 | 1.2 |
| kraeftig (strong) | 12.0 | 3.5 | 1.0 | 0 (= no extra limit, factory setting) |

Further limits: arm joints max 5°/cycle, gripper max 25°/cycle (`max_relative_target`), 30 Hz.

### Pre-flight check `preflight.py`
Run automatically by `teleop.sh` (up to 6 tries of 5 s each, in case the arm was just powered on).
Prints the leader port on success.

| Exit code | Meaning |
|---|---|
| 0 | all OK, leader port on stdout |
| 2 | leader arm (USB) not found |
| 3 | CAN adapter (PCAN) not reachable |
| 4 | robot arm not answering (power off / just switched on) → retried |

### Collision guard (software)
Enabled in `teleop.sh` (`COLLISION_GUARD=1`), switchable in the app. It detects a blocked arm joint while the leader
keeps moving: leader↔follower offset above the joint threshold **and** the follower progresses < 0.15°/cycle for
6 cycles (200 ms). The arm then freezes at its current position (no pushing on, no retreat; the gripper keeps working).
Every 5 s the log writes a `Guard-Stats` line (max tracking error, leader offset, torque per joint) for tuning.

Current settings (v3.1, all overridable via environment variables in `teleop.sh`):

| Joint | MIT kp / kd | Offset threshold | Torque trigger |
|---|---|---|---|
| shoulder_pan | 20 / 1.9 (compliant) | 6° | 1.6 Nm |
| shoulder_lift | 150 / 10 | 10° | off |
| elbow_flex | 150 / 10 | 10° | off |
| wrist_flex | 40 / 4.5 (carries the gripper; lower kp would let it sag) | 10° | off |
| wrist_yaw | 20 / 2.5 (compliant) | 8° | 1.25 Nm |
| wrist_roll | 50 / 4 | 15° | off |

Torque trigger fires only if |torque| > limit **and** offset ≥ 3° **and** progress < 0.5°/cycle for 6 cycles
(`COLLISION_TORQ_CYCLES`, `COLLISION_TORQ_MIN_LEAD_DEG`, `COLLISION_TORQ_MAX_PROGRESS_DEG`).

**Auto-resume** (`COLLISION_AUTO_RESUME=1`): once the frozen arm has stood still for 1.5 s it creeps towards the
leader at max 1°/cycle (targets start at the frozen command, so nothing sags; each target runs at most 4° (compliant
joints) or 2.5° (stiff joints) ahead of its joint). It re-freezes only on a real obstacle: the joint lags its target by
> 3°/2° and does not move for 10 cycles, or the torque trigger fires. Retries are unlimited with growing waits
(1.5 s, 3 s, 5 s, then every 5 s). Manual release: bring only the joint that collided back within 6° of the frozen
position; the log and the app name that joint in German ("Leader: Ellbogen 12.2 deg zurückführen").
Normal speed resumes once all creep targets are within 6° of the leader.

History: v1 `guard_v1` (detection + freeze), v2 `guard_v2_compliant` (compliant pan/yaw), v3 `guard_v3`
(wrist_flex, auto-resume, app self-update), v3.1 `guard_v3_1` (no dead end, fewer false alarms). Details in
`docs/CHANGELOG.md`.

### App version and self-update
The version number lives in `~/humanoid/VERSION` and is shown in the window title. When `teleop_gui.py` or `VERSION`
changes, the app reloads itself while stopped within ~10 s (logs "Aktualisiert auf Version X"); never while teleop
runs – then it shows "Update verfügbar – wird nach STOPP geladen" and reloads right after STOP. A file with a syntax
error is not loaded. The "⟳ App neu laden" button reloads manually (disabled while running).

## Safety

- **The emergency stop is the power strip** (48 V off = motors torque-free; the arm then drops!).
  Recommendation: install a real hardware e-stop (mushroom switch) in the 48 V line (see `docs/TODO.md`).
- The collision guard is **software detection only, not certified**, and not personal protection.
  It reacts only after contact (≥ 200 ms). Until then the arm pushes with its position control (up to ~13 Nm at
  shoulder/elbow, ~1.75 Nm at the compliant pan/yaw joints). Slowly yielding objects (soft items, people) are
  detected worse than a hard stop.
- Do not reach into the workspace while teleop runs. Keep children/pets away.
- Before START put both arms in rest position (gripper closed) so the gripper zero search is correct.
- Gripper "vorsichtig" does not crush cardboard but can pinch fingers.

## Contents of this folder

```
README.md            this file
VERSION              app/guard version (mirrors ~/humanoid/VERSION)
docs/CHANGELOG.md    all changes with reasons
docs/TODO.md         ideas and next steps
scripts/             current copies of the scripts in ~/humanoid
patches/             diffs original→patched, plus patched/ and original/ (plugin 1.0.5)
versions/            snapshots for rollback (each folder has a MANIFEST: file → target)
tests/               offline tests of the collision guard and the app update logic (no hardware)
rollback.sh          restore / re-apply patches
```

### Rollback
```
cd ~/humanoid/rebot-B601-RS
./rollback.sh list               # list snapshots
./rollback.sh status             # which snapshot matches the installed files
./rollback.sh -n guard_v3        # dry run: only show what would happen
./rollback.sh guard_v3           # back to version 3.0
./rollback.sh before_guard       # state before the collision guard
./rollback.sh original           # unpatched plugin
./rollback.sh apply              # re-apply patches (e.g. after a pip/uv update of the plugin)
```
The current state is always saved to `backups/<timestamp>/` first. Refuses to run while teleop runs.
Changes take effect at the next START (the app reloads itself when `teleop_gui.py`/`VERSION` change).
Snapshots older than `guard_v3_1` contain the files as they were then (German comments).

Note: after updating/reinstalling the plugin the patches are gone → `./rollback.sh apply`.

### Tests
```
cd ~/humanoid/rebot-B601-RS/tests
~/humanoid/.venv/bin/python test_collision_guard.py                      # checks the installed plugin + app
~/humanoid/.venv/bin/python replay_logs.py ../patches/patched/seeed_b601_follower.py ~/humanoid/logs/session_*.log
```

## Cameras (v3.1.1)
Two preview panels in the GUI (`camera_feeds.py`, config `~/humanoid/cameras.json`; MJPEG URL, still-image URL,
OpenCV index or empty). The iPhone browser bridge is in `tools/iphone_web_cam/` (see its README).

## Guard v3.2 (gravity-aware)
See the v3.2 entry in [docs/CHANGELOG.md](docs/CHANGELOG.md). Switches in `teleop.sh`: `COLLISION_GRAVITY`, `GRAVITY_FF`.

## Dataset recording (v3.3)
GUI panel "Datensatz aufnehmen": `recorder.py` (own process) + telemetry tap `lerobot_teleoperate_tap.py`
(started by `teleop.sh`). Datasets go to `~/humanoid/datasets/<name>`.

## Learned policy: record → clean → train → play (v3.4)
1. **Record** in the GUI (teleop running): dataset name, "● Episode starten", do the task, "✔ Episode speichern",
   then wait for "gespeichert" (~10 s) before pressing STOP. Only successful demos belong in a training set, because
   ACT imitates everything (a miss teaches misses).
2. **Clean/merge:** `.venv/bin/python tools/merge_datasets.py <out> <in…>`. Exclude bad episodes at training time with
   `EPISODES='[0,2,…]'`. Never delete inside a recorded dataset without a backup.
3. **Train:** `EPISODES='[…]' DEADLINE=05:10 ./train_act.sh <dataset> 60000` (ACT on MPS, caffeinate). Timing goes to
   `outputs/train/<run>/timing.json` and the CHANGELOG.
4. **Play:** GUI **▶ Policy abspielen** or `./play_policy.sh [checkpoint] [--seconds N] [--dry-run]`. See
   [docs/POLICY_PLAYBACK.md](docs/POLICY_PLAYBACK.md).

## SmolVLA zero-shot test (v3.5, experimental)
In the policy panel choose **SmolVLA (Text-Befehl)**, type a command (default `pick up the box and put it in the bin`),
press **▶ Policy abspielen** with "Trockenlauf (ohne Motoren)" checked first, then uncheck it for the real run.
The base model has never seen this arm: expect odd motions. Details and the honest caveats: `docs/SMOLVLA_ZERO_SHOT.md`.

## Policy command bar (v3.6)
In the policy panel choose **SmolVLA (Text-Befehl)**. Type a command in the large field (or click a
preset such as `pick up the box and put it in the bin`), press **🧪 Testen (ohne Motoren)** first, then
**▶ Ausführen (Motoren an)** once the test succeeded with that same text. Recent commands are under
**Verlauf**. ACT still uses **▶ Policy abspielen**. Details: `docs/GUI_COMMAND_BAR.md`.

## Laptop (2026-11-21)
Same project on the MacBook Pro M1 (conda env `lerobot` behind a `.venv` shim). Setup, differences and open steps:
[docs/LAPTOP_SETUP.md](docs/LAPTOP_SETUP.md).
