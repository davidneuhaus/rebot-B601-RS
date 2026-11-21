# reBot B601-RS – Humanoid Project

App code lives on `main`. The project logbook website lives on the `website` branch.

reBot B601-RS follower arm (CAN via XCAN/PEAK adapter) + reBot 102 leader arm (USB serial): LeRobot teleoperation,
dataset recording with live markers, policy playback, camera vision and automatic hand-eye calibration.
The repository mirrors the app folder `~/humanoid` on the robot laptop (synced with `tools/git_sync.sh`).

- Main documentation: [rebot-B601-RS/README.md](rebot-B601-RS/README.md); laptop notes:
  [rebot-B601-RS/docs/LAPTOP_SETUP.md](rebot-B601-RS/docs/LAPTOP_SETUP.md); operating procedure:
  [docs/OPERATING_PROCEDURE.md](docs/OPERATING_PROCEDURE.md); changes: [docs/CHANGELOG.md](docs/CHANGELOG.md).
- GUI: `Robot Teleop.command` (double-click) or `.venv/bin/python teleop_gui.py`
- Teleop CLI: `./teleop.sh [vorsichtig|mittel|kraeftig]`
- Auto calibration: `./handeye.sh run --auto` (see docs/OPERATING_PROCEDURE.md section 8); simulation `./handeye.sh run --sim --auto`
- Offline self-check: `./verify_laptop.sh`; tests: `vision/tests/test_handeye.py`, `rebot-B601-RS/tests/`
- Rollback on the laptop: `rebot-B601-RS/rollback.sh list` (snapshots in `rebot-B601-RS/versions/`, not in git)

Not in the repository: the Python environment (`.venv`, conda env `lerobot`, pins in `tools/pins.txt`), the PCBUSB
library (`lib/`), datasets, checkpoints (`outputs/`), logs, snapshots, the YOLO-World model (`vision/models/`) and
per-session calibration captures.
