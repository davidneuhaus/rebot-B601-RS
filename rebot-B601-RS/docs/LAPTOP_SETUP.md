# Laptop setup (MacBook Pro M1, 8 GB) – 2026-11-21

This laptop runs a copy of the Mac mini project (`~/humanoid`, app version 3.6). The Mac mini stays at the office and
keeps the datasets, the public datasets and the trained ACT checkpoints; none of those were copied here.

## Differences to the Mac mini

| | Mac mini | Laptop |
|---|---|---|
| Python | `~/humanoid/.venv` (uv venv, Python 3.12) | conda env `lerobot` (`/opt/homebrew/Caskroom/miniforge/base/envs/lerobot`, Python 3.12) |
| `~/humanoid/.venv` | real venv | **shim**: `bin/activate` activates the conda env, `bin/python` → conda python, `lib` → conda `lib` (so every script, the GUI and `rollback.sh` work unchanged) |
| LeRobot | 0.7.0 (PyPI) | 0.7.1, editable from `~/lerobot` (installed by another assistant; its `start_teleop.sh` / `find_cameras.sh` are left alone) |
| motorbridge | 0.6.0 | 0.6.1 |
| Robot plugin | `lerobot-robot-seeed-b601` 1.0.5 + local patches | same wheel 1.0.5 (`--no-deps`), same patches (guard v3.2, gripper homing, close-torque cap, safe_zero fix) |
| Leader plugin | `lerobot-teleoperator-rebot-arm-102` 1.0.1 | same |
| SmolVLA extras | `lerobot[smolvla]` installed | **not installed** (8 GB RAM; `transformers` missing → SmolVLA test/run fails until installed) |
| PCAN library | `/usr/local/lib/libPCBUSB.dylib` (sudo install) | `~/humanoid/lib/libPCBUSB.dylib` + `DYLD_LIBRARY_PATH` + `~/.local/lib` symlink (no sudo) |
| Leader port | autodetected `/dev/tty.usbserial-*` | same (currently `/dev/tty.usbserial-13420`) |

## One-time steps for David

1. **PCAN driver (PCBUSB)** – no admin rights on this laptop, so the library is kept in user space:
   `~/humanoid/lib/libPCBUSB.dylib` (do not delete). `~/humanoid/.venv/bin/activate` exports
   `DYLD_LIBRARY_PATH=$HOME/humanoid/lib` (sourced by teleop.sh, play_policy.sh, train_act.sh, the app launcher), and
   `~/.local/lib/libPCBUSB.dylib` is a symlink to it (motorbridge searches there too, so it also works when macOS strips
   `DYLD_*` variables). python-can uses `interface='pcan'`, channel `PCAN_USBBUS1`; the plugin uses `can0`.
   Check with the arm powered: `~/humanoid/.venv/bin/python ~/humanoid/preflight.py` (prints the leader port, exit 0).
2. **Camera permission**: System Settings → Privacy & Security → Camera → allow Terminal (and "Robot Teleop" when asked).
   For a USB camera (Innomaker) set `"source": "0"` / `"1"` in `~/humanoid/cameras.json`.
3. **Calibration**: the Mac mini calibration files (`~/.cache/huggingface/lerobot/calibration/...`) are not on this
   laptop. If teleop asks for calibration on the first START, run it once or copy the two JSON files from the Mac mini.
4. Hardware reminders: leader needs its 12 V supply (not only USB); follower 48 V, CAN adapter switch on **120R**;
   after power-on the motors need up to 30 s before all 7 answer.

## Start

- App: `~/Applications/Robot Teleop.app` (or double-click `~/humanoid/Robot Teleop.command`, or
  `~/humanoid/.venv/bin/python ~/humanoid/teleop_gui.py`).
- Command line: `~/humanoid/teleop.sh [vorsichtig|mittel|kraeftig]`.
- Offline self-check (no motor commands): `~/humanoid/verify_laptop.sh`.

## Optional

- SmolVLA on the laptop: `~/humanoid/.venv/bin/pip install -e "$HOME/lerobot[smolvla]"` (downloads transformers etc.;
  the 450M model is fetched on first use). Expect slow inference on 8 GB.
- Training (ACT) is better done on the Mac mini; 8 GB is tight.
