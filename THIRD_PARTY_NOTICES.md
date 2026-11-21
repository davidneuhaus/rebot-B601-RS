# Third-party notices

## lerobot-robot-seeed-b601 (Seeed Technology Co., Ltd.) – MIT License
- Source: https://github.com/Seeed-Projects/lerobot-robot-seeed-b601, version 1.0.5.
- Copyright (c) Seeed Technology Co., Ltd. Full license text: `rebot-B601-RS/patches/LICENSE-SEEED`.
- Used here: the unmodified plugin files in `rebot-B601-RS/patches/original/`, the modified files in
  `rebot-B601-RS/patches/patched/` (collision guard, gravity model, safe-zero changes) with their `.diff` files,
  and the patch scripts `tools/patch_*.py` that apply those modifications. Modified by David Neuhaus, 2026.

## LeRobot (Hugging Face) – Apache License 2.0
- https://github.com/huggingface/lerobot. Used as an installed dependency only (`pip install`); no LeRobot source
  code is copied into this repository.

## Other dependencies
Installed with pip (not vendored), each under its own license: motorbridge, python-can, OpenCV, NumPy, PyTorch,
Ultralytics, MobileSAM and others listed in `docs/LAPTOP_SETUP.md` / `tools/pins.txt`.
