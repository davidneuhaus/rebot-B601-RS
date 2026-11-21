#!/usr/bin/env python3
"""Merge several local LeRobot datasets (same features) into one new dataset for training. Sources stay untouched.
Usage: ~/humanoid/.venv/bin/python tools/merge_datasets.py <output_name> <src1> <src2> ...
       e.g. tools/merge_datasets.py pick_place_all pick_place_v1 pick_place_v2 pick_place_v3 pick_place_v4
Datasets live in ~/humanoid/datasets (override with DATA_ROOT). The output must not exist yet."""
import json, os, sys
from pathlib import Path
from lerobot.datasets.aggregate import aggregate_datasets

root = Path(os.path.expanduser(os.environ.get("DATA_ROOT", "~/humanoid/datasets")))
out, srcs = sys.argv[1], sys.argv[2:]
if not srcs:
    sys.exit(__doc__)
if (root / out).exists():
    sys.exit(f"{root / out} exists already – choose another output name or delete it first")
for s in srcs:
    if not (root / s / "meta" / "info.json").exists():
        sys.exit(f"not a dataset: {root / s}")
aggregate_datasets(repo_ids=[f"local/{s}" for s in srcs], aggr_repo_id=f"local/{out}",
                   roots=[root / s for s in srcs], aggr_root=root / out)
info = json.load(open(root / out / "meta" / "info.json"))
want = sum(json.load(open(root / s / "meta" / "info.json"))["total_episodes"] for s in srcs)
print(f"{out}: {info['total_episodes']} episodes, {info['total_frames']} frames (expected {want} episodes)")
sys.exit(0 if info["total_episodes"] == want else 1)
