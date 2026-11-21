#!/usr/bin/env python3
"""Build a clean training dataset from selected episodes of several local LeRobot datasets (same features).
Usage: clean_merge.py <root> <out> <src1>[:<bad_ep,...>] <src2> ...   (bad episode indices are per source)

Why a rebuild instead of aggregate_datasets + delete_episodes: in LeRobot 0.7.0 delete_episodes on an aggregated
dataset whose sources kept one episodes-metadata file per episode fails (FileNotFoundError on
meta/episodes/.../file-00N.parquet). The rebuild reads every kept frame through LeRobotDataset (decoded video) and
writes a brand-new dataset with LeRobotDataset.create/add_frame/save_episode - same features, fps, robot type and
task strings, videos re-encoded once. Sources stay untouched."""
import json, sys, shutil, time
from pathlib import Path
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = Path(sys.argv[1]); out = sys.argv[2]; specs = sys.argv[3:]
if (root / out).exists():
    sys.exit(f"{root/out} exists")
plan, keep_lens = [], []
for s in specs:
    name, _, bad = s.partition(":")
    ds = LeRobotDataset(f"local/{name}", root=root / name)
    badl = {int(b) for b in bad.split(",") if b != ""}
    lens = [int(ds.meta.episodes[i]["length"]) for i in range(ds.meta.total_episodes)]
    print(f"{name}: episodes {len(lens)} lengths {lens}" + (f"  (drop {sorted(badl)})" if badl else ""), flush=True)
    for e in range(ds.meta.total_episodes):
        if e not in badl:
            plan.append((name, ds, e)); keep_lens.append(lens[e])
src0 = plan[0][1]
feats = {k: v for k, v in src0.meta.features.items()
         if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
vid_keys = [k for k, v in feats.items() if v["dtype"] == "video"]
new = LeRobotDataset.create(repo_id=f"local/{out}", fps=src0.meta.fps, features=feats, root=root / out,
                            robot_type=src0.meta.robot_type, use_videos=True, image_writer_threads=4)
t0 = time.time()
for n, (name, ds, e) in enumerate(plan):
    ep = ds.meta.episodes[e]; a = int(ep["dataset_from_index"]); b = a + int(ep["length"])
    for idx in range(a, b):
        it = ds[idx]
        fr = {"task": it["task"]}
        for k in feats:
            v = it[k]
            if k in vid_keys:
                fr[k] = (v.permute(1, 2, 0).clamp(0, 1).numpy() * 255).round().astype(np.uint8)
            else:
                fr[k] = v.numpy().astype(np.float32)
        new.add_frame(fr)
    new.save_episode()
    print(f"  wrote episode {n} <- {name} ep{e} ({b - a} frames, {time.time() - t0:.0f} s)", flush=True)
new.finalize()
ds = LeRobotDataset(f"local/{out}", root=root / out)
lens = [int(ds.meta.episodes[i]["length"]) for i in range(ds.meta.total_episodes)]
i = json.load(open(root / out / "meta" / "info.json"))
print(f"{out}: total_episodes {i['total_episodes']} total_frames {i['total_frames']} lengths {lens} "
      f"(expected {len(keep_lens)} / {sum(keep_lens)} {keep_lens})")
assert lens == keep_lens and i["total_episodes"] == len(keep_lens) and i["total_frames"] == sum(keep_lens)
for e in range(ds.meta.total_episodes):
    ep = ds.meta.episodes[e]
    for idx in (int(ep["dataset_from_index"]), int(ep["dataset_from_index"]) + int(ep["length"]) - 1):
        it = ds[idx]
        shapes = {k.split(".")[-1]: tuple(v.shape) for k, v in it.items() if k.startswith("observation.images")}
        print(f"  ep{e} len {ep['length']} frame {idx} decoded {shapes} state0 {float(it['observation.state'][0]):.2f}")
print("CLEAN_MERGE_OK")
