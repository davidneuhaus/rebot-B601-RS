#!/usr/bin/env python3
"""Export a filtered copy of a recorded dataset by episode label (v3.8).

Labels come from <dataset>/meta/episode_labels.jsonl (written by recorder.py: success / fail / play).
Episodes without a label line count as "play". The source dataset is never changed.

  ~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1 --list
  ~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1                 # -> pick_place_v1_success
  ~/humanoid/.venv/bin/python ~/humanoid/tools/export_dataset.py pick_place_v1 --labels success,play --out pp_good
  options: --root (default ~/humanoid/datasets or $REC_ROOT), --dry-run, --force (replace an existing output)
"""
import argparse, json, os, shutil, sys
from collections import Counter

LABELS_FILE = "episode_labels.jsonl"


def read_labels(root_dir):
    out = {}
    p = os.path.join(root_dir, "meta", LABELS_FILE)
    if os.path.exists(p):
        for line in open(p):
            try:
                r = json.loads(line); out[int(r["episode_index"])] = r
            except Exception:
                pass
    return out


def total_episodes(root_dir):
    with open(os.path.join(root_dir, "meta", "info.json")) as f:
        return int(json.load(f)["total_episodes"])


def plan(root_dir, keep_labels):
    n = total_episodes(root_dir)
    lab = read_labels(root_dir)
    labels = {i: (lab.get(i) or {}).get("label", "play") for i in range(n)}
    keep = [i for i in range(n) if labels[i] in keep_labels]
    drop = [i for i in range(n) if labels[i] not in keep_labels]
    return n, labels, lab, keep, drop


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("--labels", default="success", help="comma list of labels to keep (default success)")
    ap.add_argument("--out", default=None, help="output dataset name (default <dataset>_<labels>)")
    ap.add_argument("--root", default=os.path.expanduser(os.environ.get("REC_ROOT", "~/humanoid/datasets")))
    ap.add_argument("--list", action="store_true"); ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    src = os.path.join(a.root, a.dataset)
    keep_labels = [x.strip() for x in a.labels.split(",") if x.strip()]
    n, labels, lab, keep, drop = plan(src, keep_labels)
    c = Counter(labels.values())
    print(f"{a.dataset}: {n} episodes – " + ", ".join(f"{k} {v}" for k, v in sorted(c.items())))
    if a.list:
        for i in range(n):
            r = lab.get(i) or {}
            print(f"  {i:4d}  {labels[i]:8s} {r.get('seconds', ''):>6}  {r.get('recorded_at', '')}  {r.get('task', '')}")
        return 0
    out_name = a.out or f"{a.dataset}_{'_'.join(keep_labels)}"
    dst = os.path.join(a.root, out_name)
    print(f"keep {len(keep)} ({','.join(keep_labels)}), drop {len(drop)} -> {dst}")
    if not keep:
        print("nothing to export"); return 2
    if a.dry_run:
        return 0
    if os.path.exists(dst):
        if not a.force:
            print(f"{dst} exists (use --force to replace)"); return 3
        shutil.rmtree(dst)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id=f"local/{a.dataset}", root=src)
    if drop:
        from lerobot.datasets.dataset_tools import delete_episodes
        delete_episodes(ds, episode_indices=drop, output_dir=dst, repo_id=f"local/{out_name}")
    else:
        shutil.copytree(src, dst)
    # labels for the new dataset: kept episodes are renumbered 0..k-1 in their old order
    with open(os.path.join(dst, "meta", LABELS_FILE), "w") as f:
        for new_i, old_i in enumerate(keep):
            r = dict(lab.get(old_i) or {"label": "play"}); r["episode_index"] = new_i; r["source_episode"] = old_i
            r["source_dataset"] = a.dataset
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    got = total_episodes(dst)
    print(f"done: {out_name} has {got} episodes" + ("" if got == len(keep) else f" (expected {len(keep)}!)"))
    return 0 if got == len(keep) else 4


if __name__ == "__main__":
    sys.exit(main())
