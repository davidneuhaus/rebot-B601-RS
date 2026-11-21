#!/bin/bash
# Overnight pipeline (detached): rebuild the clean dataset, verify it, then start ACT training with timing.
set -u
cd ~/humanoid && source .venv/bin/activate || exit 1
TS=$(date +%Y%m%d_%H%M%S)
for d in datasets/pick_place_clean datasets/_tmp_pick_place_clean_all; do [ -e "$d" ] && mv "$d" "datasets/_old_$(basename $d)_$TS"; done
echo "== rebuild start $(date '+%H:%M:%S %Z')"
python tools/clean_merge.py ~/humanoid/datasets pick_place_clean pick_place_v1:1 pick_place_v2 pick_place_v3 pick_place_v4 2>&1 | grep -v "it/s\]\|Svt\|mp4 @\|examples/s"
python -c "import json;d=json.load(open('datasets/pick_place_clean/meta/info.json'));print('INFO',d['total_episodes'],d['total_frames']);assert d['total_episodes']==6 and d['total_frames']==7034" || { echo "ABORT: dataset stats wrong - training NOT started"; exit 7; }
echo "== rebuild done $(date '+%H:%M:%S %Z')"
RUN=act_pick_place_clean_night1
[ -e outputs/train/$RUN ] && mv outputs/train/$RUN outputs/train/_old_${RUN}_$TS
DEADLINE=05:10 RUN_NAME=$RUN SAVE_FREQ=2000 KEEP_CKPT=3 exec ./train_act.sh pick_place_clean 60000
