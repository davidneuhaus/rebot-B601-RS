set -u
cd ~/humanoid || exit 1
if pgrep -f "lerobot-train|clean_merge" >/dev/null; then echo "ABORT: already running"; pgrep -fl "lerobot-train|clean_merge"; exit 3; fi
cp _incoming/clean_merge.py tools/clean_merge.py && cp _incoming/pipeline.sh tools/night_pipeline.sh && chmod +x tools/*.py tools/night_pipeline.sh || exit 8
[ -f train_act_v1_before_timing.sh ] && grep -q DEADLINE train_act.sh && echo "train_act.sh v3.4 in place (backup train_act_v1_before_timing.sh)"
P=logs/night_pipeline_$(date +%Y%m%d_%H%M%S).log
nohup bash tools/night_pipeline.sh > "$P" 2>&1 < /dev/null &
disown
echo "pipeline started $(date '+%H:%M:%S %Z'), log $P"
RUN=act_pick_place_clean_night1
for i in $(seq 1 44); do
  sleep 15
  if grep -q "ABORT\|Traceback\|FAILED" "$P"; then echo "!! problem"; break; fi
  if [ -f outputs/train/$RUN.log ] && [ "$(grep -c 'step:' outputs/train/$RUN.log)" -ge 3 ]; then break; fi
done
echo "== pipeline log:"; grep -v "^\s*$" "$P" | grep -v "Training:" | cut -c1-220 | tail -30
echo "== train log:"; grep -o "step:[0-9.K]* .*loss:[0-9.]*.*step_s:[0-9.]*" outputs/train/$RUN.log 2>/dev/null | tail -4
grep -i "error\|Traceback" outputs/train/$RUN.log 2>/dev/null | tail -5
pgrep -fl "lerobot-train" | cut -c1-120; df -h ~ | tail -1; date '+%H:%M:%S %Z'
