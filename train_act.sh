#!/bin/bash
# Train an ACT policy locally on the Mac mini (Apple GPU via MPS) from a dataset recorded with the teleop GUI.
#   ./train_act.sh [dataset_name] [steps]          e.g. ./train_act.sh pick_place_clean 60000
# Env overrides: BATCH (8), SAVE_FREQ (2000), NUM_WORKERS (4), DATA_ROOT (~/humanoid/datasets), RUN_NAME, RESUME=1,
#   DEADLINE=HH:MM   stop training gracefully at this Europe/Berlin wall-clock time (the newest checkpoint is kept;
#                    ACT trains with a constant learning rate, so an early stop is just a shorter run)
#   KEEP_CKPT (3)    keep only the newest N numbered checkpoints (each ~0.6 GB incl. optimizer state)
# Output: ~/humanoid/outputs/train/<run>/  (checkpoints/<step>/pretrained_model, checkpoints/last), log: <run>.log
# Timing (v3.4): <run>/timing.json (+ live copy <run>.timing.json while running) with start/end (Europe/Berlin),
#   wall-clock duration, steps reached, steps/sec; one line is appended to the CHANGELOG at the end.
# caffeinate keeps the Mac awake for the whole run (display may sleep). Nothing is pushed to the Hub.
set -u
DS="${1:-pick_place_v1}"
STEPS="${2:-${STEPS:-60000}}"
BATCH="${BATCH:-8}"
SAVE_FREQ="${SAVE_FREQ:-2000}"
NUM_WORKERS="${NUM_WORKERS:-4}"
KEEP_CKPT="${KEEP_CKPT:-3}"
DEADLINE="${DEADLINE:-}"
DATA_ROOT="${DATA_ROOT:-$HOME/humanoid/datasets}"
RUN_NAME="${RUN_NAME:-act_${DS}_$(date +%Y%m%d_%H%M)}"
OUT="$HOME/humanoid/outputs/train/$RUN_NAME"
export TZ=Europe/Berlin
cd "$HOME/humanoid" && source .venv/bin/activate || exit 2
[ -f "$DATA_ROOT/$DS/meta/info.json" ] || { echo "Dataset not found: $DATA_ROOT/$DS"; exit 2; }
read EPIS FRAMES < <(python -c "import json;d=json.load(open('$DATA_ROOT/$DS/meta/info.json'));print(d['total_episodes'],d['total_frames'])")
echo "Dataset $DS: $EPIS episodes, $FRAMES frames | ACT, device mps, steps $STEPS, batch $BATCH, checkpoint every $SAVE_FREQ${DEADLINE:+, deadline $DEADLINE}"
mkdir -p "$(dirname "$OUT")"
export PYTORCH_ENABLE_MPS_FALLBACK=1   # CPU fallback for the few ops MPS lacks
ARGS=(--dataset.repo_id="local/$DS" --dataset.root="$DATA_ROOT/$DS"
      --policy.type=act --policy.device="${POLICY_DEVICE:-mps}" --policy.push_to_hub=false
      --output_dir="$OUT" --job_name="$RUN_NAME"
      --steps="$STEPS" --batch_size="$BATCH" --save_freq="$SAVE_FREQ" --log_freq="${LOG_FREQ:-100}"
      --num_workers="$NUM_WORKERS" --trackio.enable=false)
[ -n "${EPISODES:-}" ] && ARGS+=(--dataset.episodes="$EPISODES")   # e.g. EPISODES='[0,2,3,4,5]' skips failed demos
if [ "${RESUME:-0}" = 1 ]; then
  ARGS=(--config_path="$OUT/checkpoints/last/pretrained_model/train_config.json" --resume=true)
fi
LOG="$OUT.log"
TJ="$OUT.timing.json"
START_EPOCH=$(date +%s)
echo "Log: $LOG   Timing: $TJ"

timing() {   # timing <status>  -> writes $TJ (and $OUT/timing.json once $OUT exists)
  python - "$1" <<PY
import json, os, re, sys, time, datetime, zoneinfo
tz = zoneinfo.ZoneInfo("Europe/Berlin")
st, now = $START_EPOCH, time.time()
step = 0; loss = None
try:
    for line in open("$LOG", errors="ignore"):
        m = re.search(r"step:(\d+(?:\.\d+)?)([KMB]?)\b", line)
        if m:
            step = int(float(m.group(1)) * {"": 1, "K": 1e3, "M": 1e6, "B": 1e9}[m.group(2)])
            l = re.search(r"loss:([\d.eE+-]+)", line); loss = float(l.group(1)) if l else loss
except OSError:
    pass
ck = sorted(int(d) for d in os.listdir("$OUT/checkpoints") if d.isdigit()) if os.path.isdir("$OUT/checkpoints") else []
dur = now - st; sps = step / dur if dur > 0 and step else None
d = {"run": "$RUN_NAME", "dataset": "$DS", "episodes": $EPIS, "frames": $FRAMES, "status": sys.argv[1],
     "start": datetime.datetime.fromtimestamp(st, tz).isoformat(timespec="seconds"),
     "end": datetime.datetime.fromtimestamp(now, tz).isoformat(timespec="seconds") if sys.argv[1] != "running" else None,
     "updated": datetime.datetime.fromtimestamp(now, tz).isoformat(timespec="seconds"),
     "wall_clock_s": round(dur), "wall_clock_hms": time.strftime("%H:%M:%S", time.gmtime(dur)),
     "steps_planned": $STEPS, "steps_reached_logged": step, "last_checkpoint_step": ck[-1] if ck else None,
     "steps_per_sec": round(sps, 3) if sps else None, "last_loss": loss, "batch": $BATCH, "deadline": "$DEADLINE" or None,
     "eta_planned_steps": (datetime.datetime.fromtimestamp(st + $STEPS / sps, tz).isoformat(timespec="minutes") if sps else None)}
s = json.dumps(d, indent=2)
open("$TJ", "w").write(s)
if os.path.isdir("$OUT"): open("$OUT/timing.json", "w").write(s)
print(f"[timing] {d['status']} step {step} ({d['steps_per_sec']} steps/s) loss {loss} wall {d['wall_clock_hms']}")
PY
}

prune() {   # keep only the newest $KEEP_CKPT numbered checkpoints (never the one 'last' points to)
  [ -d "$OUT/checkpoints" ] || return 0
  python - <<PY
import os, shutil
d = "$OUT/checkpoints"; last = os.path.basename(os.path.realpath(os.path.join(d, "last")))
ck = sorted((int(c) for c in os.listdir(d) if c.isdigit()))
for c in ck[:-$KEEP_CKPT] if $KEEP_CKPT > 0 else []:
    name = next(n for n in os.listdir(d) if n.isdigit() and int(n) == c)
    if name != last: shutil.rmtree(os.path.join(d, name), ignore_errors=True)
PY
}

STOPPED_BY_DEADLINE=0
caffeinate -i -s lerobot-train "${ARGS[@]}" > >(tee "$LOG") 2>&1 &
TPID=$!
DL_EPOCH=""
[ -n "$DEADLINE" ] && DL_EPOCH=$(python -c "
import datetime,zoneinfo;tz=zoneinfo.ZoneInfo('Europe/Berlin');n=datetime.datetime.now(tz)
h,m=map(int,'$DEADLINE'.split(':'));d=n.replace(hour=h,minute=m,second=0,microsecond=0)
d=d+datetime.timedelta(days=1) if d<=n else d;print(int(d.timestamp()))")
TICK=0
while kill -0 $TPID 2>/dev/null; do
  sleep 5; TICK=$((TICK + 5))
  [ $((TICK % 60)) -eq 0 ] || continue
  timing running >/dev/null 2>&1; prune
  if [ -n "$DL_EPOCH" ] && [ "$(date +%s)" -ge "$DL_EPOCH" ]; then
    echo "[deadline] $DEADLINE reached – stopping training gracefully (newest checkpoint is kept)"
    STOPPED_BY_DEADLINE=1
    pkill -INT -f "lerobot-train.*--job_name=$RUN_NAME" ; sleep 30
    pkill -TERM -f "lerobot-train.*--job_name=$RUN_NAME"; sleep 5; kill $TPID 2>/dev/null
    break
  fi
done
wait $TPID 2>/dev/null; RC=$?
prune
if [ $STOPPED_BY_DEADLINE = 1 ]; then STATUS="stopped_at_deadline"; elif [ $RC -eq 0 ]; then STATUS="finished"; else STATUS="failed_rc_$RC"; fi
timing "$STATUS"
# one CHANGELOG line per run
CL=""
for c in "$HOME/humanoid/rebot-B601-RS/docs/CHANGELOG.md" "$HOME/humanoid/docs/CHANGELOG.md"; do [ -f "$c" ] && CL="$c" && break; done
[ -z "$CL" ] && CL="$HOME/humanoid/docs/CHANGELOG.md" && mkdir -p "$(dirname "$CL")" && echo "# CHANGELOG" > "$CL"
python - "$CL" <<PY
import json, sys
d = json.load(open("$TJ"))
line = (f"- Training run {d['run']} ({d['dataset']}, {d['episodes']} episodes, {d['frames']} frames): {d['status']}; "
        f"start {d['start']}, end {d['end']}, wall clock {d['wall_clock_hms']}, steps {d['steps_reached_logged']} "
        f"(planned {d['steps_planned']}, last checkpoint {d['last_checkpoint_step']}), {d['steps_per_sec']} steps/s, "
        f"last loss {d['last_loss']}.\n")
s = open(sys.argv[1]).read()
hdr = "## Training runs (automatic, written by train_act.sh)\n"
if hdr not in s: s = s.rstrip("\n") + "\n\n" + hdr
s = s.replace(hdr, hdr + line, 1)
open(sys.argv[1], "w").write(s)
PY
if [ -L "$OUT/checkpoints/last" ] || [ -d "$OUT/checkpoints/last" ]; then
  echo "Done ($STATUS). Policy: $OUT/checkpoints/last/pretrained_model"; exit 0
fi
echo "Training FAILED ($STATUS) - see $LOG"; exit 1
