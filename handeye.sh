#!/bin/bash
# Automatic hand-eye calibration (v3.9, v3.9.1) - the "🤖 Auto-Kalibrierung" button runs `handeye.sh run --yes --auto`
# (automatic start, default) or `handeye.sh run --yes` (from the taught pose).
#   ./handeye.sh teach | plan [--auto] | run [--yes] [--auto] | run --sim [--auto] | replay <folder> | apply <folder> [--force] | status
# Same safety settings as play_policy.sh: collision guard ON with all COLLISION_/MIT_/GRAVITY_/GRIPPER_ exports of
# teleop.sh. v3.9.3: profile 'kraeftig' (factory: gripper kp 12, no extra closing cap; the gripper is held where it is). Motion (v3.9.1): approach 6 deg/s, sweep max 12 deg/s,
# LeRobot max_relative_target 4 deg; the goal stays within 3 deg of the measured position (never clamped).
# STOP: Ctrl+C or SIGTERM = immediate torque off (then a partial sweep with >= 10 samples is solved, v3.9.2). Emergency stop is still the power strip. Log: logs/handeye_*.log
cd "$HOME/humanoid" || exit 2
export DYLD_LIBRARY_PATH="$HOME/humanoid/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
# v3.9.3: 'kraeftig' profile (was 'vorsichtig' CLOSE=1.1 KP=10). Arm kp/kd, guard and gravity FF come from teleop.sh
# and are the same for every profile; near the board (<= 160 mm) the sweep runs at 8 deg/s.
CLOSE=0; KP=12.0; TQ=3.5; HOLD=1.0
eval "$(grep -E '^export (COLLISION_|MIT_|GRAVITY_|GRIPPER_)[A-Z_]*=' teleop.sh)"
export COLLISION_GUARD=1 GRIPPER_CLOSE_TQ=$CLOSE
echo "Arm-Profil: kraeftig (Greifer kp=$KP, schliessen ohne Extra-Limit), Kollisionsschutz AN, Schwerkraft-Feedforward ${GRAVITY_FF:-0} (x${GRAVITY_FF_SCALE:-})"
export PLAY_GRIPPER_KP=$KP PLAY_GRIPPER_TQ=$TQ PLAY_GRIPPER_HOLD=$HOLD
LOGDIR="${HANDEYE_LOG_DIR:-logs}"
case " $* " in *" --sim "*) LOGDIR="${HANDEYE_LOG_DIR:-${TMPDIR:-/tmp}/handeye_sim_logs}" ;; esac   # v3.9.2: sim runs never log into logs/
mkdir -p "$LOGDIR"; LOG="$LOGDIR/handeye_$(date +%Y-%m-%d_%H-%M-%S).log"
ls -1t "$LOGDIR"/handeye_*.log 2>/dev/null | tail -n +31 | while read -r f; do rm -f "$f"; done   # keep 30
echo "Log: $LOG"
set -o pipefail
# v3.9.2: STOP (SIGTERM to the process group) must only reach handeye.py: it switches the motors off and may then
# solve the partial sweep. bash keeps waiting (trap with a command; children get default handlers) and tee ignores
# the signal, so the remaining output still reaches the log and the GUI. A second STOP ends handeye.py.
trap 'true' TERM INT
"$HOME/humanoid/.venv/bin/python" -u vision/handeye.py "$@" 2>&1 | ( trap '' TERM INT; exec tee -a "$LOG" )
