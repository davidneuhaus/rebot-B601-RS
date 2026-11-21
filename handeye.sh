#!/bin/bash
# Automatic hand-eye calibration (v3.9, v3.9.1) - the "🤖 Auto-Kalibrierung" button runs `handeye.sh run --yes --auto`
# (automatic start, default) or `handeye.sh run --yes` (from the taught pose).
#   ./handeye.sh teach | plan [--auto] | run [--yes] [--auto] | run --sim [--auto] | replay <folder> | apply <folder> [--force] | status
# Same safety settings as play_policy.sh: collision guard ON with all COLLISION_/MIT_/GRAVITY_/GRIPPER_ exports of
# teleop.sh, gripper 'vorsichtig' (held where it is). Motion (v3.9.1): approach 6 deg/s, sweep max 12 deg/s,
# LeRobot max_relative_target 4 deg; the goal stays within 3 deg of the measured position (never clamped).
# STOP: Ctrl+C or SIGTERM = immediate torque off. Emergency stop is still the power strip. Log: logs/handeye_*.log
cd "$HOME/humanoid" || exit 2
export DYLD_LIBRARY_PATH="$HOME/humanoid/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
CLOSE=1.1; KP=10.0; TQ=3.5; HOLD=1.0
eval "$(grep -E '^export (COLLISION_|MIT_|GRAVITY_|GRIPPER_)[A-Z_]*=' teleop.sh)"
export COLLISION_GUARD=1 GRIPPER_CLOSE_TQ=${GRIPPER_CLOSE_TQ:-$CLOSE}
export PLAY_GRIPPER_KP=${PLAY_GRIPPER_KP:-$KP} PLAY_GRIPPER_TQ=${PLAY_GRIPPER_TQ:-$TQ} PLAY_GRIPPER_HOLD=${PLAY_GRIPPER_HOLD:-$HOLD}
mkdir -p logs; LOG="logs/handeye_$(date +%Y-%m-%d_%H-%M-%S).log"
ls -1t logs/handeye_*.log 2>/dev/null | tail -n +31 | while read -r f; do rm -f "$f"; done   # keep 30
echo "Log: $LOG"
set -o pipefail
"$HOME/humanoid/.venv/bin/python" -u vision/handeye.py "$@" 2>&1 | tee -a "$LOG"
