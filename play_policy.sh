#!/bin/bash
# Play the newest trained ACT policy on the B601-RS follower (v3.4) - the "▶ Policy abspielen" button runs this too.
#   ./play_policy.sh [checkpoint_dir] [--seconds 60] [--dry-run] [--fake-cams] [--check]
# Collision guard: the same settings as teleop (all COLLISION_*/MIT_*/GRAVITY_* exports of teleop.sh), guard forced ON.
# Gripper force: "vorsichtig" (kp 10, 1.5 Nm, hold 1.0 Nm, close 1.1 Nm). Step limit: 2°/cycle arm, 10° gripper.
# STOP: Ctrl+C or SIGTERM = immediate torque off. Emergency stop is still the power strip. Log: logs/play_*.log
cd "$HOME/humanoid" && source .venv/bin/activate || exit 2
CLOSE=1.1; KP=10.0; TQ=1.5; HOLD=1.0
eval "$(grep -E '^export (COLLISION_|MIT_|GRAVITY_|GRIPPER_)[A-Z_]*=' teleop.sh)"
export COLLISION_GUARD=1 GRIPPER_CLOSE_TQ=${GRIPPER_CLOSE_TQ:-$CLOSE}
export PLAY_GRIPPER_KP=${PLAY_GRIPPER_KP:-$KP} PLAY_GRIPPER_TQ=${PLAY_GRIPPER_TQ:-$TQ} PLAY_GRIPPER_HOLD=${PLAY_GRIPPER_HOLD:-$HOLD}
export PYTORCH_ENABLE_MPS_FALLBACK=1
mkdir -p logs; LOG="logs/play_$(date +%Y-%m-%d_%H-%M-%S).log"
ls -1t logs/play_*.log 2>/dev/null | tail -n +51 | while read -r f; do rm -f "$f"; done   # keep 50
echo "Log: $LOG"
export PLAY_LOG="$LOG"
exec python -u play_policy.py "$@"
