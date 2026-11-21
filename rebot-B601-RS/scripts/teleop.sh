#!/bin/bash
# Teleoperation: white reBot 102 (leader) drives the large B601-RS (follower). Stop with Ctrl+C.
# Usage: teleop.sh [vorsichtig|mittel|kraeftig]   (default: vorsichtig = gentle gripper)
# Joints max 5°/step, gripper max 25°/step.
STUFE="${1:-vorsichtig}"
case "$STUFE" in
  # v3.7: TQ (opening limit) 1.5/2.0 -> 3.5 Nm: at 1.5 Nm the gripper stalled half open (75-175 of ~270 deg,
  # session 13:43); at 3.5 Nm ('kraeftig', session 13:52) it opened to ~245-270 deg. Closing stays capped by CLOSE.
  vorsichtig) KP=10.0; TQ=3.5; HOLD=1.0;  CLOSE=1.1 ;;   # soft closing, for cardboard/fragile items
  mittel)     KP=12.0; TQ=3.5; HOLD=1.2;  CLOSE=1.2 ;;
  kraeftig)   KP=12.0; TQ=3.5; HOLD=1.0;  CLOSE=0   ;;   # factory setting
  *) echo "Unbekannte Stufe: $STUFE"; exit 1 ;;
esac
export GRIPPER_CLOSE_TQ=$CLOSE
# Collision guard for the 6 arm joints (software detection, NOT certified – the e-stop is still the power strip).
# Detection: leader deviates more than ERR degrees from the follower AND the follower makes no progress
# (< MIN_PROGRESS degrees per cycle) for CYCLES cycles of 33 ms -> the arm freezes.
# Release once the leader is back within RESUME degrees of the follower position. COLLISION_GUARD=0 = guard off.
export COLLISION_GUARD=${COLLISION_GUARD:-1}
export COLLISION_ERR_DEG=${COLLISION_ERR_DEG:-10}               # shoulder pan/lift, elbow
export COLLISION_ERR_WRIST_DEG=${COLLISION_ERR_WRIST_DEG:-15}   # 3 wrist axes
export COLLISION_CYCLES=${COLLISION_CYCLES:-6}
export COLLISION_MIN_PROGRESS_DEG=${COLLISION_MIN_PROGRESS_DEG:-0.15}
export COLLISION_RESUME_DEG=${COLLISION_RESUME_DEG:-6}
# Guard v2 (rotary joints): shoulder_pan and wrist_yaw are compliant (kp 20 instead of 50)
# so a hand can stop the joint instead of being pushed away. kd ~ kd*sqrt(20/50).
export MIT_KP_SHOULDER_PAN=${MIT_KP_SHOULDER_PAN:-20}
export MIT_KD_SHOULDER_PAN=${MIT_KD_SHOULDER_PAN:-1.9}
export MIT_KP_WRIST_YAW=${MIT_KP_WRIST_YAW:-20}
export MIT_KD_WRIST_YAW=${MIT_KD_WRIST_YAW:-2.5}
# wrist_flex (wrist up/down) carries the gripper (~2.3 Nm gravity): kp 40 instead of 50, NOT 20 –
# below kp ~30, kp*5° (step limit) no longer beats gravity and the wrist would sag away.
export MIT_KP_WRIST_FLEX=${MIT_KP_WRIST_FLEX:-40}
export MIT_KD_WRIST_FLEX=${MIT_KD_WRIST_FLEX:-4.5}
export COLLISION_ERR_WRIST_FLEX=${COLLISION_ERR_WRIST_FLEX:-10}   # 8 gave false alarms in the 17:50 cardboard log (wrist resting 8.3° off)
export COLLISION_ERR_SHOULDER_PAN=${COLLISION_ERR_SHOULDER_PAN:-6}
export COLLISION_ERR_WRIST_YAW=${COLLISION_ERR_WRIST_YAW:-8}
# Torque trigger (Nm, |torque| > value for TORQ_CYCLES cycles). At kp 20 max approx. 1.75 Nm (kp*5°);
# values were scaled from kp-50 measurements and tuned with David's test logs.
# v3.1: torque alone means little for compliant joints (at a full step the joint always presses
# kp*5°). So additionally: offset >= TORQ_MIN_LEAD degrees AND almost no progress, for 6 cycles (200 ms).
export COLLISION_TORQ_SHOULDER_PAN=${COLLISION_TORQ_SHOULDER_PAN:-1.6}    # v3.0: 1.45 – false alarms at 1.49 Nm
# v3.7: wrist_yaw 1.25 -> 2.4 Nm: 11 false freezes on 2026-11-21 at 1.37-2.10 Nm during normal moves (max 2.16 Nm in
# the 5 s stats). At kp 20 a static block can only press kp*5deg = 1.75 Nm, so a held wrist is now caught by the
# lag trigger (COLLISION_ERR_WRIST_YAW=8 deg, 6 cycles); the torque trigger only catches hard dynamic hits.
export COLLISION_TORQ_WRIST_YAW=${COLLISION_TORQ_WRIST_YAW:-2.4}         # v3.0: 1.0, v3.1-v3.6: 1.25
export COLLISION_TORQ_WRIST_FLEX=${COLLISION_TORQ_WRIST_FLEX:-0}         # off: at kp 40 max ~3.5 Nm = normal fast motion
export COLLISION_TORQ_CYCLES=${COLLISION_TORQ_CYCLES:-6}
export COLLISION_TORQ_MIN_LEAD_DEG=${COLLISION_TORQ_MIN_LEAD_DEG:-3}
export COLLISION_TORQ_MAX_PROGRESS_DEG=${COLLISION_TORQ_MAX_PROGRESS_DEG:-0.5}
# Auto-resume (v3): once the frozen arm has stood still for AUTO_RESUME_S seconds it creeps (max RESUME_STEP_DEG
# degrees/cycle instead of 5) towards the leader; if a joint gets stuck on an obstacle it freezes again
# (v3.1: unlimited attempts, see below). 0 = off.
export COLLISION_AUTO_RESUME=${COLLISION_AUTO_RESUME:-1}
export COLLISION_AUTO_RESUME_S=${COLLISION_AUTO_RESUME_S:-1.5}
export COLLISION_RESUME_STEP_DEG=${COLLISION_RESUME_STEP_DEG:-1}
# v3.1: no dead end – new attempts after 1.5 s, 3 s, 5 s, then every 5 s (unlimited). While creeping
# it re-freezes only on a real obstacle (joint lags > 3° (compliant) or 2° (stiff) behind its target and does not
# move for 10 cycles). Manual release: bring only the collided joint back with the leader – the rest follows slowly.
# Optional per joint: COLLISION_ERR_ELBOW_FLEX=8, COLLISION_TORQ_SHOULDER_LIFT=12 (Nm, default off); 0 = trigger off
# v3.2: gravity-aware guard. Torque triggers use |measured - g(q)| (URDF gravity model of the plugin); lag triggers get
# extra tolerance COLLISION_GRAV_LAG_GAIN * g/kp when the joint lags against gravity, and are skipped when the
# uncompensated gravity exceeds COLLISION_GRAV_SAT_FRAC of kp*5deg. COLLISION_GRAVITY=0 = exact v3.1 behaviour.
# GRAVITY_FF=1 adds GRAVITY_FF_SCALE * g(q) as MIT torque feedforward. v3.7: ON by default (ramped in over 1 s, capped
# per joint: lift 14, elbow 9, wrist_flex 3, rotary 1 Nm). Without it, lifting overhead needs up to 11.4 Nm from the
# P-term alone, which kp 150 can only just deliver inside the 5 deg step limit (13.1 Nm) -> the arm stalled.
# v3.7 (guard v3.3): COLLISION_GRAV_RES_NM - a lag against gravity on shoulder_lift/elbow_flex/wrist_flex only counts
# as a collision when the joint pushes more than this beyond the expected gravity torque (false freezes today:
# shoulder_lift 1.57/1.72 Nm, wrist_flex 1.38 Nm). With GRAVITY_FF on, a real block pushes ~kp*5deg - 0.2*g
# (lift/elbow >= 10 Nm, wrist_flex ~3 Nm), so it is still detected.
export COLLISION_GRAVITY=${COLLISION_GRAVITY:-1}
export COLLISION_GRAV_LAG_GAIN=${COLLISION_GRAV_LAG_GAIN:-1.5}
export COLLISION_GRAV_SAT_FRAC=${COLLISION_GRAV_SAT_FRAC:-0.85}
export GRAVITY_FF=${GRAVITY_FF:-1}
export GRAVITY_FF_SCALE=${GRAVITY_FF_SCALE:-0.8}
export GRAVITY_FF_RAMP_S=${GRAVITY_FF_RAMP_S:-1.0}
export COLLISION_GRAV_RES_NM=${COLLISION_GRAV_RES_NM:-4.0}                 # shoulder_lift / elbow_flex (kp 150: kp*5deg = 13.1 Nm)
export COLLISION_GRAV_RES_WRIST_FLEX=${COLLISION_GRAV_RES_WRIST_FLEX:-1.5}  # kp 40: kp*5deg = 3.5 Nm; false freeze 13:44:42 pushed 1.38 Nm
export COLLISION_GRAV_RES_JOINTS=${COLLISION_GRAV_RES_JOINTS:-shoulder_lift,elbow_flex,wrist_flex}
echo "Greifer-Stufe: $STUFE (kp=$KP, schliessen max=$CLOSE Nm, oeffnen max=$TQ Nm, halten=$HOLD Nm; 0=ohne Extra-Limit)"
echo "Kollisionsschutz: $([ "$COLLISION_GUARD" = 1 ] && echo AN || echo AUS) (Abweichung ${COLLISION_ERR_DEG}°/${COLLISION_ERR_WRIST_DEG}°, ${COLLISION_CYCLES} Zyklen; Drehen ${COLLISION_ERR_SHOULDER_PAN}°/${COLLISION_ERR_WRIST_YAW}°, Kraft ${COLLISION_TORQ_SHOULDER_PAN}/${COLLISION_TORQ_WRIST_YAW}/${COLLISION_TORQ_WRIST_FLEX} Nm ab ${COLLISION_TORQ_MIN_LEAD_DEG}°)"
echo "Schwerkraft-Schutz (v3.3): $([ "$COLLISION_GRAVITY" = 1 ] && echo AN || echo AUS), Feedforward: $([ "$GRAVITY_FF" = 1 ] && echo "AN (x$GRAVITY_FF_SCALE)" || echo AUS), Stillstand-Schwelle ${COLLISION_GRAV_RES_NM} Nm"
echo "Nachgiebig: Schulter-Drehung kp=${MIT_KP_SHOULDER_PAN} kd=${MIT_KD_SHOULDER_PAN}, Handgelenk-Drehung kp=${MIT_KP_WRIST_YAW} kd=${MIT_KD_WRIST_YAW}, Handgelenk hoch/runter kp=${MIT_KP_WRIST_FLEX} kd=${MIT_KD_WRIST_FLEX}; Auto-Weiter: $([ "$COLLISION_AUTO_RESUME" = 1 ] && echo AN || echo AUS)"
cd ~/humanoid && source .venv/bin/activate
# Pre-flight check, waits up to 30 s in case the arm was just switched on
for i in $(seq 1 6); do
  OUT=$(python preflight.py 2>/dev/null); RC=$?
  [ $RC -eq 0 ] && break
  echo "$OUT"
  [ $RC -ne 4 ] && exit $RC
  [ $i -lt 6 ] && echo "… warte auf Roboterarm ($i/6)" && sleep 5
done
[ $RC -ne 0 ] && exit $RC
LEADER_PORT="$OUT"
echo "Check OK – Leader: $LEADER_PORT, alle Motoren antworten."
# v3.3: lerobot_teleoperate_tap.py = unchanged lerobot-teleoperate + UDP telemetry tap for the dataset recorder
# (TELEMETRY_TAP=0 disables the tap). Laptop: .venv is a shim that activates the conda env "lerobot".
exec python lerobot_teleoperate_tap.py \
  --robot.type=seeed_b601_rs_follower --robot.port=can0 --robot.id=follower \
  --robot.max_relative_target='{"shoulder_pan":5,"shoulder_lift":5,"elbow_flex":5,"wrist_flex":5,"wrist_yaw":5,"wrist_roll":5,"gripper":25}' \
  --robot.gripper_mit_kp=$KP \
  --robot.gripper_mit_torque_limit=$TQ \
  --robot.gripper_mit_hold_torque_limit=$HOLD \
  --teleop.type=rebot_arm_102_leader --teleop.port=$LEADER_PORT --teleop.id=leader \
  --fps=30
