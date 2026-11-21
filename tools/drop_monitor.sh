#!/bin/bash
# Passive drop diagnostics monitor (v3.9.4): USB attach/detach, GUI camera frame stalls, log lines (ms timestamps).
#   tools/drop_monitor.sh start | stop | status
# Output: logs/dropmon_<ts>/events.jsonl. Report: .venv/bin/python tools/drop_report.py
cd "$HOME/humanoid" || exit 2
PIDF="$HOME/humanoid/logs/dropmon.current"
case "${1:-status}" in
  start)
    if [ -f "$PIDF" ] && kill -0 "$(head -1 "$PIDF")" 2>/dev/null; then echo "already running: $(cat "$PIDF")"; exit 0; fi
    OUT="$HOME/humanoid/logs/dropmon_$(date +%Y-%m-%d_%H-%M-%S)"; mkdir -p "$OUT"
    nohup "$HOME/humanoid/.venv/bin/python" -u tools/drop_monitor.py "$OUT" >> "$OUT/monitor.out" 2>&1 < /dev/null &
    echo "$!" > "$PIDF"; echo "$OUT" >> "$PIDF"; sleep 1
    kill -0 "$(head -1 "$PIDF")" 2>/dev/null && echo "running: pid $(head -1 "$PIDF") -> $OUT" || { echo "failed"; cat "$OUT/monitor.out"; exit 1; } ;;
  stop)
    [ -f "$PIDF" ] && kill "$(head -1 "$PIDF")" 2>/dev/null; rm -f "$PIDF"; echo stopped ;;
  status)
    if [ -f "$PIDF" ] && kill -0 "$(head -1 "$PIDF")" 2>/dev/null; then echo "running: $(tr '\n' ' ' < "$PIDF")"; else echo "not running"; fi ;;
esac
