#!/bin/bash
# Rollback / restore for the reBot B601-RS project (no git commands, only copies files).
#
#   ./rollback.sh list                 list available snapshots
#   ./rollback.sh <snapshot>           copy the snapshot's files to ~/humanoid or into the plugin
#   ./rollback.sh original             restore the unpatched plugin (original 1.0.5)
#   ./rollback.sh apply                re-apply the current patches (e.g. after a plugin update)
#   ./rollback.sh status               show which snapshot matches the installed files
#   option -n / --dry-run              only show, change nothing
#
# Before every change the current state is saved to backups/<timestamp>/.
# Refuses to run while lerobot-teleoperate runs (press STOP in the app first).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUM="${HUMANOID_DIR:-$HOME/humanoid}"
PLUGIN="${PLUGIN_DIR:-$HUM/.venv/lib/python3.12/site-packages/lerobot_robot_seeed_b601}"
VERS="$HERE/versions"
DRY=0
ARGS=()
for a in "$@"; do
  case "$a" in
    -n|--dry-run) DRY=1 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) ARGS+=("$a") ;;
  esac
done
CMD="${ARGS[0]:-}"

say() { echo "$@"; }
run() { if [ $DRY -eq 1 ]; then echo "  [dry-run] $*"; else "$@"; fi; }

target_path() {   # HUM:teleop.sh -> ~/humanoid/teleop.sh
  case "$1" in
    HUM:*) echo "$HUM/${1#HUM:}" ;;
    PLUGIN:*) echo "$PLUGIN/${1#PLUGIN:}" ;;
    *) echo "" ;;
  esac
}

check_not_running() {
  if pgrep -f "lerobot-teleoperate|lerobot_teleoperate" >/dev/null 2>&1; then
    echo "ABBRUCH: Teleop läuft gerade (lerobot-teleoperate). Erst in der App STOPP drücken, dann erneut." >&2
    exit 3
  fi
}

check_paths() {
  [ -d "$HUM" ] || { echo "FEHLER: $HUM nicht gefunden" >&2; exit 2; }
  [ -d "$PLUGIN" ] || { echo "FEHLER: Plugin-Ordner nicht gefunden: $PLUGIN" >&2; exit 2; }
}

backup_current() {
  local ts dst
  ts="$(date +%Y-%m-%d_%H-%M-%S)"
  dst="$HERE/backups/$ts"
  [ -e "$dst" ] && dst="$dst-$$" && ts="$ts-$$"
  say "Sicherung des aktuellen Stands -> backups/$ts/"
  run mkdir -p "$dst"
  for f in "$HUM/teleop.sh" "$HUM/teleop_gui.py" "$HUM/VERSION" "$HUM/play_policy.py" "$HUM/play_policy.sh" "$HUM/preflight.py" "$HUM/recorder.py" "$HUM/camera_feeds.py" "$HUM/lerobot_teleoperate_tap.py" "$PLUGIN/seeed_b601_follower.py" "$PLUGIN/seeed_b601_rs_follower.py"; do
    [ -f "$f" ] && run cp -p "$f" "$dst/"
  done
}

list() {
  say "Snapshots in $VERS:"
  for d in "$VERS"/*/; do
    n="$(basename "$d")"
    files="$(awk '{print $2}' "$d/MANIFEST" | tr '\n' ' ')"
    printf "  %-24s %s\n" "$n" "$files"
  done
  say ""
  say "Beispiele: ./rollback.sh laptop_v3_6     (complete laptop install state, GUI 3.6 + guard v3.2)"
  say "           ./rollback.sh before_gui_v3_6 (GUI 3.5 without the command bar)"
  say "           ./rollback.sh guard_v3_1      (plugin with guard v3.1, no gravity model)"
  say "           ./rollback.sh original        (unpatched plugin 1.0.5)"
  say "           ./rollback.sh -n laptop_v3_6  (only show)"
}

status() {
  check_paths
  for d in "$VERS"/*/; do
    n="$(basename "$d")"; ok=1
    while read -r src tgt; do
      [ -z "$src" ] && continue
      cmp -s "$d/$src" "$(target_path "$tgt")" || ok=0
    done < "$d/MANIFEST"
    if [ $ok -eq 1 ]; then say "  passt: $n"; fi
  done
  return 0
}

restore() {
  local snap="$1" d="$VERS/$1"
  [ -f "$d/MANIFEST" ] || { echo "Unbekannter Snapshot: $snap  (./rollback.sh list)" >&2; exit 1; }
  check_paths; check_not_running
  backup_current
  say "Stelle Snapshot '$snap' wieder her:"
  while read -r src tgt; do
    [ -z "$src" ] && continue
    t="$(target_path "$tgt")"
    [ -n "$t" ] || { echo "Ungültiges Ziel im MANIFEST: $tgt" >&2; exit 1; }
    if cmp -s "$d/$src" "$t"; then say "  $tgt  (schon identisch)"; continue; fi
    say "  $src -> $t"
    run cp "$d/$src" "$t"
    case "$t" in *.sh) run chmod +x "$t" ;; esac
  done < "$d/MANIFEST"
  run rm -rf "$PLUGIN/__pycache__"
  [ $DRY -eq 1 ] && say "(dry-run: nichts geändert)" || say "Fertig. Wirkt beim nächsten START."
}

apply_patches() {
  check_paths; check_not_running
  backup_current
  local P="$HERE/patches"
  for f in seeed_b601_follower.py seeed_b601_rs_follower.py; do
    t="$PLUGIN/$f"
    if cmp -s "$P/patched/$f" "$t"; then say "  $f: Patch ist schon drin"; continue; fi
    if cmp -s "$P/original/$f" "$t"; then
      say "  $f: Original erkannt -> gepatchte Version kopieren"; run cp "$P/patched/$f" "$t"; continue
    fi
    known=""
    for v in "$VERS"/*/"$f"; do [ -f "$v" ] && cmp -s "$v" "$t" && known="$(basename "$(dirname "$v")")"; done
    if [ -n "$known" ]; then
      say "  $f: bekannte ältere Version ($known) -> gepatchte Version kopieren"; run cp "$P/patched/$f" "$t"; continue
    fi
    say "  $f: unbekannte Version (Plugin-Update?) -> versuche Diff anzuwenden"
    if patch --dry-run -s -p1 -d "$PLUGIN" < "$P/$f.diff" >/dev/null 2>&1; then
      run patch -s -p1 -d "$PLUGIN" < "$P/$f.diff" && say "    Diff angewendet"
    else
      echo "    FEHLER: Diff passt nicht auf die neue Plugin-Version. Bitte manuell prüfen ($P/$f.diff)." >&2
      exit 4
    fi
  done
  run rm -rf "$PLUGIN/__pycache__"
  [ $DRY -eq 1 ] && say "(dry-run: nichts geändert)" || say "Fertig. Wirkt beim nächsten START."
}

case "$CMD" in
  list|"") list ;;
  status) status ;;
  apply) apply_patches ;;
  *) restore "$CMD" ;;
esac
