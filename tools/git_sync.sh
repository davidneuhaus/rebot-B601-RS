#!/bin/bash
# Sync the app folder ~/humanoid into the private repo davidneuhaus/rebot-B601-RS, branch main, then commit + push.
#   tools/git_sync.sh ["commit message"]     copy, check, commit, push
#   tools/git_sync.sh --dry-run              copy + check + show what would be committed, no commit/push
# Rules: only branch main (never 'website'), never force-push, never changes repo settings. Uses the git credentials
# already on this Mac (osxkeychain). Read-only for ~/humanoid; works in a separate clone ($HUMANOID_REPO).
# Excluded: see the RSYNC filter below and .gitignore (venv, caches, datasets, outputs, logs, tmp, snapshots, *.bak*,
# binaries, models, media, calibration captures, iPhone bridge secrets). Files > 1 MB are skipped.
# Scrubbed in the committed copy only: LAN IPs (192.168.x.x -> <lan-ip>), /Users/<name> -> ~.
# Before committing it refuses on: tokens (ghp_, github_pat_, sk-..., AKIA...), private keys, password assignments,
# e-mail addresses, LAN IPs, personal absolute paths, staged files > 1 MB.
set -euo pipefail
SRC="${HUMANOID_DIR:-$HOME/humanoid}"
REPO="${HUMANOID_REPO:-$HOME/humanoid_repo}"
URL="${HUMANOID_REPO_URL:-https://github.com/davidneuhaus/rebot-B601-RS}"
BRANCH=main
DRY=0; MSG=""
for a in "$@"; do case "$a" in --dry-run|-n) DRY=1 ;; *) MSG="$a" ;; esac; done
[ -n "$MSG" ] || MSG="sync v$(tr -d '[:space:]' < "$SRC/VERSION") $(date +%Y-%m-%d_%H:%M)"

if [ ! -d "$REPO/.git" ]; then
  git clone --branch "$BRANCH" --single-branch "$URL" "$REPO"
fi
cd "$REPO"
[ "$(git rev-parse --abbrev-ref HEAD)" = "$BRANCH" ] || git checkout "$BRANCH"
git pull --ff-only origin "$BRANCH"

rsync -a --delete --max-size=1m \
  --exclude='.git/' --exclude='.DS_Store' --exclude='.venv/' --exclude='venv/' --exclude='.pytest_cache/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='.laptop_install' --exclude='.env' \
  --exclude='/datasets/' --exclude='/outputs/' --exclude='/logs/' --exclude='/tmp/' --exclude='/_xfer/' \
  --exclude='/_backup*/' --exclude='*.log' --exclude='/rebot-B601-RS/versions/' --exclude='/rebot-B601-RS/backups/' \
  --exclude='*.bak*' --exclude='/lib/' --exclude='*.dylib' --exclude='*.so' --exclude='*.pt' --exclude='*.pth' \
  --exclude='*.ckpt' --exclude='*.onnx' --exclude='*.safetensors' --exclude='*.mp4' --exclude='*.mov' --exclude='*.avi' \
  --exclude='*.jpg' --exclude='*.jpeg' --include='/vision/calibration/charuco_7x5_35mm_A4.pdf' --exclude='*.pdf' \
  --exclude='/vision/models/' --exclude='/vision/calibration/handeye/' --exclude='/vision/calibration/history/' \
  --exclude='/vision/calibration/handeye.json' --exclude='/vision/calibration/handeye_teach.json' \
  --exclude='/vision/calibration/handeye_plan.json' --exclude='/vision/calibration/handeye_plan.png' \
  --exclude='/vision/calibration/handeye_auto.json' --exclude='/vision/calibration/table.json' \
  --exclude='/tools/iphone_web_cam/token.txt' --exclude='/tools/iphone_web_cam/links.txt' \
  --exclude='*.pem' --exclude='*.key' --exclude='*.crt' --exclude='*.p12' \
  "$SRC/" "$REPO/"

# scrub the committed copy (text files only)
git ls-files -co --exclude-standard -z | while IFS= read -r -d '' f; do
  [ -f "$f" ] && grep -Iq . "$f" 2>/dev/null || continue
  if grep -Eq '192\.168\.[0-9]+\.[0-9]+|/Users/[A-Za-z0-9._-]+' "$f"; then
    sed -E -i '' -e 's/192\.168\.[0-9]+\.[0-9]+/<lan-ip>/g' -e 's#/Users/[A-Za-z0-9._-]+#~#g' "$f"
    echo "scrubbed: $f"
  fi
done

git add -A
# checks on the staged content
fail=0
staged="$(git diff --cached --name-only)"
if [ -z "$staged" ]; then echo "nothing to commit"; exit 0; fi
while IFS= read -r f; do
  [ -f "$f" ] || continue
  sz=$(stat -f %z "$f"); if [ "$sz" -gt 1048576 ]; then echo "REFUSED: $f is $sz bytes (> 1 MB)"; fail=1; fi
done <<< "$staged"
PAT='ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9_-]{20,}|AKIA[0-9A-Z]{16}|-----BEGIN [A-Z ]*PRIVATE KEY|(password|passwd|api_key|apikey|secret_key)[[:space:]]*[:=][[:space:]]*["'"'"'][^"'"'"']{4,}|[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.(com|de|io|net|org)|192\.168\.[0-9]+\.[0-9]+|/Users/[A-Za-z0-9._-]+'
hits="$(git diff --cached -U0 | grep -E '^\+' | grep -Ev '^\+\+\+' | grep -En "$PAT" || true)"
if [ -n "$hits" ]; then echo "REFUSED: sensitive pattern in staged lines:"; echo "$hits" | cut -c1-160; fail=1; fi
echo "staged: $(echo "$staged" | wc -l | tr -d ' ') files"
if [ $fail -ne 0 ]; then git reset -q; echo "nothing committed"; exit 3; fi
if [ $DRY -eq 1 ]; then git diff --cached --stat | tail -3; git reset -q; echo "dry run: nothing committed"; exit 0; fi
git commit -q -m "$MSG"
git push origin "$BRANCH"          # never --force
echo "pushed $(git rev-parse --short HEAD) to $BRANCH: $MSG"
