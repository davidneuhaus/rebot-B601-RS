#!/bin/bash
# Start the iPhone browser camera bridge in the background
cd "$(dirname "$0")"
PY="$HOME/humanoid/.venv/bin/python"
IP="$(ipconfig getifaddr en0 || ipconfig getifaddr en1)"
[ -f cert.pem ] && openssl x509 -in cert.pem -noout -text | grep -q "IP Address:$IP" || ./make_cert.sh "$IP"
if [ -f server.pid ] && kill -0 "$(cat server.pid)" 2>/dev/null; then echo "already running (pid $(cat server.pid))"; else
  nohup "$PY" server.py > server.log 2>&1 &
  echo $! > server.pid; sleep 1.5
  kill -0 "$(cat server.pid)" 2>/dev/null || { echo "failed:"; cat server.log; exit 1; }
fi
echo "iPhone links (with access token): see $(pwd)/links.txt  (accept the certificate warning, tap Start)"
echo "OpenCV: http://127.0.0.1:${IPHONE_WEB_CAM_MJPEG_PORT:-8090}/stream1.mjpg  and  /stream2.mjpg"
