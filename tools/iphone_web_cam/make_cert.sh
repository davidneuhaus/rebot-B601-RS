#!/bin/bash
# Self-signed cert for the LAN IP + <LocalHostName>.local
set -e
cd "$(dirname "$0")"
IP="${1:-$(ipconfig getifaddr en0 || ipconfig getifaddr en1)}"
HOST="$(scutil --get LocalHostName 2>/dev/null || hostname -s).local"
openssl req -x509 -newkey rsa:2048 -nodes -days 825 -keyout key.pem -out cert.pem \
  -subj "/CN=$IP" -addext "subjectAltName=IP:$IP,DNS:$HOST,DNS:localhost,IP:127.0.0.1" 2>/dev/null
echo "cert for IP=$IP host=$HOST"
