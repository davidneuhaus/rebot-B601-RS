#!/bin/bash
cd "$(dirname "$0")"
if [ -f server.pid ] && kill "$(cat server.pid)" 2>/dev/null; then echo stopped; else pkill -f "iphone_web_cam/server.py" && echo stopped || echo "not running"; fi
rm -f server.pid
