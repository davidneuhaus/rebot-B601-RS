#!/bin/bash
# Starts the teleop GUI (log: /tmp/teleop_gui.log). The window can be closed; the Terminal window may stay.
cd "$HOME/humanoid" && source .venv/bin/activate && export DYLD_LIBRARY_PATH="$HOME/humanoid/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}" && nohup "$HOME/humanoid/.venv/bin/python" "$HOME/humanoid/teleop_gui.py" >> /tmp/teleop_gui.log 2>&1 &
