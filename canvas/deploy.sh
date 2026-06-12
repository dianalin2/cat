#!/bin/bash
ROBOT_IP="${1:-$(hostname -I | awk '{print $1}')}"

cd "$(dirname -- "${BASH_SOURCE[0]}")"
source setup-terminal.sh $ROBOT_IP
tmux new-session "cd src/launch && ros2 launch launch.py" \; split-window -h "npm i && npm run dev"
