#!/bin/bash
cd "$(dirname -- "${BASH_SOURCE[0]}")"
colcon build --symlink-install
source setup-terminal.sh && cd src/launch && ros2 launch launch.py
