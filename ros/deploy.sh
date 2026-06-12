#!/bin/bash
cd "$(dirname -- "${BASH_SOURCE[0]}")"
colcon build --symlink-install
source install/setup.bash && cd src/launch && ros2 launch launch.py
