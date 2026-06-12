#!/bin/bash
ROBOT_IP="${1:-$(hostname -I | awk '{print $1}')}"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$ROBOT_IP:7447\"]"
source $(dirname -- "${BASH_SOURCE[0]}")/../ros/install/setup.bash
