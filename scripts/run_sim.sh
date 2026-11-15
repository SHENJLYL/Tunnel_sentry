#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec ros2 launch tunnel_sentinel_bringup simulation.launch.py "$@"
