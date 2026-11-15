#!/usr/bin/env bash
set -e
SENTRY_SOURCE_OVERLAY=0 source "$(dirname "$0")/env.sh"
cd "$SENTRY_WS"
python3 scripts/generate_lift.py
python3 scripts/generate_meshes.py
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
