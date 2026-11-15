#!/usr/bin/env bash
SENTRY_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/jazzy/setup.bash
SENTRY_DEPS="$SENTRY_WS/.deps/root/opt/ros/jazzy"
if [[ -d "$SENTRY_DEPS" ]]; then
  export AMENT_PREFIX_PATH="$SENTRY_DEPS:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$SENTRY_DEPS:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$SENTRY_DEPS/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$SENTRY_DEPS/lib/python3.12/site-packages:$SENTRY_WS/.deps/root/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export PATH="$SENTRY_DEPS/bin:$PATH"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$SENTRY_DEPS/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
fi
if [[ "${SENTRY_SOURCE_OVERLAY:-1}" == 1 && -f "$SENTRY_WS/install/local_setup.bash" ]]; then
  source "$SENTRY_WS/install/local_setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export GZ_PARTITION="${GZ_PARTITION:-tunnel_sentinel_47}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$SENTRY_WS/src/tunnel_sentinel_bringup/config/dds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export ROS_LOG_DIR="$SENTRY_WS/log/ros"
export ROS_HOME="$SENTRY_WS/.runtime/ros"
export GZ_SIM_LOG_PATH="$SENTRY_WS/log/gazebo"
export GZ_LOG_PATH="$SENTRY_WS/log/gazebo"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$GZ_SIM_LOG_PATH"
