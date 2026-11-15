#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
mkdir -p .deps/debs .deps/root
cd .deps/debs
apt-get download python3-filelock ros-jazzy-controller-manager-msgs ros-jazzy-controller-manager ros-jazzy-forward-command-controller ros-jazzy-effort-controllers ros-jazzy-ros2-control-cmake ros-jazzy-gz-ros2-control
for package in *.deb; do
  dpkg-deb -x "$package" ../root
done
