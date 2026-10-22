#!/usr/bin/env bash
# Run on the robot after fetch-source.py. Never source Humble here.
set -eo pipefail
if (( $# )); then
  if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
    echo 'Usage: bash scripts/jazzy/build-source.bash'
    echo 'Build ros_base and demos in ~/ros2_jazzy using a clean environment.'
    exit 0
  fi
  echo 'This script accepts only --help; see scripts/README.md.' >&2
  exit 2
fi
if [[ ${HEI_JAZZY_CLEAN_ENV:-} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" \
    PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 PYTHONNOUSERSITE=1 \
    HEI_JAZZY_CLEAN_ENV=1 bash --noprofile --norc "$0" "$@"
fi
cd "$HOME/ros2_jazzy"
export ROS_HOME="$PWD/ros-home"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --merge-install --executor sequential \
  --packages-up-to ros_base demo_nodes_cpp demo_nodes_py \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DTRACETOOLS_DISABLED=ON 2>&1 | tee build-console.log
