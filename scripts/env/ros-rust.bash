# Source after /opt/ros/<distro>/setup.bash and before the workspace setup.bash.
_hei_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
_hei_rust_index="${_hei_repo}/.deps/ros-rust-index/${ROS_DISTRO:-jazzy}"
if [[ -d "${_hei_rust_index}/share" ]]; then
  export AMENT_PREFIX_PATH="${_hei_rust_index}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
fi
_hei_local_ros="${_hei_repo}/.deps/ros/opt/ros/${ROS_DISTRO:-jazzy}"
if [[ -d "${_hei_local_ros}/lib" ]]; then
  export AMENT_PREFIX_PATH="${_hei_local_ros}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${_hei_local_ros}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset _hei_repo _hei_local_ros _hei_rust_index
