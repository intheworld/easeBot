distro=$2
ros_setup=$3
shift 3
if [[ "$ros_setup" == '~/'* ]]; then ros_setup="$HOME/${ros_setup:2}"; fi
if test -z "$ros_setup"; then
  ros_setup="/opt/ros/$distro/setup.bash"
  if ! test -f "$ros_setup"; then ros_setup="$HOME/ros2_${distro}/install/setup.bash"; fi
fi
test -f "$ros_setup" || { echo "ROS setup not found: $ros_setup" >&2; exit 1; }
source "$ros_setup"
test "$ROS_DISTRO" = "$distro" || { echo 'ROS distro does not match the target config.' >&2; exit 1; }
source scripts/env/workspace.bash
command -v colcon >/dev/null || { echo 'Install colcon and the required build plugins first.' >&2; exit 1; }
args=()
if (( $# )); then args=(--packages-up-to "$@"); fi
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install "${args[@]}"
