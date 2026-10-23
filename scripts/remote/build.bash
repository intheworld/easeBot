command -v colcon >/dev/null || { echo 'Install colcon and the required build plugins first.' >&2; exit 1; }
args=()
if (( $# )); then args=(--packages-up-to "$@"); fi
colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install "${args[@]}"
