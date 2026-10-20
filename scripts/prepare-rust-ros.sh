#!/usr/bin/env bash
# Optional no-root installation of rclrs 0.7's test_msgs C typesupport dependency.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
distro="${ROS_DISTRO:?Source the ROS setup.bash first}"
prefix="${repo}/.deps/ros/opt/ros/${distro}"
if [[ -f "${prefix}/lib/libtest_msgs__rosidl_generator_c.so" ]]; then
  # Give the extracted package a usable local ROS prefix without changing /opt/ros.
  cat > "${prefix}/local_setup.bash" <<'SETUP'
_hei_local_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${_hei_local_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_hei_local_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
unset _hei_local_prefix
SETUP
  echo 'Local test_msgs is ready.'
  exit 0
fi
if [[ -f "/opt/ros/${distro}/lib/libtest_msgs__rosidl_generator_c.so" ]]; then
  echo 'test_msgs is already available.'
  exit 0
fi
package="ros-${distro}-test-msgs"
metadata="$(apt-cache show "$package")"
filename="$(awk '/^Filename: / {print $2; exit}' <<< "$metadata")"
checksum="$(awk '/^SHA256: / {print $2; exit}' <<< "$metadata")"
[[ -n "$filename" && -n "$checksum" ]]
work="$(mktemp -d)"
trap 'rm -rf -- "$work"' EXIT
# Check against the checksum in the configured apt repository metadata.
curl --fail --location --max-time 120 "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/${filename}" -o "${work}/package.deb"
printf '%s  %s\n' "$checksum" "${work}/package.deb" | sha256sum --check --status
mkdir -p "${repo}/.deps/ros"
dpkg-deb --extract "${work}/package.deb" "${repo}/.deps/ros"
# Re-enter to generate local_setup.bash now that the archive has been extracted.
exec "$0"
