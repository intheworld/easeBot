#!/usr/bin/env bash
# Populate this machine's Cargo cache. Does not open devices or start ROS nodes.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "$HOME/.cargo/env" ]]; then source "$HOME/.cargo/env"; fi
command -v cargo >/dev/null || { echo 'Install Rust/cargo first.' >&2; exit 1; }
# Avoid the workspace's ROS message Cargo patches; this backend has no ROS crates.
cd /tmp
cargo fetch --locked --manifest-path "$repo_root/ros2_ws/src/hei_hardware/motorbridge_backend/Cargo.toml"
