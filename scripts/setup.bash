#!/usr/bin/env bash
# Prepare workspace dependencies on the machine that will build/run the project.
set -eo pipefail
without_agent=false
for arg in "$@"; do
  case "$arg" in
    --without-agent-bridge) without_agent=true ;;
    -h|--help)
      cat <<'HELP'
Usage: bash scripts/setup.bash [--without-agent-bridge]

Source the ROS 2 Jazzy underlay first. Prepare build plugins, ROS Rust discovery,
motorbridge Cargo dependencies, and (by default) the pinned Agent Bridge SDK.
Existing compatible tools are reused. Local Python dependencies stay in .deps/.
Does not install ROS/the Rust toolchain/system packages, build packages, or start nodes.
After completion: source scripts/env/workspace.bash, then run colcon build.
HELP
      exit 0 ;;
    *) echo "Unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_DISTRO:-}" != jazzy || -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo 'Source the ROS 2 Jazzy underlay setup.bash first (including controller dependencies).' >&2
  exit 1
fi
source "$repo_root/scripts/env/workspace.bash"
missing=()
for tool in python3 git cmake c++ cargo rustc colcon; do
  command -v "$tool" >/dev/null || missing+=("$tool")
done
if (( ${#missing[@]} )); then
  echo "Missing base tools: ${missing[*]}. Install the ROS/Rust build toolchain first; see scripts/README.md." >&2
  exit 1
fi
# Ensure essential ROS Rust bindings already exist, before downloading other dependencies.
python3 - <<'PY'
import os
from pathlib import Path
required = {'sensor_msgs', 'nav_msgs', 'diagnostic_msgs'}
for prefix in os.environ['AMENT_PREFIX_PATH'].split(os.pathsep):
    for name in tuple(required):
        if (Path(prefix) / 'share' / name / 'rust/Cargo.toml').is_file():
            required.remove(name)
if required:
    raise SystemExit('Missing generated ROS Rust bindings: ' + ', '.join(sorted(required)) +
                     '. Use the Jazzy environment described in hei_monitor/README.md.')
PY
printf '[1/4] Build tools\n'
python3 "$repo_root/scripts/setup/prepare-build-tools.py"
# A child installer cannot change this process's PATH/PYTHONPATH.
source "$repo_root/scripts/env/workspace.bash"
printf '[2/4] ROS Rust message discovery and typesupport\n'
python3 "$repo_root/scripts/setup/prepare-rust-ros.py"
printf '[3/4] Pinned motorbridge dependencies\n'
bash "$repo_root/scripts/setup/prepare-motorbridge.bash"
if [[ "$without_agent" == false ]]; then
  printf '[4/4] Agent Bridge SDK and Python dependencies\n'
  bash "$repo_root/scripts/setup/prepare-agent-bridge.sh"
else
  printf '[4/4] Agent Bridge skipped\n'
fi
printf '\nDependencies ready. In the build terminal, run:\n  source scripts/env/workspace.bash\nThen use the existing colcon or remote-dev build command.\n'
