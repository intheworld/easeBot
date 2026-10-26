#!/usr/bin/env bash
# Prepare workspace dependencies on the machine that will build/run the project.
set -eo pipefail
without_agent=false
agent_server=false
camera_sdk=false
grasp=false
arm_motion=false
for arg in "$@"; do
  case "$arg" in
    --without-agent-bridge) without_agent=true ;;
    --agent-server) agent_server=true ;;
    --camera-sdk) camera_sdk=true ;;
    --grasp) grasp=true ;;
    --arm-motion) arm_motion=true ;;
    -h|--help)
      cat <<'HELP'
Usage: bash scripts/setup.bash [--without-agent-bridge | --agent-server | --camera-sdk | --grasp | --arm-motion]

--agent-server prepares agent/.venv with the local discovery package and Agent
LLM/MHS dependencies only; no ROS environment is required.
--camera-sdk prepares only the pinned RealSense Python SDK in .deps/.
--grasp checks the existing rebot_grasp compute environment and weights only.
--arm-motion checks only the IK/collision compute dependencies; no detector/weights required.

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
if [[ "$arm_motion" == true ]]; then
  if [[ "$agent_server" == true || "$without_agent" == true || "$camera_sdk" == true || "$grasp" == true ]]; then
    echo '--arm-motion must be used alone' >&2
    exit 2
  fi
  python3 "$repo_root/scripts/setup/prepare-grasp.py" --arm-only
  exit 0
fi
if [[ "$grasp" == true ]]; then
  if [[ "$agent_server" == true || "$without_agent" == true || "$camera_sdk" == true ]]; then
    echo '--grasp must be used alone' >&2
    exit 2
  fi
  python3 "$repo_root/scripts/setup/prepare-grasp.py"
  exit 0
fi
if [[ "$camera_sdk" == true ]]; then
  if [[ "$agent_server" == true || "$without_agent" == true ]]; then
    echo '--camera-sdk must be used alone' >&2
    exit 2
  fi
  python3 "$repo_root/scripts/setup/prepare-sensors.py"
  exit 0
fi
if [[ "$agent_server" == true ]]; then
  if [[ "$without_agent" == true ]]; then
    echo '--agent-server cannot be combined with --without-agent-bridge' >&2
    exit 2
  fi
  if [[ ! -x "$repo_root/agent/.venv/bin/python" ]]; then
    python3 -m venv "$repo_root/agent/.venv"
  fi
  env -u PYTHONPATH "$repo_root/agent/.venv/bin/python" -m pip install \
    -e "$repo_root/packages/hei-discovery" -e "$repo_root/agent[llm,mhs]"
  echo 'Server ready: agent/.venv/bin/hei-agent --backend embedded --serve'
  exit 0
fi
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
printf '[1/5] Build tools\n'
python3 "$repo_root/scripts/setup/prepare-build-tools.py"
# A child installer cannot change this process's PATH/PYTHONPATH.
source "$repo_root/scripts/env/workspace.bash"
printf '[2/5] ROS Rust message discovery and typesupport\n'
python3 "$repo_root/scripts/setup/prepare-rust-ros.py"
printf '[3/5] Pinned motorbridge dependencies\n'
bash "$repo_root/scripts/setup/prepare-motorbridge.bash"
printf '[4/5] Pinned D435 camera SDK\n'
python3 "$repo_root/scripts/setup/prepare-sensors.py"
if [[ "$without_agent" == false ]]; then
  printf '[5/5] Agent Bridge SDK and Python dependencies\n'
  bash "$repo_root/scripts/setup/prepare-agent-bridge.sh"
else
  printf '[5/5] Agent Bridge skipped\n'
fi
printf '\nDependencies ready. In the build terminal, run:\n  source scripts/env/workspace.bash\nThen use the existing colcon or remote-dev build command.\n'
