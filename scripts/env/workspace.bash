# Source after the ROS underlay and before the application's install/setup.bash.
# Loading dependencies does not install packages or start ROS nodes.
# Prefer native distribution tools over pip/Conda Python launchers. In isolated
# Python environments those launchers may exist on PATH but fail to import cmake.
# colcon-cmake supports these explicit overrides; caller choices remain intact.
if [[ -z "${CMAKE_COMMAND:-}" && -x /usr/bin/cmake ]]; then
  export CMAKE_COMMAND=/usr/bin/cmake
fi
if [[ -z "${CTEST_COMMAND:-}" && -x /usr/bin/ctest ]]; then
  export CTEST_COMMAND=/usr/bin/ctest
fi
_hei_workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -d "${_hei_workspace_root}/.deps/sensors-python" ]]; then
  export PYTHONPATH="${_hei_workspace_root}/.deps/sensors-python${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -f "$HOME/.cargo/env" ]]; then
  source "$HOME/.cargo/env"
fi
if [[ -d "${_hei_workspace_root}/.deps/build-python" ]]; then
  export PYTHONPATH="${_hei_workspace_root}/.deps/build-python${PYTHONPATH:+:${PYTHONPATH}}"
  export PATH="${_hei_workspace_root}/.deps/build-python/bin:$PATH"
fi
source "${_hei_workspace_root}/scripts/env/ros-rust.bash"
source "${_hei_workspace_root}/scripts/env/agent-bridge.bash"
unset _hei_workspace_root
