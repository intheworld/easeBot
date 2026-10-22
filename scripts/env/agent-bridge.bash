# Source after ROS setup and before starting the bridge. No system Python changes.
_hei_agent_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="${_hei_agent_root}/.deps/agnes-harness/packages/mhs/python/src:${_hei_agent_root}/.deps/agent-python${PYTHONPATH:+:${PYTHONPATH}}"
unset _hei_agent_root
