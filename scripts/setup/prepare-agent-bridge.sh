#!/usr/bin/env bash
set -euo pipefail
if (( $# )); then
  if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
    echo 'Usage: bash scripts/setup/prepare-agent-bridge.sh'
    echo 'Prepare the pinned Agnes SDK and Python dependencies in .deps/.'
    exit 0
  fi
  echo 'This script accepts only --help; see scripts/README.md.' >&2
  exit 2
fi
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
revision="$(cat "${repo}/ros2_ws/src/hei_agent_bridge/sdk-revision.txt")"
sdk="${repo}/.deps/agnes-harness"
mkdir -p "${repo}/.deps"
if [[ ! -d "${sdk}/.git" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/AgnesAI-Labs/agnes-harness.git "$sdk"
fi
if [[ "$(git -C "$sdk" rev-parse HEAD)" != "$revision" ]]; then
  git -C "$sdk" fetch --depth 1 origin "$revision"
fi
git -C "$sdk" sparse-checkout init --cone
git -C "$sdk" sparse-checkout set packages/mhs/python packages/mhs/schema
git -C "$sdk" checkout --detach "$revision"
# env/agent-bridge.bash loads SDK source directly, preserving its schema path.
if command -v uv >/dev/null; then
  UV_CACHE_DIR="${repo}/.deps/uv-cache" uv pip install --python /usr/bin/python3 \
    --target "${repo}/.deps/agent-python" \
    --requirements "${repo}/ros2_ws/src/hei_agent_bridge/requirements-mhs.lock"
else
  /usr/bin/python3 -m pip install --target "${repo}/.deps/agent-python" \
    --requirement "${repo}/ros2_ws/src/hei_agent_bridge/requirements-mhs.lock"
fi
echo 'Ready: source scripts/env/agent-bridge.bash before running hei_agent_bridge.'
