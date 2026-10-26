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
prepare_checkout() {
  local checkout=$1
  if ! git -C "$checkout" rev-parse --verify HEAD >/dev/null 2>&1 ||
     [[ "$(git -C "$checkout" rev-parse HEAD)" != "$revision" ]]; then
    git -C "$checkout" fetch --depth 1 origin "$revision"
  fi
  git -C "$checkout" sparse-checkout init --cone
  git -C "$checkout" sparse-checkout set packages/mhs/python packages/mhs/schema
  git -C "$checkout" checkout --detach "$revision"
}
if [[ -e "${sdk}/.git" ]]; then
  prepare_checkout "$sdk"
else
  # Older deployments copied a source archive without Git metadata. Fetch first
  # so a network failure never removes the currently usable SDK.
  stage="$(mktemp -d "${repo}/.deps/agnes-harness.prepare.XXXXXX")"
  trap 'rm -rf -- "$stage"' EXIT
  git init "${stage}/source"
  git -C "${stage}/source" remote add origin https://github.com/AgnesAI-Labs/agnes-harness.git
  prepare_checkout "${stage}/source"
  if [[ -e "$sdk" ]]; then
    backup="$(mktemp -d "${repo}/.deps/agnes-harness-before-setup.XXXXXX")"
    mv -- "$sdk" "${backup}/source"
    echo "Previous SDK preserved in ${backup}/source"
  fi
  mv -- "${stage}/source" "$sdk"
fi
# env/agent-bridge.bash loads SDK source directly, preserving its schema path.
if command -v uv >/dev/null; then
  UV_CACHE_DIR="${repo}/.deps/uv-cache" uv pip install --python /usr/bin/python3 \
    --target "${repo}/.deps/agent-python" \
    --reinstall-package hei-discovery \
    --requirements "${repo}/ros2_ws/src/hei_agent_bridge/requirements-mhs.lock" \
    "${repo}/packages/hei-discovery"
else
  /usr/bin/python3 -m pip install --upgrade --target "${repo}/.deps/agent-python" \
    --requirement "${repo}/ros2_ws/src/hei_agent_bridge/requirements-mhs.lock" \
    "${repo}/packages/hei-discovery"
fi
echo 'Ready: source scripts/env/agent-bridge.bash before running hei_agent_bridge.'
