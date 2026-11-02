#!/usr/bin/env bash
# Isolated cuRobo v0.7.8 build; reuse the site's CUDA-enabled Torch, never replace it.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
base_python="${GRASP_PYTHON:-$HOME/miniconda3/envs/rebotarm/bin/python}"
venv="$root/.deps/gpu-planner-venv"
source_dir="$root/.deps/curobo-v0.7.8"
"$base_python" -c 'import torch; assert torch.cuda.is_available(), "CUDA-enabled Torch required"; print(torch.__version__, torch.version.cuda)'
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
export PATH="$venv/bin:$HOME/.local/bin:$CUDA_HOME/bin:$PATH"
command -v nvcc >/dev/null
if [[ ! -x "$venv/bin/python" ]]; then
  "$base_python" -m venv --system-site-packages "$venv"
fi
if [[ ! -d "$source_dir" ]]; then
  git clone --depth 1 --branch v0.7.8 https://github.com/NVlabs/curobo.git "$source_dir"
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" == d64c4b005459db10c5dd867d8b30a87d5bda9bdb ]]
"$venv/bin/python" -m pip install --no-deps \
  python-fcl==0.7.0.11 numpy==1.26.4 yourdfpy==0.0.58 warp-lang==1.5.1 numpy-quaternion==2023.0.4 lxml==5.3.0 trimesh==4.5.3
export MAX_JOBS="${MAX_JOBS:-2}"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.7}"
"$venv/bin/python" -m pip install --no-build-isolation --no-deps "$source_dir"
"$venv/bin/python" -c 'import cv2, numpy, torch, fcl; from curobo.wrap.reacher.ik_solver import IKSolver; print("cuRobo/OpenCV import passed")'
"$venv/bin/python" - "$root" "$venv/bin/python" <<'PYCONFIG'
import json, pathlib, shutil, sys
from datetime import datetime, timezone
root=pathlib.Path(sys.argv[1])
backup=root/'log/gpu-setup'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
for name in ('arm_motion.json','right_grasp.json'):
    path=root/'ros2_ws/src/hei_agent_bridge/config'/name
    data=json.loads(path.read_text())
    backup.mkdir(parents=True,exist_ok=True)
    shutil.copy2(path,backup/name)
    data.update(worker_python=sys.argv[2],ik_backend='auto')
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,indent=2)+'\n')
    temporary.replace(path)
print('Source profiles updated; rebuild hei_agent_bridge to install them. Backup:',backup)
PYCONFIG
printf 'GPU worker Python: %s\nNo services restarted or robot commands sent.\n' "$venv/bin/python"
