#!/usr/bin/env python3
"""Check an existing grasp compute environment, without touching hardware or downloading weights."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', dest='python_path', default='~/miniconda3/envs/rebotarm/bin/python')
    parser.add_argument('--model', default='~/gitdir/rebot_grasp/models/yoloe-26s-seg.pt')
    parser.add_argument('--arm-only', action='store_true', help='Check IK/collision dependencies without detector or weights')
    args = parser.parse_args()
    python = Path(args.python_path).expanduser().resolve(strict=True)
    model = None if args.arm_only else Path(args.model).expanduser().resolve(strict=True)
    modules = 'numpy, scipy, cv2, mujoco' + ('' if args.arm_only else ', ultralytics')
    subprocess.run([str(python), '-c',
        'import json, ' + modules + '; '
        'print(json.dumps({m.__name__: m.__version__ for m in (' + modules + ')}))'],
        check=True, env={k:v for k,v in __import__('os').environ.items() if k not in ('PYTHONPATH', 'PYTHONHOME', 'PYTHONNOUSERSITE')})
    print(json.dumps({'worker_python': str(python),
                      **({'model_path':str(model), 'model_bytes':model.stat().st_size} if model else {})}))
    print(('Arm motion' if args.arm_only else 'Grasp') + ' dependencies ready; existing environment reused, no hardware opened.')


if __name__ == '__main__':
    main()
