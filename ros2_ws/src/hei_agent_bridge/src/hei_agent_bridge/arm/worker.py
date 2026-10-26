"""Isolated IK/collision compute; does not import a detector or ROS/hardware drivers."""
import json
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from hei_agent_bridge.arm.planner import compute


def main():
    directory = Path(sys.argv[1])
    try:
        result = compute(json.loads((directory/'request.json').read_text()))
    except Exception as exc:
        result = {'error':f'{type(exc).__name__}: {exc}'}
    temporary = directory/'result.part'
    temporary.write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
    temporary.replace(directory/'result.json')
    return 1 if 'error' in result else 0


if __name__ == '__main__':
    raise SystemExit(main())
