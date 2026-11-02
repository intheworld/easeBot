"""Isolated IK/collision compute; does not import a detector or ROS/hardware drivers."""
import json
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from hei_agent_bridge.arm.planner import compute


def run(directory):
    try:
        request=json.loads((directory/'request.json').read_text())
        if request.get('operation')=='warmup':
            from hei_agent_bridge.compute_warmup import warmup
            result=warmup(request)
        else:
            result=compute(request)
    except Exception as exc:
        result = {'error':f'{type(exc).__name__}: {exc}'}
    temporary = directory/'result.part'
    temporary.write_text(json.dumps(result,allow_nan=False),encoding='utf-8')
    temporary.replace(directory/'result.json')
    return 1 if 'error' in result else 0


def main():
    from hei_agent_bridge.compute_server import entry
    return entry(run)


if __name__ == '__main__':
    raise SystemExit(main())
