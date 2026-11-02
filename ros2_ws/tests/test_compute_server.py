import asyncio
import json
import os
from pathlib import Path
import sys
import pytest
from hei_agent_bridge.motion import MotionRuntime, MotionComputeError


def runtime(tmp_path):
    script = tmp_path/'worker.py'
    package = Path(__file__).parents[1]/'src/hei_agent_bridge/src'
    script.write_text(f'''import sys, json, os, time
sys.path.insert(0, {str(package)!r})
from hei_agent_bridge.compute_server import entry
count = 0
def run(directory):
    global count
    count += 1
    request = json.loads((directory/'request.json').read_text())
    print('native diagnostic', flush=True)
    os.write(1, b'native stdout\\n')
    (directory/'started').write_text(str(os.getpid()))
    time.sleep(request.get('sleep', 0))
    result = {{'count':count, 'value':request.get('value')}}
    if request.get('fail'): result = {{'error':'expected failure'}}
    (directory/'result.json').write_text(json.dumps(result))
    return int('error' in result)
raise SystemExit(entry(run))
''')
    r = MotionRuntime.__new__(MotionRuntime)
    r.profile = {'worker_python':sys.executable}
    r.worker_script = str(script)
    r.worker_cwd = str(tmp_path)
    r._compute_lock = asyncio.Lock()
    r._compute_process = None
    return r


def request(tmp_path, name, **values):
    directory = tmp_path/name
    directory.mkdir()
    (directory/'request.json').write_text(json.dumps(values))
    return directory


def test_reuse_error_and_fresh_results(tmp_path):
    async def run():
        r = runtime(tmp_path)
        try:
            a = request(tmp_path, 'a', value='cube')
            assert (await r._worker(a))['count'] == 1
            pid = r._compute_process.pid
            b = request(tmp_path, 'b', fail=True)
            with pytest.raises(MotionComputeError): await r._worker(b)
            c = request(tmp_path, 'c', value='tray')
            assert await r._worker(c) == {'count':3, 'value':'tray'}
            assert r._compute_process.pid == pid
            assert 'native stdout' in (c/'worker.log').read_text()
        finally: await r.close_worker()
    asyncio.run(run())


def test_cancel_reaps_and_next_request_restarts(tmp_path):
    async def run():
        r = runtime(tmp_path)
        a = request(tmp_path, 'a', sleep=30)
        task = asyncio.create_task(r._worker(a))
        try:
            async def started():
                while not (a/'started').exists(): await asyncio.sleep(.01)
            await asyncio.wait_for(started(), 5)
            pid = r._compute_process.pid
            task.cancel()
            with pytest.raises(asyncio.CancelledError): await task
            with pytest.raises(ProcessLookupError): os.kill(pid, 0)
            assert not (a/'result.json').exists()
            assert (await r._worker(request(tmp_path, 'b')))['count'] == 1
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            await r.close_worker()
    asyncio.run(run())


def test_detector_cache_reloads_changed_weights(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.grasp import worker
    calls = []
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=lambda path: calls.append(path) or object()))
    monkeypatch.setattr(worker, '_detector_key', None)
    monkeypatch.setattr(worker, '_detector', None)
    model = tmp_path/'weights.pt'; model.write_bytes(b'a')
    first = worker.cached_detector(model)
    assert worker.cached_detector(model) is first
    model.write_bytes(b'changed')
    assert worker.cached_detector(model) is not first
    assert len(calls) == 2


def test_serial_requests_and_dead_worker_recovery(tmp_path):
    async def run():
        r = runtime(tmp_path)
        try:
            a=request(tmp_path,'a',sleep=.05,value='a')
            b=request(tmp_path,'b',value='b')
            results=await asyncio.gather(r._worker(a),r._worker(b))
            assert results == [{'count':1,'value':'a'},{'count':2,'value':'b'}]
            r._compute_process.kill()
            await r._compute_process.wait()
            assert (await r._worker(request(tmp_path,'c')))['count'] == 1
        finally: await r.close_worker()
    asyncio.run(run())


def test_idle_timeout_override_and_parent_lifetime_default(tmp_path):
    async def run():
        r=runtime(tmp_path);r.profile['worker_idle_timeout_sec']=.05
        try:
            await r._worker(request(tmp_path,'a'))
            old=r._compute_process
            await asyncio.wait_for(old.wait(),3)
            assert await r._worker(request(tmp_path,'b'))=={'count':1,'value':None}
            assert r._compute_process.pid!=old.pid
        finally:await r.close_worker()
        r=runtime(tmp_path)
        try:
            await r._worker(request(tmp_path,'c'))
            pid=r._compute_process.pid
            await asyncio.sleep(.12)
            assert (await r._worker(request(tmp_path,'d')))['count']==2
            assert r._compute_process.pid==pid
        finally:await r.close_worker()
    asyncio.run(run())


def test_warmup_queues_on_same_process_and_never_reads_hardware(tmp_path):
    async def run():
        r=runtime(tmp_path);r.directory=tmp_path/'logs';r.urdf='not-read-by-test-worker'
        # The protocol fixture recognizes warmup separately from a regular plan.
        script=Path(r.worker_script)
        text=script.read_text().replace("result = {'count':count, 'value':request.get('value')}",
            "result = ({'operation':'warmup','status':'ready'} if request.get('operation')=='warmup' else {'count':count,'value':request.get('value')})")
        script.write_text(text)
        try:
            status=await r.warmup()
            assert status['status']=='ready'
            assert status['pid']==r._compute_process.pid
            assert not hasattr(r,'backend')  # No state/feedback or action client needed.
            assert (await r._worker(request(tmp_path,'after')))['count']==2
            assert status['pid']==r._compute_process.pid
        finally:await r.close_worker()
    asyncio.run(run())


def test_cancel_background_warmup_reaps_its_child(tmp_path):
    async def run():
        r=runtime(tmp_path);r.directory=tmp_path/'logs';r.urdf='unused'
        script=Path(r.worker_script);script.write_text(script.read_text().replace("time.sleep(request.get('sleep', 0))",'time.sleep(30)'))
        task=asyncio.create_task(r.warmup())
        try:
            async def started():
                while not list(r.directory.glob('*/started')):await asyncio.sleep(.01)
            await asyncio.wait_for(started(),5)
            pid=r._compute_process.pid;task.cancel()
            with pytest.raises(asyncio.CancelledError):await task
            with pytest.raises(ProcessLookupError):os.kill(pid,0)
            assert r.warmup_status['status']=='cancelled'
        finally:
            task.cancel();await asyncio.gather(task,return_exceptions=True);await r.close_worker()
    asyncio.run(run())


def test_keepalive_worker_exits_when_parent_pipe_closes(tmp_path):
    async def run():
        r=runtime(tmp_path)
        try:
            await r._worker(request(tmp_path,'a'))
            process=r._compute_process
            process.stdin.close();await process.stdin.wait_closed()
            await asyncio.wait_for(process.wait(),3)
            assert process.returncode==0
        finally:await r.close_worker()
    asyncio.run(run())


def test_warmup_does_not_create_plan_or_contact_robot(monkeypatch):
    import builtins
    from types import SimpleNamespace
    from hei_agent_bridge.compute_warmup import warmup
    from hei_agent_bridge.grasp import planner_backend
    from hei_agent_bridge.collision import fcl_scene
    imports=builtins.__import__;calls=[]
    def guarded(name,*args,**kwargs):
        if name.split('.')[0] in ('rclpy','serial','mujoco'):
            raise AssertionError('Hardware/simulator import in warmup: '+name)
        return imports(name,*args,**kwargs)
    import numpy as np
    planner=SimpleNamespace(lower=np.full(6,-3.),upper=np.full(6,3.),fk=lambda q:np.eye(4),
                            solve=lambda *args:calls.append('ik'))
    monkeypatch.setattr(planner_backend,'get_planner',lambda *args:planner)
    monkeypatch.setattr(fcl_scene,'robot_model',lambda *args:calls.append('mesh'))
    monkeypatch.setattr(builtins,'__import__',guarded)
    result=warmup({'urdf':'unused','profile':{'ik_backend':'scipy'}})
    assert calls==['ik','mesh']
    assert result['status']=='ready' and result['synthetic'] and not result['motion_sent']
    assert 'plan' not in result and 'validation' not in result
