"""Real serial framing against PTY motors; no physical hardware is opened."""
import csv
import errno
import os
from pathlib import Path
import select
import struct
import subprocess
import threading

import pytest

EXE = Path(__file__).resolve().parents[1]/'src/hei_hardware/motorbridge_backend/target/debug/examples/align_arm_gains'


@pytest.mark.parametrize('case', ['normal','enabled','wrong_mode','mismatch'])
def test_gain_tool_verified_writes_and_preconditions(tmp_path, case):
    assert EXE.is_file(), 'Build cargo --example align_arm_gains before running this test'
    master, slave = os.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    os.set_blocking(master,False)
    frames, errors, registers = [], [], {}
    stopped = threading.Event()
    def send(id, payload):
        os.write(master,b'\xaa\x11\x08'+struct.pack('<I',0x10+id)+payload+b'\x55')
    def serve():
        buffer=bytearray()
        try:
            while not stopped.is_set():
                if not select.select([master],[],[],.01)[0]:
                    continue
                try:
                    buffer.extend(os.read(master,8192))
                except OSError as exc:
                    if exc.errno==errno.EIO:
                        stopped.wait(.001)
                        continue
                    raise
                while len(buffer)>=30:
                    packet=bytes(buffer[:30]); del buffer[:30]
                    assert packet[:3]==b'\x55\xaa\x1e'
                    assert struct.unpack('<I',packet[13:17])[0]==0x7ff
                    data=packet[21:29];id=int.from_bytes(data[:2],'little');op=data[2];rid=data[3]
                    frames.append((id,op,rid))
                    assert 1<=id<=6
                    if op==0xcc:
                        send(id,bytes([id|(0x10 if case=='enabled' else 0)])+b'\0'*7)
                    elif op==0x33:
                        value=struct.pack('<I',1 if case=='wrong_mode' else 2) if rid==10 else registers.get((id,rid),struct.pack('<f',.1))
                        send(id,data[:4]+value)
                    else:
                        assert op==0x55 and 25<=rid<=28
                        if case!='mismatch': registers[id,rid]=data[4:]
                        send(id,data)
        except BaseException as exc:
            errors.append(exc)
    worker=threading.Thread(target=serve)
    worker.start()
    try:
        backup=tmp_path/'backup.csv'
        run=subprocess.run([str(EXE),port,str(backup),'--apply-seeedsdk-profile'],
                           capture_output=True,text=True,timeout=8)
        assert not errors, errors
        writes=[f for f in frames if f[1]==0x55]
        if case=='normal':
            assert run.returncode==0,run.stderr
            rows=list(csv.DictReader(run.stdout.splitlines()))
            assert len(rows)==24 and all(r['status']=='verified' for r in rows)
            assert len(writes)==24
            assert len(list(csv.DictReader(backup.open())))==24
        else:
            assert run.returncode!=0
            assert len(writes)==(1 if case=='mismatch' else 0)
    finally:
        stopped.set();worker.join(timeout=1);os.close(master)
