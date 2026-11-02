"""Serial, file-backed compute protocol. No robot interfaces are imported here."""
import json
import math
import os
from pathlib import Path
import select
import sys
import time


def serve(run):
    # Keep protocol output separate from Python and native library diagnostics.
    protocol = os.fdopen(os.dup(1), 'w', buffering=1)
    os.dup2(2, 1)
    timeout=float(os.environ.get('HEI_COMPUTE_IDLE_TIMEOUT_SEC','0'))
    if not math.isfinite(timeout) or timeout<0:
        raise ValueError('Compute idle timeout must be finite and nonnegative')
    while select.select([sys.stdin], [], [], timeout or None)[0]:
        line = sys.stdin.readline()
        if not line:  # Parent exited or explicitly closed the worker.
            break
        message = json.loads(line)
        directory = Path(message['directory'])
        started = time.monotonic()
        with (directory/'worker.log').open('ab', buffering=0) as log:
            saved = [os.dup(fd) for fd in (1, 2)]
            try:
                sys.stdout.flush(); sys.stderr.flush()
                os.dup2(log.fileno(), 1); os.dup2(log.fileno(), 2)
                status = run(directory)
            finally:
                sys.stdout.flush(); sys.stderr.flush()
                for fd, backup in zip((1, 2), saved):
                    os.dup2(backup, fd); os.close(backup)
        protocol.write(json.dumps({'id':message['id'], 'status':status,
                                   'pid':os.getpid(), 'compute_sec':time.monotonic()-started})+'\n')
    protocol.close()


def entry(run):
    if sys.argv[1:] == ['--serve']:
        serve(run)
        return 0
    return run(Path(sys.argv[1]))
