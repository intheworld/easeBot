"""Per-run JSONL trace, written at node boundaries and flushed immediately."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from .graph import json_data


class Trace:
    def __init__(self, run_id, directory=None):
        self.run_id = run_id
        self.events = []
        self.started = time.monotonic()
        self.path = None
        self._file = None
        if directory is not None:
            directory = Path(directory)
            directory.mkdir(parents=True, exist_ok=True)
            self.path = directory / f'{run_id}.jsonl'
            self._file = self.path.open('x', encoding='utf-8')

    def record(self, event, **data):
        entry = json_data({
            'sequence': len(self.events), 'event': event, 'run_id': self.run_id,
            'time': datetime.now(timezone.utc).isoformat(),
            'elapsed_sec': time.monotonic() - self.started, **data,
        })
        if self._file is not None:
            self._file.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
            self._file.flush()
        self.events.append(entry)

    def close(self):
        if self._file is not None:
            self._file.close()
