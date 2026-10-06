"""Bounded private evidence files; never a ROS perception/control interface."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time


def encoded_range(value: float):
    if math.isnan(value):
        return 'NaN'
    if math.isinf(value):
        return 'Infinity' if value > 0 else '-Infinity'
    return float(value)


class SurfaceMemberEvidence:
    SCHEMA = 'rm_observed_surface_members/private_v1'
    MAX_SCAN_BEAMS = 4096
    MAX_RECORDS = 2000
    MAX_RECORD_BYTES = 4 * 1024 * 1024

    def __init__(self, directory: str):
        self.root = Path(directory)
        if not self.root.is_absolute():
            raise ValueError('surface evidence directory must be absolute')
        self.root.mkdir(parents=True, exist_ok=False)
        self.records = 0
        self.failed_reason = ''
        self.closed = False
        self.write_timings = []

    @property
    def complete(self):
        return not self.failed_reason

    def fail(self, reason: str):
        if self.failed_reason:
            return
        self.failed_reason = reason
        try:
            with (self.root / 'incomplete.json').open('x') as stream:
                json.dump({'schema': self.SCHEMA, 'complete': False,
                           'records_written': self.records, 'reason': reason}, stream)
                stream.write('\n')
        except OSError:
            # Caller also reports this state; an unwritable sink cannot claim evidence.
            pass

    def write(self, record: dict):
        if self.closed:
            self.fail('write attempted after evidence close')
            return False
        if not self.complete:
            return False
        if self.records >= self.MAX_RECORDS:
            self.fail('scan event budget exceeded')
            return False
        payload = dict(record, schema=self.SCHEMA, ordinal=self.records)
        started = time.perf_counter()
        try:
            data = (json.dumps(payload, separators=(',', ':'), allow_nan=False) + '\n').encode()
            if len(data) > self.MAX_RECORD_BYTES:
                raise ValueError('complete record byte budget exceeded')
            prefix = 'map' if record.get('status') == 'map' else 'scan'
            with (self.root / f'{prefix}_{self.records:06d}.json').open('xb') as stream:
                stream.write(data)
        except (OSError, ValueError, TypeError) as error:
            self.fail(str(error))
            return False
        self.records += 1
        self.write_timings.append({'ordinal': self.records - 1,
                                   'write_elapsed_seconds': time.perf_counter() - started})
        return True

    def close(self):
        if self.closed:
            return
        self.closed = True
        summary = {'schema': self.SCHEMA, 'complete': self.complete,
                   'records_written': self.records, 'reason': self.failed_reason,
                   'write_timings': self.write_timings,
                   'timing_scope': 'Serialization and file IO only; not full callback or physical response delay'}
        try:
            with (self.root / 'summary.json').open('x') as stream:
                json.dump(summary, stream, allow_nan=False)
                stream.write('\n')
        except (OSError, ValueError) as error:
            self.fail('evidence close failed: ' + str(error))
