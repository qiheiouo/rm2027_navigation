"""Compare only A02 consumption mathematics; never import its tracker/frontend/output."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


def main(probe):
    root = Path(__file__).resolve().parents[3]
    intake = json.loads((root/'docs/dynamic_navigation/r4_consumer_library_sources.json').read_text())
    for item in intake['algorithm_references']:
        assert hashlib.sha256((root/item['path']).read_bytes()).hexdigest() == item['sha256']
    sys.path.insert(0, str(root/'experiments/r4_hws_prediction_consumption'))
    from r4_hws.soft_field import TemporalSoftField  # noqa: E402

    source = 1_820_000_000_123_456_789
    track = SimpleNamespace(origin=(-.1, 0.), cells=((0, 0), (2, 0), (3, 0)),
                            resolution=.05, anchor=(1.05, 0.), velocity=(.5, 0.),
                            source_ns=source, observation_ns=source-100_000_000, track_id=7)
    rows = subprocess.check_output([probe], text=True).splitlines()
    assert len(rows) == 48
    for row in rows:
        yaw, stage, x, y, residual, gx, gy, clearance, plateau = map(float, row.split(','))
        class ActualBodyReference(TemporalSoftField):
            physical_half_extents = (.32, .27)
            robot_padding = .02
        snapshot = SimpleNamespace(state=(0., 0., yaw), tracks=(track,),
            stage_epochs=tuple(source+50_000_000+k*50_000_000 for k in range(31)))
        expected = ActualBodyReference(snapshot).sample((x, y), int(stage))
        for actual, reference in zip((residual, gx, gy), (expected.residual, *expected.gradient)):
            assert math.isclose(actual, reference, rel_tol=1e-10, abs_tol=1e-10), (row, expected)
        assert (math.isnan(clearance) and expected.clearance is None) or math.isclose(
            clearance, expected.clearance, rel_tol=1e-10, abs_tol=1e-10), (row, expected)
        assert bool(plateau) == expected.plateau
    print('48 C++ soft-field queries match fixed A02 math with explicit actual body parameters.')


if __name__ == '__main__':
    main(sys.argv[1])
