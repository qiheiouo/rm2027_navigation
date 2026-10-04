from dataclasses import replace
import json
import pytest
from run_experiment import simulate
from verify_evidence import verify
from temporal_mpc.fixtures import SCENARIOS
from temporal_mpc.solver import Config


def fixture(tmp_path):
    scenario = replace(SCENARIOS[1], duration=.1)
    report = simulate(scenario, "observed_polygon", "temporal", Config(), tmp_path)
    (tmp_path / "summary.json").write_text(json.dumps({"reports": [report]}))
    (tmp_path / "manifest.json").write_text(json.dumps({"arguments": {
        "scenarios": [scenario.name], "modes": ["observed_polygon"], "consumptions": ["temporal"], "horizons": [1.5]}}))
    return next(tmp_path.glob("*.jsonl"))


def test_independent_execution_replay_and_recorded_metric_agreement(tmp_path):
    fixture(tmp_path)
    assert verify(tmp_path)["verdict"] == "EXECUTED SWEEP AND METRICS REPLAY PASS"


def test_corrupted_held_control_is_detected_by_independent_replay(tmp_path):
    path = fixture(tmp_path)
    row = json.loads(path.read_text())
    row["acceleration"][0] += .01
    path.write_text(json.dumps(row) + "\n")
    with pytest.raises(AssertionError):
        verify(tmp_path)
