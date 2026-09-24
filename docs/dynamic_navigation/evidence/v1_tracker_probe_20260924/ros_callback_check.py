#!/usr/bin/env python3
"""Container-only smoke test for Humble prediction and diagnostic serialization."""
import json
import tempfile
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray, DynamicObstaclePrediction
from live_nodes import PredictionRecorder


def main():
    with tempfile.TemporaryDirectory() as root:
        rclpy.init()
        node = PredictionRecorder(Path(root))
        diagnostic = DiagnosticArray()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'tracker'
        diagnostic.status = [status]
        node.on_diagnostic(diagnostic)
        prediction = DynamicObstaclePredictionArray()
        track = DynamicObstaclePrediction()
        track.track_id = 1
        track.state = DynamicObstaclePrediction.STATE_CONFIRMED
        prediction.tracks = [track]
        node.on_prediction(prediction)
        node.destroy_node()
        rclpy.shutdown()
        recorded_diag = json.loads((Path(root) / 'diagnostics.jsonl').read_text())
        recorded_prediction = json.loads((Path(root) / 'predictions.jsonl').read_text())
        assert recorded_diag['statuses'][0]['level'] == 0
        assert recorded_prediction['tracks'][0]['state'] == 2
        print('Humble callback serialization 2/2 passed')


if __name__ == '__main__':
    main()
