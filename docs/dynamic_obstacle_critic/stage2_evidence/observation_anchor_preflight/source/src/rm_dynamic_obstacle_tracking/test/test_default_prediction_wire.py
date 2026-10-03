"""CDR canonicalization cannot conceal mutations of a semantic field."""
import importlib.util
from pathlib import Path

import pytest
pytest.importorskip('rclpy')
pytest.importorskip('rm_competition_interfaces.msg')
from rclpy.serialization import serialize_message
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array
from rm_competition_interfaces.msg import DynamicObstaclePrediction as Track

path = Path(__file__).resolve().parents[1]/'tools/audit_default_prediction_wire.py'
spec = importlib.util.spec_from_file_location('wire_canonicalization_test', path)
wire = importlib.util.module_from_spec(spec); spec.loader.exec_module(wire)


def message():
    result = Array(); result.schema = Array.SCHEMA
    result.authority = Array.AUTHORITY_SHADOW_ONLY; result.header.frame_id = 'map'
    result.header.stamp.sec = 1700000000; result.header.stamp.nanosec = 123456789
    result.processing_stamp = result.header.stamp
    result.prediction_dt = .1; result.prediction_steps = 30
    result.complete = True; result.total_track_count = 1
    track = Track(); track.track_id = 7; track.state = Track.STATE_CONFIRMED
    track.position.x = 1.25; track.position.y = -0.0; track.velocity.y = -.5
    track.size.x = .2; track.size.y = .3; track.last_observation_stamp = result.header.stamp
    result.tracks = [track]
    return result


def test_only_structural_padding_can_change():
    value = message(); payload = serialize_message(value)
    canonical, padding = wire.canonical_cdr(value, payload)
    assert padding
    changed = bytearray(payload)
    for offset in padding: changed[offset] ^= 0xff
    assert wire.canonical_cdr(value, bytes(changed))[0] == canonical
    for offset in set(range(len(payload)))-set(padding):
        changed = bytearray(payload); changed[offset] ^= 1
        with pytest.raises(ValueError):
            wire.canonical_cdr(value, bytes(changed))


def test_truncation_trailing_and_signed_zero_are_semantic():
    value = message(); payload = serialize_message(value)
    for changed in (payload[:-1], payload+b'\x00'):
        with pytest.raises(ValueError): wire.canonical_cdr(value, changed)
    value.tracks[0].position.y = 0.0
    with pytest.raises(ValueError): wire.canonical_cdr(value, payload)
