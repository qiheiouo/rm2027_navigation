import copy
from types import SimpleNamespace as NS
import numpy as np
import pytest

from temporal_mpc.contracts import (ContractError, Geometry, PublicAdapter, Snapshot,
                                    Track, V1, V2, predict, polygon)


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def stamp(sec=1, nanosec=0):
    return NS(sec=sec, nanosec=nanosec)


def message():
    t = NS(track_id=1, state=2, position=vector(2, 0), velocity=vector(.5, 0),
           size=vector(.45, .55), last_observation_stamp=stamp(),
           prediction=[vector(2 + .05 * k, 0) for k in range(1, 16)])
    return NS(header=NS(frame_id="map", stamp=stamp()), schema=V2, authority="shadow_only",
              complete=True, total_track_count=1, prediction_dt=.1, prediction_steps=15, tracks=[t])


def test_absolute_source_age_and_zero_index():
    snapshot = PublicAdapter().consume(message(), 1_200_000_000)
    result = predict(snapshot, 1_200_000_000, [0., .1, 1.5])
    np.testing.assert_allclose(result.centers[0][:, 0], [2.1, 2.15, 2.85])
    assert result.relative_times[0] == 0


def test_coasting_anchor_is_not_advanced_twice():
    msg = message()
    msg.header.stamp = stamp(1, 100_000_000)
    msg.tracks[0].state = 3
    msg.tracks[0].position.x = 2.05  # Already at the array epoch.
    snapshot = PublicAdapter().consume(msg, 1_200_000_000)
    assert predict(snapshot, 1_200_000_000, [0., .1]).centers[0][0, 0] == pytest.approx(2.1)


@pytest.mark.parametrize("field,value", [
    ("schema", "unknown"), ("authority", "controller"), ("complete", False),
    ("complete", 1), ("total_track_count", 2), ("prediction_dt", .2),
    ("prediction_dt", float("nan")), ("prediction_steps", 14),
])
def test_array_contract_rejection_replaces_usable_cache(field, value):
    adapter, msg = PublicAdapter(), message()
    adapter.consume(msg, 1_000_000_000)
    bad = copy.deepcopy(msg)
    bad.header.stamp = stamp(1, 100_000_000)
    setattr(bad, field, value)
    with pytest.raises(ContractError):
        adapter.consume(bad, 1_100_000_000)
    assert adapter.usable is None


@pytest.mark.parametrize("mutate", [
    lambda m: setattr(m.header, "frame_id", "base_link"),
    lambda m: setattr(m.header.stamp, "nanosec", 1_000_000_000),
    lambda m: setattr(m.tracks[0].velocity, "x", float("nan")),
    lambda m: setattr(m.tracks[0].velocity, "x", 3.01),
    lambda m: setattr(m.tracks[0].velocity, "z", .1),
    lambda m: setattr(m.tracks[0].size, "x", 0.),
    lambda m: setattr(m.tracks[0].size, "y", 3.01),
    lambda m: setattr(m.tracks[0], "state", 4),
    lambda m: setattr(m.tracks[0], "track_id", -1),
    lambda m: setattr(m.tracks[0].last_observation_stamp, "nanosec", 1),
    lambda m: setattr(m.tracks[0], "prediction", []),
    lambda m: setattr(m.tracks[0].prediction[0], "x", float("inf")),
])
def test_track_contract_rejections(mutate):
    msg = message()
    mutate(msg)
    with pytest.raises(ContractError):
        PublicAdapter().consume(msg, 1_000_000_000)


def test_duplicate_stale_future_jump_and_backwards_source():
    msg = message()
    msg.tracks *= 2
    msg.total_track_count = 2
    with pytest.raises(ContractError):
        PublicAdapter().consume(msg, 1_000_000_000)
    for evaluation in (999_999_999, 1_400_000_001):
        with pytest.raises(ContractError):
            PublicAdapter().consume(message(), evaluation)
    adapter = PublicAdapter()
    adapter.consume(message(), 1_000_000_000)
    with pytest.raises(ContractError):
        adapter.consume(message(), 1_000_000_000)
    msg = message()
    msg.header.stamp = msg.tracks[0].last_observation_stamp = stamp(1, 100_000_000)
    msg.tracks[0].position.x = 4
    with pytest.raises(ContractError):
        adapter.consume(msg, 1_100_000_000)


def test_nominal_diameter_uses_full_D_and_observed_shape_cannot_be_invented():
    snapshot = PublicAdapter(geometry_mode="nominal_diameter").consume(message(), 1_000_000_000)
    assert snapshot.tracks[0].geometry.radius == pytest.approx(1.6970562748477143)
    with pytest.raises(ContractError):
        PublicAdapter(geometry_mode="observed_polygon")


def test_tentative_is_current_only_and_empty_does_not_bypass_freshness():
    msg = message()
    msg.tracks[0].state = 1
    snapshot = PublicAdapter().consume(msg, 1_000_000_000)
    result = predict(snapshot, 1_000_000_000, [0., .1, 1.5])
    np.testing.assert_allclose(result.centers[0], [[2, 0]] * 3)
    msg.tracks, msg.total_track_count = [], 0
    snapshot = PublicAdapter().consume(msg, 1_000_000_000)
    assert not snapshot.tracks
    with pytest.raises(ContractError):
        predict(snapshot, 1_500_000_000, [0., .1])


@pytest.mark.parametrize("points", [[], [(0, 0), (1, 1)],
    [(0, 0), (1, 0), (.1, .1), (1, 1), (0, 1)],
    [(0, 0), (1, 1), (0, 1), (1, 0)], [(0, 0), (0, 0), (1, 1)]])
def test_polygon_rejects_nonconvex_or_degenerate_geometry(points):
    with pytest.raises(ContractError):
        polygon(points)


def test_geometry_is_immutable_and_lost_is_unknown():
    shape = Geometry("polygon", ((-.2, -.3), (.2, -.3), (.2, .3), (-.2, .3)), source="fixture")
    snapshot = Snapshot(0, (Track(1, (2, 0), (0, 0), 0, "lost", shape),))
    with pytest.raises(ContractError):
        predict(snapshot, 0, [0., .1])
