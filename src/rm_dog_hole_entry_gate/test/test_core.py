import pytest

from rm_dog_hole_entry_gate.core import (
    DogHoleEntryPauseGate,
    GateState,
    LocalizationReadinessGate,
    project_path_distance,
)
from rm_path_annotations.core import PathPose, RegionContractError, parse_region_set


SHA256 = "1" * 64


def _pose(x: float, y: float = 0.0) -> PathPose:
    return PathPose(x=x, y=y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)


def _region(region_id: str, region_type: str, start: float, end: float) -> dict:
    return {
        "id": region_id,
        "type": region_type,
        "polygon": [[start, -1.0], [end, -1.0], [end, 1.0], [start, 1.0]],
    }


def _region_set(*regions: dict):
    return parse_region_set(
        {
            "schema": "rm_semantic_regions/v1",
            "region_set_id": "connected_terrain",
            "revision": "rev1",
            "map_binding": {
                "frame_id": "map",
                "map_id": "field_map",
                "map_revision": "r1",
                "manifest_sha256": SHA256,
            },
            "regions": list(regions),
        }
    )


def _gate() -> DogHoleEntryPauseGate:
    return DogHoleEntryPauseGate(
        _region_set(
            _region("entry", "dog_hole_approach", -1.0, 0.0),
            _region("tunnel", "committed_corridor", 0.0, 2.0),
            _region("exit", "dog_hole_approach", 2.0, 3.0),
        ),
        brake_settle_sec=0.5,
        hold_sec=5.0,
        rearm_clear_sec=1.0,
    )


def test_path_projection_uses_arc_length_and_prefers_later_tie() -> None:
    path = (_pose(0.0), _pose(2.0), _pose(2.0, 2.0))
    assert project_path_distance(path, 1.0, 0.0) == pytest.approx(1.0)
    assert project_path_distance(path, 2.0, 1.0) == pytest.approx(3.0)


def test_crossing_stops_then_holds_five_seconds_before_commit() -> None:
    gate = _gate()
    gate.set_path((_pose(-2.0), _pose(4.0)), now=0.0)
    assert gate.state == GateState.ARMED
    assert not gate.must_stop(pose_fresh=True)

    gate.update_pose(-0.5, 0.0, now=1.0)
    assert gate.state == GateState.BRAKING
    assert gate.must_stop(pose_fresh=True)
    gate.tick(1.5)
    assert gate.state == GateState.HOLDING
    gate.tick(6.49)
    assert gate.state == GateState.HOLDING
    gate.tick(6.5)
    assert gate.state == GateState.RELEASED
    assert not gate.must_stop(pose_fresh=False)

    gate.update_pose(1.0, 0.0, now=7.0)
    assert gate.state == GateState.COMMITTED
    gate.update_pose(3.5, 0.0, now=8.0)
    assert gate.state == GateState.PASSED
    gate.tick(9.0)
    assert gate.state == GateState.ARMED


def test_corridor_behind_does_not_retrigger_in_exit_approach() -> None:
    gate = _gate()
    gate.set_path((_pose(-2.0), _pose(4.0)), now=0.0)
    gate.update_pose(2.5, 0.0, now=1.0)
    assert gate.state == GateState.ARMED
    assert not gate.snapshot().committed_corridor_ahead


def test_non_crossing_path_passes_without_global_pose() -> None:
    gate = _gate()
    gate.set_path((_pose(-2.0, 3.0), _pose(4.0, 3.0)), now=0.0)
    assert not gate.path_crosses_corridor
    assert not gate.must_stop(pose_fresh=False)


def test_crossing_path_is_fail_closed_until_pose_is_fresh() -> None:
    gate = _gate()
    assert gate.must_stop(pose_fresh=False)
    gate.set_path((_pose(-2.0), _pose(4.0)), now=0.0)
    assert gate.must_stop(pose_fresh=False)
    gate.update_pose(-2.0, 0.0, now=0.1)
    assert not gate.must_stop(pose_fresh=True)


def test_starting_inside_committed_corridor_is_rejected() -> None:
    gate = _gate()
    gate.set_path((_pose(1.0), _pose(4.0)), now=0.0)
    gate.update_pose(1.0, 0.0, now=0.1)
    assert gate.state == GateState.INVALID_ENTRY
    assert gate.must_stop(pose_fresh=True)


def test_invalid_entry_clears_only_after_continuous_time_outside_regions() -> None:
    gate = _gate()
    gate.set_path((_pose(1.0), _pose(4.0)), now=0.0)
    gate.update_pose(1.0, 0.0, now=0.1)
    assert gate.state == GateState.INVALID_ENTRY

    gate.update_pose(3.5, 0.0, now=0.2)
    gate.tick(1.19)
    assert gate.state == GateState.INVALID_ENTRY
    gate.update_pose(2.5, 0.0, now=1.19)
    gate.update_pose(3.5, 0.0, now=1.20)
    gate.tick(2.19)
    assert gate.state == GateState.INVALID_ENTRY
    gate.tick(2.20)
    assert gate.state == GateState.ARMED


def test_stale_pose_resets_invalid_entry_clear_confirmation() -> None:
    gate = _gate()
    gate.set_path((_pose(1.0), _pose(4.0)), now=0.0)
    gate.update_pose(1.0, 0.0, now=0.1)
    gate.update_pose(3.5, 0.0, now=0.2)
    gate.tick(0.7, pose_fresh=False)
    gate.update_pose(3.5, 0.0, now=0.8)
    gate.tick(1.79)
    assert gate.state == GateState.INVALID_ENTRY
    gate.tick(1.80)
    assert gate.state == GateState.ARMED


def test_localization_readiness_requires_continuously_valid_signal() -> None:
    readiness = LocalizationReadinessGate(stable_sec=1.0)
    assert not readiness.ready(0.0)
    readiness.update(True, 0.1)
    assert not readiness.ready(1.09)
    readiness.update(False, 1.09)
    readiness.update(True, 1.10)
    assert not readiness.ready(2.09)
    assert readiness.ready(2.10)


def test_localization_readiness_rejects_invalid_duration() -> None:
    with pytest.raises(RegionContractError, match="stable_sec"):
        LocalizationReadinessGate(stable_sec=-0.1)


def test_contract_requires_both_region_roles() -> None:
    with pytest.raises(RegionContractError, match="dog_hole_approach"):
        DogHoleEntryPauseGate(
            _region_set(_region("tunnel", "committed_corridor", 0.0, 2.0))
        )
    with pytest.raises(RegionContractError, match="committed_corridor"):
        DogHoleEntryPauseGate(
            _region_set(_region("entry", "dog_hole_approach", -1.0, 0.0))
        )
