from copy import deepcopy
import math
from pathlib import Path

import pytest

import rm_path_annotations.core as core
from rm_path_annotations.core import (
    AdmissionPolicy,
    PathPose,
    RegionContractError,
    RegionType,
    TraversalPolicy,
    annotate_path,
    compute_path_revision,
    load_region_set,
    parse_region_set,
    validate_map_binding,
)
from rm_path_annotations.validate_regions import main as validate_regions_main


SHA256 = "1" * 64


def _document(regions: list[dict] | None = None) -> dict:
    return {
        "schema": "rm_semantic_regions/v1",
        "region_set_id": "field_a",
        "revision": "rev1",
        "map_binding": {
            "frame_id": "map",
            "map_id": "field_map",
            "map_revision": "r1",
            "manifest_sha256": SHA256,
        },
        "regions": [] if regions is None else regions,
    }


def _square(
    region_id: str,
    region_type: str,
    start_x: float,
    end_x: float,
    **values: object,
) -> dict:
    return {
        "id": region_id,
        "type": region_type,
        "polygon": [
            [start_x, -1.0],
            [end_x, -1.0],
            [end_x, 1.0],
            [start_x, 1.0],
        ],
        **values,
    }


def _pose(x: float, y: float = 0.0) -> PathPose:
    return PathPose(x=x, y=y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0)


def test_region_schema_is_strict_and_map_bound() -> None:
    region_set = parse_region_set(
        _document([_square("slow_1", "slow_zone", 0.0, 2.0, max_linear_speed=0.6)])
    )
    assert region_set.region_set_id == "field_a"
    assert region_set.regions[0].region_type == RegionType.SLOW_ZONE
    validate_map_binding(
        region_set,
        expected_map_id="field_map",
        expected_map_revision="r1",
        expected_manifest_sha256=SHA256,
    )
    with pytest.raises(RegionContractError, match="map_revision"):
        validate_map_binding(
            region_set,
            expected_map_id="field_map",
            expected_map_revision="r2",
            expected_manifest_sha256=SHA256,
        )


def test_map_binding_accepts_map_tools_timestamp_revision() -> None:
    document = _document()
    document["map_binding"]["map_revision"] = "20260830T092655Z"
    region_set = parse_region_set(document)
    assert region_set.map_binding.map_revision == "20260830T092655Z"

    document["map_binding"]["map_revision"] = "../escaped"
    with pytest.raises(RegionContractError, match="map_revision"):
        parse_region_set(document)


def test_region_contract_hash_binds_semantics_not_yaml_order() -> None:
    slow = _square("slow", "slow_zone", 0.0, 2.0, max_linear_speed=0.6)
    narrow = _square("narrow", "no_spin", 1.0, 2.0)
    forward = parse_region_set(_document([slow, narrow]))
    reversed_order = parse_region_set(_document([narrow, slow]))
    changed = deepcopy(slow)
    changed["max_linear_speed"] = 0.5
    changed_set = parse_region_set(_document([changed, narrow]))
    assert len(forward.contract_sha256) == 64
    assert forward.contract_sha256 == reversed_order.contract_sha256
    assert forward.contract_sha256 != changed_set.contract_sha256


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda document: document.update({"unknown": 1}), "unknown keys"),
        (lambda document: document.update({"schema": "future"}), "schema must"),
        (
            lambda document: document["map_binding"].update(
                {"manifest_sha256": "A" * 64}
            ),
            "lowercase hex",
        ),
        (
            lambda document: document.update(
                {
                    "regions": [
                        _square("duplicate", "no_spin", 0.0, 1.0),
                        _square("duplicate", "no_spin", 2.0, 3.0),
                    ]
                }
            ),
            "unique",
        ),
    ],
)
def test_invalid_documents_fail_closed(mutation: object, message: str) -> None:
    document = _document()
    mutation(document)
    with pytest.raises(RegionContractError, match=message):
        parse_region_set(document)


def test_polygon_self_intersection_is_rejected() -> None:
    region = {
        "id": "bowtie",
        "type": "no_spin",
        "polygon": [[0.0, 0.0], [2.0, 2.0], [0.0, 2.0], [2.0, 0.0]],
    }
    with pytest.raises(RegionContractError, match="self-intersecting"):
        parse_region_set(_document([region]))


def test_region_type_defines_admission_and_commit_contract() -> None:
    regions = [
        _square("approach", "dog_hole_approach", 0.0, 1.0),
        _square("corridor", "committed_corridor", 1.0, 2.0),
    ]
    region_set = parse_region_set(_document(regions))
    assert region_set.regions[0].admission_policy == AdmissionPolicy.DYNAMIC_CLEARANCE
    assert region_set.regions[0].traversal_policy == TraversalPolicy.PREEMPTIBLE
    assert region_set.regions[1].admission_policy == AdmissionPolicy.DYNAMIC_CLEARANCE
    assert region_set.regions[1].traversal_policy == TraversalPolicy.COMMITTED

    invalid = deepcopy(regions[1])
    invalid["traversal_policy"] = "preemptible"
    with pytest.raises(RegionContractError, match="conflicts"):
        parse_region_set(_document([invalid]))


def test_loader_rejects_duplicate_keys_aliases_and_symlinks(tmp_path: Path) -> None:
    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text("schema: one\nschema: two\n", encoding="utf-8")
    with pytest.raises(RegionContractError, match="duplicate YAML key"):
        load_region_set(duplicate)

    alias = tmp_path / "alias.yaml"
    alias.write_text("schema: &schema value\ncopy: *schema\n", encoding="utf-8")
    with pytest.raises(RegionContractError, match="aliases are forbidden"):
        load_region_set(alias)

    target = tmp_path / "target.yaml"
    target.write_text("not: relevant\n", encoding="utf-8")
    symlink = tmp_path / "regions.yaml"
    symlink.symlink_to(target)
    with pytest.raises(RegionContractError, match="non-symlink"):
        load_region_set(symlink)


def test_path_revision_binds_frame_stamp_and_all_pose_values() -> None:
    poses = (_pose(0.0), _pose(1.0))
    revision = compute_path_revision("map", 123, poses)
    assert revision == compute_path_revision("map", 123, poses)
    assert len(revision) == 64
    assert revision != compute_path_revision("map", 124, poses)
    assert revision != compute_path_revision("map", 123, (_pose(0.0), _pose(1.1)))
    with pytest.raises(RegionContractError, match="positive integer"):
        compute_path_revision("map", 0, poses)


def test_single_region_produces_exact_arc_length_interval() -> None:
    region_set = parse_region_set(
        _document([_square("slow", "slow_zone", 0.0, 2.0, max_linear_speed=0.6)])
    )
    result = annotate_path((_pose(-1.0), _pose(3.0)), region_set)
    assert result.path_length == pytest.approx(4.0)
    assert len(result.segments) == 1
    segment = result.segments[0]
    assert segment.start_distance == pytest.approx(1.0)
    assert segment.end_distance == pytest.approx(3.0)
    assert segment.region_ids == ("slow",)
    assert segment.max_linear_speed == pytest.approx(0.6)


def test_overlapping_regions_resolve_to_most_restrictive_intent() -> None:
    region_set = parse_region_set(
        _document(
            [
                _square("slow", "slow_zone", 0.0, 3.0, max_linear_speed=0.8),
                _square(
                    "narrow",
                    "no_spin",
                    1.0,
                    2.0,
                    max_linear_speed=0.4,
                ),
            ]
        )
    )
    result = annotate_path((_pose(-1.0), _pose(4.0)), region_set)
    assert [(item.start_distance, item.end_distance) for item in result.segments] == [
        pytest.approx((1.0, 2.0)),
        pytest.approx((2.0, 3.0)),
        pytest.approx((3.0, 4.0)),
    ]
    overlap = result.segments[1]
    assert overlap.region_ids == ("narrow", "slow")
    assert overlap.max_linear_speed == pytest.approx(0.4)
    assert overlap.no_spin


def test_forbidden_and_committed_regions_are_explicit() -> None:
    region_set = parse_region_set(
        _document(
            [
                _square("blocked", "forbidden", 0.0, 1.0),
                _square("tunnel", "committed_corridor", 2.0, 3.0),
            ]
        )
    )
    result = annotate_path((_pose(-1.0), _pose(4.0)), region_set)
    assert result.segments[0].blocked
    assert not result.segments[1].blocked
    assert result.segments[1].admission_policy == AdmissionPolicy.DYNAMIC_CLEARANCE
    assert result.segments[1].traversal_policy == TraversalPolicy.COMMITTED


def test_incompatible_heading_constraints_reject_whole_annotation() -> None:
    region_set = parse_region_set(
        _document(
            [
                _square(
                    "east",
                    "slow_zone",
                    0.0,
                    2.0,
                    max_linear_speed=0.8,
                    required_heading=0.0,
                    heading_tolerance=0.1,
                ),
                _square(
                    "north",
                    "no_spin",
                    1.0,
                    3.0,
                    required_heading=math.pi / 2.0,
                    heading_tolerance=0.1,
                ),
            ]
        )
    )
    with pytest.raises(RegionContractError, match="incompatible headings"):
        annotate_path((_pose(-1.0), _pose(4.0)), region_set)


def test_empty_region_set_preserves_path_length_without_annotations() -> None:
    region_set = parse_region_set(_document())
    result = annotate_path((_pose(0.0), _pose(3.0, 4.0)), region_set)
    assert result.path_length == pytest.approx(5.0)
    assert result.segments == ()


def test_annotation_complexity_limit_fails_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    region_set = parse_region_set(
        _document([_square("slow", "slow_zone", 0.0, 2.0, max_linear_speed=0.6)])
    )
    monkeypatch.setattr(core, "MAX_ANNOTATION_OPERATIONS", 3)
    with pytest.raises(RegionContractError, match="intersection exceeds"):
        annotate_path((_pose(-1.0), _pose(1.0)), region_set)


def test_cli_validates_binding_and_reports_summary(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    regions = tmp_path / "regions.yaml"
    regions.write_text(
        """\
schema: rm_semantic_regions/v1
region_set_id: field_a
revision: rev1
map_binding:
  frame_id: map
  map_id: field_map
  map_revision: r1
  manifest_sha256: "1111111111111111111111111111111111111111111111111111111111111111"
regions: []
""",
        encoding="utf-8",
    )
    result = validate_regions_main(
        [
            "--regions",
            str(regions),
            "--expected-map-id",
            "field_map",
            "--expected-map-revision",
            "r1",
            "--expected-manifest-sha256",
            SHA256,
        ]
    )
    assert result == 0
    output = capsys.readouterr().out
    assert '"region_count": 0' in output
    assert '"map_id": "field_map"' in output
    assert '"region_set_sha256":' in output
