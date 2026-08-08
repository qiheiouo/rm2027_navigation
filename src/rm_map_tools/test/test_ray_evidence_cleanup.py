import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from rm_map_tools.ray_evidence_cleanup import (
    RayObservation,
    clean_with_ray_evidence,
    load_ray_observations,
    main,
    traverse_voxels,
)
from rm_map_tools.map_export import write_ascii_pcd


def test_dda_excludes_origin_and_endpoint_voxels() -> None:
    traversed = traverse_voxels((0.1, 0.1, 0.1), (3.1, 0.1, 0.1), 1.0)
    assert traversed == ((1, 0, 0), (2, 0, 0))


def test_sidecar_requires_map_frame_and_monotonic_time(tmp_path: Path) -> None:
    sidecar = tmp_path / "rays.jsonl"
    sidecar.write_text(
        "\n".join(
            [
                json.dumps(
                    {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
                ),
                json.dumps(
                    {
                        "type": "observation",
                        "stamp": 2.0,
                        "origin": [0.0, 0.0, 0.0],
                        "endpoints": [[1.0, 0.0, 0.0]],
                    }
                ),
                json.dumps(
                    {
                        "type": "observation",
                        "stamp": 1.0,
                        "origin": [0.0, 0.0, 0.0],
                        "endpoints": [[1.0, 0.0, 0.0]],
                    }
                ),
            ]
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        load_ray_observations(sidecar)


def test_ray_evidence_removes_transient_and_keeps_static() -> None:
    points = np.asarray(
        [
            [2.1, 1.1, 0.1],  # transient obstacle
            [2.2, 1.1, 0.1],
            [3.1, 0.1, 0.1],  # persistent wall
        ],
        dtype=np.float32,
    )
    observations = []
    for index in range(2):
        observations.append(
            RayObservation(
                1.0 + index,
                (0.1, 0.1, 0.1),
                ((2.1, 1.1, 0.1), (3.1, 0.1, 0.1)),
            )
        )
    for index in range(5):
        observations.append(
            RayObservation(
                3.0 + index,
                (0.1, 1.1, 0.1),
                ((4.1, 1.1, 0.1), (3.1, 0.1, 0.1)),
            )
        )

    result = clean_with_ray_evidence(
        points,
        observations,
        voxel_resolution=1.0,
        min_pass_observations=4,
        pass_to_hit_ratio=2.0,
    )

    assert result.kept_points.shape == (1, 3)
    assert np.allclose(result.kept_points[0], [3.1, 0.1, 0.1])
    assert result.report["removed_points"] == 2
    assert result.report["removed_voxels"] == 1


def test_cli_never_overwrites_source_and_requires_explicit_candidate(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.pcd"
    observations = tmp_path / "rays.jsonl"
    report = tmp_path / "report.json"
    candidate = tmp_path / "candidate.pcd"
    write_ascii_pcd(
        source,
        np.asarray(
            [[1.1, 0.1, 0.1], [0.1, 2.1, 0.1]],
            dtype=np.float32,
        ),
    )
    observations.write_text(
        "\n".join(
            [
                json.dumps(
                    {"schema": "rm_map_ray_observations/v1", "frame_id": "map"}
                ),
                *[
                    json.dumps(
                        {
                            "type": "observation",
                            "stamp": float(index + 1),
                            "origin": [0.1, 0.1, 0.1],
                            "endpoints": [
                                [2.1, 0.1, 0.1],
                                [0.1, 2.1, 0.1],
                            ],
                        }
                    )
                    for index in range(4)
                ],
            ]
        ),
        encoding="utf-8",
    )
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()

    assert main(
        [
            "--input-pcd",
            str(source),
            "--observations",
            str(observations),
            "--report",
            str(report),
            "--voxel-resolution",
            "1.0",
            "--min-pass-observations",
            "4",
        ]
    ) == 0

    assert report.exists()
    assert not candidate.exists()
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash

    write_report = tmp_path / "write_report.json"
    assert main(
        [
            "--input-pcd",
            str(source),
            "--observations",
            str(observations),
            "--report",
            str(write_report),
            "--output-pcd",
            str(candidate),
            "--write-candidate",
            "--voxel-resolution",
            "1.0",
            "--min-pass-observations",
            "4",
        ]
    ) == 0

    assert candidate.exists()
    write_summary = json.loads(write_report.read_text(encoding="utf-8"))
    assert write_summary["candidate_written"]
    assert write_summary["kept_points"] == 1
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
