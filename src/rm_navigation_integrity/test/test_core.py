import math
import unittest

from rm_navigation_integrity.core import (
    IntegrityMetrics,
    IntegrityState,
    IntegrityThresholds,
    OccupancyDistanceField,
    Pose2D,
    Transform3D,
    compose_pose,
    correction_from_global_and_odom,
    evaluate_integrity,
    inverse_pose,
    pose_delta,
    scan_map_metrics,
)


def _ready_metrics(**updates):
    values = dict(
        map_ready=True,
        global_pose_ready=True,
        scan_ready=True,
        odom_ready=True,
        tf_ready=True,
        scan_map_ready=True,
        global_pose_age_sec=0.01,
        scan_age_sec=0.01,
        pose_scan_dt_sec=0.01,
        pose_odom_dt_sec=0.01,
        tf_scan_dt_sec=0.0,
        valid_scan_points=100,
        scan_points_in_map=90,
        scan_map_agreement=0.80,
        scan_map_mean_residual_m=0.10,
    )
    values.update(updates)
    return IntegrityMetrics(**values)


class CoreTest(unittest.TestCase):
    def test_pose_composition_and_inverse_round_trip(self):
        pose = Pose2D(2.0, -1.0, 0.7)
        identity = compose_pose(pose, inverse_pose(pose))
        self.assertAlmostEqual(identity.x, 0.0, places=12)
        self.assertAlmostEqual(identity.y, 0.0, places=12)
        self.assertAlmostEqual(identity.yaw, 0.0, places=12)

    def test_map_to_odom_correction_reconstructs_global_pose(self):
        map_to_base = Pose2D(3.0, -1.0, 0.5)
        odom_to_base = Pose2D(0.4, 0.2, 0.1)
        correction = correction_from_global_and_odom(map_to_base, odom_to_base)
        reconstructed = compose_pose(correction, odom_to_base)
        translation, yaw = pose_delta(reconstructed, map_to_base)
        self.assertAlmostEqual(translation, 0.0, places=12)
        self.assertAlmostEqual(yaw, 0.0, places=12)

    def test_distance_field_respects_rotated_map_origin(self):
        data = [0] * 16
        data[1 * 4 + 2] = 100
        field = OccupancyDistanceField(
            4, 4, 1.0, Pose2D(10.0, 20.0, math.pi / 2.0), data
        )
        self.assertEqual(field.world_to_grid(8.5, 22.5), (2, 1))
        self.assertAlmostEqual(field.distance_at_world(8.5, 22.5), 0.0)

    def test_scan_map_metrics_applies_timestamped_sensor_extrinsic(self):
        data = [0] * (20 * 10)
        data[10] = 100
        field = OccupancyDistanceField(
            20, 10, 0.1, Pose2D(0.0, 0.0, 0.0), data
        )
        metrics = scan_map_metrics(
            ranges=[0.8],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            map_to_base=Transform3D(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            ),
            base_to_scan=Transform3D(
                (0.2, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            ),
            distance_field=field,
        )
        self.assertEqual(metrics.valid_point_count, 1)
        self.assertEqual(metrics.points_in_map, 1)
        self.assertAlmostEqual(metrics.agreement, 1.0)
        self.assertAlmostEqual(metrics.mean_residual_m, 0.0)

    def test_integrity_is_good_when_no_reason_is_triggered(self):
        result = evaluate_integrity(_ready_metrics(), IntegrityThresholds())
        self.assertEqual(result.state, IntegrityState.GOOD)
        self.assertEqual(result.reasons, ())

    def test_missing_evidence_is_unknown_not_rejected(self):
        result = evaluate_integrity(
            _ready_metrics(map_ready=False, scan_map_agreement=None),
            IntegrityThresholds(),
        )
        self.assertEqual(result.state, IntegrityState.UNKNOWN)
        self.assertIn("MAP_UNAVAILABLE", result.reasons)

    def test_unavailable_scan_map_evidence_is_unknown(self):
        result = evaluate_integrity(
            _ready_metrics(scan_map_ready=False, scan_map_agreement=None),
            IntegrityThresholds(),
        )
        self.assertEqual(result.state, IntegrityState.UNKNOWN)
        self.assertIn("SCAN_MAP_EVIDENCE_UNAVAILABLE", result.reasons)

    def test_bad_scan_alone_is_suspect_not_rejected(self):
        result = evaluate_integrity(
            _ready_metrics(scan_map_agreement=0.10), IntegrityThresholds()
        )
        self.assertEqual(result.state, IntegrityState.SUSPECT)
        self.assertIn("SCAN_MAP_AGREEMENT_LOW", result.reasons)

    def test_independent_hard_jump_and_bad_scan_produce_shadow_reject(self):
        result = evaluate_integrity(
            _ready_metrics(
                scan_map_agreement=0.10,
                correction_translation_jump_m=0.90,
            ),
            IntegrityThresholds(),
        )
        self.assertEqual(result.state, IntegrityState.REJECT)
        self.assertIn("CORRECTION_TRANSLATION_JUMP", result.reasons)
        self.assertIn("SCAN_MAP_AGREEMENT_LOW", result.reasons)


if __name__ == "__main__":
    unittest.main()
