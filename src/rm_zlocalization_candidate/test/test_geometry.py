"""Three source-time full-SE(3) geometries, including dynamic lever arm."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from geometry import matrix, quaternion, body_pose


def rotation(axis, angle, translation=(0., 0., 0.)):
    return matrix(translation, [*(np.asarray(axis) * np.sin(angle / 2)), np.cos(angle / 2)])


@pytest.mark.parametrize('yaw,pitch,lever', [(0., 0., (0., 0., 0.)),
    (.8, 0., (0., 0., .2)), (-.6, .35, (.3, -.1, .4))])
def test_dynamic_sensor_to_base(yaw, pitch, lever):
    expected_base = rotation((0, 0, 1), .4, (2., -1., .3)) @ rotation((1, 0, 0), .2)
    base_sensor = rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch, lever)
    sensor = expected_base @ base_sensor
    recovered, covariance = body_pose(sensor, base_sensor, np.diag([.1]*3 + [.01]*3))
    np.testing.assert_allclose(recovered, expected_base, atol=1.e-12)
    np.testing.assert_allclose(matrix(recovered[:3, 3], quaternion(recovered[:3, :3])), expected_base, atol=1.e-12)
    assert np.linalg.eigvalsh(covariance).min() > 0
    # Reconstruct the same correction through canonical odom, independent of raw basis.
    canonical_base = rotation((0, 0, 1), -.2, (1., .4, -.1))
    correction = recovered @ np.linalg.inv(canonical_base)
    np.testing.assert_allclose(correction @ canonical_base @ base_sensor, sensor, atol=1.e-12)
