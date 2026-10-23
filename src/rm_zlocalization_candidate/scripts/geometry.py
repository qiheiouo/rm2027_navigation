"""Rigid pose and conditional lever-arm covariance, ROS order xyz/rotation."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def matrix(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1.e-12:
        raise ValueError('invalid orientation')
    q = q / norm
    cross = skew(q[:3])
    result = np.eye(4)
    result[:3, :3] += 2 * q[3] * cross + 2 * cross @ cross
    result[:3, 3] = position
    if not np.isfinite(result).all():
        raise ValueError('invalid position')
    return result


def quaternion(rotation):
    # Symmetric quaternion eigenproblem; output order x/y/z/w.
    r = rotation
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]])
    return np.linalg.eigh(k)[1][:, -1]


def body_pose(sensor_pose, base_from_sensor, sensor_covariance):
    result = sensor_pose @ np.linalg.inv(base_from_sensor)
    lever = result[:3, 3] - sensor_pose[:3, 3]
    jacobian = np.eye(6)
    jacobian[:3, 3:] = -skew(lever)
    covariance = jacobian @ sensor_covariance @ jacobian.T
    return result, (covariance + covariance.T) * .5
