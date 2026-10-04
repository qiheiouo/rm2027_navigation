"""Vectorized model clearance. Oracle deliberately uses a separate algorithm."""
import numpy as np


def clearance(states, centers, geometry, half_extents):
    yaw = states[:, 2]
    ux = np.c_[np.cos(yaw), np.sin(yaw)]
    uy = np.c_[-np.sin(yaw), np.cos(yaw)]
    relative = centers - states[:, :2]
    hx, hy = half_extents
    if geometry.kind == "circle":
        local = np.c_[np.sum(relative * ux, axis=1), np.sum(relative * uy, axis=1)]
        q = np.abs(local) - (hx, hy)
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0) - geometry.radius
    vertices = np.asarray(geometry.offsets)
    edges = np.roll(vertices, -1, axis=0) - vertices
    normals = np.c_[-edges[:, 1], edges[:, 0]]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    axes = np.concatenate((ux[:, None, :], uy[:, None, :],
                           np.broadcast_to(normals, (len(states), len(normals), 2))), axis=1)
    r = hx * np.abs(np.einsum("kli,ki->kl", axes, ux)) + hy * np.abs(np.einsum("kli,ki->kl", axes, uy))
    anchor = np.einsum("ki,kli->kl", relative, axes)
    projections = np.einsum("mi,kli->klm", vertices, axes)
    minimum = anchor + projections.min(axis=2)
    maximum = anchor + projections.max(axis=2)
    # Positive max separating-axis gap is a lower bound on Euclidean distance.
    return np.maximum(minimum - r, -r - maximum).max(axis=1)


def boundary_clearance(states, domain, half_extents):
    xmin, xmax, ymin, ymax = domain
    hx, hy = half_extents
    c, s = np.abs(np.cos(states[:, 2])), np.abs(np.sin(states[:, 2]))
    rx, ry = c * hx + s * hy, s * hx + c * hy
    return np.c_[states[:, 0] - rx - xmin, xmax - states[:, 0] - rx,
                 states[:, 1] - ry - ymin, ymax - states[:, 1] - ry]
