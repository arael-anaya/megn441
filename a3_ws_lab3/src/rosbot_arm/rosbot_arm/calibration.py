"""Map where the camera sees a block to where the arm has to go to grab it.

calibrate_node puts a block down at known IK targets, looks at it from the look pose and records where the vision
node reports it. A 2D affine fit of those pairs (detected -> commanded) absorbs every smooth error between the two:
camera tilt / yaw, wrong height or intrinsics (scale), a mirrored axis, and servo error at floor height. It is only
valid for the look pose it was measured from.
"""
import json
import math
import os

import numpy as np


def fit_affine(detected, commanded):
    """Least-squares A (2x2), b with commanded ~= A @ detected + b. Needs 3+ points not in a line.
    Returns (A, b, residuals in m per point)."""
    d = np.asarray(detected, dtype=float)
    c = np.asarray(commanded, dtype=float)
    if len(d) < 3:
        raise ValueError(f'need at least 3 points, got {len(d)}')
    design = np.column_stack([d, np.ones(len(d))])
    params, _, rank, _ = np.linalg.lstsq(design, c, rcond=None)
    if rank < 3:
        raise ValueError('points are in a line; spread them out in both x and y')
    A = params[:2].T
    b = params[2]
    residuals = np.linalg.norm(design @ params - c, axis=1)
    return A, b, residuals


def describe(A, b):
    """What the fit says is wrong, in words: rotation, scale, mirror and shift."""
    u, s, vt = np.linalg.svd(A)
    rotation = u @ vt
    mirrored = np.linalg.det(A) < 0
    if mirrored:
        # Closest rotation for a mirrored map: flip one axis back so the angle is meaningful
        rotation = u @ np.diag([1.0, -1.0]) @ vt
    angle = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return (f'rotation {angle:+.1f} deg, scale {s[0]:.3f} / {s[1]:.3f}, '
            f'{"MIRRORED, " if mirrored else ""}shift ({1000 * b[0]:+.0f}, {1000 * b[1]:+.0f}) mm')


def apply(A, b, x, y, yawRad):
    """Corrected (x, y, yaw). The block's edge direction goes through the same linear map as its position."""
    cx, cy = A @ [x, y] + b
    edge = A @ [math.cos(yawRad), math.sin(yawRad)]
    return float(cx), float(cy), math.atan2(edge[1], edge[0])


def save(path, A, b, lookXyz, residuals):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump({'A': np.asarray(A).tolist(), 'b': np.asarray(b).tolist(), 'look_xyz': list(lookXyz),
                   'max_residual_m': float(np.max(residuals))}, f, indent=2)


def load(path):
    """(A, b, look_xyz) or None if there is no calibration file."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        data = json.load(f)
    return np.array(data['A']), np.array(data['b']), data['look_xyz']
