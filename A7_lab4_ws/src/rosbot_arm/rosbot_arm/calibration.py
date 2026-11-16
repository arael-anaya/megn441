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

from rosbot_arm.armConfig import (PICK_OFFSET, REACH_ERROR, GRIPPER_OPEN, GRIPPER_CLOSED, GRASP_Z, towardBase)

CALIBRATION_FILE = '~/.ros/block_calibration.json'
MAPS_TO = 'ik_target_v2'


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
        rotation = u @ np.diag([1.0, -1.0]) @ vt
    angle = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return (f'rotation {angle:+.1f} deg, scale {s[0]:.3f} / {s[1]:.3f}, '
            f'{"MIRRORED, " if mirrored else ""}shift ({1000 * b[0]:+.0f}, {1000 * b[1]:+.0f}) mm')


def apply(A, b, x, y, yawRad):
    """Corrected (x, y, yaw). The block's edge direction goes through the same linear map as its position."""
    cx, cy = A @ [x, y] + b
    edge = A @ [math.cos(yawRad), math.sin(yawRad)]
    return float(cx), float(cy), math.atan2(edge[1], edge[0])


def pickTarget(calib, x, y, yawRad):
    """IK target (x, y) and block yaw for grabbing a block the camera reports at (x, y, yawRad).

    With a calibration (A, b) that is the measured map. Without one the camera position is trusted and only the
    arm's known overreach is taken off. PICK_OFFSET is added either way.
    """
    if calib is not None:
        tx, ty, yawRad = apply(*calib, x, y, yawRad)
    else:
        tx, ty = towardBase(x, y, REACH_ERROR)
    return tx + PICK_OFFSET[0], ty + PICK_OFFSET[1], yawRad


def _read(path):
    try:
        with open(os.path.expanduser(path)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _write(path, data):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def save(path, A, b, lookXyz, residuals):
    data = _read(path)
    data.update({'A': np.asarray(A).tolist(), 'b': np.asarray(b).tolist(), 'look_xyz': list(lookXyz),
                 'maps_to': MAPS_TO, 'max_residual_m': float(np.max(residuals))})
    _write(path, data)


def saveRobot(path, gripperClosed, gripperOpen, graspZ):
    """The hardware numbers that differ between robots (measured by calibrate_node's grip phase). Kept in the same
    file as the camera fit and independent of the look pose."""
    data = _read(path)
    data['robot'] = {'gripper_closed': int(gripperClosed), 'gripper_open': int(gripperOpen),
                     'grasp_z': float(graspZ)}
    _write(path, data)


def robotSettings(path=CALIBRATION_FILE):
    """{'gripper_closed', 'gripper_open', 'grasp_z'} as measured on this robot, armConfig defaults for any missing."""
    saved = _read(path).get('robot', {})
    return {'gripper_closed': int(saved.get('gripper_closed', GRIPPER_CLOSED)),
            'gripper_open': int(saved.get('gripper_open', GRIPPER_OPEN)),
            'grasp_z': float(saved.get('grasp_z', GRASP_Z))}


def load(path):
    """(A, b, look_xyz) or None if there is no camera fit in the calibration file."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    data = _read(path)
    if 'A' not in data:
        return None
    return np.array(data['A']), np.array(data['b']), data['look_xyz']


def loadFor(lookXyz, path=CALIBRATION_FILE):
    """The saved (A, b) if it was measured from this look pose, else None; plus a line saying which, for the log."""
    loaded = load(path)
    if loaded is None:
        return None, (f'No calibration at {path}; using raw camera positions (picks may miss). '
                      'Run: ros2 launch rosbot_arm stack_launch.py mode:=calibrate')
    A, b, measuredFrom = loaded
    with open(os.path.expanduser(path)) as f:
        if json.load(f).get('maps_to') != MAPS_TO:
            return None, (f'Calibration {path} was measured before the camera pose and reach handling were fixed; '
                          'not using it. Run: ros2 launch rosbot_arm stack_launch.py mode:=calibrate')
    if any(abs(a - float(c)) > 1e-3 for a, c in zip(measuredFrom, lookXyz)):
        return None, (f'Calibration was measured from look pose {measuredFrom}, not {list(lookXyz)}; not using it. '
                      'Recalibrate after changing the look pose.')
    return (A, b), f'Using calibration {path}: {describe(A, b)}'
