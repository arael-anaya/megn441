import math

import numpy as np
import pytest

from rosbot_arm import calibration

GRID = [(x, y) for x in (0.19, 0.22, 0.25) for y in (-0.05, 0.0, 0.05)]


def camera_error(points, angleDeg, scale, shift, mirror=False):
    """Where a miscalibrated camera reports blocks that really are at `points`."""
    a = math.radians(angleDeg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]) * scale
    if mirror:
        R = R @ np.diag([1.0, -1.0])
    return np.asarray(points) @ R.T + shift


@pytest.mark.parametrize('angle, scale, shift, mirror', [
    (6.0, 1.0, (0.0, 0.0), False),        # camera yaw: error grows with distance, different for every block
    (0.0, 0.9, (0.02, -0.01), False),     # wrong camera height / intrinsics plus a shift
    (-3.0, 1.05, (0.01, 0.03), True),     # mirrored image axis
])
def test_fit_undoes_position_dependent_error(angle, scale, shift, mirror):
    detected = camera_error(GRID, angle, scale, shift, mirror)
    detected += np.random.default_rng(0).normal(0, 0.001, detected.shape)  # 1 mm detection noise
    A, b, residuals = calibration.fit_affine(detected, GRID)
    assert residuals.max() < 0.004
    # A new block anywhere in the area lands where the arm has to go
    for true in [(0.20, 0.03), (0.24, -0.04)]:
        seen = camera_error([true], angle, scale, shift, mirror)[0]
        x, y, _ = calibration.apply(A, b, seen[0], seen[1], 0.0)
        assert math.hypot(x - true[0], y - true[1]) < 0.003
    assert ('MIRRORED' in calibration.describe(A, b)) == mirror


def test_yaw_follows_the_rotation():
    detected = camera_error(GRID, 10.0, 1.0, (0.0, 0.0))
    A, b, _ = calibration.fit_affine(detected, GRID)
    _, _, yaw = calibration.apply(A, b, 0.2, 0.0, math.radians(10.0))
    assert abs(math.degrees(yaw)) < 0.5


def test_points_in_a_line_are_rejected():
    with pytest.raises(ValueError):
        calibration.fit_affine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)], [(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)])


def test_save_and_load_round_trip(tmp_path):
    A, b = np.array([[1.0, 0.1], [-0.1, 1.0]]), np.array([0.01, -0.02])
    path = str(tmp_path / 'cal.json')
    calibration.save(path, A, b, [0.14, 0.0, 0.2], np.array([0.001]))
    A2, b2, look = calibration.load(path)
    assert np.allclose(A, A2) and np.allclose(b, b2) and look == [0.14, 0.0, 0.2]
    assert calibration.load(str(tmp_path / 'missing.json')) is None


def test_load_for_only_accepts_the_look_pose_it_was_measured_from(tmp_path):
    path = str(tmp_path / 'cal.json')
    assert calibration.loadFor([0.2, 0.0, 0.14], path)[0] is None
    calibration.save(path, np.eye(2), np.zeros(2), [0.2, 0.0, 0.14], np.array([0.001]))
    assert calibration.loadFor([0.2, 0.0, 0.14], path)[0] is not None
    assert calibration.loadFor([0.2, 0.0, 0.20], path)[0] is None


def test_old_format_file_is_not_used(tmp_path):
    # Measured by the first calibrate_node, which aimed REACH_ERROR short of every point
    path = str(tmp_path / 'cal.json')
    with open(path, 'w') as f:
        f.write('{"A": [[1, 0], [0, 1]], "b": [0, 0], "look_xyz": [0.2, 0.0, 0.14], "max_residual_m": 0.001}')
    calib, status = calibration.loadFor([0.2, 0.0, 0.14], path)
    assert calib is None and 'calibrate' in status


def test_pick_target_without_calibration_takes_off_the_reach_error():
    x, y, yaw = calibration.pickTarget(None, 0.20, 0.0, 0.3)
    assert abs(x - (0.20 - calibration.REACH_ERROR)) < 1e-9 and y == 0.0 and yaw == 0.3
    x, y, _ = calibration.pickTarget(None, 0.0, -0.20, 0.0)
    assert abs(y + (0.20 - calibration.REACH_ERROR)) < 1e-9 and abs(x) < 1e-9


def test_pick_target_with_calibration_is_the_measured_map():
    A, b = np.array([[1.0, 0.0], [0.0, 1.0]]), np.array([-0.02, 0.01])
    x, y, yaw = calibration.pickTarget((A, b), 0.25, 0.03, 0.2)
    assert abs(x - 0.23) < 1e-9 and abs(y - 0.04) < 1e-9 and abs(yaw - 0.2) < 1e-9
