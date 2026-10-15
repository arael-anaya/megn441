import numpy as np

from rosbot_vision.depthUtils import median_depth, deproject, camera_to_arm


def test_median_ignores_invalid_pixels():
    depth = np.full((480, 640), 500, dtype=np.uint16)
    depth[240, 320] = 0
    depth[241, 320] = 9000
    assert median_depth(depth, 320, 240) == 500


def test_median_all_zero_is_none():
    assert median_depth(np.zeros((480, 640), np.uint16), 320, 240) is None


def test_median_at_corner():
    depth = np.full((480, 640), 500, dtype=np.uint16)
    assert median_depth(depth, 0, 0) == 500


def test_deproject():
    assert deproject(320, 240, 0.5, 600, 600, 320, 240) == (0, 0, 0.5)
    x, y, z = deproject(620, 240, 0.5, 600, 600, 320, 240)
    assert np.isclose(x, 0.25) and y == 0 and z == 0.5
    x, y, z = deproject(320, 540, 0.5, 600, 600, 320, 240)
    assert x == 0 and np.isclose(y, 0.25)


def test_camera_to_arm_identity_and_translation():
    p = camera_to_arm([1, 2, 3], [0, 0, 0], [0, 0, 0])
    assert np.allclose(p, [1, 2, 3])
    p = camera_to_arm([1, 2, 3], [0.1, 0.2, 0.3], [0, 0, 0])
    assert np.allclose(p, [1.1, 2.2, 3.3])


def test_camera_to_arm_yaw_90():
    p = camera_to_arm([1, 0, 0], [0, 0, 0], [0, 0, 90])
    assert np.allclose(p, [0, 1, 0], atol=1e-9)
