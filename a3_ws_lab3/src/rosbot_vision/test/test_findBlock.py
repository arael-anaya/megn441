import cv2
import numpy as np

from rosbot_vision.findBlock import find_block


def square_image(color_bgr, angle, center=(320, 240), size=80):
    img = np.zeros((480, 640, 3), np.uint8)
    box = cv2.boxPoints((center, (size, size), angle))
    cv2.fillPoly(img, [box.astype(np.int32)], color_bgr)
    return img


def test_finds_center_of_red_square():
    cx, cy, _ = find_block(square_image((0, 0, 255), 0), 'red')
    assert abs(cx - 320) < 2
    assert abs(cy - 240) < 2


def test_other_colors_not_found():
    img = square_image((0, 0, 255), 0)
    assert find_block(img, 'blue') is None
    assert find_block(img, 'green') is None


def test_yaw_wraps_into_45_degree_range():
    for trueAngle in [-40, -20, 0, 20, 40, 60, 90]:
        _, _, yaw = find_block(square_image((0, 0, 255), trueAngle), 'red')
        assert -45 <= yaw < 45
        # a square repeats every 90 degrees
        diff = (yaw - trueAngle + 45) % 90 - 45
        assert abs(diff) < 3


def test_small_blob_is_ignored():
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (100, 100), (110, 110), (0, 0, 255), -1)
    assert find_block(img, 'red') is None
