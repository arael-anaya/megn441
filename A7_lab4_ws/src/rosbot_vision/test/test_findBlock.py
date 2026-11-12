import numpy as np

from rosbot_vision.findBlock import block_candidates


def test_small_distant_blob_survives_cleanup():
    bgr = np.full((480, 640, 3), 120, np.uint8)
    bgr[200:208, 300:310] = (0, 0, 220)
    found = block_candidates(bgr, 'red', minArea=40)
    assert len(found) == 1
    assert 60 <= len(found[0][0]) <= 140


def test_other_colors_not_found():
    bgr = np.full((480, 640, 3), 120, np.uint8)
    bgr[200:240, 300:340] = (0, 0, 220)
    assert block_candidates(bgr, 'blue') == []
