import cv2
import numpy as np

# ranges of Hue Saturation Value for different colors. Kept wide on purpose: glare and shadow on a block drop its
# saturation and value, and anything flat on the table that happens to match is thrown out later by the depth check.
# Override per color with the hsv_<color> parameters in vision_params.yaml.
HSV_RANGES = {
    'red':   [((0, 90, 50), (10, 255, 255)), ((165, 90, 50), (179, 255, 255))],
    'green': [((35, 60, 40), (85, 255, 255))],
    'blue':  [((95, 80, 40), (130, 255, 255))],
}


def colorMask(bgr, color, ranges=None):
    # blur first so sensor noise does not break the block into speckles
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in (ranges or HSV_RANGES[color]):
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    return mask


def cleanMask(mask):
    # 5 by 5 square of ones
    kernel = np.ones((5,5), np.uint8)

    # open removes white spects smaller than kernel, close fills black holes smaller than kernel
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def maskPixels(bgr, color, ranges=None):
    """Number of pixels matching the color range before any cleanup, for debugging."""
    return int(np.count_nonzero(colorMask(bgr, color, ranges)))


def block_candidates(bgr, color, minArea=150, ranges=None, maxCandidates=5):
    """Every blob of `color` at least minArea pixels big, biggest first, as (us, vs) arrays of its pixel coordinates.

    Holes inside a blob (glare on the top face) are filled in. The caller decides which blob is really the block.
    """
    mask = cleanMask(colorMask(bgr, color, ranges))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= minArea]
    contours.sort(key=cv2.contourArea, reverse=True)

    candidates = []
    for c in contours[:maxCandidates]:
        filled = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(filled, [c], -1, 255, thickness=cv2.FILLED)
        vs, us = np.nonzero(filled)
        candidates.append((us, vs))
    return candidates


def find_block(bgr, color, minArea = 500):
    """Centre (pixels) and in-image yaw (deg) of the biggest `color` blob, or None. Color only, no depth."""
    mask = cleanMask(colorMask(bgr, color))

    # list of outlines
    contours , _ = cv2.findContours (mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # pick outline with biggest area
    c = max(contours, key=cv2.contourArea)

    if cv2.contourArea(c) < minArea:
        return None

    m = cv2.moments(c)

    cx = m['m10'] / m['m00']
    cy = m['m01'] / m['m00']

    (_,_), (w,h), angle = cv2.minAreaRect(c)

    yaw = ((angle+45) % 90) - 45

    return cx, cy, yaw
