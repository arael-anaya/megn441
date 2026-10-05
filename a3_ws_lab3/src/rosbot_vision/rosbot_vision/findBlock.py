import cv2
import numpy as np

# ranges of Hue Saturation Value for different colors
HSV_RANGES = {
    'red':   [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 80, 70), (80, 255, 255))],
    'blue':  [((100, 120, 70), (130, 255, 255))],
}


def find_block(bgr, color, minArea = 500):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

    # 5 by 5 square of ones
    kernel = np.ones((5,5), np.uint8)

    # open removes white spects smaller than kernel, close fills vlack holes smaller than kernel
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

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


    



