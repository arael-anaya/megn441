import cv2
import numpy as np

from rosbot_vision.findBlock import find_block

img = np.zeros((480, 640, 3) , np.uint8)
pts = np.array([[300,200], [380, 220], [360,290], [280, 270]])
cv2.fillPoly(img, [pts], (0,0,255))

print('red:', find_block(img, 'red'))
print('blue:', find_block(img, 'blue'))

print('--rotations sweep---')

for true_angle in [-40, -20, 0, 20, 40, 60, 90]:
    img = np.zeros((480, 640, 3), np.uint8)
    box = cv2.boxPoints(((320, 240), (80, 80), true_angle))
    cv2.fillPoly(img, [box.astype(np.int32)], (0, 0, 255))
    cx, cy, yaw = find_block(img, 'red')
    print(f'true {true_angle:4d}   ->  yaw{yaw:7.2f} ')



