import numpy as np
from rosbot_vision.depthUtils import median_depth, deproject

depth = np.full((480, 640), 500, dtype=np.uint16)
depth[240, 320] = 0
depth[241, 320] = 9000

print('median:', median_depth(depth, 320, 240))
print('all zero:', median_depth(np.zeros((480, 640), np.uint16), 320, 240))
print('corner:', median_depth(depth, 0, 0))

print('center:', deproject(320, 240, 0.5, 600, 600, 320, 240))
print('right :', deproject(620, 240, 0.5, 600, 600, 320, 240))
print('down  :', deproject(320, 540, 0.5, 600, 600, 320, 240))
