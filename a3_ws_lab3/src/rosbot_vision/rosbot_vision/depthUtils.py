import numpy as np

def median_depth(depth, u, v, half=3):
    h, w = depth.shape[:2]
    u = int(round(u))
    v = int(round(v))

    patch = depth[max(v - half, 0):min(v + half + 1, h),
                    max(u - half, 0):min(u + half + 1, w)]

    valid = patch[patch>2]
    if valid.size == 0:
        return None

    return float(np.median(valid))

def deproject(u, v, z, fx, fy, cx, cy):
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z


def camera_to_arm(point, translation, rpyDeg):

    r, p, y = np.deg2rad(rpyDeg)
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    rot = rz @ ry @ rx
    return rot @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float)
