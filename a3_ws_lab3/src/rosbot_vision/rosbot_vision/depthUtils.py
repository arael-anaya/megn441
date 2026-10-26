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


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fit_table_plane(depth, fx, fy, cx, cy, depthScale, step=8, iterations=150, tol=0.006, seed=0):
    """RANSAC plane through the depth image, in camera optical coordinates.

    Returns (height, tiltDeg, inlierFraction, normal) where height is how far the camera is from the plane, tiltDeg
    is how far the optical axis points below the plane's horizon (90 = looking straight at the plane) and normal is
    the unit plane normal in camera coordinates pointing from the camera toward the plane, or None.
    """
    h, w = depth.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth[vs, us].astype(float) * depthScale
    good = (z > 0.05) & (z < 1.5)
    us, vs, z = us[good], vs[good], z[good]
    if z.size < 200:
        return None
    pts = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])

    rng = np.random.default_rng(seed)
    best, bestCount = None, 0
    for _ in range(iterations):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        count = int(np.count_nonzero(np.abs((pts - a) @ n) < tol))
        if count > bestCount:
            best, bestCount = (n, float(n @ a)), count
    if best is None:
        return None
    n, dist = best
    if dist < 0:
        n, dist = -n, -dist
    tilt = np.degrees(np.arcsin(min(1.0, abs(n[2]))))
    return dist, float(tilt), bestCount / len(pts), n


def level_rotation(rotation, normal):
    """Correct a camera -> arm rotation so the arm's z axis matches the measured table normal.

    The arm's pitch joints all turn about the arm's y axis, so a wrong joint angle in TF tilts the camera about that
    axis. Keep TF's y axis (heading), take "up" from the table plane and rebuild x from the two.
    """
    zc = -np.asarray(normal, dtype=float)
    yc = rotation[1, :] - (rotation[1, :] @ zc) * zc
    yc /= np.linalg.norm(yc)
    xc = np.cross(yc, zc)
    return np.vstack([xc, yc, zc])
