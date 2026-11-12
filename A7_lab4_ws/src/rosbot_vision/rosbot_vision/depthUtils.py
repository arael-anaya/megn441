import math

import numpy as np


def depth_to_meters(depth, depthScale):
    """Depth image in meters as float. 32FC1 images are already in meters; 16UC1 ones are scaled by depthScale."""
    if np.issubdtype(depth.dtype, np.floating):
        z = depth.astype(float)
        z[~np.isfinite(z)] = 0.0
        return z
    return depth.astype(float) * depthScale


def pixel_rays(us, vs, fx, fy, cx, cy):
    """Camera rays through pixels, scaled so their optical z is 1 (a ray times the depth is the 3D point)."""
    us = np.asarray(us, dtype=float)
    vs = np.asarray(vs, dtype=float)
    return np.column_stack([(us - cx) / fx, (vs - cy) / fy, np.ones(len(us))])


def intersect_plane(rays, normal, offset):
    """Points where camera rays hit the plane normal . p = offset. Rays parallel to or facing away from it are dropped."""
    denom = rays @ normal
    good = denom > 1e-6
    return rays[good] * (offset / denom[good])[:, None]


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def fit_table_plane(depth, fx, fy, cx, cy, depthScale, step=8, iterations=300, tol=0.006, seed=0,
                    expectedNormal=None, maxTiltDeg=12.0):
    """RANSAC plane through the depth image, in camera optical coordinates, refined by least squares on the inliers.

    Returns (height, tiltDeg, inlierFraction, normal) where height is how far the camera is from the plane, tiltDeg
    is how far the optical axis points below the plane's horizon (90 = looking straight at the plane) and normal is
    the unit plane normal in camera coordinates pointing from the camera toward the plane, or None.

    expectedNormal (from TF) restricts the search to planes within maxTiltDeg of it, so walls and furniture in view
    cannot be mistaken for the floor.
    """
    h, w = depth.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth[vs, us].astype(float) * depthScale
    good = (z > 0.05) & (z < 1.5)
    us, vs, z = us[good], vs[good], z[good]
    if z.size < 200:
        return None
    pts = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])

    cosLimit = math.cos(math.radians(maxTiltDeg))
    rng = np.random.default_rng(seed)
    best, bestCount = None, 0
    for _ in range(iterations):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if expectedNormal is not None and abs(n @ expectedNormal) < cosLimit:
            continue
        count = int(np.count_nonzero(np.abs((pts - a) @ n) < tol))
        if count > bestCount:
            best, bestCount = (n, float(n @ a)), count
    if best is None:
        return None
    n, dist = best

    # The 3-point plane is only as good as those 3 points; fit all of its inliers for a steadier height and tilt
    inliers = pts[np.abs(pts @ n - dist) < tol]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    n = vt[2]
    dist = float(n @ centroid)

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
