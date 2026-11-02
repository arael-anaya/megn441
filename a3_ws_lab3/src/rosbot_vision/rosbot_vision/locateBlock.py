"""Block position from a color blob, the depth image and the table plane.

The old way (depth at the blob centre, deprojected) lands behind the block whenever that depth comes from the table:
an edge pixel, a depth hole or a small color / depth misregistration slides the point down the camera ray to where the
ray meets the table, which is past the block. Here depth is only used to decide which pixels are the block's top face.
Their position comes from where their camera rays cross the plane one block height above the table, which only needs
the table plane (fit from thousands of points) and the camera intrinsics.
"""
import math

import cv2
import numpy as np

from rosbot_vision.depthUtils import pixel_rays, intersect_plane
from rosbot_vision.findBlock import block_candidates


def _largest_component(us, vs, shape):
    """Keep the biggest connected group of pixels so stray depth noise elsewhere in the blob is ignored."""
    mask = np.zeros(shape, np.uint8)
    mask[vs, us] = 255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 2:
        return us, vs
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    keep = labels[vs, us] == biggest
    return us[keep], vs[keep]


def _rect_yaw(xy):
    """minAreaRect of arm-frame x / y points: (yawDeg in [-45, 45), sorted side lengths in m)."""
    rect = cv2.minAreaRect((xy * 1000.0).astype(np.float32))
    box = cv2.boxPoints(rect)
    edge = box[1] - box[0]
    yawDeg = ((math.degrees(math.atan2(edge[1], edge[0])) + 45.0) % 90.0) - 45.0
    return yawDeg, sorted(s / 1000.0 for s in rect[1])


def _fit_square(xy, yawDeg, size, cameraXy):
    """Centre of a size x size square with the given yaw from the blob projected onto the top-face plane.

    In that projection the top face lands exactly where it is, and the side faces the camera can see land next to it
    on the camera's side. So along each edge direction the extreme facing away from the camera is a true top-face
    edge, and the centre is half a block back from it.
    """
    yaw = math.radians(yawDeg)
    toCamera = np.asarray(cameraXy) - np.median(xy, axis=0)
    centre = np.zeros(2)
    for axis in (np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])):
        proj = xy @ axis
        if toCamera @ axis > 0:
            centre += (np.percentile(proj, 1) + size / 2) * axis
        else:
            centre += (np.percentile(proj, 99) - size / 2) * axis
    return centre


def _locate_candidate(us, vs, depthM, intrinsics, plane, rotation, translation, blockHeight, maxHeight, reasons,
                      requireRaised=True):
    fx, fy, cx, cy = intrinsics
    h, w = depthM.shape[:2]
    if us.min() <= 1 or vs.min() <= 1 or us.max() >= w - 2 or vs.max() >= h - 2:
        # The image edge would be taken for the block's edge and the position would be off by up to a block
        reasons.append('blob is cut off by the image edge')
        return None
    rays = pixel_rays(us, vs, fx, fy, cx, cy)
    z = depthM[vs, us]
    valid = (z > 0.05) & (z < 3.0)

    if plane is None:
        # No table plane: best we can do is the median of the block surface seen in depth
        if np.count_nonzero(valid) < 10:
            reasons.append('no table plane and no depth on the blob')
            return None
        surface = rays[valid] * z[valid, None]
        armPts = surface @ rotation.T + translation
        yawDeg, sides = _rect_yaw(armPts[:, :2])
        x, y, zArm = np.median(armPts, axis=0)
        return {
            'x': float(x), 'y': float(y), 'z': float(zArm), 'yaw_deg': yawDeg,
            'pixel': (float(np.mean(us)), float(np.mean(vs))), 'top_px': len(surface), 'size': sides,
        }

    normal, dist = plane
    blobUs, blobVs = us, vs
    topXy = None
    raised = False
    if np.count_nonzero(valid) >= 20:
        heights = dist - (rays[valid] * z[valid, None]) @ normal
        meanDepth = float(np.median(z[valid]))
        blockPx = blockHeight * fx / meanDepth
        elevated = (heights > 0.4 * blockHeight) & (heights < maxHeight)
        raised = np.count_nonzero(elevated) >= max(15, 0.2 * blockPx ** 2)
        if not raised:
            reasons.append(f'only {np.count_nonzero(elevated)} px stand above the table (flat on the table or '
                           f'too tall; median height {1000 * np.median(heights):.0f} mm)')
            if requireRaised:
                return None
    # Not raised (or no depth on the blob): go on the color blob alone
    if raised:
        top = (heights > 0.7 * blockHeight) & (heights < maxHeight)
        pick = top if np.count_nonzero(top) >= 15 else elevated
        topUs, topVs = _largest_component(us[valid][pick], vs[valid][pick], depthM.shape[:2])
        topPts = intersect_plane(pixel_rays(topUs, topVs, fx, fy, cx, cy), normal, dist - blockHeight)
        if len(topPts) >= 10:
            topXy = (topPts @ rotation.T + translation)[:, :2]

        # Drop blob pixels that depth puts on the table (a same-colored patch touching the block would stretch it),
        # except right next to the raised pixels, where depth may just be a few pixels misaligned with color
        raisedUs, raisedVs = _largest_component(us[valid][elevated], vs[valid][elevated], depthM.shape[:2])
        near = np.zeros(depthM.shape[:2], np.uint8)
        near[raisedVs, raisedUs] = 255
        near = cv2.dilate(near, np.ones((13, 13), np.uint8))
        offTable = np.zeros(len(us), bool)
        offTable[np.flatnonzero(valid)[heights >= 0.15 * blockHeight]] = True
        keep = (near[vs, us] > 0) | offTable
        blobUs, blobVs = us[keep], vs[keep]

    blobPts = intersect_plane(pixel_rays(blobUs, blobVs, fx, fy, cx, cy), normal, dist - blockHeight)
    if len(blobPts) < 10:
        reasons.append('blob rays miss the table plane')
        return None
    armPts = blobPts @ rotation.T + translation
    blobXy = armPts[:, :2]

    blobYaw, sides = _rect_yaw(blobXy)
    # Side faces seen at a shallow angle stretch the blob toward the camera, so allow it to be long
    if sides[0] < 0.5 * blockHeight or sides[1] > 2.6 * blockHeight:
        reasons.append(f'blob is {1000 * sides[0]:.0f} x {1000 * sides[1]:.0f} mm on the table, '
                       f'not a {1000 * blockHeight:.0f} mm block')
        return None
    # The top face alone gives the cleanest edge directions; the whole blob is skewed by the side faces
    yawDeg = _rect_yaw(topXy)[0] if topXy is not None and len(topXy) >= 30 else blobYaw

    x, y = _fit_square(blobXy, yawDeg, blockHeight, translation[:2])
    # Points are on the top-face plane; the block centre is half a block below it
    zArm = float(np.median(armPts[:, 2]) - blockHeight / 2)
    return {
        'x': float(x), 'y': float(y), 'z': zArm, 'yaw_deg': yawDeg,
        'pixel': (float(np.mean(blobUs)), float(np.mean(blobVs))), 'top_px': len(blobUs), 'size': sides,
    }


def locate_block(bgr, depthM, color, intrinsics, plane, rotation, translation, blockHeight,
                 minAreaPx=150, maxHeight=0.06, ranges=None, reasons=None, requireRaised=True):
    """Centre of the `color` block in the arm frame, or None.

    depthM:      depth image in meters, registered to the color image
    intrinsics:  (fx, fy, cx, cy) of the color camera
    plane:       (normal, dist) of the table in camera coordinates, normal pointing from the camera to the table, or None
    rotation, translation: camera -> arm frame
    reasons:     optional list that collects why blobs were rejected, for logging
    requireRaised: reject blobs that depth shows lying flat on the table; False keeps them on color alone

    Returns a dict with x, y, z (block centre, m), yaw_deg (arm frame, [-45, 45)), pixel, top_px and size, or None.
    """
    if reasons is None:
        reasons = []
    best = None
    for us, vs in block_candidates(bgr, color, minAreaPx, ranges):
        result = _locate_candidate(us, vs, depthM, intrinsics, plane, rotation, translation,
                                   blockHeight, maxHeight, reasons, requireRaised)
        if result is not None and (best is None or result['top_px'] > best['top_px']):
            best = result
    if best is None and not reasons:
        reasons.append(f'no {color} blob >= {minAreaPx} px')
    return best
