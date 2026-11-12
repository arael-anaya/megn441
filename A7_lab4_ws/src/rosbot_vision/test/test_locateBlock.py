import math

import numpy as np

from rosbot_vision.depthUtils import fit_table_plane, level_rotation
from rosbot_vision.locateBlock import locate_block

FX = FY = 450.0
CX, CY = 320.0, 240.0
W, H = 640, 480
SIZE = 0.0254
TABLE_BGR = (120, 120, 120)


def camera_pose(tiltDeg, position=(0.05, 0.0, 0.30)):
    """Camera -> arm rotation for a camera facing +x, tilted tiltDeg below horizontal, and its position."""
    t = math.radians(tiltDeg)
    forward = np.array([math.cos(t), 0.0, -math.sin(t)])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(forward, right)
    return np.column_stack([right, down, forward]), np.array(position)


def render(rotation, translation, blocks, patches=()):
    """Ray cast a table at z = 0 with cubes (x, y, yawDeg, bgr) on it and flat colored squares (x, y, half, bgr)."""
    vs, us = np.mgrid[0:H, 0:W]
    raysCam = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones_like(us, float)], axis=-1).reshape(-1, 3)
    raysArm = raysCam @ rotation.T

    # table hit, as distance along the camera-z-normalised ray (= optical depth)
    tHit = -translation[2] / raysArm[:, 2]
    tHit[raysArm[:, 2] >= 0] = np.inf
    bgr = np.tile(np.array(TABLE_BGR, np.uint8), (len(tHit), 1))
    hitTable = translation + raysArm * tHit[:, None]
    for x, y, half, colour in patches:
        inside = (np.abs(hitTable[:, 0] - x) < half) & (np.abs(hitTable[:, 1] - y) < half)
        bgr[inside] = colour

    for x, y, yawDeg, colour in blocks:
        c, s = math.cos(math.radians(yawDeg)), math.sin(math.radians(yawDeg))
        toLocal = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        o = toLocal @ (translation - np.array([x, y, SIZE / 2]))
        d = raysArm @ toLocal.T
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (-SIZE / 2 - o) / d
            t2 = (SIZE / 2 - o) / d
        tNear = np.nanmax(np.minimum(t1, t2), axis=1)
        tFar = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tNear <= tFar) & (tNear > 0) & (tNear < tHit)
        tHit[hit] = tNear[hit]
        bgr[hit] = colour

    depth = np.where(np.isfinite(tHit), tHit * 1000.0, 0).round().astype(np.uint16)
    return bgr.reshape(H, W, 3), depth.reshape(H, W)


def locate(bgr, depth, rotation, translation, color='red'):
    depthM = depth.astype(float) * 0.001
    height, _, _, normal = fit_table_plane(depthM, FX, FY, CX, CY, 1.0)
    rot = level_rotation(rotation, normal)
    trans = np.array([translation[0], translation[1], height])
    return locate_block(bgr, depthM, color, (FX, FY, CX, CY), (normal, height), rot, trans, SIZE)


def test_oblique_view_lands_on_block_centre():
    rotation, translation = camera_pose(50)
    for bx, by, yaw in [(0.25, 0.0, 0), (0.30, 0.06, 20), (0.22, -0.05, -30)]:
        bgr, depth = render(rotation, translation, [(bx, by, yaw, (0, 0, 255))])
        det = locate(bgr, depth, rotation, translation)
        assert det is not None
        assert abs(det['x'] - bx) < 0.004, det
        assert abs(det['y'] - by) < 0.004, det
        assert abs(det['z'] - SIZE / 2) < 0.004, det
        assert abs((det['yaw_deg'] - yaw + 45) % 90 - 45) < 5, det


def test_misregistered_depth_does_not_push_target_behind_block():
    rotation, translation = camera_pose(50)
    bgr, depth = render(rotation, translation, [(0.25, 0.0, 0, (0, 0, 255))])
    # Depth that is a few pixels off from color, the way a registered depth camera often is
    for shift in (-4, 4):
        det = locate(bgr, np.roll(depth, shift, axis=0), rotation, translation)
        assert det is not None
        assert abs(det['x'] - 0.25) < 0.006, det


def test_flat_colored_patch_on_table_is_ignored():
    rotation, translation = camera_pose(50)
    # A red square of paper bigger than the block, and the real block next to it
    bgr, depth = render(rotation, translation, [(0.22, 0.05, 0, (0, 0, 255))],
                        patches=[(0.28, -0.04, 0.03, (0, 0, 255))])
    det = locate(bgr, depth, rotation, translation)
    assert det is not None
    assert abs(det['x'] - 0.22) < 0.004 and abs(det['y'] - 0.05) < 0.004, det


def test_touching_patch_with_depth_holes_does_not_stretch_block():
    rotation, translation = camera_pose(55, (0.05, 0.0, 0.205))
    bgr, depth = render(rotation, translation, [(0.282, -0.057, -34, (0, 0, 220))],
                        patches=[(0.342, -0.107, 0.025, (0, 0, 220))])
    depth[np.random.default_rng(0).random(depth.shape) < 0.05] = 0
    det = locate(bgr, depth, rotation, translation)
    assert det is not None
    assert abs(det['x'] - 0.282) < 0.006 and abs(det['y'] + 0.057) < 0.006, det


def test_block_cut_off_by_image_edge_is_not_reported():
    rotation, translation = camera_pose(71, (0.05, 0.0, 0.263))
    bgr, depth = render(rotation, translation, [(0.317, -0.020, 6, (0, 0, 220))])
    assert locate(bgr, depth, rotation, translation) is None


def test_finds_each_color_and_misses_absent_one():
    rotation, translation = camera_pose(60)
    blocks = [(0.20, 0.04, 10, (0, 0, 200)), (0.24, -0.03, 0, (0, 180, 0)), (0.28, 0.06, -15, (200, 60, 0))]
    bgr, depth = render(rotation, translation, blocks)
    for color, (bx, by, _, _) in zip(['red', 'green', 'blue'], blocks):
        det = locate(bgr, depth, rotation, translation, color)
        assert det is not None, color
        assert abs(det['x'] - bx) < 0.004 and abs(det['y'] - by) < 0.004, (color, det)

    bgr, depth = render(rotation, translation, blocks[:2])
    assert locate(bgr, depth, rotation, translation, 'blue') is None


def test_distant_block_with_stretched_side_face_is_found():
    # Seen from far away at a shallow angle the camera-facing side lands on the table as a long streak behind
    # the block (30 x 94 mm at 0.8 m); that is still one block, not an oversized blob.
    rotation, translation = camera_pose(30, (0.05, 0.0, 0.30))
    for dist in (0.5, 0.8, 1.1):
        bgr, depth = render(rotation, translation, [(dist, 0.02, 10, (0, 0, 220))])
        det = locate(bgr, depth, rotation, translation)
        assert det is not None, dist
        assert abs(det['x'] - dist) < 0.02 and abs(det['y'] - 0.02) < 0.02, (dist, det)


def test_floor_fit_prefers_floor_over_a_wall_in_view():
    rotation, translation = camera_pose(20, (0.05, 0.0, 0.25))
    _, depth = render(rotation, translation, [])
    depthM = depth.astype(float) * 0.001
    vs, us = np.mgrid[0:H, 0:W]
    raysArm = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones_like(us, float)], axis=-1).reshape(-1, 3) @ rotation.T
    # A wall 0.7 m ahead fills the whole top half of the image, with more points than the floor below it
    wall = (0.7 - translation[0]) / raysArm[:, 0]
    top = (np.arange(H * W) // W) < 300
    depthM = depthM.reshape(-1)
    depthM[top] = wall[top]
    depthM = depthM.reshape(H, W)
    free = fit_table_plane(depthM, FX, FY, CX, CY, 1.0)
    assert free is not None and abs(free[0] - 0.25) > 0.05, 'wall should fool the unguided fit, else this test proves nothing'
    guided = fit_table_plane(depthM, FX, FY, CX, CY, 1.0, expectedNormal=-rotation[2, :])
    assert guided is not None
    assert abs(guided[0] - 0.25) < 0.01, guided[:3]
