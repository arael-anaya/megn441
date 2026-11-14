"""The camera pose is computed from commanded servo pulses; it has to agree with the IK that placed the arm."""
import math

import numpy as np

from rosbot_arm import inverseKinematics as ik
from rosbot_arm.armConfig import CAMERA_OFFSET
from rosbot_vision import cameraModel
from rosbot_vision.locateBlock import locate_block

from test_locateBlock import render, FX, FY, CX, CY, SIZE

LOOK = (0.20, 0.0, 0.14)


def look_angles(x, y, z):
    pulses = ik.solvePulses(x, y, z, 0.0, -90.0)
    assert pulses is not None
    return cameraModel.pulses_to_angles(dict(zip(cameraModel.ARM_JOINTS, pulses)))


def test_look_pose_puts_the_camera_above_the_floor_looking_down():
    rotation, translation = cameraModel.optical_pose(look_angles(*LOOK))
    # ahead of the tool point (plus the measured camera offset) and 0.11 m above it
    assert np.allclose(translation, [0.25 + CAMERA_OFFSET[0], CAMERA_OFFSET[1], 0.247], atol=0.004), translation
    assert np.allclose(rotation[:, 2], [0, 0, -1], atol=0.02), rotation[:, 2]   # lens points at the floor
    assert np.allclose(rotation[:, 0], [0, -1, 0], atol=0.02), rotation[:, 0]   # image right = arm -y
    assert np.allclose(rotation[:, 1], [-1, 0, 0], atol=0.02), rotation[:, 1]   # image down = toward the robot


def test_camera_turns_with_joint_1():
    rotation, translation = cameraModel.optical_pose(look_angles(0.16, 0.12, 0.14))
    heading = math.atan2(0.12, 0.16)
    # In the arm's own frame the camera is where it was at heading 0, whichever way the arm turns
    c, s = math.cos(heading), math.sin(heading)
    ahead = translation[0] * c + translation[1] * s
    left = -translation[0] * s + translation[1] * c
    assert abs(ahead - (0.25 + CAMERA_OFFSET[0])) < 0.004 and abs(left - CAMERA_OFFSET[1]) < 0.004, (ahead, left)


def test_tool_point_from_the_same_chain_is_the_ik_target():
    # Same joints the camera hangs off: if these disagreed with the IK, the camera would be misplaced too
    for x, y, z in [(0.20, 0.0, 0.14), (0.22, 0.05, 0.0), (0.15, -0.10, 0.10)]:
        angles = look_angles(x, y, z)
        R, p = cameraModel.camera_link_pose(angles)
        # the measured camera offset is not part of the URDF chain
        heading = math.radians(angles[0])
        p = p - cameraModel.rot_z(heading) @ [CAMERA_OFFSET[0], CAMERA_OFFSET[1], 0.0]
        # depth_cam_link back to link 4's tip: undo the mount, then go down the gripper
        link4 = R @ cameraModel.rot_x(-cameraModel.CAMERA_LINK_RPY[0]) @ cameraModel.rot_y(-cameraModel.CAMERA_LINK_RPY[1]) \
            @ cameraModel.rot_z(-cameraModel.CAMERA_LINK_RPY[2]) @ cameraModel.rot_z(-cameraModel.CAMERA_CONNECT_YAW)
        origin4 = p - link4 @ cameraModel.rot_z(cameraModel.CAMERA_CONNECT_YAW) @ [0, 0, cameraModel.CAMERA_LINK_Z] \
            - link4 @ cameraModel.CAMERA_CONNECT_XYZ
        tool = origin4 + link4 @ [0, 0, ik.l3 + ik.tool_link]
        # 0.24 deg per pulse of rounding and URDF links 0.6 mm shorter than the IK's: a few millimetres
        assert np.allclose(tool, [x, y, z], atol=0.004), (x, y, z, tool)


def test_blocks_are_located_from_color_and_the_model_alone():
    """No depth at all (the camera's minimum range, or the topic missing): the floor plane comes from the arm model."""
    rotation, translation = cameraModel.optical_pose(look_angles(*LOOK))
    plane = cameraModel.floor_plane(rotation, translation)
    for bx, by, yaw in [(0.25, 0.0, 0), (0.30, 0.08, 25), (0.28, -0.10, -30), (0.33, 0.02, 40)]:
        bgr, _ = render(rotation, translation, [(bx, by, yaw, (0, 0, 220))])
        det = locate_block(bgr, np.zeros(bgr.shape[:2]), 'red', (FX, FY, CX, CY), plane, rotation, translation, SIZE)
        assert det is not None, (bx, by)
        assert abs(det['x'] - bx) < 0.003 and abs(det['y'] - by) < 0.003, (bx, by, det)
        assert abs((det['yaw_deg'] - yaw + 45) % 90 - 45) < 4, (yaw, det)


def test_driver_extrinsic_moves_the_lens_sideways():
    angles = look_angles(*LOOK)
    _, nominal = cameraModel.optical_pose(angles)
    # color sensor 2 cm to the left of the housing origin (link y)
    _, shifted = cameraModel.optical_pose(angles, (cameraModel.OPTICAL_IN_LINK, [0.0, 0.02, 0.0]))
    assert np.allclose(shifted - nominal, [0.0, 0.02, 0.0], atol=1e-3), shifted - nominal


def test_extrinsic_sanity_check():
    assert cameraModel.plausible_extrinsic(cameraModel.OPTICAL_IN_LINK, [0.0, 0.0125, 0.0])
    assert not cameraModel.plausible_extrinsic(np.eye(3), [0.0, 0.0, 0.0])
    assert not cameraModel.plausible_extrinsic(cameraModel.OPTICAL_IN_LINK, [0.3, 0.0, 0.0])
