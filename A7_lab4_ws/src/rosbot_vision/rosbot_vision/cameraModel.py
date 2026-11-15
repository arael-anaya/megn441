"""Where the camera is, worked out from the servo pulses the arm was sent to. No TF, no /joint_states.

The robot's own bringup (ros2_ws, started at boot) also publishes /joint_states and runs a joint_state_publisher and a
robot_state_publisher. Its arm angles have the opposite sign to ours and its joint_state_publisher re-stamps whichever
value it heard last 20 times a second, so base_footprint <- camera in TF can be a mirrored or minutes-old arm that
still looks fresh. Every block then lands centimetres (or a whole arm) away from where it is.

The camera is bolted to link 4, so the commanded pulses and the URDF link lengths give its pose directly. The numbers
are from jetrover_description (car_acker, connect, arm and depth_camera xacro).

Frames: the arm / IK frame has its origin on the joint 1 axis at floor level, x forward, y left, z up. The optical
frame is the usual camera one: x right, y down, z out of the lens.
"""
import math

import numpy as np

from rosbot_arm.armConfig import CAMERA_OFFSET

JOINT1_Z = 0.05 + 0.0654867643253843 + 0.0774026880954513
JOINT2_Z = 0.0338648012164686
JOINT3_Z = 0.129416446394797
JOINT4_Z = 0.129444631186569

CAMERA_CONNECT_XYZ = np.array([-0.0507060266977644, 0.0, 0.0505384841187764])
CAMERA_CONNECT_YAW = -math.pi / 2
CAMERA_LINK_Z = 0.014475
CAMERA_LINK_RPY = (math.pi, -math.pi / 2, -math.pi / 2)

OPTICAL_IN_LINK = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])

PULSE_LIMITS = {2: (125, 775)}
ARM_JOINTS = (1, 2, 3, 4, 5)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def clamp_pulse(jointId, pulse):
    lo, hi = PULSE_LIMITS.get(jointId, (0, 1000))
    return min(max(int(pulse), lo), hi)


def pulses_to_angles(pulses):
    """{joint id: pulse} for joints 1-5 -> servo angles in degrees, the convention inverseKinematics uses."""
    return np.array([(clamp_pulse(i, pulses[i]) - 500) * 120.0 / 500.0 for i in ARM_JOINTS])


def camera_link_pose(servoDeg):
    """(rotation, position) of depth_cam_link in the arm frame for servo angles 1-5 in degrees.

    Servo 1 turns the arm counter-clockwise seen from above (IK: angle = atan2(y, x)). Servos 2-4 lean the arm
    forward for negative angles, which is a positive turn about the URDF's +y joint axes.
    """
    j = np.radians(np.asarray(servoDeg, dtype=float))
    R = rot_z(j[0])
    p = np.array([0.0, 0.0, JOINT1_Z])
    for length, angle in ((JOINT2_Z, j[1]), (JOINT3_Z, j[2]), (JOINT4_Z, j[3])):
        p = p + R @ np.array([0.0, 0.0, length])
        R = R @ rot_y(-angle)
    p = p + R @ CAMERA_CONNECT_XYZ
    p = p + rot_z(j[0]) @ np.array([CAMERA_OFFSET[0], CAMERA_OFFSET[1], 0.0])
    R = R @ rot_z(CAMERA_CONNECT_YAW)
    p = p + R @ np.array([0.0, 0.0, CAMERA_LINK_Z])
    roll, pitch, yaw = CAMERA_LINK_RPY
    R = R @ rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)
    return R, p


def optical_pose(servoDeg, extrinsic=None):
    """Camera optical frame -> arm frame as (rotation, translation).

    extrinsic is the camera driver's optical frame in depth_cam_link as (rotation, translation); it carries the few
    centimetres between the color sensor and the housing. Without it the lens is taken to be at the link origin.
    """
    R, p = camera_link_pose(servoDeg)
    if extrinsic is None:
        return R @ OPTICAL_IN_LINK, p
    extR, extT = extrinsic
    return R @ np.asarray(extR, dtype=float), p + R @ np.asarray(extT, dtype=float)


def plausible_extrinsic(rotation, translation, maxAngleDeg=15.0, maxOffset=0.08):
    """True if a looked-up optical <- link transform is a camera's: near the standard optical rotation, lens within
    a few centimetres of the housing. Guards against a TF tree that connects the two frames some other way."""
    relative = np.asarray(rotation, dtype=float).T @ OPTICAL_IN_LINK
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(relative) - 1.0) / 2.0))))
    return angle <= maxAngleDeg and float(np.linalg.norm(translation)) <= maxOffset


def floor_plane(rotation, translation):
    """The floor (z = 0 in the arm frame) in camera coordinates as (normal, distance), normal pointing from the
    camera to the floor. Same form as depthUtils.fit_table_plane measures from depth."""
    return -np.asarray(rotation)[2, :], float(translation[2])
