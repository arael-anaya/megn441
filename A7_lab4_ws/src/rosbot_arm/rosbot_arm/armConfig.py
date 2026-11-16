"""Arm facts shared by every node that moves the arm, so each number exists once."""
import math

BLOCK_SIZE = 0.028575

GRIPPER_ID = 10
GRIPPER_OPEN = 300
GRIPPER_CLOSED = 600
GRIPPER_EMPTY_MARGIN = 25

STOW_XYZ = [0.1, 0.0, 0.24]
STOW_PITCH = -60.0

LOOK_XYZ = [0.20, 0.0, 0.14]
LOOK_PITCH = -60.0

GRASP_Z = 0.0

REACH_ERROR = 0.013

CAMERA_OFFSET = (0.03175, 0.0127)
CAMERA_AHEAD = 0.05 + CAMERA_OFFSET[0]

BLOCK_YAW_SIGN = -1.0
ROLL_SIGN = -1.0

PICK_OFFSET = (-BLOCK_SIZE / 2, 0.0)

# Where the joint 1 axis is relative to base_link (m, x forward, y left): the same point.
ARM_BASE_OFFSET = (0.0, 0.0)
# How far (m) ahead of the joint 1 axis nav_manager parks the robot from a block the arm cannot reach from where it
# is: 6.5-7 in, comfortably inside the arm's reach for a floor pick.
ARM_STANDOFF = 0.1


def towardBase(x, y, distance):
    """(x, y) moved `distance` closer to the joint 1 axis."""
    radius = math.hypot(x, y)
    if radius <= distance + 1e-6:
        return x, y
    scale = (radius - distance) / radius
    return x * scale, y * scale


def gripRoll(x, y, blockYawRad):
    """Joint 5 angle (deg) that lines the fingers up with the sides of a block at (x, y) whose edges point along
    blockYawRad in the arm frame.

    Joint 1 swings the whole gripper round by the arm's heading, so joint 5 has to take that out again and then
    turn to the block's own yaw (ROLL_SIGN and BLOCK_YAW_SIGN are the measured signs). A square block looks the same
    every 90 degrees, so the result is wrapped into [-45, 45).
    """
    roll = ROLL_SIGN * (math.degrees(math.atan2(y, x)) + BLOCK_YAW_SIGN * math.degrees(blockYawRad))
    return ((roll + 45.0) % 90.0) - 45.0
