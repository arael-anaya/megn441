"""Arm facts shared by every node that moves the arm, so each number exists once."""
import math

# Edge of the (cube) blocks, m: 1 1/8 in. The vision node sizes blobs with it and the stacker stacks with it.
BLOCK_SIZE = 0.028575

# Gripper servo (id 10) pulses
GRIPPER_ID = 10
GRIPPER_OPEN = 300
GRIPPER_CLOSED = 600
# Closing on nothing, the gripper servo gets (nearly) to GRIPPER_CLOSED; a block stops it short. A position read back
# within this many pulses of GRIPPER_CLOSED means the gripper is empty. ik_node logs the reading after every grasp:
# set this halfway between the empty reading and the reading with a block in the fingers.
GRIPPER_EMPTY_MARGIN = 25

# Carry pose (tool point, IK frame) while driving
STOW_XYZ = [0.1, 0.0, 0.24]
STOW_PITCH = -60.0

# Tool target (gripper straight down) the camera looks at the floor from. With the URDF camera mount this puts the
# camera at about (0.25, 0, 0.25), 0.22 m above the block tops: as low as the depth camera's ~0.2 m minimum range
# allows. The block calibration is only valid for the look pose it was measured from.
LOOK_XYZ = [0.20, 0.0, 0.14]

# Tool point height (m) the gripper closes at to grab a block lying on the floor. Used by every pick and by the
# calibration, so they all meet the floor the same way. It must NOT press the fingertips into the floor: a pressed arm
# flexes forward, by a different amount with a block in the fingers than without, and the gripper then comes down
# millimetres ahead of a block it set down itself. Fingertips a few millimetres clear of the floor is right.
GRASP_Z = 0.0

# Without a calibration: the gripper lands this far (m) further from the base than the IK target, measured on the
# robot. The calibration (calibrate_node) measures the same thing per position and replaces this.
REACH_ERROR = 0.013

# The real camera is further out and further left than the URDF chain puts it, in the arm's own frame (x along the
# arm's heading, y to its left), m. Measured on the robot with no calibration: a block straight ahead was picked 31.75 mm
# short and 12.7 mm to the right, the same every time, and the stack seen with the arm turned 90 degrees was short and
# right by about the same amounts. Both mean the camera reads blocks too close and too far right. Everything the camera
# measures is shifted by this, so it also moves the look-at-the-stack pose.
CAMERA_OFFSET = (0.03175, 0.0127)
# How far (m) the camera's centre is ahead of the tool point, along the arm
CAMERA_AHEAD = 0.05 + CAMERA_OFFSET[0]

# Which way a block's yaw turns joint 5. The fingers close across the arm's heading; joint 5 turns them clockwise
# seen from above for a positive roll (opposite to joint 1), so a block turned counter-clockwise needs a negative
# roll. Flip to +1.0 if the fingers end up turned the wrong way on a block sitting at an angle.
BLOCK_YAW_SIGN = -1.0
# Direction joint 5 turns for a positive roll command. -1 after the first test on the robot: the roll worked out from
# the heading and the block's yaw came out turned the wrong way, so the whole joint 5 command is negated.
ROLL_SIGN = -1.0

# Added to every pick target (m, arm frame x forward, y left), on top of the camera reading and the reach correction.
# Half a block back: the gripper was closing half a block too far forward.
PICK_OFFSET = (-BLOCK_SIZE / 2, 0.0)


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
