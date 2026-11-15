import math

from rosbot_arm.armConfig import ROLL_SIGN, gripRoll, towardBase


def finger_axis_deg(x, y, rollDeg):
    """World direction the fingers close along, for a gripper pointing straight down. Joint 1 turns the gripper
    with the arm's heading and joint 5 turns it by ROLL_SIGN * the roll command, counter-clockwise from above for a
    positive ROLL_SIGN * roll."""
    return math.degrees(math.atan2(y, x)) + 90.0 - ROLL_SIGN * rollDeg


def test_fingers_line_up_with_the_block_edges_anywhere_in_reach():
    for x, y in [(0.20, 0.0), (0.18, 0.10), (0.15, -0.12), (0.0, -0.20)]:
        for yawDeg in (-40, -20, 0, 15, 30, 44):
            roll = gripRoll(x, y, math.radians(yawDeg))
            assert -45.0 <= roll < 45.0
            off = (finger_axis_deg(x, y, roll) - yawDeg + 45.0) % 90.0 - 45.0
            assert abs(off) < 1e-6, (x, y, yawDeg, roll, off)


def test_toward_base_shortens_the_radius_only():
    x, y = towardBase(0.12, -0.16, 0.013)
    assert abs(math.hypot(x, y) - (0.20 - 0.013)) < 1e-9
    assert abs(math.atan2(y, x) - math.atan2(-0.16, 0.12)) < 1e-9
    assert towardBase(0.005, 0.0, 0.013) == (0.005, 0.0)
