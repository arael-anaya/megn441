"""Plain geometry for nav_manager, kept free of ROS imports so it can be unit tested anywhere."""
import math


def standoff_pose(robot, target_arm, standoff, arm_offset=(0.0, 0.0)):
    """Map pose [x, y, yaw] to drive to so a target the arm cannot reach ends up `standoff` m ahead of the arm.

    robot:      (x, y, yaw) of base_link in the map
    target_arm: (x, y) of the target in the arm frame (x forward from the joint 1 axis, y left), as the camera reports it
    arm_offset: (x, y) of the joint 1 axis in base_link
    The robot faces the target when it stops, so the arm's heading lines up with it.
    """
    rx, ry, ryaw = robot
    c, s = math.cos(ryaw), math.sin(ryaw)
    ox, oy = arm_offset
    # Arm frame and base_link share their orientation, so only the offset separates them
    tx, ty = target_arm[0] + ox, target_arm[1] + oy
    target = (rx + c * tx - s * ty, ry + s * tx + c * ty)
    axis = (rx + c * ox - s * oy, ry + s * ox + c * oy)
    heading = math.atan2(target[1] - axis[1], target[0] - axis[0])
    ch, sh = math.cos(heading), math.sin(heading)
    gx, gy = target[0] - standoff * ch, target[1] - standoff * sh  # where the arm axis should end up
    return [gx - (ch * ox - sh * oy), gy - (sh * ox + ch * oy), heading]
