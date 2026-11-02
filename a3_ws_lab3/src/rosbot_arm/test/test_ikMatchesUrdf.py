"""The IK and TF must describe the same arm: TF places the camera with the URDF chain fed by arm_state_publisher's
joint mapping, the IK places the gripper with its own link lengths. If they disagree, every block position is off.

URDF numbers are from jetrover_description (connect.urdf.xacro, arm.urdf.xacro); base_footprint -> base_link z is
the part of ik.l0 below joint 1.
"""
import numpy as np

from rosbot_arm import inverseKinematics as ik

JOINT1_XYZ = (0.0251328065010765, 0.0, ik.l0 - 0.0338648012164686)  # base_footprint -> joint1, via the IK base height
JOINT2_Z = 0.0338648012164686
JOINT3_Z = 0.129416446394797
JOINT4_Z = 0.129444631186569
JOINT5_Z = 0.0544833339503674

# arm_state_publisher: urdf = (-j1, -j2, -j3, joint4_sign * j4, -j5)
JOINT4_SIGN = -1.0


def rot(axis, angle):
    axis = np.asarray(axis, float)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def urdf_tool(servoDeg):
    """Tool point (IK frame: joint 1 axis at the origin, z = ground) and gripper axis from the URDF chain."""
    j = np.radians(servoDeg)
    u = [-j[0], -j[1], -j[2], JOINT4_SIGN * j[3], -j[4]]
    R = rot([0, 0, -1], u[0])
    p = np.array([0.0, 0.0, JOINT1_XYZ[2]])
    for length, angle in [(JOINT2_Z, u[1]), (JOINT3_Z, u[2]), (JOINT4_Z, u[3])]:
        p = p + R @ [0, 0, length]
        R = R @ rot([0, 1, 0], angle)
    p = p + R @ [0, 0, JOINT5_Z + ik.tool_link]
    return p, R[:, 2]


def test_ik_target_is_where_the_urdf_puts_the_tool():
    for x, y, z in [(0.18, 0.0, 0.03), (0.22, 0.05, 0.03), (0.15, -0.10, 0.10), (0.14, 0.0, 0.20)]:
        angles = ik.solveIK(x, y, z, 0.0, -90.0, 1)
        assert angles is not None
        tool, axis = urdf_tool(angles)
        # URDF links are 0.6 mm shorter than the IK's 0.130 m, so allow a couple of millimetres
        assert np.allclose(tool, [x, y, z], atol=0.003), (x, y, z, tool)
        assert np.allclose(axis, [0, 0, -1], atol=0.01), axis
