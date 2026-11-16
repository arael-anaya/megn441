import numpy as np

from rosbot_arm import inverseKinematics as ik


def forward(angles):
    """Planar forward kinematics for joints 1-4; returns (r, z, pitch) of the gripper tip."""
    a2, a3, a4 = np.deg2rad(angles[1] + 90.0), np.deg2rad(angles[2]), np.deg2rad(angles[3])
    r = (ik.l1 * np.cos(a2) + ik.l2 * np.cos(a2 + a3)
         + (ik.l3 + ik.tool_link) * np.cos(a2 + a3 + a4))
    z = (ik.l0 + ik.l1 * np.sin(a2) + ik.l2 * np.sin(a2 + a3)
         + (ik.l3 + ik.tool_link) * np.sin(a2 + a3 + a4))
    return r, z, a2 + a3 + a4


def test_ik_round_trips_through_forward_kinematics():
    for x, y, z in [(0.20, 0.0, 0.0), (0.15, -0.10, 0.02), (0.0, -0.20, 0.05)]:
        for constraint in (1, -1):
            angles = ik.solveIK(x, y, z, 0.0, -90.0, constraint)
            assert angles is not None
            r, zz, pitch = forward(angles)
            assert np.isclose(r, np.hypot(x, y), atol=1e-6)
            assert np.isclose(zz, z, atol=1e-6)
            assert np.isclose(pitch, np.deg2rad(-90.0), atol=1e-6)
            assert np.isclose(np.deg2rad(angles[0]), np.arctan2(y, x))


def test_roll_passes_through_to_joint_5():
    angles = ik.solveIK(0.2, 0.0, 0.0, 30.0, -90.0, 1)
    assert np.isclose(angles[4], 30.0)


def test_out_of_reach_returns_none():
    assert ik.solveIK(1.0, 0.0, 0.0, 0.0, -90.0, 1) is None


def test_solve_with_tilt_matches_straight_down_when_it_fits():
    pulses, pitch = ik.solveWithTilt(0.20, 0.0, 0.14, 0.0, -90.0)
    assert pitch == -90.0
    assert pulses == ik.solvePulses(0.20, 0.0, 0.14, 0.0, -90.0)


def test_solve_with_tilt_tilts_when_straight_down_fails():
    # Low and far: straight down leaves the wrist out of reach, a tilted gripper fits (same fallback as ik_node)
    for r in np.arange(0.15, 0.40, 0.01):
        if ik.solvePulses(r, 0.0, 0.0, 0.0, -90.0) is None:
            result = ik.solveWithTilt(r, 0.0, 0.0, 0.0, -90.0)
            if result is not None:
                assert result[1] > -90.0
                return
    raise AssertionError('no radius found where tilting helps')


def test_solve_with_tilt_none_when_out_of_reach():
    assert ik.solveWithTilt(1.0, 0.0, 0.0, 0.0, -90.0) is None
