import math

from nav_manager.geometry import standoff_pose


def close(a, b):
    return all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(a, b))


def test_target_straight_ahead():
    # Block 0.5 m ahead of the arm: stop 0.17 m short of it, same heading
    assert close(standoff_pose((1.0, 2.0, 0.0), (0.5, 0.0), 0.17), [1.33, 2.0, 0.0])


def test_target_to_the_left_turns_toward_it():
    pose = standoff_pose((0.0, 0.0, 0.0), (0.0, 0.6), 0.2)
    assert close(pose, [0.0, 0.4, math.pi / 2])


def test_robot_rotated_in_map():
    pose = standoff_pose((0.0, 0.0, math.pi / 2), (0.5, 0.0), 0.17)  # target is at map (0, 0.5)
    assert close(pose, [0.0, 0.33, math.pi / 2])


def test_arm_offset_moves_the_base_not_the_arm_axis():
    pose = standoff_pose((0.0, 0.0, 0.0), (0.5, 0.0), 0.17, arm_offset=(0.1, 0.0))
    # arm axis ends 0.17 m short of the target (which is at base x = 0.6), so the base is 0.1 further back
    assert close(pose, [0.33, 0.0, 0.0])
