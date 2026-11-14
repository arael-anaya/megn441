"""Measure how the vision node's block positions map to IK targets, using the arm itself.

    ros2 launch rosbot_arm stack_launch.py mode:=calibrate

The arm goes down to grab height at the first point with the gripper open: slide the calibration block in between the
fingers. It grabs the block, then for every point sets it down there, backs off to the look pose, records where the camera sees it and
picks it up again. Setting down and picking up at the same commanded pose lands the block exactly under the gripper,
so no ruler is needed and the arm's own reach and floor-height errors are part of what gets measured. The points are
sent to the IK as they are, so the fit maps a camera position straight to the IK target that grabs a block there. It
is saved to calibration.CALIBRATION_FILE and stacker_node / nav_manager / pick_node apply it to every detection.
"""
import numpy as np
import rclpy

from rosbot_arm import calibration
from rosbot_arm.armConfig import GRIPPER_OPEN, GRIPPER_CLOSED
from rosbot_arm.stacker_node import StackerNode, UnreachableError


# IK targets (m) to set the block down at; keep them inside the look pose's view. Gripper-down picks reach about
# 0.25 m from joint 1, so nothing past that. The first point is where the arm waits for the block to be put in, so
# it is the far, centre one.
POINTS = [(0.23, 0.0), (0.23, -0.05), (0.23, 0.05),
          (0.20, -0.05), (0.20, 0.0), (0.20, 0.05),
          (0.17, -0.05), (0.17, 0.0), (0.17, 0.05)]
COLOR = 'red'
PLACE_BLOCK_TIME = 10.0
# A set-down block should be seen within this many metres of where it was put (the camera error is a few cm). More
# than that means the block was dropped or bumped, or the camera saw something else: stop the run.
MAX_ERROR = 0.08
# Refuse to save a fit whose worst point is further off than this (m)
MAX_RESIDUAL = 0.015


class CalibrateNode(StackerNode):
    def __init__(self):
        super().__init__('calibrate_node', applyCalibration=False)
        # True: the arm never grabs the block. At every point it goes down with the gripper open and waits while you
        # slide the block in between the fingers, then backs off to look. Works even if the grasp itself is unreliable.
        self.declare_parameter('calib_manual', False)
        self.manual = self.get_parameter('calib_manual').value

    def visit(self, x, y, atBottom):
        self.touchDown(x, y, self.graspZ, 0.0, atBottom, move=self.moveRaw)

    def waitForBlock(self, x, y, then):
        self.get_logger().warn(
            f'Slide the {COLOR} block on the floor in between the open fingers at ({x:.3f}, {y:.3f}), square to '
            f'them. {then} in {PLACE_BLOCK_TIME:.0f} s.')
        self.wait(PLACE_BLOCK_TIME)

    def run(self):
        if not self.waitForSubscribers():
            return

        def grab():
            self.moveGripper(GRIPPER_CLOSED)

        def release():
            self.moveGripper(GRIPPER_OPEN)

        # Wait down at grab height with the gripper open, so the block just slides in between the fingers
        x0, y0 = POINTS[0]

        def loadBlock():
            self.waitForBlock(x0, y0, 'Grabbing')
            grab()

        release()
        if not self.manual:
            self.visit(x0, y0, loadBlock)

        commanded, detected = [], []
        try:
            for i, (x, y) in enumerate(POINTS):
                self.visit(x, y, (lambda: self.waitForBlock(x, y, 'Looking')) if self.manual else release)
                self.moveToLookPose()
                d = self.detectBlocks([COLOR]).get(COLOR)
                if d is not None:
                    miss = np.hypot(d.x - x, d.y - y)
                    if miss > MAX_ERROR:
                        self.get_logger().error(
                            f'Point {i + 1}: camera says ({d.x:.3f}, {d.y:.3f}), {100 * miss:.0f} cm from where the '
                            f'block was set down ({x:.3f}, {y:.3f}). The block was dropped or bumped, or the camera '
                            'saw something else. Stopping; nothing saved.')
                        return
                    commanded.append((x, y))
                    detected.append((d.x, d.y))
                    self.get_logger().info(
                        f'Point {i + 1}/{len(POINTS)}: set down at ({x:.3f}, {y:.3f}), camera says '
                        f'({d.x:.3f}, {d.y:.3f}), error ({1000 * (d.x - x):+.0f}, {1000 * (d.y - y):+.0f}) mm')
                else:
                    self.get_logger().warn(f'Point {i + 1}: block not seen at ({x:.3f}, {y:.3f}); skipping it.')
                if i + 1 < len(POINTS) and not self.manual:
                    self.visit(x, y, grab)
        except UnreachableError as e:
            self.get_logger().error(f'{e}. Fitting what was measured so far.')

        if len(commanded) < 3:
            self.get_logger().error(f'Only {len(commanded)} points measured; need 3. Nothing saved.')
            return
        try:
            A, b, residuals = calibration.fit_affine(detected, commanded)
        except ValueError as e:
            self.get_logger().error(f'Fit failed: {e}')
            return

        scales = np.linalg.svd(A, compute_uv=False)
        if scales.min() < 0.85 or scales.max() > 1.15:
            self.get_logger().error(
                f'The fit stretches the camera by {scales[0]:.2f} / {scales[1]:.2f}; a camera that measures in metres '
                'is within a few percent of 1. The points are inconsistent (block placed by hand somewhere other than '
                'under the gripper, a bumped block, a bad detection). NOT saved. '
                f'Per-point residuals (mm): {np.round(1000 * residuals).astype(int).tolist()}')
            return
        if residuals.max() > MAX_RESIDUAL:
            self.get_logger().error(
                f'Worst point is still {1000 * residuals.max():.0f} mm off after the fit (limit '
                f'{1000 * MAX_RESIDUAL:.0f} mm), so the error is not a smooth shift / rotation / scale. '
                f'Per-point residuals (mm): {np.round(1000 * residuals).astype(int).tolist()}. '
                'NOT saved; the existing calibration file is left as it was.')
            return
        calibration.save(calibration.CALIBRATION_FILE, A, b, self.lookXyz, residuals)
        raw = np.linalg.norm(np.subtract(detected, commanded), axis=1)
        self.get_logger().info(
            f'Calibration saved to {calibration.CALIBRATION_FILE}. Camera vs arm: {calibration.describe(A, b)}. '
            f'Error before: mean {1000 * raw.mean():.0f} mm, max {1000 * raw.max():.0f} mm; '
            f'after: mean {1000 * residuals.mean():.1f} mm, max {1000 * residuals.max():.1f} mm.')
        if residuals.max() > 0.008:
            self.get_logger().warn(
                'Some points are still off by more than 8 mm after the fit: the error is not a smooth '
                'rotation / scale / shift. Check those points in the log above (block bumped, bad detection).')


def main():
    rclpy.init()
    node = CalibrateNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
