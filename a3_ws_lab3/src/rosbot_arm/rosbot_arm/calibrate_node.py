"""Measure how the vision node's block positions map to IK targets, using the arm itself.

    ros2 launch rosbot_arm stack_launch.py mode:=calibrate

The arm goes down to grab height at the first point with the gripper open: slide the calibration block in between the
fingers. It grabs the block, then for every point sets it down there, backs off to the look pose, records where the camera sees it and
picks it up again. Setting down and picking up at the same commanded pose lands the block exactly under the gripper,
so no ruler is needed and the arm's own floor-height error is part of what gets measured. The fit is saved to
calibration_file and stacker_node applies it to every detection.
"""
import numpy as np
import rclpy

from rosbot_arm import calibration
from rosbot_arm.stacker_node import StackerNode, UnreachableError


class CalibrateNode(StackerNode):
    def __init__(self):
        super().__init__('calibrate_node', applyCalibration=False)
        # IK targets [x0, y0, x1, y1, ...] (m) to set the block down at; keep them inside the look pose's view.
        # Gripper-down picks reach about 0.255 m from joint 1, so nothing past that. The first point is where the
        # arm waits for you to put the block in, so it is the far, centre one.
        self.declare_parameter('calib_points', [0.23, 0.0, 0.23, -0.05, 0.23, 0.05,
                                                0.20, -0.05, 0.20, 0.0, 0.20, 0.05,
                                                0.17, -0.05, 0.17, 0.0, 0.17, 0.05])
        self.declare_parameter('calib_color', 'red')
        self.declare_parameter('place_block_time', 10.0)

        flat = [float(v) for v in self.get_parameter('calib_points').value]
        self.points = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.color = self.get_parameter('calib_color').value
        self.placeBlockTime = float(self.get_parameter('place_block_time').value)

    def grab(self, x, y):
        self.moveArm(x, y, self.graspZ + self.hover, 0.0)
        self.moveArm(x, y, self.graspZ, 0.0)
        self.moveGripper(self.gripperClosed)
        self.moveArm(x, y, self.graspZ + self.hover, 0.0)

    def release(self, x, y):
        self.moveArm(x, y, self.graspZ + self.hover, 0.0)
        self.moveArm(x, y, self.graspZ, 0.0)
        self.moveGripper(self.gripperOpen)
        self.moveArm(x, y, self.graspZ + self.hover, 0.0)

    def run(self):
        if not self.waitForSubscribers():
            self.get_logger().error('ik_node / arm_control are not running (nobody on inverse_topic or gripper_topic).')
            return
        if len(self.points) < 3:
            self.get_logger().error('calib_points needs at least 3 (x, y) points.')
            return

        # Wait down at grab height with the gripper open, so the block just slides in between the fingers
        x0, y0 = self.points[0]
        self.moveGripper(self.gripperOpen)
        self.moveArm(x0, y0, self.graspZ + self.hover, 0.0)
        self.moveArm(x0, y0, self.graspZ, 0.0)
        self.get_logger().warn(
            f'Slide the {self.color} block on the floor in between the open fingers, square to them. '
            f'Grabbing in {self.placeBlockTime:.0f} s.')
        self.wait(self.placeBlockTime)
        self.moveGripper(self.gripperClosed)
        self.moveArm(x0, y0, self.graspZ + self.hover, 0.0)

        commanded, detected = [], []
        try:
            for i, (x, y) in enumerate(self.points):
                self.release(x, y)
                self.moveToLookPose()
                seen = self.detectBlocks([self.color])
                if self.color in seen:
                    d = seen[self.color]
                    commanded.append((x, y))
                    detected.append((d.x, d.y))
                    self.get_logger().info(
                        f'Point {i + 1}/{len(self.points)}: set down at ({x:.3f}, {y:.3f}), camera says '
                        f'({d.x:.3f}, {d.y:.3f}), error ({1000 * (d.x - x):+.0f}, {1000 * (d.y - y):+.0f}) mm')
                else:
                    self.get_logger().warn(f'Point {i + 1}: block not seen at ({x:.3f}, {y:.3f}); skipping it.')
                if i + 1 < len(self.points):
                    self.grab(x, y)
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

        calibration.save(self.calibrationFile, A, b, self.lookXyz, residuals)
        raw = np.linalg.norm(np.subtract(detected, commanded), axis=1)
        self.get_logger().info(
            f'Calibration saved to {self.calibrationFile}. Camera vs arm: {calibration.describe(A, b)}. '
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
