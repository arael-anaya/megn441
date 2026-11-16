"""Measure how the vision node's block positions map to IK targets, using the arm itself.

    ros2 launch rosbot_arm stack_launch.py mode:=calibrate

The arm goes down to grab height at the first point with the gripper open: slide the calibration block in between the
fingers. It grabs the block, then for every point sets it down there, backs off to the look pose, records where the camera sees it and
picks it up again. Setting down and picking up at the same commanded pose lands the block exactly under the gripper,
so no ruler is needed and the arm's own reach and floor-height errors are part of what gets measured. The points are
sent to the IK as they are, so the fit maps a camera position straight to the IK target that grabs a block there. It
is saved to calibration.CALIBRATION_FILE and stacker_node / nav_manager / pick_node apply it to every detection.
"""
import math

import numpy as np
import rclpy

from rosbot_arm import calibration
from rosbot_arm.armConfig import GRASP_Z, GRIPPER_OPEN, GRIPPER_CLOSED
from rosbot_arm.stacker_node import StackerNode, UnreachableError


# The nearest row used to be x = 0.17: the servo bumped the chassis there and the look pose cannot see it, so vision
# raised the camera and the point was measured from a different pose. All rows are now within the look pose's view.
POINTS = [(0.26, 0.0), (0.26, -0.05), (0.26, 0.05),
          (0.23, -0.05), (0.23, 0.0), (0.23, 0.05),
          (0.20, -0.05), (0.20, 0.0), (0.20, 0.05)]
COLOR = 'red'
PLACE_BLOCK_TIME = 10.0
MAX_ERROR = 0.08
MAX_RESIDUAL = 0.015
MIN_SCALE, MAX_SCALE = 0.5, 2.0

# Grip tuning. A robot's gripper servo and arm differ from the next one's, so the first thing calibration does is find
# a grasp that holds on THIS robot: close on the block, lift, and if it slipped out try a higher grasp and a firmer
# squeeze. What worked is saved with the camera fit and used by every other node.
SQUEEZE = 60            # pulses past where the fingers stop on the block
SQUEEZE_STEP = 30       # extra pulses after a block slipped
GRIP_MAX = 900          # never command the gripper servo past this
Z_STEP = 0.005          # grasp higher by this much after a block slipped
GRIP_TRIES = 4
REGRAB_TRIES = 3
HOLD_TIME = 1.5         # lift, then wait this long before judging whether the block is still held


class CalibrateNode(StackerNode):
    def __init__(self):
        super().__init__('calibrate_node', applyCalibration=False)
        self.declare_parameter('calib_manual', False)
        self.manual = self.get_parameter('calib_manual').value
        self.contact = None

        # Calibration measures these, so it must not start from what an earlier run saved (a bad earlier run would
        # make every later one start in the wrong place). It starts from armConfig, or grasp_z if given.
        self.gripOpen = GRIPPER_OPEN
        self.gripClosed = GRIPPER_CLOSED
        given = float(self.get_parameter('grasp_z').value)
        self.graspZ = GRASP_Z if math.isnan(given) else given
        self.get_logger().info(
            f'Calibrating from grasp z {self.graspZ:.3f} m, gripper open {self.gripOpen} / closed {self.gripClosed}.')

    def visit(self, x, y, atBottom):
        self.touchDown(x, y, self.graspZ, 0.0, atBottom, move=self.moveRaw)

    def waitForBlock(self, x, y, then):
        self.get_logger().warn(
            f'Slide the {COLOR} block on the floor in between the open fingers at ({x:.3f}, {y:.3f}), square to '
            f'them. {then} in {PLACE_BLOCK_TIME:.0f} s.')
        self.wait(PLACE_BLOCK_TIME)

    def grab(self):
        self.moveGripper(self.gripClosed)
        self.contact = self.gripperPulse()

    def release(self):
        self.moveGripper(self.gripOpen)

    def saveGrip(self):
        calibration.saveRobot(calibration.CALIBRATION_FILE, self.gripClosed, self.gripOpen, self.graspZ)

    def blockHeld(self):
        """Lifted and still holding the block after HOLD_TIME? (True if the servo cannot be read.)"""
        self.wait(HOLD_TIME)
        held = self.isHolding()
        if held is None:
            self.get_logger().warn('Could not read the gripper servo; assuming the block is held.')
            return True
        return held

    def tuneGrip(self):
        """Grab the block at the first point and lift it, raising the grasp and squeezing harder until it stays in
        the gripper. Returns False if no setting held it."""
        x0, y0 = POINTS[0]

        def loadBlock():
            self.waitForBlock(x0, y0, 'Grabbing')
            self.grab()

        for attempt in range(GRIP_TRIES):
            self.release()
            self.visit(x0, y0, loadBlock)
            empty = self.contact is None or not self.isHolding()
            if empty:
                self.get_logger().warn('The gripper closed on nothing: no block between the fingers. Trying again '
                                       'at the same height.')
                continue
            if self.blockHeld():
                self.gripClosed = int(min(GRIP_MAX, max(self.gripClosed, self.contact + SQUEEZE)))
                self.moveGripper(self.gripClosed)
                self.saveGrip()
                self.get_logger().info(
                    f'Grip works: grasp z {self.graspZ:.3f} m, fingers stopped at {self.contact}, gripper closed '
                    f'command {self.gripClosed}. Saved.')
                return True
            self.graspZ += Z_STEP
            self.gripClosed = int(min(GRIP_MAX, self.gripClosed + SQUEEZE_STEP))
            self.get_logger().warn(
                f'The block slipped out while lifting (attempt {attempt + 1}/{GRIP_TRIES}). Next: grasp z '
                f'{self.graspZ:.3f} m, gripper closed command {self.gripClosed}. Pick up the block if it fell and '
                'put it back between the fingers when asked.')
        self.get_logger().error('No grasp height / squeeze held the block. Check the gripper fingers and the block '
                                'size, or the servo (is it weak or loose?). Nothing saved.')
        return False

    def pickUp(self, x, y, placedByHand=False):
        """Grab the block at (x, y) and lift it. If it slips out, ask for it back, squeeze harder and try again, so
        a drop does not end the calibration. False if it still cannot be held."""
        for attempt in range(REGRAB_TRIES + 1):
            if attempt == 0 and not placedByHand:
                atBottom = self.grab
            else:
                def atBottom():
                    self.waitForBlock(x, y, 'Grabbing')
                    self.grab()
            self.visit(x, y, atBottom)
            if self.blockHeld():
                return True
            self.gripClosed = int(min(GRIP_MAX, self.gripClosed + SQUEEZE_STEP))
            self.saveGrip()
            self.get_logger().warn(
                f'The block slipped out lifting from ({x:.3f}, {y:.3f}). Squeezing harder (closed command '
                f'{self.gripClosed}); put the block back between the fingers when asked '
                f'(retry {attempt + 1}/{REGRAB_TRIES}).')
            self.release()
        return False

    def run(self):
        if not self.waitForSubscribers():
            return

        self.release()
        if not self.manual and not self.tuneGrip():
            return

        commanded, detected = [], []
        try:
            for i, (x, y) in enumerate(POINTS):
                self.visit(x, y, (lambda: self.waitForBlock(x, y, 'Looking')) if self.manual else self.release)
                self.moveToLookPose()
                # no_raise: the fit is only valid for detections made from the look pose, so a block the camera cannot
                # see from there is skipped, not viewed from a raised camera.
                d = self.detectBlocks([COLOR], noRaise=True).get(COLOR)
                moved = False
                if d is not None:
                    miss = np.hypot(d.x - x, d.y - y)
                    if miss > MAX_ERROR:
                        self.get_logger().warn(
                            f'Point {i + 1}: camera says ({d.x:.3f}, {d.y:.3f}), {100 * miss:.0f} cm from where the '
                            f'block was set down ({x:.3f}, {y:.3f}). The block was bumped or the camera saw '
                            'something else. Skipping this point; put the block back when asked.')
                        d = None
                        moved = True
                if d is not None:
                    commanded.append((x, y))
                    detected.append((d.x, d.y))
                    self.get_logger().info(
                        f'Point {i + 1}/{len(POINTS)}: set down at ({x:.3f}, {y:.3f}), camera says '
                        f'({d.x:.3f}, {d.y:.3f}), error ({1000 * (d.x - x):+.0f}, {1000 * (d.y - y):+.0f}) mm')
                else:
                    self.get_logger().warn(f'Point {i + 1}: block not seen at ({x:.3f}, {y:.3f}); skipping it.')
                if i + 1 < len(POINTS) and not self.manual:
                    if not self.pickUp(x, y, placedByHand=moved):
                        self.get_logger().error(
                            'Could not keep hold of the block even after squeezing harder. Fitting what was '
                            'measured so far.')
                        break
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
        # How consistent the points are (residuals, checked below) decides whether the fit is trusted. The scale only
        # catches nonsense: it differs from 1 by tens of percent on a robot whose camera height or tilt differs from
        # the reference robot's, and that is a real correction the fit should make, not an error.
        if scales.min() < MIN_SCALE or scales.max() > MAX_SCALE:
            self.get_logger().error(
                f'The fit stretches the camera by {scales[0]:.2f} / {scales[1]:.2f}, further than any real camera '
                f'difference ({MIN_SCALE} to {MAX_SCALE}). The points are inconsistent (block placed by hand somewhere '
                'other than under the gripper, a bumped block, a bad detection). NOT saved. '
                f'Per-point residuals (mm): {np.round(1000 * residuals).astype(int).tolist()}')
            return
        if scales.min() < 0.85 or scales.max() > 1.15:
            xs, ys = np.transpose(detected)
            self.get_logger().warn(
                f'The fit stretches the camera by {scales[0]:.2f} / {scales[1]:.2f}. The points agree with each other '
                f'(worst residual {1000 * residuals.max():.0f} mm), so it is saved, but a camera this far from 1 means '
                'its height or tilt in vision_node is off on this robot (look for "Floor from depth" / "depth plane '
                f'rejected" in the vision log). Only trust picks inside the measured area: x {xs.min():.2f} to '
                f'{xs.max():.2f}, y {ys.min():.2f} to {ys.max():.2f} m as the camera sees them.')
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
