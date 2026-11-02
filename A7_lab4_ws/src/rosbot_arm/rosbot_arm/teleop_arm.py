import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from rosbot_msgs.msg import TargetPose, GripperControl


class TeleopArm(Node):
    """Joystick shifts a target pose that is streamed to the IK node."""

    def __init__(self):
        super().__init__('teleop_arm')

        self.declare_parameter('start_xyz', [0.18, 0.0, 0.20])
        self.declare_parameter('start_pitch', -90.0)
        self.declare_parameter('start_roll', 0.0)

        self.declare_parameter('max_speed', 0.08)         
        self.declare_parameter('max_pitch_rate', 45.0)    
        self.declare_parameter('max_roll_rate', 90.0)    
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('rate', 20.0)              
        self.declare_parameter('move_time', 0.15)        


        self.declare_parameter('min_xyz', [0.05, -0.25, 0.02])
        self.declare_parameter('max_xyz', [0.30, 0.25, 0.35])


        self.declare_parameter('axis_x', 1)      
        self.declare_parameter('axis_y', 0)       
        self.declare_parameter('axis_z', 3)       
        self.declare_parameter('axis_roll', 2)    
        self.declare_parameter('button_pitch_up', 5)
        self.declare_parameter('button_pitch_down', 4)
        self.declare_parameter('button_open', 1)
        self.declare_parameter('button_close', 0)
        self.declare_parameter('button_reset', 3)

        self.declare_parameter('open_pulse', 600)
        self.declare_parameter('close_pulse', 300)
        self.declare_parameter('grip_duration', 0.5)

        p = lambda name: self.get_parameter(name).value
        self.startXyz = list(p('start_xyz'))
        self.startPitch = p('start_pitch')
        self.startRoll = p('start_roll')
        self.maxSpeed = p('max_speed')
        self.maxPitchRate = p('max_pitch_rate')
        self.maxRollRate = p('max_roll_rate')
        self.deadzone = p('deadzone')
        self.moveTime = p('move_time')
        self.minXyz = p('min_xyz')
        self.maxXyz = p('max_xyz')
        self.axisX = p('axis_x')
        self.axisY = p('axis_y')
        self.axisZ = p('axis_z')
        self.axisRoll = p('axis_roll')
        self.buttonPitchUp = p('button_pitch_up')
        self.buttonPitchDown = p('button_pitch_down')
        self.buttonOpen = p('button_open')
        self.buttonClose = p('button_close')
        self.buttonReset = p('button_reset')
        self.openPulse = p('open_pulse')
        self.closePulse = p('close_pulse')
        self.gripDuration = p('grip_duration')

        self.dt = 1.0 / p('rate')

        self.xyz = list(self.startXyz)
        self.pitch = self.startPitch
        self.roll = self.startRoll
        self.joy = None
        self.lastButtons = []

        self.targetPosePublisher = self.create_publisher(TargetPose, 'inverse_topic', 10)
        self.gripperPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)
        self.joySubscription = self.create_subscription(
            Joy, 'ros_robot_controller/joy', self.joyCallback, 10)
        self.timer = self.create_timer(self.dt, self.tick)

        self.joyTimeout = 1.0
        self.lastJoyTime = None
        self.joyLost = False
        self.watchdogTimer = self.create_timer(1.0, self.checkJoy)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def checkJoy(self):
        if self.lastJoyTime is None:
            self.get_logger().warn(
                'No joystick messages on ros_robot_controller/joy. Is the ros_robot_controller '
                'driver running and the controller powered on / paired?')
        elif self.now() - self.lastJoyTime > self.joyTimeout:
            if not self.joyLost:
                self.get_logger().warn('Controller stopped publishing - holding the arm still.')
            self.joyLost = True
            self.joy = None  # stop integrating the last stick values

    def axis(self, index):
        if self.joy is None or index >= len(self.joy.axes):
            return 0.0
        value = self.joy.axes[index]
        return 0.0 if abs(value) < self.deadzone else value

    def button(self, buttons, index):
        return index < len(buttons) and buttons[index] == 1

    def pressed(self, buttons, index):
     
        was = index < len(self.lastButtons) and self.lastButtons[index] == 1
        return self.button(buttons, index) and not was

    def joyCallback(self, msg):
        self.joy = msg
        self.lastJoyTime = self.now()
        if self.joyLost:
            self.get_logger().info('Controller is publishing again.')
            self.joyLost = False
        buttons = msg.buttons

        if self.pressed(buttons, self.buttonReset):
            self.xyz = list(self.startXyz)
            self.pitch = self.startPitch
            self.roll = self.startRoll
            self.get_logger().info('Target reset to start pose')
            self.publishTarget(duration=1.5)

        if self.pressed(buttons, self.buttonOpen):
            self.publishGripper(self.openPulse)
        if self.pressed(buttons, self.buttonClose):
            self.publishGripper(self.closePulse)

        self.lastButtons = list(buttons)

    def tick(self):
        if self.joy is None:
            return

        dx = self.axis(self.axisX)
        dy = self.axis(self.axisY)
        dz = self.axis(self.axisZ)
        dRoll = self.axis(self.axisRoll)
        dPitch = (float(self.button(self.joy.buttons, self.buttonPitchUp))
                  - float(self.button(self.joy.buttons, self.buttonPitchDown)))

        if not (dx or dy or dz or dRoll or dPitch):
            return 

        step = self.maxSpeed * self.dt
        delta = [dx * step, dy * step, dz * step]
        for i in range(3):
            self.xyz[i] = min(max(self.xyz[i] + delta[i], self.minXyz[i]), self.maxXyz[i])
        self.pitch += dPitch * self.maxPitchRate * self.dt
        self.roll += dRoll * self.maxRollRate * self.dt

        self.publishTarget(duration=self.moveTime)

    def publishTarget(self, duration):
        msg = TargetPose()
        msg.x = float(self.xyz[0])
        msg.y = float(self.xyz[1])
        msg.z = float(self.xyz[2])
        msg.roll = float(self.roll)
        msg.pitch = float(self.pitch)
        msg.duration = float(duration)
        self.targetPosePublisher.publish(msg)

    def publishGripper(self, pulse):
        msg = GripperControl()
        msg.pulse = int(pulse)
        msg.duration = float(self.gripDuration)
        self.gripperPublisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
