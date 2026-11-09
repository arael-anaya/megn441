import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rosbot_arm import inverseKinematics


import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rxclpy.executors import MultiThreadedExecutor

from rosbot_msgs.action import Pick, Place
from rosbot_msgs.msg import  TargetPose , ForwardPose, GripperControl

# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]


# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000

def angle_to_pulse(angle):

    pulsePerAngle = 500/120
    pulse = 500 + angle * pulsePerAngle
    return pulse

def pulse_to_angle(pulse):

    anglePerPulse = 120/500
    angle = (pulse-500) * anglePerPulse
    return angle

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.targetPoseSubscription = self.create_subscription(TargetPose , "inverse_topic" , self.targetPoseCallback , 10)
        self.forwardPosePublisher = self.create_publisher(ForwardPose, "forward_topic" , 10)

        self.ikResultPublisher = self.create_publisher(Bool, "ik_result", 10)

        self.jointIds = [1, 2, 3, 4, 5]

        self.declare_parameter('stow_xyz' , [.1, 0.0,0.24])
        self.declare_parameter('stow_pitch' , -45.0)
        self.declare_parameter('grasp_pitch' , -90.0)
        self.declare_parameter('gripper_open', 300)
        self.declare_parameter('gripper_closed', 600)
        self.declare_parameter('gripper_open', 300)
        self.declare_parameter('hover_offset', 0.08)
        self.declare_parameter('move_time', 2.0)
        self.declare_parameter('grip_time', 1.0)
        self.declare_parameter('settle_time', 0.5)

        p  = lambda name: self.get_parameter(name).value
        self.stowXyz = p('stow_xys')
        self.stowPitch = p('stow_pitch')
        self.graspPitch = p('grasp_pitch')
        self.gripperOpen = p('gripper_open')
        self.gripperClosed = p('gripper_closed')
        self.hoverOffset = p('hover_offset')
        self.moveTime = p('move_time')
        self.gripTime = p('grip_time')
        self.settleTime = p('settle_time')

        self.gripperPublisher = self.create_publisher(GripperControl, 'gripper_topic', 10)
        self.busy = False

        self.cbGroup = ReentrantCallbackGroup() #Multithreaded bs callback stuff
        self.pickServer = ActionServer(
            self, Pick, 'pick',
            execute_callback = self.executePick,
            goal_callback = self.goalCallback,
            cancel_callback = self.cancelCallback,
            callback_group = self.cbGroup)

        self.placeServer = ActionServer(
                self, Place, 'place',
                execute_callback = self.executePlace,
                goal_callback = self.goalCallback,
                cancel_callback = self.goalCallback,
                callback_group = self.cbGroup)

    def solvePulses(self, msg):

        # Straight down (-90) puts the wrist ~0.17 m above the tip, which leaves low targets beyond ~0.15 m
        # out of reach. If the requested pitch fails, tilt the gripper toward horizontal until it fits.
        pitches = [msg.pitch] + [p for p in range(-80, -29, 5) if p > msg.pitch]
        for pitch in pitches:
            for upDownConstraint in (1, -1):
                jointAngles = inverseKinematics.solveIK(msg.x, msg.y, msg.z, msg.roll, pitch, upDownConstraint)
                if jointAngles is None:
                    continue

                pulses = [int(round(angle_to_pulse(a))) for a in jointAngles]
                if all(lo <= p <= hi for p, lo, hi in zip(pulses, min_pulses, max_pulses)):
                    if pitch != msg.pitch:
                        self.get_logger().info(f'Target needed pitch {pitch} instead of {msg.pitch}.')
                    return pulses
        return None

    def targetPoseCallback(self, msg):
        pulses = self.solvePulses(msg)

        if pulses is None:
            self.get_logger().warn(
                f'Target ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) pitch={msg.pitch} '
                'is unreachable or outside joint limits. Not sending.')
            self.ikResultPublisher.publish(Bool(data=False))
            return

        goal = ForwardPose()
        goal.ids = self.jointIds
        goal.pulses = pulses
        goal.duration = msg.duration
        self.forwardPosePublisher.publish(goal)
        self.ikResultPublisher.publish(Bool(data=True))

    def moveTo(self, x, y, z, pitch, roll=0.0):
        pose = TargetPose(x=float(x), y=float(y), z=float(z), roll=float(roll), pitch=float(pitch), duration=float(self.moveTime))
        pulses = self.solvePulses(pose)
        if pulses is None:
            self.get_logger().error(f'Unreachable: ({x:.3f}, {y:.3f}, {z:.3f}) pitch={pitch}')
            return False
        cmd = ForwardPose()
        cmd.ids = self.jointIds
        cmd.pulses = pulses
        cmd.duration = float(self.moveTime)
        self.forwardPosePublisher.publish(cmd)
        time.sleep(self.moveTime + self.settleTime)
        return True

    def moveGripper(self,pulse):
        self.gripperPublisher.publishh(GripperControl(pulse=int(pulse), duration=float(self.gripTime)))
        time.sleep(self.gripTime + self.settleTime)
        return True
    
    def stow(self):
        x,y,z = self.stowXyz
        return self.moveTo(x, y, z , self.stowPitch)
    
        


def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
