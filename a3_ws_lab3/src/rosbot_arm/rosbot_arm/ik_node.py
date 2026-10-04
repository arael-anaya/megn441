import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import TargetPose , ForwardPose
from rosbot_arm import inverseKinematics

# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]


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


        self.jointIds = [1, 2, 3, 4, 5]

    def solvePulses(self, msg):

        for upDownConstraint in (1, -1):
            jointAngles = inverseKinematics.solveIK(msg.x, msg.y, msg.z, msg.roll, msg.pitch, upDownConstraint)
            if jointAngles is None:
                continue

            pulses = [int(round(angle_to_pulse(a))) for a in jointAngles]
            if all(lo <= p <= hi for p, lo, hi in zip(pulses, min_pulses, max_pulses)):
                return pulses
        return None

    def targetPoseCallback(self, msg):
        pulses = self.solvePulses(msg)

        if pulses is None:
            self.get_logger().warn(
                f'Target ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) pitch={msg.pitch} '
                'is unreachable or outside joint limits. Not sending.')
            return

        goal = ForwardPose()
        goal.ids = self.jointIds
        goal.pulses = pulses
        goal.duration = msg.duration
        self.forwardPosePublisher.publish(goal)

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
