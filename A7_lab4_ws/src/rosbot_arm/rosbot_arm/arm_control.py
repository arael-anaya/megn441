import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board
from rosbot_msgs.msg import ForwardPose , GripperControl
from rosbot_msgs.srv import GetAngles


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')

        self.board = Board()
        self.board.enable_reception()

        # Writes get their own group so a servo read that never returns cannot hold them up
        writes = ReentrantCallbackGroup()
        self.forwardControlSubscription = self.create_subscription(ForwardPose , 'forward_topic' , self.forwardPoseCallback, 10, callback_group=writes)
        self.gripperControlSubscription = self.create_subscription(GripperControl , 'gripper_topic' , self.gripperControlCallback , 10, callback_group=writes)

        self.getAngleSrv = self.create_service(GetAngles , 'get_angles' , self.getAnglesCallback,
                                               callback_group=MutuallyExclusiveCallbackGroup())

        self.jointIds = [1, 2, 3, 4, 5, 10]

    def forwardPoseCallback(self, msg):

        if len(msg.ids) != len(msg.pulses):
            self.get_logger().warn('ForwardPose ids and pulses differ in length. Ignoring.')
            return

        limits = {2: (125, 775), 10: (0, 650)}
        targets = [(i, min(max(p, limits[i][0]), limits[i][1]) if i in limits else p)
                    for i, p in zip(msg.ids, msg.pulses)]

        self.board.bus_servo_set_position(msg.duration, targets)


    def gripperControlCallback(self, msg):
        if msg.pulse > 650: msg.pulse = 650

        self.board.bus_servo_set_position(msg.duration, [(10, msg.pulse)])

    def readPulse(self, jointId, timeout=0.5):
        """Servo position, or None. The SDK read waits on its reply queue forever (holding servo_read_lock) if the
        reply is lost, e.g. the stock stack on the same serial port ate it; that used to freeze this whole node, so
        the read runs in a worker thread and is abandoned after `timeout`."""
        lock = self.board.servo_read_lock
        if not lock.acquire(blocking=False):
            return None  # an earlier read is still stuck holding the lock
        lock.release()
        out = []
        worker = threading.Thread(target=lambda: out.append(self.board.bus_servo_read_position(jointId)), daemon=True)
        worker.start()
        worker.join(timeout)
        if worker.is_alive():
            self.get_logger().warn(f'Servo {jointId} read timed out (serial contention?).', throttle_duration_sec=2.0)
            return None
        return out[0] if out else None

    def getAnglesCallback(self, request, response):

        for jointId in self.jointIds:
            currPulse = self.readPulse(jointId)

            if currPulse is not None:
                currPulse = currPulse[0]
                if currPulse < 0:
                    currPulse = 0
                response.ids.append(jointId)
                response.pulses.append(currPulse)

        return response



def main():
    rclpy.init()
    node = ArmControl()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
