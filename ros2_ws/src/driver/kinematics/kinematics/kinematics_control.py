#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# Call robotic arm kinematics
from kinematics_msgs.srv import SetRobotPose, SetJointValue

def set_pose_target(position, pitch, pitch_range=[-180.0, 180.0], resolution=1.0):
    '''
    Given a coordinate and a pitch angle, return the inverse kinematics solution
    position: [x, y, z]，the target position in a list [x, y, z] with units of meters
    pitch: the target pitch angle in degrees, ranging from -180 to 180
    pitch_range: if a solution cannot be found at the target pitch angle, search for a solution within this range
    resolution: the resolution of the pitch_range in degrees
    return: whether the call is successful, the target positions of the servo, 
        the current positions of the servo, the target posture of the robotic arm, 
        and the changes in the rotation of all servos for the optimal solution
    '''
    msg = SetRobotPose.Request()
    msg.position = [float(i) for i in position]
    msg.pitch = float(pitch)
    msg.pitch_range = [float(i) for i in pitch_range]
    msg.resolution = float(resolution)
    return msg

def set_joint_value_target(joint_value):
    '''
    Given the rotation angles of each servo, return the target position and posture of the robotic arm
    joint_value: [joint1, joint2, joint3, joint4, joint5]，
    (the rotation angle of each servo in a list [joint1, joint2, joint3, joint4, and joint5] 
    with units of pulse width)
    return: geometry_msgs/Pose(the 3D coordinates and posture of the target position in the format geometry_msgs/Pose)
    '''
    msg = SetJointValue.Request()
    msg.joint_value = [float(i) for i in joint_value]
    return msg
    
if __name__ == "__main__":
    import time
    import rclpy
    from rclpy.node import Node
    import kinematics.transform as transform

    rclpy.init()
    client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
    while True:
        t = time.time()
        res = node.set_pose_target([transform.link3 + transform.tool_link, 0.0, 0.36], 0.0, [-180.0, 180.0], 1.0)
        print(time.time() - t)
    rclpy.logging.get_logger('p2').info(str(res[1]))
    # print('ik', res)
    # if res[1] != []:
        # res = set_joint_value_target(res[1])
        # print('fk', res)
    node.destroy_node()
    rclpy.shutdown()
