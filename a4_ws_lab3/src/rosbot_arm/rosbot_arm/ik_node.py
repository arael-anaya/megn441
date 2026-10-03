import rclpy
from rclpy.node import Node
 
import numpy as np


##  Joint Links ##
# l0 refers to the height of the arm base wrt base_link
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117

lengths = [l0, l1, l2, l3+tool_link, 0]


arm_ids = [1, 2, 3, 4, 5]

# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]



# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000

# TODO: Write helper scripts to convert from one to the other.

def angle_to_pulse(angle):
    # TODO
    return pulse

def pulse_to_angle(pulse):
    # TODO
    return angle

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
    # TODO: Write inverse kinematics node

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
