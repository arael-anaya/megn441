#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/04

import os
import time
import rclpy
import psutil
from ros_robot_controller_msgs.msg import BuzzerState, OLEDState

ROBOT_NUMBER = 12

NAMESPACE = "ros_robot_controller"

def get_cpu_serial_number():
    # device_serial_number = open("/proc/device-tree/serial-number")
    # serial_num = device_serial_number.readlines()[0][-10:-1]

    HW_WIFI_AP_SSID = ''.join(["ROSBOT-", {}.format(ROBOT_NUMBER)])

    return HW_WIFI_AP_SSID

# def get_cpu_serial_number():
#     device_serial_number = open("/proc/device-tree/serial-number")
#     serial_num = device_serial_number.readlines()[0][-10:-1]

#     sn = (serial_num + "00000000000000000000000000")[:32]
#     HW_WIFI_AP_SSID = ''.join(["HW-", sn[0:8]])

#     return HW_WIFI_AP_SSID

def get_wlan():
    ip = ''
    info = psutil.net_if_addrs()
    for k, v in info.items():
        if 'wlan0' in k:
            for i in v:
                if i[2] is not None:
                    ip = i[1]
                    break
                else:
                    ip = None

    if ip != '' and ip is not None:
        return ip
    else:
        return '0.0.0.0'


def main():
    rclpy.init()
    node = rclpy.create_node('startup')
    buzzer_pub = node.create_publisher(BuzzerState, '/' + NAMESPACE + '/set_buzzer', 1)
    oled_pub = node.create_publisher(OLEDState, '/' + NAMESPACE + '/set_oled', 1)
    time.sleep(10)
    
    msg = BuzzerState()
    msg.freq = 1900
    msg.on_time = 0.1
    msg.off_time = 0.01
    msg.repeat = 2
    buzzer_pub.publish(msg)

    msg = OLEDState()
    msg.index = 1
    msg.text = 'SSID:' + get_cpu_serial_number()
    oled_pub.publish(msg)
    time.sleep(0.2)
    msg = OLEDState()
    msg.index = 2
    msg.text = 'IP:' + get_wlan()
    oled_pub.publish(msg)

if __name__ == '__main__':
    main()
