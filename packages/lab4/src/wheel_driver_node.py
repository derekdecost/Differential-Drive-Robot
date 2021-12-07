#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg     import Float32
from lib.wheel_driver import WheelDriver

class DuckiebotDriver(self):
    def __init__(self):
        self.__linear_xdd  = 0
        self.__linear_xd   = 0
        self.__linear_x    = 0

        self.__angular_xdd = 0
        self.__angular_xd  = 0
        self.__angular_x   = 0

        self.__linear_control  = 0
        self.__angular_control = 0

    def iterate(self, dt):
        self.__linear_xdd = self.__linear_control
        self.__linear_xd += self.__linear_xdd * dt
        self.__linear_x  += self.__linear_xd  * dt

        self.__angular_xdd = self.__angular_control
        self.__angular_xd += self.__angular_xdd * dt
        self.__angular_x  += self.__angular_xd  * dt
        return

    def __cb_update_angular_control(self, msg):
        self.__angular_control = msg.data
        return

    def __cb_update_linear_control(self, msg):
        self.__linear_control = msg.data
        return

if __name__ == "__main__":
    goal_angle    = 0  # degrees
    goal_distance = 10 # centimeters
    rate = rospy.Rate(10)

    rospy.init_node("wheel_driver_node", anonymous=True)
