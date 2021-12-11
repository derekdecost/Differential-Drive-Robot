#!/usr/bin/env python3

import rospy
import numpy as np

from duckietown_msgs.msg import Twist2DStamped

WHEEL_CMD_TOPIC = "/beezchurger/car_cmd_switch_node/cmd"
WHEEL_CMD_TYPE  = Twist2DStamped

ROBOT_AXLE_LENGTH = 0.07

class WheelDriver:
    """
        @fn     __init__
        @brief  Initializes the WheelDriver class, and sets the initial values for messages used to communicate with the wheel 
                tick topics on the duckiebot.

        @param  None.

        @return Returns a WheelDriver object.
    """
    def __init__(self, pause_rate=0.5):
        self.wheel_twist_pub = rospy.Publisher(WHEEL_CMD_TOPIC, WHEEL_CMD_TYPE, queue_size=1)

        self.wheel_twist       = Twist2DStamped()
        self.wheel_twist.header.seq         = 0
        self.wheel_twist.header.stamp.secs  = 0
        self.wheel_twist.header.stamp.nsecs = 0
        self.wheel_twist.header.frame_id    = ""
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = 0

    """
        @fn     stop
        @brief  Stops the duckiebot in place by setting the angular and linear velocities of the robot to zero (0).

        @param  None.

        @return None.
    """
    def stop(self):
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = 0
        self.wheel_twist_pub.publish(self.wheel_twist)
        return

    """
        @fn     drive
        @brief  Based on the provided parameters, sets the linear and angular velocity of the duckiebot.

        @param  linear_velocity -- The linear velocity of the duckiebot to be set in meters/second.
        @param  theta           -- Angle in radians between the linear velocity vector and its radial component.
        @param  radius          -- The radius of the circle that will be used to calculate the angular velocity of the duckiebot.
        @param  l_distance      -- Distance in meters that the left wheel should travel before the duckiebot should stop.
        @param  r_distance      -- Distance in meters that the right wheel should travel before the duckiebot should stop.

        @return None.
    """
    def drive(self, linear_velocity, omega):
        self.wheel_twist.v     = linear_velocity
        self.wheel_twist.omega = omega
        self.wheel_twist_pub.publish(self.wheel_twist)
        return
