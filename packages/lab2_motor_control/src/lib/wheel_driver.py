#!/usr/bin/env python3

import rospy
import numpy as np

from duckietown_msgs.msg import Twist2DStamped
from odometry_hw.msg     import DistWheel

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
        rospy.Subscriber("/lab2/wheel_distance_delta", DistWheel, self.__cb_update_distance_traveled)
        self.wheel_twist_pub = rospy.Publisher(WHEEL_CMD_TOPIC, WHEEL_CMD_TYPE, queue_size=10)

        self.wheel_twist       = Twist2DStamped()
        self.wheel_twist.header.seq         = 0
        self.wheel_twist.header.stamp.secs  = 0
        self.wheel_twist.header.stamp.nsecs = 0
        self.wheel_twist.header.frame_id    = ""
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = 0
        self.l_wheel_distance_traveled = 0
        self.r_wheel_distance_traveled = 0 
        self.pause_rate = rospy.Rate(pause_rate) #TODO: Do I need this?

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
        self.__reset_distance_traveled()
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
    def drive(self, linear_velocity, theta, radius, l_distance=None, r_distance=None, omega=None):
        # Caclulate and publish the twist of the robot.
        self.wheel_twist.v = linear_velocity

        # The method allows for a user to hard set the omega value. If this value is set,
        # then the twist omega value will be set to the parameter set by the user.
        if omega is None:
            self.wheel_twist.omega = (linear_velocity * np.sin(theta)) / radius
        else:
            self.wheel_twist.omega = omega

        self.wheel_twist_pub.publish(self.wheel_twist)

        # If the l_distance or r_distance ketword parameters are not set, return.
        if l_distance is None or r_distance is None:
            return

        # Tracks the distance that the robot's wheels have travelled, stops the robot once it has travelled the 
        # specified distance.
        while self.l_wheel_distance_traveled < l_distance and self.r_wheel_distance_traveled < r_distance:
            pass
        
        self.stop()
        return

    """
        @fn     turn_in_place
        @brief  Based on the angle provided by the user, the duckiebot turns in place at an angle specified in the angle parameter.

        @param  speed   -- Angular velocity at which the duckiebot should turn in place.
        @param  angle   -- Angle in radians the duckiebot should turn w.r.t it's current position.

        @return None.
    """
    def turn_in_place(self, speed=8.3, angle=None):
        if angle is None:
            pass
        else:
            distance_to_travel = (ROBOT_AXLE_LENGTH / 2) * abs(angle)

        # Set the speed of each wheel.
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = speed
        self.wheel_twist_pub.publish(self.wheel_twist)

        if angle is None:
            return
        else:
            while abs(self.l_wheel_distance_traveled) < distance_to_travel and abs(self.r_wheel_distance_traveled) < distance_to_travel:
                rospy.logdebug(f"L Distance: {self.l_wheel_distance_traveled}, R Distance: {self.r_wheel_distance_traveled}, Set: {distance_to_travel}")
                pass
        
        self.stop()
        return

    """
        @fn     __cb_update_distance_traveled
        @brief  Updates the distance traveled by a command in this class by summing the distace deltas received from the wheel_distance_delta topic.

        @param  msg -- Data received in the wheels_distance_delta topic. This is a DistWheel type message.

        @return None.
    """
    def __cb_update_distance_traveled(self, msg):
        self.l_wheel_distance_traveled = self.l_wheel_distance_traveled + msg.dist_wheel_left
        self.r_wheel_distance_traveled = self.r_wheel_distance_traveled + msg.dist_wheel_right
        return

    """
        @fn     __reset_distance_traveled
        @brief  Method called by the stop method to set the distance traveled by the robot to zero (0). Distances are only tracked in between commands.

        @param  None.

        @return None.
    """
    def __reset_distance_traveled(self):
        self.l_wheel_distance_traveled = 0
        self.r_wheel_distance_traveled = 0
        return
        
