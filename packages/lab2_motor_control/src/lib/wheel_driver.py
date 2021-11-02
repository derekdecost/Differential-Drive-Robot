#!/usr/bin/env python3

import rospy
import numpy as np

from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped
from odometry_hw.msg     import DistWheel

WHEEL_CMD_TOPIC = "/beezchurger/lane_controller_node/car_cmd"
WHEEL_CMD_TYPE  = Twist2DStamped

ROBOT_AXLE_LENGTH = 0.1

class WheelDriver:
    def __init__(self, parent, pause_rate=0.5):
        # rospy.init_node(f"wheel_driver_{parent}", anonymous=True)
        rospy.Subscriber("/lab2/wheel_distance_delta", DistWheel, self.__cb_update_distance_traveled)
        self.wheel_twist_pub = rospy.Publisher(WHEEL_CMD_TOPIC, WHEEL_CMD_TYPE, queue_size=10)
        
        # Create the velocity setting message and the initial values
        # self.wheel_velocity            = WheelsCmdStamped()
        # self.wheel_velocity.header.seq = 1800
        # self.wheel_velocity.header.stamp.secs = 0
        # self.wheel_velocity.header.stamp.nsecs = 0
        # self.wheel_velocity.frame_id = ''
        # self.wheel_velocity.vel_left   = 0
        # self.wheel_velocity.vel_right  = 0
        # self.wheel_velocity_pub.publish(self.wheel_velocity)

        self.wheel_twist       = Twist2DStamped()
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = 0
        self.l_wheel_distance_traveled = 0
        self.r_wheel_distance_traveled = 0 
        
        self.pause_rate = rospy.Rate(pause_rate)

    def stop(self):
        self.wheel_twist.v     = 0
        self.wheel_twist.omega = 0
        self.wheel_twist_pub.publish(self.wheel_twist)
        return

    # def drive(self, speed, l_ratio=1, r_ratio=1, l_distance=None, r_distance=None):
    #     # Set the speed of each wheel
    #     self.wheel_velocity.vel_left  = l_ratio * speed
    #     self.wheel_velocity.vel_right = r_ratio * speed
    #     self.wheel_velocity_pub.publish(self.wheel_velocity)

    #     # If wheel distances are specified, continue to track the distance traveled
    #     # then, set the robot speed to zero (0).
    #     if l_distance is None or r_distance is None:
    #         return
        
    #     # Stop the robot's motion after it has traveled the input distance.
    #     while self.l_wheel_distance_traveled < l_distance and self.r_wheel_distance_traveled < r_distance:
    #         continue
        
    #     self.stop()
    #     self.__reset_distance_traveled()
    #     return


    """
        @fn drive
        @brief Based on the provided parameters, sets the linear and angular velocity of the duckiebot.

        @param linear_velocity -- The linear velocity of the duckiebot to be set in meters/second.
        @param theta           -- Angle in radians between the linear velocity vector and its radial component.
        @param radius          -- The radius of the circle that will be used to calculate the angular velocity of the duckiebot.
        @param l_distance      -- Distance in meters that the left wheel should travel before the duckiebot should stop.
        @param r_distance      -- Distance in meters that the right wheel should travel before the duckiebot should stop.

        @return None.
    """
    def drive(self, linear_velocity, theta, radius, l_distance=None, r_distance=None):
        # Caclulate and publish the twist of the robot.
        self.wheel_twist.v     = linear_velocity
        self.wheel_twist.omega = (linear_velocity * np.sin(theta)) / radius
        self.wheel_twist_pub.publish(self.wheel_twist)

        # If the l_distance or r_distance ketword parameters are not set, return.
        if l_distance is None or r_distance is None:
            return

        # Tracks the distance that the robot's wheels have travelled, stops the robot once it has travelled the 
        # specified distance.
        while self.l_wheel_distance_traveled < l_distance or self.r_wheel_distance_traveled < r_distance:
            continue
        
        self.stop()
        self.__reset_distance_traveled()
        return

    #TODO: Combine the ccw and cw turn_in_place functions
    def turn_in_place_ccw(self, speed, angle=None):
        # Calculate the circumferance of the circle to travel.
        # This circle is formed using the robot's axle length as the diameter.
        if angle is not None:
            distance_to_travel = (ROBOT_AXLE_LENGTH / 2) * abs(angle)
            pass

        # Set the speed of each wheel 
        self.wheel_velocity.vel_left  = -1 * speed
        self.wheel_velocity.vel_right = speed
        self.wheel_velocity_pub.publish(self.wheel_velocity)

        # If the angle of the turn is specified, continue to track the distance traveled
        # then, set the robot speed to zero (0).
        if angle is None:
            return

        # Stop the robot's turning motion after it has completed the full travel distance
        while self.l_wheel_distance_traveled < distance_to_travel and self.r_wheel_distance_traveled < distance_to_travel:
            continue

        self.stop()
        self.__reset_distance_traveled()
        return

    def turn_in_place_cw(self, speed, angle=None):
        # Calculate the circumferance of the circle to travel.
        # This circle is formed using the robot's axle length as the diameter.
        if angle is not None:
            distance_to_travel = (ROBOT_AXLE_LENGTH / 2) * abs(angle)
            pass
        
        # Set the speed of each wheel 
        self.wheel_velocity.vel_left  = speed
        self.wheel_velocity.vel_right = -1 * speed
        self.wheel_velocity_pub.publish(self.wheel_velocity)

        # If the angle of the turn is specified, continue to track the distance traveled
        # then, set the robot speed to zero (0).
        if angle is None:
            return
        
        # Stop the robot's turning motion after it has completed the full travel distance
        while self.l_wheel_distance_traveled < distance_to_travel and self.r_wheel_distance_traveled < distance_to_travel:
            continue

        self.stop()
        self.__reset_distance_traveled()
        return

    def __cb_update_distance_traveled(self, msg):
        self.l_wheel_distance_traveled = self.l_wheel_distance_traveled + msg.dist_wheel_left
        self.r_wheel_distance_traveled = self.r_wheel_distance_traveled + msg.dist_wheel_right
        return

    def __reset_distance_traveled(self):
        self.l_wheel_distance_traveled = 0
        self.r_wheel_distance_traveled = 0
        return
        
