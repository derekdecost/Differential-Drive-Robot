#!/usr/bin/env python3

import rospy

from duckietown_msgs.msg import WheelCmd
from odometry_hw.msg     import DistWheel

WHEEL_CMD_TOPIC = "/beezchurger/wheels_driver_node/wheels_cmd_executed"
WHEEL_CMD_TYPE  = WheelCmd

ROBOT_AXLE_LENGTH = 0.1

class WheelDriver:
    def __init__(self, pause_rate=0.5):
        rospy.Subscriber("/lab2/wheel_distance_delta", DistWheel, self.__cb_update_distance_traveled)
        self.wheel_velocity_pub = rospy.Publisher(WHEEL_CMD_TOPIC, WHEEL_CMD_TYPE, queue_size=10)
        
        # Create the velocity setting message and the initial values
        self.wheel_velocity            = WheelCmd()
        self.wheel_velocity.vel_left   = 0
        self.wheel_velocity.vel_right  = 0
        self.l_wheel_distance_traveled = 0
        self.r_wheel_distance_traveled = 0 
        self.wheel_velocity_pub.publish(self.wheel_velocity)
        self.pause_rate = rospy.Rate(pause_rate)

    def stop(self):
        self.wheel_velocity.vel_left  = 0
        self.wheel_velocity.vel_right = 0
        self.wheel_velocity_pub.publish(self.wheel_velocity)
        return

    def drive(self, speed, l_ratio=1, r_ratio=1, l_distance=None, r_distance=None):
        # Set the speed of each wheel
        self.wheel_velocity.vel_left  = l_ratio * speed
        self.wheel_velocity.vel_right = r_ratio * speed
        self.wheel_velocity_pub.publish(self.wheel_velocity)

        # If wheel distances are specified, continue to track the distance traveled
        # then, set the robot speed to zero (0).
        if l_distance is None or r_distance is None:
            return
        
        # Stop the robot's motion after it has traveled the input distance.
        while self.l_wheel_distance_traveled < l_distance and self.r_wheel_distance_traveled < r_distance:
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
        
