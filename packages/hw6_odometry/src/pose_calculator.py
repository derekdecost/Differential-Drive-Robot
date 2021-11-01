#!/usr/bin/env python3

import rospy
import numpy as np

# from lib.fsm_state_controller import StateController
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D

class PoseCalculator:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        # Subscriber that attaches to the topic calculating the distance
        # travelled by each wheel
        rospy.Subscriber(input_topic, input_type, self.callback_calculate_pose)
        
        # Publisher that outputs the calculated pose of the robot
        self.pose_pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        
        # Pose message object
        self.pose = output_type()

        # Class parameters
        self.axle_length_m      = 0.1
        self.half_axle_length_m = self.axle_length_m / 2

        # Pose information
        self.x_position     = 0
        self.y_position     = 0
        self.theta_position = 0

    def callback_calculate_pose(self, msg):
        # The message provides the change in wheel distance (the delta)
        del_sr    = msg.dist_wheel_right
        del_sl    = msg.dist_wheel_left
        del_s     = (del_sl + del_sr) / 2
        del_theta = (del_sr - del_sl) / self.axle_length_m
        
        # Calculate the change in x and y positions based on the distance travelled
        del_x = del_s * np.cos([self.theta_position + (del_theta / 2)])
        del_y = del_s * np.sin([self.theta_position + (del_theta / 2)])
        
        # Calculate the new x, y, and theta positions
        self.x_position     = self.x_position + del_x
        self.y_position     = self.y_position + del_y
        self.theta_position = self.theta_position + del_theta
        
        # Set the values of the pose message
        self.pose.x     = self.x_position
        self.pose.y     = self.y_position
        self.pose.theta = self.theta_position

        # Publish the pose message to the output topic
        self.pose_pub.publish(self.pose)
        rospy.loginfo(f"Pose = x:{self.pose.x}, y:{self.pose.y}, theta:{self.pose.theta}")

        return

if __name__ == "__main__":
    # duckiebot_sc = StateController(parent="pose_calculator")
    rospy.init_node("hw6_pose_calculator", anonymous=True)
    PoseCalculator(
        input_topic='/dist_wheel',
        output_topic='/pose',
        input_type=DistWheel,
        output_type=Pose2D,
        queue_val=10
    )
    rospy.spin()