#!/usr/bin/env python3

# This node subscribes to the sensor topic that is output points that
# it detects, then transforms those data points from points in the 
# sensor frame of reference to points in the frame of reference in the 
# robot frame and the world frame.

import rospy
import numpy as np
from duckietown_msgs.msg import Vector2D

class ReferenceFrameTransformer:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        # Coordinate transforms for transforming the frame of the 
        # sensor to the frame of the robot and from the sensor to 
        # the frame of the world.
        self.T_sensor_to_robot = np.matrix('-1 0 -2; 0 -1 0; 0 0 1')
        self.T_sensor_to_world = np.matrix('0.707 0.707 3.414; -0.707 0.707 5.586; 0 0 1')
        self.P_sensor_to_robot = Vector2D()
        self.P_sensor_to_world = Vector2D()

        # Subscriber that listens for input points from the sensor
        rospy.Subscriber(input_topic, input_type, self.callback)

        # Publishers that output the transformed point from the
        # sensor to a point on the robot coordinate frame, and a 
        # point on the world frame
        self.s2r_publisher = rospy.Publisher(output_topic[0], output_type, queue_size=queue_val)
        self.s2w_publisher = rospy.Publisher(output_topic[1], output_type, queue_size=queue_val)

         

    def callback(self, msg):
        # Create the point from the data received from the topic
        P_sensor_point = np.matrix(f'{msg.x}; {msg.y}; 1')

        # Perform the coordinate transformations for the robot and world frames
        P_robot_point  = np.matmul(self.T_sensor_to_robot, P_sensor_point)
        P_world_point  = np.matmul(self.T_sensor_to_world, P_sensor_point)

        # Create the points to be published
        self.P_sensor_to_robot.x = P_robot_point[0][0]
        self.P_sensor_to_robot.y = P_robot_point[1][0]

        self.P_sensor_to_world.x = P_world_point[0][0]
        self.P_sensor_to_world.y = P_world_point[1][0]

        # Publish the values to their respective topics
        self.s2r_publisher.publish(self.P_sensor_to_robot)
        self.s2w_publisher.publish(self.P_sensor_to_world)

        # Output the published data to the rqt_console
        rospy.loginfo(f"hw5_frame_transformer published to /hw5/s2r: {self.P_sensor_to_robot.x}, {self.P_sensor_to_robot.y}")
        rospy.loginfo(f"hw5_frame_transformer published to /hw5/s2w: {self.P_sensor_to_world.x}, {self.P_sensor_to_world.y}")
        
        return

if __name__ == '__main__':
    rospy.init_node('hw5_transformer', anonymous=True)
    ReferenceFrameTransformer(input_topic='/hw5/sensor_points',
                              output_topic=['/hw5/s2r', '/hw5/s2w'],
                              input_type=Vector2D,
                              output_type=Vector2D,
                              queue_val=10)             
    rospy.spin()
