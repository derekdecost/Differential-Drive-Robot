#!/usr/bin/env python3

# This node subscribes to the sensor topic that is output points that
# it detects, then transforms those data points from points in the 
# sensor frame of reference to points in the frame of reference in the 
# robot frame and the world frame.

import rospy
import numpy as np
from   geometry_msgs import Point

class ReferenceFrameTransformer:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        # Coordinate transforms for transforming the frame of the 
        # sensor to the frame of the robot and from the sensor to 
        # the frame of the world.
        # self.T_sensor_to_robot = np.array([[-1,  0, -2], 
        #                                    [ 0, -1,  0], 
        #                                    [ 0,  0,  1]])
        self.T_sensor_to_robot = np.matrix('-1 0 -2; 0 -1 0; 0 0 1')
        # self.T_sensor_to_world = np.array([[ 0.707,  0.707, 3.414], 
        #                                    [-0.707,  0.707, 5.586],
        #                                    [ 0,      0,     1]])
        self.T_sensor_to_world = np.matrix('0.707 0.707 3.414; -0.707 0.707 5.586; 0 0 1')
        self.P_sensor_to_robot = Point()
        self.P_sensor_to_world = Point()

        # Subscriber that listens for input points from the sensor
        rospy.Subscriber(input_topic, input_type, self.callback)

        # Publishers that output the transformed point from the
        # sensor to a point on the robot coordinate frame, and a 
        # point on the world frame
        self.s2r_publisher = rospy.Publisher(output_topic[0], output_type, queue_size=queue_val)
        self.s2w_publisher = rospy.Publisher(output_topic[1], output_type, queue_size=queue_val)

         

    def callback(self, msg):
        # Create the point from the data received from the topic
        P_sensor_point = np.matrix(f'{msg.x.data}; {msg.y.data}; 1')

        # Perform the coordinate transformations for the robot and world frames
        P_robot_point  = np.matmul(self.T_sensor_to_robot, P_sensor_point)
        P_world_point  = np.matmul(self.T_sensor_to_world, P_sensor_point)

        # Create the points to be published
        self.P_sensor_to_robot.x.data = P_robot_point[0][0]
        self.P_sensor_to_robot.y.data = P_robot_point[0][1]
        self.P_sensor_to_robot.z.data = 1

        self.P_sensor_to_world.x.data = P_world_point[0][0]
        self.P_sensor_to_world.y.data = P_world_point[0][1]
        self.P_sensor_to_world.z.data = 1

        # Publish the values to their respective topics
        self.s2r_publisher(self.P_sensor_to_robot)
        self.s2w_publisher(self.P_sensor_to_world)

        # Output the published data to the rqt_console
        rospy.loginfo(f"hw5_frame_transformer published to /hw5/s2r: {self.P_sensor_to_robot.x.data}, {self.P_sensor_to_robot.y.data}, {self.P_sensor_to_robot.z.data}")
        rospy.loginfo(f"hw5_frame_transformer published to /hw5/s2w: {self.P_sensor_to_world.x.data}, {self.P_sensor_to_world.y.data}, {self.P_sensor_to_world.z.data}")
        
        return

if __name__ == '__main__':
    rospy.init_node('hw5_transformer', anonymous=True)
    ReferenceFrameTransformer(input_topic='/hw5/sensor_points',
                              output_topic=['/hw5/s2r', '/hw5/s2w'],
                              input_type=Point,
                              output_type=Point,
                              queue_val=10)
    rospy.spin()
