#!/usr/bin/env python3

# This node simulates a sensor located on a robot and sends points 
# it has "detected" to a topic that will then transform these 
# points from the sensor's frame of reference to the frame of 
# reference of the robot, and the world.

import rospy
from duckietown_msgs.msg import Vector2D

class Sensor:
    def __init__(self, output_topic, output_type, queue_val):
        self.sensor_data_publisher = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        self.sensor_data = Vector2D()

    def send_data(self, x, y):
        self.sensor_data.x = x
        self.sensor_data.y = y
        
        self.sensor_data_publisher.publish(self.sensor_data)

if __name__ == '__main__':
    rospy.init_node('hw5_sensor', anonymous=True)
    sensor = Sensor('/hw5/sensor_points', Vector2D, 10)
    rate = rospy.Rate(1)

    rate.sleep()
    sensor.send_data(17, 13)
    rate.sleep()
    sensor.send_data(5, -19)
    rate.sleep()
    sensor.send_data(-16, 19)
    rate.sleep()
    sensor.send_data(-11, -7)
    rate.sleep()

    exit