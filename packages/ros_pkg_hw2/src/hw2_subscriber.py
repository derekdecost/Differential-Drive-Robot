#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class FibonacciListener:
    def __init__(self):
        rospy.Subscriber('output1', Float32, self.callback)

    def callback(self, msg):
        rospy.loginfo(f"{rospy.get_caller_id()} published {msg.data}")

if __name__ == '__main__':
    rospy.init_node('hw2_subscriber', anonymous=True)
    FibonacciListener()
    rospy.spin()