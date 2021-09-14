#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class FibonacciListener:
    def __init__(self):
        # Listen to the output1 topic of mystery_node
        
        # Student note:
    	# The topic must also specify the namespace in the case of communication 
    	# between packages. In this case, we are sending data between the "hw2" 
    	# nodes via the "mystery_node" in homework 1 and the namespace must
    	# be specified as: 
    	#               v The topic name I want to subscribe to
    	# /mystery/<topic name>
        #    ^ mystery_node namespace
        rospy.Subscriber('/mystery/output1', Float32, self.callback)

    def callback(self, msg):
        rospy.loginfo(f"/mystery/output1 published {msg.data}")

if __name__ == '__main__':
    # Start the node and begin listening to the output1 topic
    rospy.init_node('hw2_subscriber', anonymous=True)
    FibonacciListener()
    rospy.spin()
