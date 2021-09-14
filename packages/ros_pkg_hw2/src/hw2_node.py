#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class FibonacciTalker:
    def __init__(self):
    	# Student note:
    	# The topic must also specify the namespace in the case of communication 
    	# between packages. In this case, we are sending data between the "hw2" 
    	# nodes via the "mystery_node" in homework 1 and the namespace must
    	# be specified as: 
    	#               v The topic name I want to publish to
    	# /mystery/<topic name>
        #    ^ mystery_node namespace
        self.pub = rospy.Publisher('/mystery/input', Float32, queue_size=10)
        
        # Initial Fibonacci values
        self.fib1 = 0
        self.fib2 = 0

    def talk(self):
        input_val = 0

        # Construct the fibonacci sequence.
        # On the first pass, the value will be zero (0), then self.fib2 is
        # assigned a value of one (1). This assignment allows for the 
        # sequence to continue.
        if self.fib2 == 0:
            input_val = self.fib2
            self.fib2 = 1
        # As the publishing continues, the fibonacci value will be calculated 
        # and the values will be assigned as such:
        # F1 = F2
        # F2 = F1 + F2
        else:
            input_val = self.fib1 + self.fib2
            self.fib1 = self.fib2
            self.fib2 = input_val 

        # Output the value to the terminal, write it to the logfile, and write
        # it to rosout
        # rospy.loginfo(input_val)

        # Publish the fibonacci value to the topic 'input'
        self.pub.publish(input_val)

if __name__ == '__main__':
    try:
        # Register the node with ROS
        rospy.init_node('hw2_node', anonymous=True)

        # Initialize the Fibonacci talker node
        fib_talker  = FibonacciTalker()

        # Set the sleep rate to 1Hz
        rate        = rospy.Rate(1)

        # Run the talker node until ROS exits
        while not rospy.is_shutdown():
            fib_talker.talk()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
