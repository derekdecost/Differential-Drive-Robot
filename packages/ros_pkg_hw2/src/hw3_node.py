#!/usr/bin/env python3

import  rospy
from    mystery_package.msg import UnitsLabelled

class FeetConverter:
    def __init__(self, input_node, output_node, input_type, output_type, queue_val):
    	# Create the publisher and subscriber components of the node.
        rospy.Subscriber(input_node, input_type, self.callback)
        self.pub           = rospy.Publisher(output_node, output_type, queue_size=queue_val)
        
        # Set the initial values of the message that the node will publish.
        self.pub_msg       = UnitsLabelled()
        self.pub_msg.units = "feet"

    def callback(self, msg):
    	# Set the value of the message equal to the value of the topic that the subscriber
    	# is listening to after converting the units from meters to feet.
        self.pub_msg.value = self.meter2feet(msg.value)
        
        # Output the value of conversion to the publisher's topic.
        # This occurs as a part of the callback of the subscriber.
        # Once the subscriber receives a value, it will immediately convert the value from
        # meters to feet, and output the value to the log and the publisher's topic.
        self.pub.publish(self.pub_msg)
        rospy.loginfo(f"/ft_measurement/output published {self.pub_msg.value} {self.pub_msg.units}")
        return

    def meter2feet(self, val):
    	# Perform the conversion of meter's to feet.
    	# 1 meter = 3.28084 feet, Source: Google.com searching "meters to feet"
        return (val * 3.28084)        

if __name__ == "__main__":
    # Initialize the node and start the subscriber-publisher node.
    rospy.init_node('hw3_node', anonymous=True)
    FeetConverter(input_node="/mystery/output2", 
                  output_node='/ft_measurement/output', 
                  input_type=UnitsLabelled, 
                  output_type=UnitsLabelled, 
                  queue_val=10)
    rospy.spin()

