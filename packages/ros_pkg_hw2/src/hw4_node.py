#!/usr/bin/env python3

import  rospy
from    mystery_package.msg import UnitsLabelled

class FeetConverter:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
    	# Create the publisher and subscriber components of the node.
        rospy.Subscriber(input_topic, input_type, self.callback)
        self.pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        
        # Set the initial values of the message that the node will publish.
        self.pub_msg = UnitsLabelled()

        if not rospy.has_param("/hw4/conversion_type"):
            exit

        # Set the conversion unit of of the node based on the configured parameter
        if rospy.get_param("/hw4/conversion_type") == "meter":
            self.pub_msg.units = "meters"
        elif rospy.get_param("/hw4/conversion_type") == "feet":
            self.pub_msg.units = "feet"
        elif rospy.get_param("/hw4/conversion_type") == "Smoot":
            self.pub_msg.units = "Smoot"
        else:
            # Default to convert to meters if no unit has been specified
            self.pub_msg.units = "meter"

    def callback(self, msg):
        # Based on the unit type set by the parameter /hw4/conversion_type, perform
        # a unit conversion and return its value to self.pub_msg.value
    	if self.pub_msg.units == "meter":
            self.pub_msg.value = msg.value
        elif self.pub_msg.units == "feet":
            self.pub_msg.value = self.meter2feet(msg.value)
        elif self.pub_msg.value == "Smoot":
            self.pub_msg.value = self.meter2smoot(msg.value)
        
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
    
    def meter2smoot(self, val):
        # Perform the conversion of meter's to Smoots.
        # 1 meter = 0.587613 Smoots
        return (val * 0.587613)

if __name__ == "__main__":
    # Initialize the node and start the subscriber-publisher node.
    rospy.init_node('hw3_node', anonymous=True)
    FeetConverter(input_topic="/mystery/output2", 
                  output_topic='/ft_measurement/output', 
                  input_type=UnitsLabelled, 
                  output_type=UnitsLabelled, 
                  queue_val=10)
    rospy.spin()

