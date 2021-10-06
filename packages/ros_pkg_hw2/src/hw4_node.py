#!/usr/bin/env python3

import  rospy
from    mystery_package.msg import UnitsLabelled
from    std_msgs.msg        import Float32

class FeetConverter:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        # Create the publisher and subscriber components of the node.
        rospy.Subscriber(input_topic, input_type, self.callback)
        self.pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        
        # Set the initial values of the message that the node will publish.
        self.pub_msg = UnitsLabelled()

        if not rospy.has_param("/hw4/conversion_type"):
            exit
        else:
            self.units = rospy.get_param("/hw4/conversion_type")

    def callback(self, msg):
        # Based on the unit type set by the parameter /hw4/conversion_type, perform
        # a unit conversion and return its value to self.pub_msg.value
        if self.units == "meter":
            self.pub_msg.value = msg.data
        elif self.units == "feet":
            self.pub_msg.value = self.meter2feet(msg.data)
        elif self.units == "Smoot":
            self.pub_msg.value = self.meter2smoot(msg.data)
        
        # Output the value of conversion to the publisher's topic.
        # This occurs as a part of the callback of the subscriber.
        # Once the subscriber receives a value, it will immediately convert the value from
        # meters to feet, and output the value to the log and the publisher's topic.
        self.pub.publish(Float32(self.pub_msg.value))
        rospy.loginfo(f"hw4_node published {self.pub_msg.value} {self.pub_msg.units} to topic /mystery/input")
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
    FeetConverter(input_topic='/hw4/hw4_input', 
                  output_topic='/mystery/input', 
                  input_type=Float32, 
                  output_type=Float32, 
                  queue_val=10)
    rospy.spin()

