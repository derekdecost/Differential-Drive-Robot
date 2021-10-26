import rospy
import sys
import robot_params
import time

from std_msgs.msg import Int32
from std_msgs.msg import Bool
from duckietown_msgs.msg import WheelCmd


class WheelDriver:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        rospy.Subscriber(input_topic, input_type, self.drive_state_callback)
        self.wheel_velocity_pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        self.reset_rotation_calculator_pub = rospy.Publisher("/lab2/reset_rotation_calculator", Bool, queue_size=queue_val)
        
        self.reset_state = Bool()
        self.reset_state.data = False

        self.wheel_cmd = WheelCmd()
        self.wheel_cmd.vel_right = 0
        self.wheel_cmd.vel_left  = 0
        
        self.wheel_velocity_pub.publish(self.wheel_cmd)
        self.drive_state = False

    def drive_state_callback(self, msg):
        robot_speed = 1

        # If the drive state has not changed, return
        if msg.data == self.drive_state:
            return
        else:
            self.drive_state = msg.data
        
        if self.drive_state in [1, 5, 9, 13]:
            self.wheel_cmd.vel_right = robot_speed
            self.wheel_cmd.vel_left  = robot_speed
            self.reset_state.data    = True
            self.wheel_velocity_pub.publish(self.wheel_cmd)
            self.reset_rotation_calculator_pub.publish(self.reset_state)
        
        elif self.drive_state in [3, 7, 11, 15]:
            self.wheel_cmd.vel_right = robot_speed
            self.wheel_cmd.vel_left  = -1 * robot_speed
            self.reset_state.data    = True
            self.wheel_velocity_pub.publish(self.wheel_cmd)
            self.reset_rotation_calculator_pub.publish(self.reset_state)
        
        elif self.drive_state in [2, 4, 6, 8, 10, 12, 14, 16]:
            self.wheel_cmd.vel_right = 0
            self.wheel_cmd.vel_left  = 0
            self.reset_state.data    = True
            self.wheel_velocity_pub.publish(self.wheel_cmd)
            self.reset_rotation_calculator_pub.publish(self.reset_state)
        
        else:
            sys.exit
        
        return

if __name__ == "__main__":
    try:
        rospy.init_node('circular_wheel_driver', anonymous=True)
        wheel_driver = WheelDriver("/lab2/drive_state",
                                   "/beezchurger/wheels_driver_node/wheels_cmd_executed",
                                   Int32,
                                   WheelCmd,
                                   10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
