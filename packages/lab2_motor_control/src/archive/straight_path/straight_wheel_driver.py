import rospy
import sys
import robot_params
import time

from std_msgs.msg import Bool
from duckietown_msgs.msg import WheelCmd


class WheelDriver:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        rospy.Subscriber(input_topic, input_type, self.drive_state_callback)
        self.wheel_velocity_pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        self.wheel_cmd = WheelCmd()
        self.wheel_cmd.vel_right = 0
        self.wheel_cmd.vel_left  = 0
        self.wheel_velocity_pub.publish(self.wheel_cmd)
        self.drive_state = False

    def drive_state_callback(self, msg):
        self.drive_state = msg.data
        if self.drive_state == False:
            self.wheel_cmd.vel_right = 0
            self.wheel_cmd.vel_left  = 0
            self.wheel_velocity_pub.publish(self.wheel_cmd)
            sys.exit()
        return

if __name__ == "__main__":
    try:
        rospy.init_node('circular_wheel_driver', anonymous=True)
        robot_speed = 1 # m/s (Assumed)
        wheel_driver = WheelDriver("/lab2/drive_state",
                                "/beezchurger/wheels_driver_node/wheels_cmd_executed",
                                Bool,
                                WheelCmd,
                                10)

        wheel_driver.wheel_cmd.vel_right = robot_speed
        wheel_driver.wheel_cmd.vel_left  = robot_speed
        wheel_driver.wheel_velocity_pub(wheel_driver.wheel_cmd)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
