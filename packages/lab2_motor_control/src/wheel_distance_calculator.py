#!/usr/bin/env python3

from _typeshed import Self
import rospy
import math

from duckietown_msgs.msg import WheelEncoderStamped
from odometry_msgs.msg import DistWheel

WHEEL_DIAMETER      = 0.065
WHEEL_RADIUS        = WHEEL_DIAMETER / 2
WHEEL_CIRCUMFERANCE = WHEEL_DIAMETER * math.pi

class WheelDistanceCalculator:
    def __init__(self):
        rospy.Subscriber("/beezchurger/left_wheel_encoder_node/tick", WheelEncoderStamped, self.__cb_left_wheel)
        rospy.Subscriber("/beezchurger/right_wheel_encoder_node/tick", WheelEncoderStamped, self.__cb_right_wheel)
        self.pub      = rospy.Publisher("/lab2/wheel_distance_delta", DistWheel, queue_size=10)
        self.l_ticks  = 0
        self.r_ticks  = 0

    def __cb_left_wheel(self, msg):
        self.l_ticks = self.l_ticks + 1
        return

    def __cb_right_wheel(self, msg):
        self.r_ticks = self.r_ticks + 1
        return

    def reset(self):
        self.l_ticks = 0
        self.r_ticks = 0
        return


if __name__ == "__main__":
    try:
        rospy.init('wheel_distance_calculator', anonymous=True)
        wdc  = WheelDistanceCalculator()
        wd   = DistWheel()
        rate = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            wd.dist_wheel_left  = WHEEL_CIRCUMFERANCE * (wdc.l_ticks / 135)
            wd.dist_wheel_right = WHEEL_CIRCUMFERANCE * (wdc.r_ticks / 135)
            wdc.pub.publish(wd)
            wdc.reset()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
