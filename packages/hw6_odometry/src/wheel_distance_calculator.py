#!/usr/bin/env python3

import rospy
import math
from rospy.timer import Rate

# from lib.fsm_state_controller import StateController
from std_msgs.msg import Bool
from duckietown_msgs.msg      import WheelEncoderStamped
from odometry_hw.msg          import DistWheel

# Calculate wheel measurements used to calculate the distance travelled based on the number of ticks incremented
WHEEL_DIAMETER      = 0.065
WHEEL_CIRCUMFERANCE = WHEEL_DIAMETER * math.pi

class WheelDistanceCalculator:
    def __init__(self, poll_rate=2):
        rospy.Subscriber("/beezchurger/left_wheel_encoder_node/tick", WheelEncoderStamped, self.__cb_left_wheel)
        rospy.Subscriber("/beezchurger/right_wheel_encoder_node/tick", WheelEncoderStamped, self.__cb_right_wheel)
        rospy.Subscriber("/lab2/wdc_ready", Bool, self.__cb_check_ready_state)
        self.pub       = rospy.Publisher("/lab2/wheel_distance_delta", DistWheel, queue_size=10)
        self.ready_pub = rospy.Publisher("/lab2/wdc_ready", Bool, queue_size=10)
        self.ready_message      = Bool()
        self.ready_message.data = False
        self.ready_state        = None
        self.l_ticks  = 0
        self.r_ticks  = 0
        self.l_tick_prev_state = 0
        self.r_tick_prev_state = 0
        self.l_tick_initial_state = False
        self.r_tick_initial_state = False
        self.poll_rate = Rate(poll_rate)

    def set_state(self, state):
        self.ready_message.data = state
        self.ready_pub.publish(self.ready_message)
        self.poll_rate.sleep()
        while self.ready_state != self.ready_message.data or self.ready_state is None:
            self.ready_pub.publish(self.ready_message)
            self.poll_rate.sleep()

        return

    def __cb_left_wheel(self, msg):
        # Get the initial wheel tick count if not already acquired
        if self.l_tick_initial_state == False:
            self.l_tick_initial_state = True
            self.l_tick_prev_state    = msg.data

        # Calculate the number of ticks since last callback
        self.l_ticks           = self.l_ticks + (msg.data - self.l_tick_prev_state)
        self.l_tick_prev_state = msg.data
        return

    def __cb_right_wheel(self, msg):
        # Get the initial wheel tick count if not already acquired
        if self.r_tick_initial_state == False:
            self.r_tick_initial_state = True
            self.r_tick_prev_state    = msg.data
        
        # Calculate the number of ticks since last callback
        self.r_ticks           = self.r_ticks + (msg.data - self.r_tick_prev_state)
        self.r_tick_prev_state = msg.data
        return

    def __cb_check_ready_state(self, msg):
        self.ready_state = msg.data
        return

    def reset(self):
        self.l_tick_initial_state = False
        self.r_tick_initial_state = False
        self.l_ticks = 0
        self.r_ticks = 0
        return


if __name__ == "__main__":
    try:
        # duckiebot_sc = StateController("wheel_distance_calculator")
        rospy.init_node('wheel_distance_calculator', anonymous=True)
        wdc  = WheelDistanceCalculator()
        wd   = DistWheel()
        ready_pub = rospy.Publisher("/lab2/wdc_ready", Bool, queue_size=10)
        wdc.set_state(False)
        
        rate = Rate(2)

        l_wes = WheelEncoderStamped()
        r_wes = WheelEncoderStamped()
        l_wes.data = 0
        r_wes.data = 0

        # Reset the wheel encoder upon initialization
        l_tick_pub = rospy.Publisher("/beezchurger/left_wheel_encoder_node/tick", WheelEncoderStamped, queue_size=10)
        r_tick_pub = rospy.Publisher("/beezchurger/right_wheel_encoder_node/tick", WheelEncoderStamped, queue_size=10)
        l_tick_pub.publish(l_wes)
        r_tick_pub.publish(r_wes)
        
        del l_tick_pub
        del r_tick_pub
        del l_wes
        del r_wes

        
        rate = rospy.Rate(3)

        wdc.set_state(True)

        while not rospy.is_shutdown():
            # Calculate the distance traveled as a function of the tick count
            wd.dist_wheel_left  = WHEEL_CIRCUMFERANCE * (wdc.l_ticks / 135)
            wd.dist_wheel_right = WHEEL_CIRCUMFERANCE * (wdc.r_ticks / 135)
            
            wdc.pub.publish(wd)
            wdc.reset()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
