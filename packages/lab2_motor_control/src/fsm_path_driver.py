#!/usr/bin/env python3

import rospy
import math
import sys

from std_msgs.msg import Bool
from lib.wheel_driver    import WheelDriver
from duckietown_msgs.msg import FSMState
# from lib.fsm_state_controller import StateController

# All values are in meters unless specified otherwise.
# Physical dimensions of the robot.
AXLE_LENGTH              = 0.07
WHEEL_LENGTH_FROM_CENTER = 0.035

# Circular path dimensions.
PATH_DIAMETER            = 1   
PATH_RADIUS              = 0.5 

ROBOT_CENTER_RADIUS      = PATH_RADIUS
ROBOT_LEFT_WHEEL_RADIUS  = ROBOT_CENTER_RADIUS - WHEEL_LENGTH_FROM_CENTER
ROBOT_RIGHT_WHEEL_RADIUS = ROBOT_CENTER_RADIUS + WHEEL_LENGTH_FROM_CENTER

PATH_CENTER_LENGTH       = PATH_DIAMETER * math.pi 
PATH_LEFT_LENGTH         = ROBOT_LEFT_WHEEL_RADIUS  * 1.2 * math.pi
PATH_RIGHT_LENGTH        = ROBOT_RIGHT_WHEEL_RADIUS * 1.2 * math.pi

# Velocity ratios.
VELOCITY_RATIO_LEFT      = ROBOT_LEFT_WHEEL_RADIUS  / ROBOT_CENTER_RADIUS
VELOCITY_RATIO_RIGHT     = ROBOT_RIGHT_WHEEL_RADIUS / ROBOT_CENTER_RADIUS

class DuckiebotFSM:
    def __init__(self):
        rospy.Subscriber("/beezchurger/fsm_node/mode", FSMState, self.__cb_duckiebot_fsm)
        rospy.Subscriber("/lab2/wdc_ready", Bool, self.__cb_wdc_ready)
        self.fsm_pub            = rospy.Publisher("/beezchurger/fsm_node/mode", FSMState, queue_size=10)
        self.duckiebot_fsmstate = FSMState()
        self.duckiebot_fsmstate.header.seq         = 0
        self.duckiebot_fsmstate.header.stamp.secs  = 0
        self.duckiebot_fsmstate.header.stamp.nsecs = 0
        self.duckiebot_fsmstate.header.frame_id    = ""
        self.duckiebot_fsmstate.state = self.duckiebot_fsmstate.LANE_FOLLOWING
        self.duckiebot_current_state  = self.duckiebot_fsmstate.NORMAL_JOYSTICK_CONTROL
        self.wdc_ready = False

    def publish_state(self, state):
        self.duckiebot_fsmstate.state = state
        self.fsm_pub.publish(self.duckiebot_fsmstate)
        return

    def __cb_duckiebot_fsm(self, msg):
        self.duckiebot_current_state = msg.state
        return

    def __cb_wdc_ready(self, msg):
        self.wdc_ready = msg.data
        return

if __name__ == "__main__":
    rospy.init_node('fsm_path_driver', anonymous=True)

    # Get the path type that the robot will travel.
    if not rospy.has_param("/lab2/path_type"):
        sys.exit
    else:
        path_type = rospy.get_param("/lab2/path_type")
    
    poll_rate = rospy.Rate(2)
    rate      = rospy.Rate(1)
    duckiebot_fsm = DuckiebotFSM()
    duckiebot_wd  = WheelDriver()
    rospy.on_shutdown(duckiebot_wd.stop)

    while duckiebot_fsm.duckiebot_current_state != duckiebot_fsm.duckiebot_fsmstate.LANE_FOLLOWING:
        # Publish the lane following state every second until it changes.
        duckiebot_fsm.publish_state(duckiebot_fsm.duckiebot_fsmstate.LANE_FOLLOWING)
        poll_rate.sleep()

    while duckiebot_fsm.wdc_ready == False:
        poll_rate.sleep()

    if path_type == "Line":
        print("Driving in 1m line.")
        
        duckiebot_wd.drive(0.2, 0, 1, l_distance=0.8, r_distance=0.8)
        sys.exit()

    elif path_type == "Square":
        print("Driving in 1m square")

        # Drive straight for 1m at 0.25m/s.
        duckiebot_wd.drive(0.3, 0, 1, l_distance=0.7, r_distance=0.9)
        rate.sleep()
        # Turn in place 90 degrees CCW.
        duckiebot_wd.turn_in_place(speed=6, angle=(math.pi / 2))
        rate.sleep()
        # Drive straight for 1m at 0.25m/s.
        duckiebot_wd.drive(0.3, 0, 1, l_distance=0.7, r_distance=0.9)
        # rate.sleep()
        # Turn in place 90 degrees CCW.
        duckiebot_wd.turn_in_place(speed=6, angle=(math.pi / 2))
        rate.sleep()
        # Drive straight for 1m at 0.25m/s.
        duckiebot_wd.drive(0.3, 0, 1, l_distance=0.7, r_distance=0.9)
        rate.sleep()
        # Turn in place 90 degrees CCW
        duckiebot_wd.turn_in_place(speed=6, angle=(math.pi / 2))
        rate.sleep()
        # Drive straight for 1m at 0.25m/s.
        duckiebot_wd.drive(0.3, 0, 1, l_distance=0.7, r_distance=0.9)
        rate.sleep()
        # Turn in place 90 degrees CCW.
        duckiebot_wd.turn_in_place(speed=6, angle=(math.pi / 2))
        rate.sleep()
        # Stop the duckiebot. The duckiebot should be in its starting position and angle.
        duckiebot_wd.stop()
        sys.exit()
            
    elif path_type == "Circle":
        print("Driving in 1m diameter circle")

        duckiebot_wd.drive(0.3, (math.pi / 2), 2, l_distance=PATH_LEFT_LENGTH, r_distance=PATH_RIGHT_LENGTH, omega=2.5)
        sys.exit()

    elif path_type == "TIP Test":
        print("Running Turn-In-Place Test")

        # duckiebot_wd.drive(0.1, 0, 1)
        # rate.sleep()
        # duckiebot_wd.stop()
        duckiebot_wd.turn_in_place(speed=5, angle=(math.pi / 2))
        # duckiebot_wd.drive(0, 0, 0, omega=(math.pi / 2))
        while not rospy.is_shutdown():
            pass
        # rate.sleep()
        # duckiebot_wd.stop()
        sys.exit

