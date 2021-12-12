#!/usr/bin/env python3

import rospy
import numpy as np

from lib.pid_controller  import PIDController
from lib.wheel_driver    import WheelDriver
from std_msgs.msg        import Float32
from duckietown_msgs.msg import LanePose
from duckietown_msgs.msg import FSMState

class LanePoseTracker:
    def __init__(self):
        # Goals
        self.__d_goal   = 0.0
        self.__phi_goal = 0.0

        # FSM Control
        self.duckiebot_fsm_state                    = FSMState()
        self.duckiebot_fsm_state.header.seq         = 0
        self.duckiebot_fsm_state.header.stamp.secs  = 0
        self.duckiebot_fsm_state.header.stamp.nsecs = 0
        self.duckiebot_fsm_state.header.frame_id    = ""
        self.duckiebot_fsm_state.state = self.duckiebot_fsm_state.LANE_FOLLOWING
        self.duckiebot_current_state   = self.duckiebot_fsm_state.NORMAL_JOYSTICK_CONTROL

        # Controllers
        self.__d_pid   = PIDController(rospy.get_param("gains/d/Kp"),
                                       rospy.get_param("gains/d/Ki"),
                                       rospy.get_param("gains/d/Kd"))
                                       
        self.__phi_pid = PIDController(rospy.get_param("gains/phi/Kp"),
                                       rospy.get_param("gains/phi/Ki"),
                                       rospy.get_param("gains/phi/Kd"))

        # Drivers
        self.__wd = WheelDriver()

        # Publishers
        self.fsm_pub = rospy.Publisher("/beezchurger/fsm_node/mode", FSMState, queue_size=1)

        # Subscribers
        rospy.Subscriber("/beezchurger/lane_filter_node/lane_pose", LanePose, self.__cb_lane_position)
        rospy.Subscriber("/beezchurger/fsm_node/mode", FSMState, self.__cb_duckiebot_fsm)

    def __cb_lane_position(self, msg):
        # if msg.in_lane == False:
        #     self.__wd.stop()
        #     self.__d_pid.reset()
        #     self.__phi_pid.reset()
        #     return

        d_err, phi_err = self.calculate_errors(msg.d, msg.phi)
        self.__d_pid.step(d_err,
                          rospy.get_param("gains/d/Kp"),
                          rospy.get_param("gains/d/Ki"),
                          rospy.get_param("gains/d/Kd"))
        self.__phi_pid.step(phi_err,
                            rospy.get_param("gains/phi/Kp"),
                            rospy.get_param("gains/phi/Ki"),
                            rospy.get_param("gains/phi/Kd"))

        omega = self.__d_pid.output_sum + self.__phi_pid.output_sum
        self.__wd.drive(0.2, omega * 10.25)
        return

    def __cb_duckiebot_fsm(self, msg):
        # Records the new FSM state when the state changes.
        self.duckiebot_current_state = msg.state
        return
    
    def calculate_errors(self, d, phi):
        # Calculate the d and phi errors, setting the error to zero (0) if 
        # the value is within a specified threshold.
        d_err   = self.__d_goal   - d
        if d_err < 0.05 and d_err > -0.05:
            d_err = 0

        phi_err = self.__phi_goal - phi
        if phi_err < 0.06 and phi_err > -0.06:
            phi_err = 0

        return d_err, phi_err

    def publish_state(self, state):
        # Set the state of the robot and publish to the fsm state topic.
        self.duckiebot_fsm_state.state = state
        self.fsm_pub.publish(self.duckiebot_fsm_state)
        return

    def stop(self):
        self.__wd.stop()
        return

if __name__ == "__main__":
    rospy.init_node("lane_pose_tracker", anonymous=True)
    
    rate = rospy.Rate(1)

    lane_pose_tracker = LanePoseTracker()
    rospy.on_shutdown(lane_pose_tracker.stop)

    # Set the duckiebot into LANE_FOLLOWING mode to allow for an external node to control the robot wheels.
    while lane_pose_tracker.duckiebot_current_state != lane_pose_tracker.duckiebot_fsm_state.LANE_FOLLOWING:
        # Publish the lane following state every second until it changes.
        lane_pose_tracker.publish_state(lane_pose_tracker.duckiebot_fsm_state.LANE_FOLLOWING)
        rospy.logwarn("Setting FSM to LANE_FOLLOWING...")
        rate.sleep()

    rospy.logwarn("wheel_driver_node: Duckiebot FSM state set to LANE_FOLLOWING mode.")

    while not rospy.is_shutdown():
        pass