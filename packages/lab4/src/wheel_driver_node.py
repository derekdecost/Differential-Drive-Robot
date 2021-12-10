#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg        import Float32
from lib.wheel_driver    import WheelDriver
from duckietown_msgs.msg import FSMState

class DuckiebotDriver:
    def __init__(self):
        self.__linear_control   = 0
        self.__angular_control  = 0
        self.__wd                = WheelDriver()
        self.duckiebot_fsm_state = FSMState()
        self.duckiebot_fsm_state.state = self.duckiebot_fsm_state.LANE_FOLLOWING
        self.duckiebot_current_state   = self.duckiebot_fsm_state.NORMAL_JOYSTICK_CONTROL

        # Topics used to set the state to drive programatically.
        rospy.Subscriber("/beezchurger/fsm_node/mode", FSMState, self.__cb_duckiebot_fsm)
        self.fsm_pub = rospy.Publisher("/beezchurger/fsm_node/mode", FSMState, queue_size=1)

        if rospy.has_param("topics/inputs/v_pid_in"):
            rospy.Subscriber(rospy.get_param("topics/inputs/v_pid_in"), Float32, self.__cb_update_linear_control)
        else:
            rospy.logwarn("wheel_driver_node: Parameter 'topics/inputs/v_pid_in' does not exist!")

        if rospy.has_param("topics/inputs/omega_pid_in"):
            rospy.Subscriber(rospy.get_param("topics/inputs/omega_pid_in"), Float32, self.__cb_update_angular_control)
        else:
            rospy.logwarn("wheel_driver_node: Parameter 'topics/inputs/omega_pid_in' does not exist!")

    def publish_state(self, state):
        # Set the state of the robot and publish to the fsm state topic.
        self.duckiebot_fsm_state.state = state
        self.fsm_pub.publish(self.duckiebot_fsm_state)
        return
        
    def __cb_duckiebot_fsm(self, msg):
        # Records the new FSM state when the state changes.
        self.duckiebot_current_state = msg.state
        return

    def __cb_update_angular_control(self, msg):
        # Saves the new value for the angular control and sets the wheel speeds.
        self.__angular_control += msg.data
        self.__wd.drive(self.__linear_control, self.__angular_control)
        return

    def __cb_update_linear_control(self, msg):
        # Saves the new value for the linear control and sets the wheel speeds.
        self.__linear_control += msg.data
        self.__wd.drive(self.__linear_control, self.__angular_control)
        return

if __name__ == "__main__":
    try:
        rospy.init_node("wheel_driver_node", anonymous=True)
        rate = rospy.Rate(1)

        # Set the linear velocity controller to ready.
        while not rospy.is_shutdown():
            if rospy.has_param("/flags/wheel_driver_ready"):
                if rospy.get_param("/flags/wheel_driver_ready") == "false":
                    rospy.logdebug("wheel_driver_node: Linear velocity PID controller started!")
                    rospy.set_param("/flags/wheel_driver_ready", "true")
                    break
            else:
                rospy.logwarn("wheel_driver_node: Parameter '/flags/wheel_driver_ready' does not exist!")
                exit()
            rate.sleep()

        # Check if the test flag is set, if not, check for the readiness of other nodes.
        if rospy.has_param("/flags/test"):
            if rospy.get_param("/flags/test") == "false":
                # Wait until param says positional tracker is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/positional_tracker_ready"):
                        if rospy.get_param("/flags/positional_tracker_ready") == "true":
                            break
                    else:
                        rospy.logwarn("wheel_driver_node: Parameter '/flags/positional_tracker_ready' does not exist!")
                        
                    rospy.logwarn("Waiting for /flags/positional_tracker_ready to be true")
                    rate.sleep()

                # Wait until param says angular controller is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/angular_controller_ready"):
                        if rospy.get_param("/flags/angular_controller_ready") == "true":
                            break
                    else:
                        rospy.logwarn("wheel_driver_node: Parameter '/flags/angular_controller_ready' does not exist!")
                    rospy.logdebug("Waiting for /flags/angular_controller_ready to be true")
                    rate.sleep()

                # Wait until param says linear controller is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/linear_controller_ready"):
                        if rospy.get_param("/flags/linear_controller_ready") == "true":
                            break
                    else:
                        rospy.logwarn("wheel_driver_node: Parameter '/flags/linear_controller_ready' does not exist!")

                    rospy.logdebug("Waiting for /flags/linear_controller_ready to be true")
                    rate.sleep()

        

        # Create the WheelDriver using the launch file parameters.
        duckiebot_driver = DuckiebotDriver()

        # Set the duckiebot into LANE_FOLLOWING mode to allow for an external node to control the robot wheels.
        while duckiebot_driver.duckiebot_current_state != duckiebot_driver.duckiebot_fsm_state.LANE_FOLLOWING:
            # Publish the lane following state every second until it changes.
            duckiebot_driver.publish_state(duckiebot_driver.duckiebot_fsm_state.LANE_FOLLOWING)
            rate.sleep()
        rospy.logdebug("wheel_driver_node: Duckiebot FSM state set to LANE_FOLLOWING mode.")

        while not rospy.is_shutdown():
            pass
    except:
        pass


    
