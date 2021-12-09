#!/usr/bin/env python3

from numpy.core.arrayprint import dtype_is_implied
import rospy
import numpy as np

from std_msgs.msg     import Float32
from lib.wheel_driver import WheelDriver

class DuckiebotDriver:
    def __init__(self):
        if rospy.has_param("topics/inputs/v_pid_in"):
            rospy.Subscriber(rospy.get_param("topics/inputs/v_pid_in"), Float32, self.__cb_update_linear_control)
        else:
            rospy.logwarn("wheel_driver_node: Parameter 'topics/inputs/v_pid_in' does not exist!")

        if rospy.has_param("topics/inputs/omega_pid_in"):
            rospy.Subscriber(rospy.get_param("topics/inputs/omega_pid_in"), Float32, self.__cb_update_angular_control)
        else:
            rospy.logwarn("wheel_driver_node: Parameter 'topics/inputs/omega_pid_in' does not exist!")

        self.__t_prev = None
        self.__t      = None

        self.__v      = 0
        self.__omega  = 0
        self.__linear_control  = 0
        self.__angular_control = 0
        self.__iterating = False
        self.__wd = WheelDriver()
        rospy

    def iterate(self):
        if (self.__t_prev is None) and (self.__t is None):
            self.__t_prev = rospy.get_time()

        try:
            self.__t      = rospy.get_time()
            dt            = self.__t - self.__t_prev
            self.__t_prev = self.__t

            self.__v     += self.__linear_control  * dt
            self.__omega += self.__angular_control * dt
            self.__wd.drive(self.__v, self.__omega)
        except:
            pass

        return

    #TODO: If there are strang patterns in running, it could result from here.
    def __cb_update_angular_control(self, msg):
        self.__angular_control = msg.data
        while(self.__iterating):
            pass
        self.__iterating = True
        self.iterate()
        self.__iterating = False
        return

    def __cb_update_linear_control(self, msg):
        self.__linear_control = msg.data
        while(self.__iterating):
            pass
        self.__iterating = True
        self.iterate()
        self.__iterating = False
        return

if __name__ == "__main__":
    try:
        rospy.init_node("wheel_driver_node", anonymous=True)
        rate = rospy.Rate(1)

        # Set the linear velocity controller to ready.
        while not rospy.is_shutdown():
            if rospy.has_param("/flags/wheel_driver_ready"):
                if rospy.get_param("/flags/wheel_driver_ready") == "false":
                    rospy.logwarn("wheel_driver_node: Linear velocity PID controller started!")
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
                    rospy.logwarn("Waiting for /flags/positional_tracker_ready to be true")
                    rate.sleep()

                # Wait until param says angular controller is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/angular_controller_ready"):
                        if rospy.get_param("/flags/angular_controller_ready") == "true":
                            break
                    rospy.logwarn("Waiting for /flags/angular_controller_ready to be true")
                    rate.sleep()

                # Wait until param says linear controller is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/linear_controller_ready"):
                        if rospy.get_param("/flags/linear_controller_ready") == "true":
                            break
                    rospy.logwarn("Waiting for /flags/linear_controller_ready to be true")
                    rate.sleep()

        # Create the WheelDriver using the launch file parameters.
        duckiebot_driver = DuckiebotDriver()

        while not rospy.is_shutdown():
            pass
    except:
        pass


    
