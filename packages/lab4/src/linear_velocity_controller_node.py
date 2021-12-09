#!/usr/bin/env python3

import rospy

from lib.pid_controller import PIDController
from std_msgs.msg       import Float32

class Controller:
    def __init__(self, Kp, Ki, Kd, dt, input_topic, output_topic):
        self.run   = False
        self.__PID = PIDController(Kp, Ki, Kd, dt)
        self.__pub = rospy.Publisher(output_topic, Float32, queue_size=10)
        rospy.Subscriber(input_topic, Float32, self.__cb_step)

    def __cb_step(self, msg):
        if self.run == False:
            return
            
        error = msg.data
        self.__PID.step(error)
        self.__pub.publish(Float32(data=self.__PID.output_sum))

if __name__ == "__main__":
    try:
        rospy.init_node("angular_velocity_controller_node", anonymous=True)
        rate = rospy.Rate(1)

        # Get all parameters for the controller from the launch file.
        if rospy.has_param("definitions/gains/Kp"):
            Kp = rospy.get_param("definitions/gains/Kp")

        if rospy.has_param("definitions/gains/Ki"):
            Ki = rospy.get_param("definitions/gains/Ki")
        
        if rospy.has_param("definitions/gains/Kd"):
            Kd = rospy.get_param("definitions/gains/Kd")

        if rospy.has_param("definitions/periods/dt"):
            dt = rospy.get_param("definitions/periods/dt")

        if rospy.has_param("topics/inputs/pid_error"):
            input_topic = rospy.get_param("topics/inputs/pid_error")

        if rospy.has_param("topics/outputs/pid_out"):
            output_topic = rospy.get_param("topics/outputs/pid_out")

        # Create the PID controller using the launch file parameters.
        pid_controller = Controller(Kp, Ki, Kd, dt, input_topic, output_topic)

        # Set the linear velocity controller to ready.
        while not rospy.is_shutdown():
            if rospy.has_param("/flags/linear_controller_ready"):
                if rospy.get_param("/flags/linear_controller_ready") == "false":
                    rospy.logwarn("linear_velocity_controller_node: Linear velocity PID controller started!")
                    rospy.set_param("/flags/linear_controller_ready", "true")
                    break
            else:
                rospy.logwarn("linear_velocity_controller_node: Parameter 'topics/inputs/start' does not exist!")
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

                # Wait until param says wheel driver is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/wheel_driver_ready"):
                        if rospy.get_param("/flags/wheel_driver_ready") == "true":
                            break
                    rospy.logwarn("Waiting for /flags/wheel_driver_ready to be true")
                    rate.sleep()

        # Create the PID controller using the launch file parameters.
        pid_controller = Controller(Kp, Ki, Kd, dt, input_topic, output_topic)
        
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass


        
