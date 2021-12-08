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
        
        #TODO: Set angular PID to ready
        #TODO: Check for wheel driver to be ready.
        while not rospy.is_shutdown():
            if rospy.has_param("topics/inputs/start"):
                if rospy.get_param("topics/inputs/start") == True:
                    rospy.logwarn("angular_velocity_controller_node: Linear velocity PID controller started!")
                    pid_controller.run = True
                    break
            else:
                rospy.logwarn("angular_velocity_controller_node: Parameter 'topics/inputs/start' does not exist!")
                exit()
            pass
            
        while not rospy.is_shutdown():
            if rospy.has_param("topics/inputs/start"):
                if rospy.get_param("topics/inputs/start") == False:
                    rospy.logwarn("angular_velocity_controller_node: Linear velocity PID controller has stopped!")
                    pid_controller.run = False
                    break

    except rospy.ROSInterruptException:
        pass


        
