#!/usr/bin/env python3

import rospy

from pid_controller import PIDController
from std_msgs.msg import Float32

class Controller:
    def __init__(self):
        kp       = 0.00030
        ki       = 0.0017
        kd       = 0.35
        dt       = 0.1
        self.pid = PIDController(kp, ki, kd, dt)
        self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
        rospy.Subscriber("error", Float32, self.step)

    def step(self, msg):
        error = msg.data
        self.pid.step(error)
        self.pub.publish(Float32(data=self.pid.output_sum))

if __name__ ==  "__main__":
    try:
        rospy.init_node('pid_controller_node', anonymous=True)
        pid_controller = Controller()
        rate           = rospy.Rate(0.5)

        # Set the controller node to ready
        while not rospy.is_shutdown():
            if rospy.has_param("controller_ready"):
                if rospy.get_param("controller_ready") == "false":
                    rospy.loginfo("pid_controller_node: Setting 'controller_ready' to 'true'.")
                    rospy.set_param("controller_ready", "true")
                    break
            else:
                rospy.logwarn("pid_controller_node: Parameter 'controller_ready' does not exist!")
                exit()
            rate.sleep()

        rospy.loginfo("pid_controller_node: Starting controller.")

        # Run the controller until the "controller_ready" parameter is set to "false"
        while not rospy.is_shutdown():
            if rospy.has_param("controller_ready"):
                if rospy.get_param("controller_ready") == "false":
                    rospy.logwarn("pid_controller_node: Controller has been stopped.")
                    exit()
            pass

    except rospy.ROSInterruptException:
        pass
