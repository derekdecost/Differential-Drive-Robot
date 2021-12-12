#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg        import Float32
from duckietown_msgs.msg import AprilTagDetectionArray

V_THRESHOLD_H = -0.01
V_THRESHOLD_L =  0.01

OMEGA_THRESHOLD_H =  0.01
OMEGA_THRESHOLD_L = -0.01

class PositionalTracker:
    def __init__(self):
        self.__x_goal =  0.0    # Goal to make robot face tag head on.
        self.__z_goal =  0.1    # Goal to get robot 10cm from tag.

        if rospy.has_param("topics/inputs/apriltag_position"):
            rospy.Subscriber(rospy.get_param("topics/inputs/apriltag_position"), AprilTagDetectionArray, self.__cb_apriltag_position, queue_size=1)

        if rospy.has_param("topics/outputs/v_error"):
            self.__v_error_pub = rospy.Publisher(rospy.get_param("topics/outputs/v_error"), Float32, queue_size=10)

        if rospy.has_param("topics/outputs/omega_error"):
            self.__omega_error_pub = rospy.Publisher(rospy.get_param("topics/outputs/omega_error"), Float32, queue_size=10)

    def __cb_apriltag_position(self, msg):
        v_error     = 0.0
        omega_error = 0.0
        detections  = msg.detections

        # If there are no detected april tags are found, stop the robot.
        if len(detections) == 0:
            self.__v_error_pub.publish(Float32(data=0))
            self.__omega_error_pub.publish(Float32(data=0))
            return

        # Find the april tag associated with the selected sign.
        for apriltag in detections:
            # Skip all april tags that do not have the specified id_tag value.
            if apriltag.tag_id == 96:
                # Calculate errors for linear and angular velocity and publish.
                # If the calculated error is within a certain threshold, then the 
                # error will be treated as zero (0).
                v_error     = apriltag.transform.translation.z - self.__z_goal
                if v_error > V_THRESHOLD_L and v_error < V_THRESHOLD_H:
                    v_error = 0

                omega_error = self.__x_goal - apriltag.transform.translation.x
                if omega_error > OMEGA_THRESHOLD_L and omega_error < OMEGA_THRESHOLD_H:
                    omega_error = 0
                
                # Publish the error values.
                self.__v_error_pub.publish(Float32(data=v_error))
                self.__omega_error_pub.publish(Float32(data=omega_error))
                break
        return

if __name__ == "__main__":
    try:
        rospy.init_node("positional_tracker_node", anonymous=True)
        rate = rospy.Rate(1)

        # Set the positional tracker node to ready
        while not rospy.is_shutdown():
            if rospy.has_param("/flags/positional_tracker_ready"):
                if rospy.get_param("/flags/positional_tracker_ready") == "false":
                    rospy.logwarn("positional_tracker_node: Positional tracker started!")
                    rospy.set_param("/flags/positional_tracker_ready", "true")
                    break
            else:
                rospy.logwarn("positional_tracker_node: Parameter '/flags/positional_tracker_ready' does not exist!")
                exit()
            rate.sleep()

        # Check if the test flag is set, if not, check for the readiness of other nodes.
        if rospy.has_param("/flags/test"):
            if rospy.get_param("/flags/test") == "false":
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

                # Wait until param says wheel driver is ready
                while not rospy.is_shutdown():
                    if rospy.has_param("/flags/wheel_driver_ready"):
                        if rospy.get_param("/flags/wheel_driver_ready") == "true":
                            break
                    rospy.logwarn("Waiting for /flags/wheel_driver_ready to be true")
                    rate.sleep()
        
        # Start the posiitional tracker once all nodes have been started.
        positional_tracker = PositionalTracker()

        while not rospy.is_shutdown():
            pass
    
    except rospy.ROSInterruptException:
        pass