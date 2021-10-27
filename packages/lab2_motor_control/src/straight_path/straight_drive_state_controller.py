import rospy
import straight_path_params

from std_msgs.msg import Float32
from std_msgs.msg import Bool

class DriveStateController:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        rospy.Subscriber(input_topic[0], input_type, self.right_wheel_callback)
        rospy.Subscriber(input_topic[1], input_type, self.left_wheel_callback)
        self.drive_state_pub = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        
        self.drive_state = Bool()
        self.drive_state.data = True

        self.r_wheel_rotations = 0
        self.l_wheel_rotations = 0

        # Start the wheels moving
        self.drive_state_pub.publish(self.drive_state)

    def right_wheel_callback(self, msg):
        self.r_wheel_rotations = msg.data
        return

    def left_wheel_callback(self, msg):
        self.l_wheel_rotations = msg.data
        return


if __name__ == "__main__":
    try:
        rospy.init_node('drive_state_controller', anonymous=True)
        drive_state_controller = DriveStateController(
            input_topic=["/lab2/right_wheel_rotations", "/lab2/left_wheel_rotations"],
            output_topic="/lab2/drive_state",
            input_type=Float32,
            output_type=Bool,
            queue_val=10)

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            if (drive_state_controller.r_wheel_rotations >= straight_path_params.WHEEL_ROTATIONS_TO_COMPLETE_RIGHT_PATH) and (drive_state_controller.l_wheel_rotations >= straight_path_params.WHEEL_ROTATIONS_TO_COMPLETE_LEFT_PATH):
                drive_state_controller.drive_state.data = False
                drive_state_controller.drive_state_pub.publish(drive_state_controller.drive_state)

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

