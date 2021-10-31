import rospy

from rotation_calculator import RotationCalculator
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float32

if __name__ == "__main__":
    try:
        rospy.init('left_wheel_rotation_calculator', anonymous=True)
        rotation_calculator = RotationCalculator("/beezchurger/left_wheel_encoder_node/tick",
                                                 "/lab2/left_wheel_rotations",
                                                 WheelEncoderStamped,
                                                 Float32,
                                                 10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass