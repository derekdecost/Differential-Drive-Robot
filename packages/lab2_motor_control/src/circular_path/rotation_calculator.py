import rospy
import sys
from std_msgs.msg import Float32
from std_msgs.msg import Bool

class RotationCalculator:
    def __init__(self, input_topic, output_topic, input_type, output_type, queue_val):
        rospy.Subscriber(input_topic, input_type, self.tick_callback)
        rospy.Subscriber("/lab2/reset_rotation_calculator", Bool, self.reset)
        self.rotation_pub   = rospy.Publisher(output_topic, output_type, queue_size=queue_val)
        self.reset_rotation_calculator_pub = rospy.Publisher("/lab2/reset_rotation_calculator", Bool, queue_size=queue_val)
        
        # Topic Types
        # @param wheel_rotations - Float32 data that publishes the number of complete and partial wheel rotations.
        self.wheel_rotations  = Float32()
        self.reset_state      = Bool()
        self.reset_state.data = False
        # Parameters
        # @param tick_count       - Stores the number of ticks counted from the wheel encoders. Resets at 135.
        # @param full_rotations   - Integer value tracking the number of complete wheel rotations.
        self.tick_count      = 0
        self.full_rotations  = 0

    def tick_callback(self, msg):
        self.tick_count = self.tick_count + 1
        if self.tick_count == 135:
            self.tick_count     = 0
            self.full_rotations = self.full_rotations + 1

        self.wheel_rotations.data = self.full_rotations + (self.tick_count / 135)
        self.rotation_pub.publish(self.wheel_rotations)
        return

    def drive_state_callback(self, msg):
        if msg.data == False:
            sys.exit()
        else:
            return

    def reset(self, msg):
        if msg.data == False:
            return

        self.tick_count     = 0
        self.full_rotations = 0
        self.reset_rotation_calculator_pub.publish(self.reset_state)