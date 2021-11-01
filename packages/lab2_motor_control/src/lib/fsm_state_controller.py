#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import Int16

class StateController:
    def __init__(self, parent):
        rospy.init_node(f"fsm_{parent}", anonymous=True)
        rospy.Subscriber("/fsm_state", Int16, self.__cb_state_actions)
        self.state_controller_pub = rospy.Publisher("/fsm_state", Int16, queue_size=10)
        self.state = Int16()
        self.state.data = 0
        self.state_controller_pub.publish(self.state)
        self.parent = parent

    def __cb_state_actions(self, msg):
        if msg.data == 0:
            return 0

        # Kill all nodes state.
        if msg.data == -1:
            print(f"Killing Node {self.parent}")
            sys.exit

        return

    def kill_all_nodes(self):
        self.state.data = -1
        self.state_controller_pub.publish(self.state)
        return

        