#!/usr/bin/env python3

import rospy
import actionlib
import hw10.msg
from hw10.srv import *

def fib_srv(fib_cnt):
    rospy.wait_for_service("calc_fibonacci")
    fib_seq = rospy.ServiceProxy("calc_fibonacci", Fibonacci)
    rospy.loginfo("fib_srv: Requesting service...")

    try:
        
        fib_recv = fib_seq(fib_cnt)
        rospy.loginfo("fib_srv: Request completed!")
    except rospy.ServiceException as exception:
        rospy.logwarn(f"fib_srv: Service returned exception '{str(exception)}'")

    rospy.loginfo("fib_srv: Service completed!")
    return fib_recv

def fib_action(fib_cnt):
    client = actionlib.SimpleActionClient("fibonacci", hw10.msg.FibonacciAction)
    client.wait_for_server()
    goal = hw10.msg.FibonacciGoal(order=fib_cnt)

    rospy.loginfo("fib_action: Requesting service...")
    client.send_goal(goal)
    rospy.loginfo("fib_action: Request completed!")
    rospy.loginfo("fib_action: Waiting for result...")
    client.wait_for_result()
    rospy.loginfo("fib_action: Result received!")
    return client.get_result()

if __name__ == "__main__":
    rospy.init_node("hw10_client_node", anonymous=True)
    rospy.loginfo("Requesting service 3")
    fib_srv(3)
    rospy.loginfo("Requesting service 15")
    fib_srv(15)
    rospy.loginfo("Requesting action 3")
    fib_action(3)
    rospy.loginfo("Requesting action 15")
    fib_action(15)