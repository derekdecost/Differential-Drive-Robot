#!/usr/bin/env python3

import rospy
import math

from lib.wheel_driver         import WheelDriver
from lib.fsm_state_controller import StateController

# All values are in meters unless specified otherwise.
# Physical dimensions of the robot.
AXLE_LENGTH              = 0.1
WHEEL_LENGTH_FROM_CENTER = 0.05

# Circular path dimensions.
PATH_DIAMETER            = 1   
PATH_RADIUS              = 0.5 

ROBOT_CENTER_RADIUS      = PATH_RADIUS
ROBOT_LEFT_WHEEL_RADIUS  = ROBOT_CENTER_RADIUS - WHEEL_LENGTH_FROM_CENTER
ROBOT_RIGHT_WHEEL_RADIUS = ROBOT_CENTER_RADIUS + WHEEL_LENGTH_FROM_CENTER

PATH_CENTER_LENGTH       = PATH_DIAMETER * math.pi 
PATH_LEFT_LENGTH         = ROBOT_LEFT_WHEEL_RADIUS  * 2 * math.pi
PATH_RIGHT_LENGTH        = ROBOT_RIGHT_WHEEL_RADIUS * 2 * math.pi

# Velocity ratios.
VELOCITY_RATIO_LEFT      = ROBOT_LEFT_WHEEL_RADIUS  / ROBOT_CENTER_RADIUS
VELOCITY_RATIO_RIGHT     = ROBOT_RIGHT_WHEEL_RADIUS / ROBOT_CENTER_RADIUS

if __name__ == "__main__":
    duckiebot_sc = StateController(parent="fsm_path_driver")

    # Get the path type that the robot will travel.
    if not rospy.has_param("/lab2/path_type"):
            StateController.kill_all_nodes()
    else:
        path_type = rospy.get_param("/lab2/path_type")
    
    duckiebot_wd = WheelDriver()
    if path_type == "Line":
        # Drive straight for 1m at 0.25m/s then exit.
        while not rospy.is_shutdown():
            duckiebot_wd.drive(0.25, l_distance=1, r_distance=1)
            duckiebot_sc.kill_all_nodes()

    elif path_type == "Square":
        # Drive in a 1m square, then kill all nodes.
        while not rospy.is_shutdown():
            # Drive straight for 1m at 0.25m/s.
            duckiebot_wd.drive(0.25, l_distance=1, r_distance=1)

            # Turn in place 90 degrees CCW.
            duckiebot_wd.turn_in_place_ccw(0.1, angle=(math.pi / 2))

            # Drive straight for 1m at 0.25m/s.
            duckiebot_wd.drive(0.25, l_distance=1, r_distance=1)

            # Turn in place 90 degrees CCW.
            duckiebot_wd.turn_in_place_ccw(0.1, angle=(math.pi / 2))

            # Drive straight for 1m at 0.25m/s.
            duckiebot_wd.drive(0.25, l_distance=1, r_distance=1)

            # Turn in place 90 degrees CCW
            duckiebot_wd.turn_in_place_ccw(0.1, angle=(math.pi / 2))

            # Drive straight for 1m at 0.25m/s.
            duckiebot_wd.drive(0.25, l_distance=1, r_distance=1)

            # Turn in place 90 degrees CCW.
            duckiebot_wd.turn_in_place_ccw(0.1, angle=(math.pi / 2))

            # Stop the duckiebot. The duckiebot should be in its starting position and angle.
            duckiebot_wd.stop()

            # Kill all nodes upon completion of the 1m square.
            duckiebot_sc.kill_all_nodes()

    elif path_type == "Circle":
        while not rospy.is_shutdown():
            # Drive in a 1m diameter circle at 0.25m/s then kill all nodes.
            duckiebot_wd.drive(0.25, l_ratio=VELOCITY_RATIO_LEFT, r_ratio=VELOCITY_RATIO_RIGHT, l_distance=PATH_LEFT_LENGTH, r_distance=PATH_RIGHT_LENGTH)
            duckiebot_sc.kill_all_nodes()


