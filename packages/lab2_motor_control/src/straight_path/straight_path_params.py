import math
import robot_params

PATH_DIAMETER                           = 1 # meter
PATH_RADIUS                             = PATH_DIAMETER / 2
PATH_CENTER_LENGTH                      = PATH_DIAMETER * math.pi
PATH_RIGHT_LENGTH                       = 1 # meter
PATH_LEFT_LENGTH                        = 1 # meter

WHEEL_ROTATIONS_TO_COMPLETE_CENTER_PATH = PATH_CENTER_LENGTH / robot_params.WHEEL_CIRCUMFERANCE
WHEEL_ROTATIONS_TO_COMPLETE_RIGHT_PATH  = PATH_RIGHT_LENGTH / robot_params.WHEEL_CIRCUMFERANCE
WHEEL_ROTATIONS_TO_COMPLETE_LEFT_PATH   = PATH_LEFT_LENGTH / robot_params.WHEEL_CIRCUMFERANCE