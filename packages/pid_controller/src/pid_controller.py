#!/usr/bin/env python3

class PIDController:
    """
    """
    def __init__(self, kp, ki, kd, dt):
        self.__kp              = kp        # Proportional weight.
        self.__ki              = ki        # Integral weight.
        self.__kd              = kd        # Differential weight.
        self.__dt              = dt        # Time interval.
        self.__error_prev      = None      # Previous calculated error.
        self.__error_integrate = 0         # Controller error.
        self.output_list       = [0, 0, 0] # Controller output list.
        self.output_sum        = 0         # Controller output value.  

    """
    @fn     step
    @brief  
    """
    def step(self, err):
        # Calculate the errors of the controller.
        error                   = err
        self.__error_integrate += error

        if self.__error_prev is None:
            error_differential = 0.0
        else:
            error_differential = (error - self.__error_prev) / self.__dt

        self.__error_prev = error

        # Controller components.
        proportional = self.__kp * error
        integral     = self.__ki * self.__error_integrate
        differential = self.__kd * error_differential

        # Assign and calculate outputs for the controller.
        self.output_list  = [proportional, integral, differential]
        self.output_sum   = sum(self.output_list)