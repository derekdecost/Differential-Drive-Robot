#!/usr/bin/env bash

rosparam set /controllers/angular_velocity/definitions/gains/Kp "$1"
rosparam set /controllers/angular_velocity/definitions/gains/Ki "$2"
rosparam set /controllers/angular_velocity/definitions/gains/Kd "$3"