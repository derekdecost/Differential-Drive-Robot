#!/usr/bin/env python3

#TODO: Try using the message_filters library on my laptop.
import sys
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge       import CvBridge

class LaneDetector:
    def __init__(self): 
        self.bridge = CvBridge()
        self.cropped_edges_pub = rospy.Publisher("image_edges",        Image, queue_size=10)
        self.white_lines_pub   = rospy.Publisher("image_lines_white",  Image, queue_size=10)
        self.yellow_lines_pub  = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.all_lines_pub     = rospy.Publisher("image_lines_all",    Image, queue_size=10)
        rospy.Subscriber("image_cropped", Image, self.__cb_cropped_lane_detector)
        rospy.Subscriber("image_white",   Image, self.__cb_white_lane_detector)
        rospy.Subscriber("image_yellow",  Image, self.__cb_yellow_lane_detector)
    
    def __cb_cropped_lane_detector(self, msg):
        return    

    def __cb_white_lane_detector(self, msg):
        return

    def __cb_yellow_lane_detector(self, msg):
        return