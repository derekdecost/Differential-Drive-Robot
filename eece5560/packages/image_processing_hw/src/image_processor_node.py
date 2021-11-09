#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge       import CvBridge

class ImageProcessor:
    def __init__(self, image_percentage=0.45):
        self.__image_percentage = image_percentage

        self.bridge             = CvBridge()
        self.cropped_img_pub    = rospy.Publisher("image_cropped", Image, queue_size=10)
        self.white_img_pub      = rospy.Publisher("image_white",   Image, queue_size=10)
        self.yellow_img_pub     = rospy.Publisher("image_yellow",  Image, queue_size=10)
        rospy.Subscriber("image",         Image, self.__cb_cropper)
        rospy.Subscriber("image_cropped", Image, self.__cb_filter_white)
        rospy.Subscriber("image_cropped", Image, self.__cb_filter_yellow)

    def __cb_cropper(self, msg):
        cv_img  = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img  = cv_img[int(cv_img.shape[0] - (cv_img.shape[0] * self.__image_percentage)):cv_img.shape[0], 0:cv_img.shape[1]]
        
        ros_img = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
        self.cropped_img_pub.publish(ros_img)

    def __cb_filter_white(self, msg):
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        erode_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cv_img        = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img        = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        filtered_img  = cv2.inRange(cv_img, (0, 0, 200), (180, 50, 255))
        filtered_img  = cv2.erode(filtered_img,  erode_kernel)
        filtered_img  = cv2.dilate(filtered_img, dilate_kernel)

        ros_img       = self.bridge.cv2_to_imgmsg(filtered_img, "passthrough")
        self.white_img_pub.publish(ros_img)

    def __cb_filter_yellow(self, msg):
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        erode_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cv_img        = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img        = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        filtered_img  = cv2.inRange(cv_img, (24, 155, 210), (32, 255, 255))
        filtered_img  = cv2.erode(filtered_img,  erode_kernel)
        filtered_img  = cv2.dilate(filtered_img, dilate_kernel)
        
        ros_img       = self.bridge.cv2_to_imgmsg(filtered_img, "passthrough")
        self.yellow_img_pub.publish(ros_img)


if __name__ == "__main__":
    rospy.init_node("image_processor_node")
    ImageProcessor()
    while not rospy.is_shutdown():
        pass