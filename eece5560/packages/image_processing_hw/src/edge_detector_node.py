#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge       import CvBridge

class EdgeDetector:
    def __init__(self): 
        self.w_lines = None
        self.y_lines = None
        self.bridge = CvBridge()
        self.cropped_edges_pub = rospy.Publisher("image_edges",        Image, queue_size=10)
        self.white_lines_pub   = rospy.Publisher("image_lines_white",  Image, queue_size=10)
        self.yellow_lines_pub  = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.all_lines_pub     = rospy.Publisher("image_lines_all",    Image, queue_size=10)
        rospy.Subscriber("image_cropped", Image, self.__cb_cropped_lane_detector)
        rospy.Subscriber("image_white",   Image, self.__cb_white_lane_detector)
        rospy.Subscriber("image_yellow",  Image, self.__cb_yellow_lane_detector)
    
    def __cb_cropped_lane_detector(self, msg):
        # Convert to a CV image and apply the Canny edge detection.
        cv_img   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_edges = cv2.Canny(cv_img, 100, 200)

        # Convert to a ROS Image message and publish.
        ros_img  = self.bridge.cv2_to_imgmsg(cv_edges, "passthrough")
        self.cropped_edges_pub.publish(ros_img)

        while (self.w_lines is None) and (self.y_lines is None):
            pass

        # Publish the white detected lines overlaying the cropped image.
        w_overlay = self.output_lines(cv_img, self.w_lines)
        w_overlay = self.bridge.cv2_to_imgmsg(w_overlay, "bgr8")
        self.white_lines_pub.publish(w_overlay)

        # Publish the yellow detected lines overlaying the cropped image.
        y_overlay = self.output_lines(cv_img, self.y_lines)
        y_overlay = self.bridge.cv2_to_imgmsg(y_overlay, "bgr8")
        self.yellow_lines_pub.publish(y_overlay)

        # Publish both the white and yellow detected lines overlaying the cropped image.
        all_overlay = self.output_lines(cv_img, self.w_lines)
        all_overlay = self.output_lines(all_overlay, self.y_lines)
        all_overlay = self.bridge.cv2_to_imgmsg(all_overlay, "bgr8")
        self.all_lines_pub.publish(all_overlay)

        return    

    def __cb_white_lane_detector(self, msg):
        # Convert to a CV image and apply the Canny edge detection.
        cv_img   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img   = cv2.GaussianBlur(cv_img, (5,5), 0)
        cv_edges = cv2.Canny(cv_img, 100, 200)
        cv_edges = cv2.GaussianBlur(cv_edges, (3, 3), 0)
        cv_img   = cv2.bitwise_and(cv_img, cv_img, mask=cv_edges)
        #TODO: Apply dilate to the edges

        # Apply the Hough transform and combine the lines with the lane image.
        hough_lines  = cv2.HoughLinesP(cv_edges, 1, np.pi/180, 1, minLineLength=20, maxLineGap=4)
        self.w_lines = hough_lines
        cv_img       = self.output_lines(cv_img, hough_lines)

        # Convert to a ROS Image message and publish.
        ros_img  = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")

        return

    def __cb_yellow_lane_detector(self, msg):
        # Convert to a CV image and apply the Canny edge detection.
        cv_img   = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_img   = cv2.GaussianBlur(cv_img, (5,5), 0)
        cv_edges = cv2.Canny(cv_img, 100, 200)
        cv_edges = cv2.GaussianBlur(cv_edges, (3, 3), 0)
        cv_img   = cv2.bitwise_and(cv_img, cv_img, mask=cv_edges)
        #TODO: Apply dilate to the edges

        # Apply the Hough transform and combine the lines with the lane image.
        hough_lines  = cv2.HoughLinesP(cv_edges, 1, np.pi/180, 4, minLineLength=15, maxLineGap=2)
        self.y_lines = hough_lines
        cv_img       = self.output_lines(cv_img, hough_lines)

        # Convert to a ROS Image message and publish.
        ros_img  = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")

        return

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

if __name__ == "__main__":
    rospy.init_node("edge_detector_node")
    EdgeDetector()
    while not rospy.is_shutdown():
        pass