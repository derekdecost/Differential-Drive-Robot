#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from duckietown_msgs.msg import SegmentList, Segment, Vector2D
from sensor_msgs.msg     import CompressedImage, Image
from cv_bridge           import CvBridge

class LaneDetector:
    def __init__(self):
        # Class constants
        self.__YELLOW_LOWER__ = ( 15,  80, 150)
        self.__YELLOW_UPPER__ = ( 50, 255, 255)
        self.__WHITE_LOWER__  = (  0,   0, 152)
        self.__WHITE_UPPER__  = (180,  70, 255)

        # Class members
        self.__bridge             = CvBridge()
        self.__arr_cutoff         = 0
        self.__arr_ratio          = 0
        self.__y_hough_lines      = None
        self.__w_hough_lines      = None
        self.__y_hough_lines_norm = None
        self.__w_hough_lines_norm = None
        self.__offset             = 0

        # Class publishers
        # self.__cropped_img_pub  = rospy.Publisher("img_cropped",  Image, queue_size=10)  # Debug Only
        # self.__white_img_pub    = rospy.Publisher("image_white",  Image, queue_size=10)  # Debug only
        # self.__yellow_img_pub   = rospy.Publisher("image_yellow", Image, queue_size=10)  # Debug only
        self.__overlay_img_pub  = rospy.Publisher("overlay",      Image, queue_size=10)
        self.__segment_list_pub = rospy.Publisher("/beezchurger/line_detector_node/segment_list", SegmentList, queue_size=10)

        # Class subscribers
        rospy.Subscriber("/beezchurger/camera_node/image/compressed", CompressedImage, self.__cb_lane_detector, queue_size=1, buff_size=2**24)

    """
        @fn     __crop_img
        @brief  Accepts an input image and crops the image to be x% height based on the img_percent parameter.

        @param  msg - An image message passed by ROS.
        @return Returns a cropped image object.

        @note   Image Length Calculation
                length = The image length of the image. 0 indicates the top of the image and length - 1 indicates the bottom of the image.

                length * (1 - length_percent)          = The index at the 1 - length_percent point of the image.
                (length * (1 - length_percent)):length = the bottom length_percent% of the image being sliced.
    """
    def __crop_img(self, img, img_size=(160, 120), img_percent=0.50):
        img           = cv2.resize(img, img_size, interpolation=cv2.INTER_NEAREST)  # Resize the compressed image.
        self.__offset = int(img.shape[0] * (1 - img_percent))                       # Save the offset value.
        img           = img[self.__offset:, :]                                      # Crop to only the bottom 45% of the image.
        
        self.__arr_cutoff = np.array([0, self.__offset, 0, self.__offset])
        self.__arr_ratio  = np.array([1. / img_size[0], 1. / img_size[1], 1. / img_size[0], 1. / img_size[1]])

        # Publish the image for debugging.
        # ros_img = self.__bridge.cv2_to_imgmsg(img, "bgr8")
        # self.__cropped_img_pub.publish(ros_img)

        return img

    def __filter_white(self, img):
        # Create image filter kernels
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        erode_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        base_img      = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                                # Convert to HSV color space.
        filtered_img  = cv2.inRange(base_img, self.__WHITE_LOWER__, self.__WHITE_UPPER__)   # Filter to create mask of white areas.
        filtered_img  = cv2.erode(filtered_img,  erode_kernel)                              # Erode noise.
        filtered_img  = cv2.dilate(filtered_img, dilate_kernel)                             # Dilate to emphasize white areas.
        filtered_img  = cv2.bitwise_and(base_img, base_img, mask=filtered_img)              # Mask the original image to show only white areas.
        filtered_img  = cv2.cvtColor(filtered_img, cv2.COLOR_HSV2BGR)                       # Convert to BGR color space.
        
        # Publish the image for debugging
        # ros_img       = self.__bridge.cv2_to_imgmsg(filtered_img, "bgr8")
        # self.__white_img_pub.publish(ros_img)

        return filtered_img

    def __filter_yellow(self, img):
        # Create image filter kernels
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        erode_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        base_img      = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                                # Convert to HSV color space.
        filtered_img  = cv2.inRange(base_img, self.__YELLOW_LOWER__, self.__YELLOW_UPPER__) # Filter to create mask of yellow areas.
        filtered_img  = cv2.erode(filtered_img,  erode_kernel)                              # Erode noise.
        filtered_img  = cv2.dilate(filtered_img, dilate_kernel)                             # Dilate to emphasize yellow areas.
        filtered_img  = cv2.bitwise_and(base_img, base_img, mask=filtered_img)              # Mask the original image to show only yellow areas.
        filtered_img  = cv2.cvtColor(filtered_img, cv2.COLOR_HSV2BGR)                       # Convert to BGR color space.
        
        # Publish the image for debugging
        # ros_img       = self.__bridge.cv2_to_imgmsg(filtered_img, "bgr8")
        # self.__yellow_img_pub.publish(ros_img)

        return filtered_img

    def __w_lane_detector(self, img):
        # Convert to a CV image and apply the Canny edge detection.
        cv_img   = cv2.GaussianBlur(img, (5,5), 0)
        cv_edges = cv2.Canny(cv_img, 100, 200)
        cv_img   = cv2.bitwise_and(cv_img, cv_img, mask=cv_edges)

        # Apply the Hough transform and combine the lines with the lane image.
        hough_lines               = cv2.HoughLinesP(cv_edges, 1, np.pi/180, 1, minLineLength=20, maxLineGap=5)
        self.__w_hough_lines      = hough_lines

        if self.__w_hough_lines is not None:
            self.__w_hough_lines_norm = (self.__w_hough_lines + self.__arr_cutoff) * self.__arr_ratio

        return

    def __y_lane_detector(self, img):
        # Convert to a CV image and apply the Canny edge detection.
        cv_img   = cv2.GaussianBlur(img, (5,5), 0)
        cv_edges = cv2.Canny(cv_img, 100, 200)
        cv_img   = cv2.bitwise_and(cv_img, cv_img, mask=cv_edges)

        # Apply the Hough transform and combine the lines with the lane image.
        hough_lines               = cv2.HoughLinesP(cv_edges, 1, np.pi/180, 4, minLineLength=4, maxLineGap=1)
        self.__y_hough_lines      = hough_lines

        if self.__y_hough_lines is not None:
            self.__y_hough_lines_norm = (self.__y_hough_lines + self.__arr_cutoff) * self.__arr_ratio
        
        return

    def __draw_lines(self, img, lines, line_color=(255, 0, 0)):
        output = np.copy(img)

        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]                                                         # Get the line segment object.
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), line_color, 2, cv2.LINE_AA)  # Draw the line segment.
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))                           # Draw the circular endpoint.
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))                           # Draw the circular endpoint.

        return output

    def __cb_lane_detector(self, msg):
        segment_list     = []
        segment_list_msg = SegmentList()
        segment_msg      = Segment()

        # Convert to a cv2 image and crop the image. to 45% height.
        img = self.__bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        img = self.__crop_img(img)
        
        # Filter the yellow and white colors into masks.
        w_img = self.__filter_white(img)
        y_img = self.__filter_yellow(img)
        
        # Perform a Hough transform on the yellow and white edges.
        self.__w_lane_detector(w_img)
        self.__y_lane_detector(y_img)

        # draw the Hough lines on the cropped image and convert the image to an Image message.
        lane_img = self.__draw_lines(img, self.__w_hough_lines, line_color=(255, 0, 0))
        lane_img = self.__draw_lines(lane_img, self.__y_hough_lines, line_color=(0, 0, 255))
        lane_img = self.__bridge.cv2_to_imgmsg(lane_img, "bgr8")
        
        self.__overlay_img_pub.publish(lane_img)

        # Create the SegmentList to send to the ground projection node.
        # Iterate through each yellow hough line vector.
        if self.__y_hough_lines_norm is not None:
            for i in range(len(self.__y_hough_lines_norm)):
                line = self.__y_hough_lines_norm[i][0]

                # Declare a new Segment object and add it to the list of Segments.
                segment_list.append(Segment(color=1, pixels_normalized=np.array([Vector2D(x=line[0], y=line[1]), Vector2D(x=line[2], y=line[3])])))

        # Iterate through each white hough line vector.
        if self.__w_hough_lines_norm is not None:
            for i in range(len(self.__w_hough_lines_norm)):
                line = self.__w_hough_lines_norm[i][0]

                # Declare a new Segment object and add it to the list of Segments.
                segment_list.append(Segment(color=0, pixels_normalized=np.array([Vector2D(x=line[0], y=line[1]), Vector2D(x=line[2], y=line[3])])))

        # Add all the Hough lines to the SegmentList message and publish.
        segment_list_msg.segments = segment_list
        self.__segment_list_pub.publish(segment_list_msg)
        return

if __name__ == "__main__":
    rospy.init_node("lab3_lane_detector_node", anonymous=True)
    rospy.loginfo("Running Derek DeCost's Lab 3")
    LaneDetector()
    while not rospy.is_shutdown():
        pass