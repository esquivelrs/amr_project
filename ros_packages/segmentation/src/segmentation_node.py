#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

class SegmentationNode:
    def __init__(self):
        self.N = 1 
        self.FOCAL_LENGTH = 1017.66
        self.BALL_RADIUS = 1.0

        # Initialize the filters with a window size of N
        self.circle_x_filter = MovingAverageFilter(self.N)
        self.circle_y_filter = MovingAverageFilter(self.N)
        self.circle_r_filter = MovingAverageFilter(self.N)

        self.circle_pub = rospy.Publisher('/iio_position', Point, queue_size=1)
        self.sonar_sub = rospy.Subscriber('/bluerov2/sonar_front_front', LaserScan, self.sonar_callback)
        self.camera_sub = rospy.Subscriber('/bluerov2/camera_front/camera_image', Image, self.camera_callback)

    def sonar_callback(self, data):
        # Implement sonar callback here
        #print ranges
        #print(data.ranges)
        # get the min range
        self.sonar_dist = min(data.ranges)

        rospy.loginfo(f'sonar min: {self.sonar_dist}')
        
        #pass

    def camera_callback(self, image):
        if self.sonar_dist < np.inf:
            lower_mask = tuple(eval(rospy.get_param("~lower_mask")))
            upper_mask = tuple(eval(rospy.get_param("~upper_mask")))

            br = CvBridge()
            cv_image = br.imgmsg_to_cv2(image, desired_encoding="bgr8")
            cv_image_resized = cv2.resize(cv_image, (cv_image.shape[1]//2, cv_image.shape[0]//2))

            mask = cv2.inRange(cv_image_resized, lower_mask, upper_mask)
            cv_image_filtered = cv2.bitwise_and(cv_image_resized, cv_image_resized, mask=mask)

            gray = cv2.cvtColor(cv_image_filtered, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            thresh = cv2.dilate(thresh, kernel)
            cv2.imshow("thresh", thresh)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            thresh = cv2.erode(thresh, kernel)

            edges = cv2.Canny(thresh, 50, 150, apertureSize=3)
            circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 1100, param1=150, param2=5, minRadius=30, maxRadius=0)
            if circles is not None:
                circles = circles[0]
                for circle in circles:
                    x, y, r = circle
                    cv2.circle(cv_image_resized, (x, y), r, (0, 255, 0), 4)

            if circles is not None and len(circles) == 1:
                x, y, r = circles[0]
                # x and y with respect to the center of the image
                x = x - cv_image_resized.shape[1] // 2
                y = y - cv_image_resized.shape[0] // 2

                # Calculate the distance to the ball
                # z = self.BALL_RADIUS * self.FOCAL_LENGTH / r
                dist = self.sonar_dist




                # x and y in meters
                x = x * dist / self.FOCAL_LENGTH
                y = y * dist / self.FOCAL_LENGTH

                #rospy.loginfo(f'Calculated values: x={x}, y={y}, z={z}')

                # Create a Point message
                circle_msg = Point()

                # Set the values
                circle_msg.x = self.circle_x_filter.update(dist)
                circle_msg.y = self.circle_y_filter.update(-x)
                circle_msg.z = self.circle_r_filter.update(-y)

                # Publish the message
                self.circle_pub.publish(circle_msg)

            cv2.imshow("image", cv_image_resized)
            cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('segmentation_node')
    rospy.loginfo("segmentation_node started!")

    node = SegmentationNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")