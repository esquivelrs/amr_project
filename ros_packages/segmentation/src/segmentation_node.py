#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

N = 25 

# Initialize the filters with a window size of N
circle_x_filter = MovingAverageFilter(N)
circle_y_filter = MovingAverageFilter(N)
circle_r_filter = MovingAverageFilter(N)

circle_x_pub = rospy.Publisher('/circle_x', Float32, queue_size=1)
circle_y_pub = rospy.Publisher('/circle_y', Float32, queue_size=1)
circle_r_pub = rospy.Publisher('/circle_r', Float32, queue_size=1)

def camera_callback(image):
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
        y = cv_image_resized.shape[0] // 2 - y


        circle_x_pub.publish(circle_x_filter.update(x))
        circle_y_pub.publish(circle_y_filter.update(y))
        circle_r_pub.publish(circle_r_filter.update(r))

    cv2.imshow("image", cv_image_resized)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('segmentation_node')
    rospy.loginfo("segmentation_node started!")

    camera_sub = rospy.Subscriber('/bluerov2/camera_front/camera_image', Image, camera_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")