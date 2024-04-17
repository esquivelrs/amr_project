#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def ooi_simple_move_publisher():
    """
    Moving target front and back with a given period and speed
    """
    rospy.init_node('simple_movement', anonymous=True)

    # Get parameters from ROS parameter server
    period = rospy.get_param('~period', 6.0)
    speed = rospy.get_param('~speed', 0.5)

    pub = rospy.Publisher('ooi/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        twist_msg = Twist()
        twist_msg.linear.y = speed  # Initial linear velocity

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

        # Change linear velocity to -speed
        twist_msg.linear.y = -speed

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

        twist_msg.linear.y = 0 

        twist_msg.linear.z = speed  # Initial linear velocity

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/4:
            pub.publish(twist_msg)
            rate.sleep()

        # Change linear velocity to -speed
        twist_msg.linear.z = -speed

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/4:
            pub.publish(twist_msg)
            rate.sleep()
        
        



if __name__ == '__main__':
    try:
        ooi_simple_move_publisher()
    except rospy.ROSInterruptException:
        pass