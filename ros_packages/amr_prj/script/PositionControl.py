from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from PID.PIDRegulator import PIDRegulator
import rospy
import numpy as np


import math
import tf.transformations as trans


class PositionControlNode:
    def __init__(self):
        self.target_distance = 1.0  # The desired distance to the object
        self.object_sub = rospy.Subscriber('/iio_position', Point, self.object_callback)
        self.cmd_pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=1)

        self.robot_pose = None
        self.object_position = None

        # Initialize the PID controller
        self.pid = PIDRegulator(5.0, 0.005, 0.0005, 1.0)


    def object_callback(self, data):
        self.p = data
        rospy.loginfo(f'Control LOOP: {self.p}')



        # Calculate the distance to the object
        dx = self.p.x
        dy = self.p.y
        dz = self.p.z

        e_pos = np.array([dx - 1.0, dy, dz])
        t = rospy.get_time()

        rospy.loginfo(f'Control ERROR: {e_pos}')




        # Error angles
        e_rot = np.array([0.0,0.0, math.atan2(dy, dx)])

        # Create a Twist message


        v_linear = self.pid.regulate(e_pos, t)
        v_angular = self.pid.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = Twist()
        cmd_vel.linear = Vector3(*v_linear)
        cmd_vel.angular = Vector3(*v_angular)
 

        # Publish the command
        self.cmd_pub.publish(cmd_vel)

if __name__ == '__main__':
    rospy.init_node('position_control_node')
    rospy.loginfo("position_control_node started!")

    node = PositionControlNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")