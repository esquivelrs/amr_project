from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
from PID.PIDRegulator import PIDRegulator
import rospy
import numpy as np
from std_msgs.msg import Float64


import math
import tf.transformations as trans


class PositionControlNode:
    def __init__(self):
        self.target_distance = 1.0  # The desired distance to the object
        self.object_sub = rospy.Subscriber('/iio_position', Point, self.object_callback)
        self.object_offset = rospy.Subscriber('/iio_offset', Point, self.offset_callback)  # Changed to Point
        self.cmd_pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=1)
        self.offset_pub = rospy.Publisher('/iio_offset_setpoint', Point, queue_size=1)

        self.robot_pose = None
        self.object_position = None


        # Initialize the PID controller
        #self.pid = PIDRegulator(2.0, 0.005, 0.0005, 2.0)
        self.pid = PIDRegulator(2.0, 0.01, 0.001, 1.5)

        # default x= 1.0, y=0.0, z=0.0
        self.offset = Point(1.0, 0.0, 0.0) 

    def offset_callback(self, msg):
        self.offset = msg  # No need to use .data
        rospy.loginfo(f'SET Control offset: {self.offset}')


    def object_callback(self, data):
        self.p = data
        rospy.loginfo(f'Control LOOP: {self.p}')

        rospy.loginfo(f'Control offset: {self.offset}')



        # Calculate the distance to the object
        dx = self.p.x
        dy = self.p.y - self.offset.y
        dz = self.p.z - self.offset.z

        e_pos = np.array([dx - self.offset.x, dy, dz])
        t = rospy.get_time()

        #rospy.loginfo(f'Control ERROR: {e_pos}')




        # Error angles
        e_rot = np.array([0.0,math.atan2(dz, dx), math.atan2(dy, dx)])

        # Create a Twist message


        v_linear = self.pid.regulate(e_pos, t)
        v_angular = self.pid.regulate(e_rot, t)

        # Convert and publish vel. command:
        cmd_vel = Twist()
        cmd_vel.linear = Vector3(*v_linear)
        cmd_vel.angular = Vector3(*v_angular)
 

        # Publish the command
        self.cmd_pub.publish(cmd_vel)


        # Publish the offset
        self.offset_pub.publish(self.offset) 

if __name__ == '__main__':
    rospy.init_node('position_control_node')
    rospy.loginfo("position_control_node started!")

    node = PositionControlNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")