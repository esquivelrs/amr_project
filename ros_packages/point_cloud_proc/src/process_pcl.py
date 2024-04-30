from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import rospy
import numpy as np


class PclNode:
    def __init__(self):
        self.pcl_sub = rospy.Subscriber('/sonar_cloud', PointCloud2, self.pcl_callback)
        self.pcl_pub = rospy.Publisher('/sonar_cloud_filtered', PointCloud2, queue_size=1)
        self.detection_pub = rospy.Publisher('/detection_cloud', PointCloud2, queue_size=1)
        self.surroundings_pub = rospy.Publisher('/surroundings_cloud', PointCloud2, queue_size=1)

    def pcl_callback(self, cloud):
        # Convert the PointCloud2 message to a list of tuples
        pcl_cloud = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)

        # Create two new lists for detection and surroundings
        detection_cloud = []
        surroundings_cloud = []

        # Iterate over the points in the point cloud
        for point in pcl_cloud:
            # Calculate the distance of the point from the origin
            distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

            # If the distance is less than or equal to 0.5, add the point to the detection point cloud
            if 1.5 >= distance >= 0.8:
                detection_cloud.append(point)
            # Otherwise, add the point to the surroundings point cloud
            else:
                surroundings_cloud.append(point)

        # Convert the detection and surroundings point clouds back to PointCloud2 messages
        detection_msg = pc2.create_cloud_xyz32(cloud.header, detection_cloud)
        surroundings_msg = pc2.create_cloud_xyz32(cloud.header, surroundings_cloud)

        # Publish the detection and surroundings point clouds
        self.detection_pub.publish(detection_msg)
        self.surroundings_pub.publish(surroundings_msg)

        # Publish the original point cloud
        self.pcl_pub.publish(cloud)

if __name__ == '__main__':
    rospy.init_node('pcl_node')
    rospy.loginfo("pcl_node started!")

    node = PclNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")