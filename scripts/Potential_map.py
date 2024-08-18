#!/usr/bin/python
################################################################################
#
# Node to wrap the OccupancyGrid2d class.
#
################################################################################

import rospy
import sys
import tf2_ros
import tf
import math


from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

import numpy as np

class OccupancyGrid2d(object):
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # Initialization and loading parameters.
    def Initialize(self):
        self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        if not self.LoadParameters():
            rospy.logerr("%s: Error loading parameters.", self._name)
            return False

        # Register callbacks.
        if not self.RegisterCallbacks():
            rospy.logerr("%s: Error registering callbacks.", self._name)
            return False
    
        return True

    def LoadParameters(self):

        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")

        # -- self._vis_topic

        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sensor_topic,
                                            OccupancyGrid,
                                            self.SensorCallback,
                                            queue_size=None)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        PointCloud2,
                                        queue_size=1)


        return True

    # Callback to process sensor measurements.
    
    def SensorCallback(self, msg):

        width = msg.info.width
        height = msg.info.height
        x,y = np.meshgrid(np.linspace(-30,30,width), np.linspace(-30,30,height))
        data = np.array(msg.data)/100
        data_reshape = np.reshape(data,(width,height))
        self.points = np.array([x,y,data_reshape,data_reshape]).reshape(4,-1).T
        self.Visualize()

           
    def Visualize(self):
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
        
        
        header = Header()
        header.frame_id = self._fixed_frame
        header.stamp = rospy.Time.now()
        m = point_cloud2.create_cloud(header, fields, self.points)
        self._vis_pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("mapping_node")

    og = OccupancyGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()
