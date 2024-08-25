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
import message_filters

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from own_package.msg import Num

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

        if not rospy.has_param("~topics/sensor2"):
            return False
        self._sensor_topic2 = rospy.get_param("~topics/sensor2")
        # -- self._vis_topic

        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # -- self._fixed_frame

        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")


        if not rospy.has_param("~x/num"):
            return False
        self._x_num = rospy.get_param("~x/num")

        # -- self._x_min
        if not rospy.has_param("~x/min"):
            return False
        self._x_min = rospy.get_param("~x/min")

        # -- self._x_max
        if not rospy.has_param("~x/max"):
            return False
        self._x_max = rospy.get_param("~x/max")

        # -- self._x_res # The resolution in x. Note: This isn't a ROS parameter. What will you do instead?

        self._x_res = (self._x_max-self._x_min)/self._x_num

        # -- self._y_num
        if not rospy.has_param("~y/num"):
            return False
        self._y_num = rospy.get_param("~y/num")

        # -- self._y_min
        if not rospy.has_param("~y/min"):
            return False
        self._y_min = rospy.get_param("~y/min")

        # -- self._y_max
        if not rospy.has_param("~y/max"):
            return False
        self._y_max = rospy.get_param("~y/max")

        self._y_res = (self._y_max-self._y_min)/self._y_num

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = message_filters.Subscriber(self._sensor_topic,
                                            OccupancyGrid)
        rospy.logerr(self._sensor_sub)
        self._sensor_sub2 = message_filters.Subscriber(self._sensor_topic2,
                                            PoseStamped)
        
        ts = message_filters.ApproximateTimeSynchronizer([self._sensor_sub, self._sensor_sub2], 2,0.1)

        ts.registerCallback(self.SensorCallback)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        OccupancyGrid,
                                        queue_size=1)


        return True

    # Callback to process sensor measurements.
    
    def SensorCallback(self, Grid, Pose):
        self.height = Grid.info.height
        self.width = Grid.info.width
        self.res = Grid.info.resolution
        data = np.array(Grid.data)
        data_reshape = np.reshape(data,(self.width,self.height))
        sensor_x = Pose.pose.position.x
        sensor_y = Pose.pose.position.z
        self.grid_x = int((sensor_x - self._x_min) / self._x_res)
        self.grid_y = int((sensor_y - self._y_min) / self._y_res)
        self.num1 = self.grid_x -25
        num2 = self.grid_x +25 
        self.num3 = self.grid_y -25
        num4 = self.grid_y +25 
        data_reshape = data_reshape[self.num1:num2,self.num3:num4]
  
        self._map = np.ones((data_reshape.shape[0], data_reshape.shape[1]))

        for idx, r in np.ndenumerate(data_reshape):
            
            if r > 50:
                self._map[idx[0],idx[1]] = 100
            elif r < 50:
                self._map[idx[0],idx[1]] = 0
            elif r == 50:
                self._map[idx[0],idx[1]] = 50



        self.Visualize()

           

    def Visualize(self):
        arr = self._map.ravel()
        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        np_round= np.around(arr,0)
        int_map= map(int,np_round)
        m.data = list(int_map)
        m.info.map_load_time = rospy.Time.now()
        m.info.height = self._map.shape[0]
        m.info.width = self._map.shape[1]
        m.info.resolution = self.res
        m.info.origin.position = Point((self.num1-150)/5,(self.num3-150)/5,0)
        m.info.origin.orientation = Quaternion(0.7071,0.7071,0,0)
        
        self._vis_pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("mapping_node")

    og = OccupancyGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()
