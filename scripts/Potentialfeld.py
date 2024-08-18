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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
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

        # -- self._vis_topic

        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # -- self._fixed_frame

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
                                        OccupancyGrid,
                                        queue_size=1)


        return True

    # Callback to process sensor measurements.
    
    def SensorCallback(self, msg):
        self.height = msg.info.height
        self.width = msg.info.width
        self.res = msg.info.resolution
        data = np.array(msg.data)
        data_reshape = np.reshape(data,(self.width,self.height))
        # frontier = list()
        # for idx, r in np.ndenumerate(data_reshape):

        #     if (data_reshape[idx[0]-1,idx[1]] == 50 or data_reshape[idx[0]+1,idx[1]] == 50 or data_reshape[idx[0],idx[1]-1] == 50 or data_reshape[idx[0],idx[1]+1] == 50) and (r == 0):
                                                                                                                                                                            
        #         frontier.append(idx)
        
        self._map = np.ones((self.width, self.height))
        
        # for point in frontier:
        #     wert = 0
        #     for point2 in frontier:
        #         if abs(point2[0]-point[0]) < 60  or abs(point2[1]-point[1]) < 60 : 
        #             wert = wert - math.exp(-((math.pow((point[0]-point2[0])*0.2,2)+math.pow((point[1]-point2[1])*0.2,2))/(2*math.pow(20*0.2,2))))
        #             if abs(wert) < 100: 
        #                 self._map[point[0],point[1]] = wert 
        
        for idx, r in np.ndenumerate(data_reshape):
            
            if r == 50:
                self._map[idx[0],idx[1]] = 0
            elif r == 100:
                self._map[idx[0],idx[1]] = 100
            elif r == 0: 
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
        m.info.height = self.height
        m.info.width = self.width
        m.info.resolution = self.res
        m.info.origin.position = Point(-30,-30,0)
        m.info.origin.orientation = Quaternion(0.7071,0.7071,0,0)
        
        self._vis_pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("mapping_node")

    og = OccupancyGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()
