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
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from own_package.msg import Num


import numpy as np

class pointcloud_2_OGrid2d(object):
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

        self._kf_scan_array = PoseArray.poses
        self._kf_scan_old = PoseArray.poses
        self.length_kf = 1
        self.pose = Pose
        self.skip = 0
        self.ranges1 = []
        self.range = []
        self._initialized = True
        self.increments = 0
        self.länge = 0

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
        self._sensor_sub = message_filters.Subscriber(self._sensor_topic,
                                            LaserScan)
        self._kf_sub = message_filters.Subscriber('/orb_slam3/kf_markers_poses',
                                            PoseArray)
        
        ts = message_filters.TimeSynchronizer([self._sensor_sub, self._kf_sub], 10)
        ts.registerCallback(self.SensorCallback)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Num,
                                        queue_size=1)
        



    

        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, laser_msg,kf_msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return
        

        if len(kf_msg.poses)-1 > self.länge:
        
            self.skip = 0
            

            if len(kf_msg.poses)-1 != self.length_kf:
                rospy.logerr("Falsche Länge")


            if self.length_kf > 1:
                for i in range(len(self._kf_scan_old)-1):

                    if abs(self._kf_scan_old[i].position.x - kf_msg.poses[i].position.x) > 0.05:
                        # rospy.logerr("x")
                        # rospy.logerr(kf_msg.poses[i].position.x)
                        # rospy.logerr(self._kf_scan_old[i].position.x)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].position.y - kf_msg.poses[i].position.y) > 0.05:
                        # rospy.logerr("y")
                        # rospy.logerr(kf_msg.poses[i].position.y)
                        # rospy.logerr(self._kf_scan_old[i].position.y)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].position.z - kf_msg.poses[i].position.z) > 0.05:
                        # rospy.logerr("z")
                        # rospy.logerr(kf_msg.poses[i].position.z)
                        # rospy.logerr(self._kf_scan_old[i].position.z)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].orientation.x - kf_msg.poses[i].orientation.x) > 0.01:
                        # rospy.logerr("ox")
                        # rospy.logerr(kf_msg.poses[i].orientation.x)
                        # rospy.logerr(self._kf_scan_old[i].orientation.x)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].orientation.y - kf_msg.poses[i].orientation.y) > 0.01:
                        # rospy.logerr("oy")
                        # rospy.logerr(kf_msg.poses[i].orientation.y)
                        # rospy.logerr(self._kf_scan_old[i].orientation.y)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].orientation.z - kf_msg.poses[i].orientation.z) > 0.01:
                        # rospy.logerr("oz")
                        # rospy.logerr(kf_msg.poses[i].orientation.z)
                        # rospy.logerr(self._kf_scan_old[i].orientation.z)
                        self.skip = 1

                    elif abs(self._kf_scan_old[i].orientation.w - kf_msg.poses[i].orientation.w) > 0.01:
                        # rospy.logerr("ow")
                        # rospy.logerr(kf_msg.poses[i].orientation.w)
                        # rospy.logerr(self._kf_scan_old[i].orientation.w)
                        self.skip = 1 
                        
            self._kf_scan_array = kf_msg.poses[:-1]
            self.pose = kf_msg.poses[len(kf_msg.poses)-2]            
            

            self.increments = (laser_msg.angle_max - laser_msg.angle_min)//laser_msg.angle_increment
            
            self.range = laser_msg.ranges

            if len(kf_msg.poses)> self.length_kf-1: 
                
                self.ranges1.extend(laser_msg.ranges)
                
                
                self.length_kf = self.length_kf +1


            self._kf_scan_old = kf_msg.poses

            self.länge = self.länge + 1



            self.Visualize()
        else: 
            self.skip = 2
            self.Visualize()

    def Visualize(self):

        m = Num()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        m.poses = self._kf_scan_array
        m.pose = self.pose
        m.range = self.range
        m.skip = self.skip
        m.ranges = self.ranges1
        m.increments = self.increments
        self._vis_pub.publish(m)
    


if __name__ == "__main__":
    rospy.init_node("all_kf_scan")
    og = pointcloud_2_OGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the kf scan node.")
        sys.exit(1)

    rospy.spin()
