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

        # Set up the map.
        self._map = np.ones((self._x_num, self._y_num))*0.5

        self._initialized = True
        return True

    def LoadParameters(self):
        # Random downsampling fraction, i.e. only keep this fraction of rays.
        if not rospy.has_param("~random_downsample"):
            return False
        self._random_downsample = rospy.get_param("~random_downsample")

        # Dimensions and bounds.
        # TODO! You'll need to set values for class variables called:

        #-- self._x_num
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

        # -- self._y_res # The resolution in y. Note: This isn't a ROS parameter. What will you do instead?

        self._y_res = (self._y_max-self._y_min)/self._y_num

        # Update parameters.
        if not rospy.has_param("~update/occupied"):
            return False
        self._occupied_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied"))

        if not rospy.has_param("~update/occupied_threshold"):
            return False
        self._occupied_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/occupied_threshold"))

        if not rospy.has_param("~update/free"):
            return False
        self._free_update = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free"))

        if not rospy.has_param("~update/free_threshold"):
            return False
        self._free_threshold = self.ProbabilityToLogOdds(
            rospy.get_param("~update/free_threshold"))

        # Topics.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_topic

        if not rospy.has_param("~topics/sensor"):
            return False
        self._sensor_topic = rospy.get_param("~topics/sensor")

        # -- self._vis_topic

        if not rospy.has_param("~topics/vis"):
            return False
        self._vis_topic = rospy.get_param("~topics/vis")

        # Frames.
        # TODO! You'll need to set values for class variables called:
        # -- self._sensor_frame

        if not rospy.has_param("~frames/sensor"):
            return False
        self._sensor_frame = rospy.get_param("~frames/sensor")

        # -- self._fixed_frame

        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = rospy.Subscriber(self._sensor_topic,
                                            LaserScan,
                                            self.SensorCallback,
                                            queue_size=1)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        OccupancyGrid,
                                        queue_size=10)


        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        # Get our current pose from TF.
        try:
            self.pose1 = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
        # assuming that the turtlebot is on the ground plane.
        sensor_x = self.pose1.transform.translation.x
        sensor_y = self.pose1.transform.translation.y
        if abs(self.pose1.transform.translation.z) > 0.05:
            rospy.logwarn("%s: Turtlebot is not on ground plane.", self._name)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [self.pose1.transform.rotation.x, self.pose1.transform.rotation.y,
             self.pose1.transform.rotation.z, self.pose1.transform.rotation.w])
        rospy.logerr(roll)
        rospy.logerr(pitch)
        rospy.logerr(yaw)
        if abs(roll) > 0.1 or abs(pitch) > 0.1:
            rospy.logwarn("%s: Turtlebot roll/pitch is too large.", self._name)
        [self.grid_x,self.grid_y]= self.PointToVoxel(sensor_x,sensor_y)
        #rospy.logerr(self.grid_x)
        # Loop over all ranges in the LaserScan.
        self._map = np.ones((self._x_num, self._y_num))*0.5
        for idx, r in enumerate(msg.ranges):

            if np.isinf(r):
                continue
            else:    

            # Get angle of this ray in fixed frame.

                angle = msg.angle_min + idx*msg.angle_increment + yaw
           

            # Throw out this point if it is too close or too far away.
                if r > msg.range_max:
                    rospy.logwarn("%s: Range %f > %f was too large.",
                                self._name, r, msg.range_max)
                    continue
                if r < msg.range_min:
                    rospy.logwarn("%s: Range %f < %f was too small.",
                                self._name, r, msg.range_min)
                    continue
                



                rangeinVoxel = round(r//self._x_res,0)

                scan_x = int(round(self.grid_x + rangeinVoxel*math.cos(angle),0))
                scan_y = int(round(self.grid_y + rangeinVoxel*math.sin(angle),0))
                if self.ProbabilityToLogOdds(self._map[scan_x,scan_y]+0.001) > self._occupied_threshold:
                    self._map[scan_x,scan_y] = self._map[scan_x,scan_y]

                elif self.ProbabilityToLogOdds(self._map[scan_x,scan_y]+0.001) < self._occupied_threshold: 

                    if self._map[scan_x,scan_y] == 0:
                        for ii in range(-1,2):
                            for jj in range(-1,2):
                                if self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) <self._occupied_threshold:
                                    self._map[scan_x+ii,scan_y+jj] = self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) + self._occupied_update
                                    self._map[scan_x+ii,scan_y+jj] = self.LogOddsToProbability(self._map[scan_x+ii,scan_y+jj])

                    else:
                        for ii in range(-1,2):
                            for jj in range(-1,2):
                                if self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) <self._occupied_threshold:
                                    self._map[scan_x+ii,scan_y+jj] = self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) + self._occupied_update
                                    self._map[scan_x+ii,scan_y+jj] = self.LogOddsToProbability(self._map[scan_x+ii,scan_y+jj])

                self.scan_x_j = scan_x
                self.scan_y_j = scan_y

                for ii in range(int((rangeinVoxel+1)*4)):
                                    
                    scan_x_i = int(round(self.grid_x + (ii/4)*math.cos(angle),0))
                    scan_y_i = int(round(self.grid_y + (ii/4)*math.sin(angle),0))



                    if scan_x_i != scan_x or self.scan_x_j != scan_x_i or scan_y_i != scan_y or self.scan_y_j != scan_y_i:

                        if self.ProbabilityToLogOdds(self._map[scan_x_i,scan_y_i]+0.001) < self._free_threshold: 

                            self._map[scan_x_i,scan_y_i] = self._map[scan_x_i,scan_y_i]
                        
                        elif self.ProbabilityToLogOdds(self._map[scan_x_i,scan_y_i]+0.001) > self._free_threshold: 

                            self._map[scan_x_i,scan_y_i]= self.ProbabilityToLogOdds(self._map[scan_x_i,scan_y_i]+0.001) +self._free_update
                            self._map[scan_x_i,scan_y_i] = self.LogOddsToProbability(self._map[scan_x_i,scan_y_i])

                    else:

                        self._map[scan_x_i,scan_y_i] = self._map[scan_x_i,scan_y_i]

                    self.scan_x_j = scan_x_i
                    self.scan_y_j = scan_y_i
                    #rospy.logerr(self.ProbabilityToLogOdds(self._map[scan_x,scan_y]) + self.ProbabilityToLogOdds(self._occupied_update))
                # Walk along this ray from the scan point to the sensor.
                # Update log-odds at each voxel along the way.
                # Only update each voxel once. 
                # The occupancy grid is stored in self._map
                # TODO!

        # Visualize.
        self.Visualize()

    # Convert (x, y) coordinates in fixed frame to grid coordinates.
    def PointToVoxel(self, x, y):
        grid_x = int((x - self._x_min) / self._x_res)
        grid_y = int((y - self._y_min) / self._y_res)

        return (grid_x, grid_y)

    # Get the center point (x, y) corresponding to the given voxel.
    def VoxelCenter(self, ii, jj):
        center_x = self._x_min + (0.5 + ii) * self._x_res
        center_y = self._y_min + (0.5 + jj) * self._y_res

        return (center_x, center_y)

    # Convert between probabity and log-odds.
    def ProbabilityToLogOdds(self, p):
        return np.log(p / (1.0 - p))

    def LogOddsToProbability(self, l):
        return 1.0 / (1.0 + np.exp(-l))

    # Colormap to take log odds at a voxel to a RGBA color.
    def Colormap(self, ii, jj):
        p = self.LogOddsToProbability(self._map[ii, jj])

        c = ColorRGBA()
        c.r = p
        c.g = 0.1
        c.b = 1.0 - p
        c.a = 0.75
        return c

    # Visualize the map as a collection of flat cubes instead of
    # as a built-in OccupancyGrid message, since that gives us more
    # flexibility for things like color maps and stuff.
    # See http://wiki.ros.org/rviz/DisplayTypes/Marker for a brief tutorial.

    def Visualize(self):
        arr = self._map.ravel()
        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._fixed_frame
        np_round= np.around(arr*100,0)
        int_map= map(int,np_round)
        #rospy.logerr(np_round[np_round != 0])
        m.data = list(int_map)
        m.info.map_load_time = rospy.Time.now()
        m.info.height = self._map.shape[0]
        m.info.width = self._map.shape[1]
        m.info.resolution = self._x_res
        m.info.origin.position = Point(-30,-30,0)
        m.info.origin.orientation = Quaternion(0.7071,0.7071,0,0)
        
        # radius = round(msg.range_max//self._x_res)
        # for ii in range(self.grid_x-radius-1,self.grid_x+radius+1):
        #     for jj in range(self.grid_y-radius-1,self.grid_y+radius+1):
        #         p = Point(0.0, 0.0, 0.0)
        #         (p.x, p.y) = self.VoxelCenter(ii, jj)

        #         m.points.append(p)
        #         m.colors.append(self.Colormap(ii, jj))

        self._vis_pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("mapping_node")

    og = OccupancyGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()
