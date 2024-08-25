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

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid


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

        # Set up the map.
        self._map = np.zeros((self._x_num, self._y_num))

        self._initialized = True

        self.update = 0
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
                                            PointCloud2,
                                            self.SensorCallback,
                                            queue_size=1)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        OccupancyGrid,
                                        queue_size=1)


        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):
        if not self._initialized:
            rospy.logerr("%s: Was not initialized.", self._name)
            return

        type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                        (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                        (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
        pftype_to_nptype = dict(type_mappings)
        self.DUMMY_FIELD_PREFIX = '__'
        offset = 0
        np_dtype_list = []
        pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                        PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}
        for f in msg.fields:
            while offset < f.offset:
            #  might be extra padding between fields
                np_dtype_list.append(('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1

            dtype = pftype_to_nptype[f.datatype]
            if f.count != 1:
                dtype = np.dtype((dtype, f.count))

            np_dtype_list.append((f.name, dtype))
            offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
        while offset < msg.point_step:
            np_dtype_list.append(('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1
        
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = np_dtype_list
 
        # parse the cloud into an array
        cloud_array = np.fromstring(msg.data, dtype_list)

        # remove the dummy fields that were added
        cloud_array = cloud_array[
            [fname for fname, _type in dtype_list if not (fname[:len(self.DUMMY_FIELD_PREFIX)] == self.DUMMY_FIELD_PREFIX)]]
     
        if True and msg.height == 1:
            np.reshape(cloud_array, (msg.width,))
        else:
            np.reshape(cloud_array, (msg.height, msg.width))
        
        if True:
            mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
            cloud_array = cloud_array[mask]
     
        # pull out x, y, and z values
        points = np.zeros(cloud_array.shape + (3,), dtype=np.float)
        points[...,0] = cloud_array['x']
        points[...,1] = cloud_array['y']
        points[...,2] = cloud_array['z']
        self.array = points


        for i in range(len(points)):

            if points[i,1] < -1 or points[i,1] > 1:
                continue

            else: 
                #if points[i,1] > -1 or points[i,1] < 1:

                [grid_x,grid_y] = self.PointToVoxel(points[i,0],points[i,2])

                if grid_x < self._x_num and grid_y < self._y_num and grid_x > 0 and grid_y > 0:
                    self._map[grid_x,grid_y] = 1
            
        #if self.update == 50:
        for ii in range(1,int(self._x_num-1)):
            for jj in range(1,int(self._y_num-1)):
                    
                    
                sum = np.array([self._map[ii,jj],self._map[ii,jj+1],self._map[ii+1,jj],self._map[ii-1,jj],self._map[ii,jj-1],self._map[ii+1,jj+1],self._map[ii-1,jj-1],self._map[ii+1,jj-1],self._map[ii-1,jj+1]])
                
                if np.sum(sum) > 1:
                    self._map[ii,jj] = self._map[ii,jj]
                else:
                    self._map[ii,jj] = 0
        # Visualize.
            #self.update = 0    
        self.Visualize()

        self.update = self.update +1

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
        rospy.logerr(np_round[np_round != 0])
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
        rate = rospy.Rate(1)
        self._vis_pub.publish(m)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pointcloud_mapper")
    og = pointcloud_2_OGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()
