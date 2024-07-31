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

        # Set up the map.
        self._map = np.ones((self._x_num, self._y_num))*0.5
        self.length  = 0
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
                                            Num,
                                            self.SensorCallback,
                                            queue_size=None)

        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        OccupancyGrid,
                                        queue_size=1)


        return True

    # Callback to process sensor measurements.
    def SensorCallback(self, msg):

        if msg.skip == 0:
            self.length = len(msg.poses)
            if not self._initialized:
                rospy.logerr("%s: Was not initialized.", self._name)
                return

            # Get our current pose from TF.
            for idx, r in enumerate(msg.range):

                
                #rospy.logerr(idx)
            # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
            # assuming that the turtlebot is on the ground plane.
                sensor_x = msg.pose.position.x
                sensor_y = msg.pose.position.z
            

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [msg.pose.orientation.x, msg.pose.orientation.z,
                    -msg.pose.orientation.y, msg.pose.orientation.w])
                
    
                [self.grid_x,self.grid_y]= self.PointToVoxel(sensor_x,sensor_y)
        
                if np.isinf(r):
                    continue
                else:    
                    idx_modulo = idx%70
                # Get angle of this ray in fixed frame.

                    angle = 0.7 + idx_modulo*0.025 + yaw


                    rangeinVoxel = round(r//self._x_res,0)

                    scan_x = int(round(self.grid_x + rangeinVoxel*math.cos(angle),0))
                    scan_y = int(round(self.grid_y + rangeinVoxel*math.sin(angle),0))
                    if self.ProbabilityToLogOdds(self._map[scan_x,scan_y]+0.001) > self._occupied_threshold:
                        self._map[scan_x,scan_y] = self._map[scan_x,scan_y]

                    elif self.ProbabilityToLogOdds(self._map[scan_x,scan_y]+0.001) < self._occupied_threshold: 
                            for ii in range(-1,2):
                                for jj in range(-1,2):
                                    if self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) <self._occupied_threshold:
                                        self._map[scan_x+ii,scan_y+jj] = self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) + self._occupied_update
                                        self._map[scan_x+ii,scan_y+jj] = self.LogOddsToProbability(self._map[scan_x+ii,scan_y+jj])

                    pts = self.get_grid_cells_btw(self.grid_x,self.grid_y,scan_x,scan_y)

                    for x in pts:
    
                        if self.ProbabilityToLogOdds(self._map[x[0],x[1]]+0.001) > self._free_threshold:

                            self._map[x[0],x[1]]= self.ProbabilityToLogOdds(self._map[x[0],x[1]]+0.001) + self._free_update
                            self._map[x[0],x[1]] = self.LogOddsToProbability(self._map[x[0],x[1]])

            # Visualize.
            self.Visualize()


        elif msg.skip == 1:
            self._map = np.ones((self._x_num, self._y_num))*0.5
            self.length = len(msg.poses)
            rospy.logerr("im neuen")
            if not self._initialized:
                rospy.logerr("%s: Was not initialized.", self._name)
                return

            angles = np.array([])

            for position in msg.poses:

                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [position.orientation.x, position.orientation.z,
                    -position.orientation.y, position.orientation.w])
                
                angles = np.append(angles,[yaw])
            
            ranges = np.array(msg.ranges)
     
            # Get our current pose from TF.
            for idx, r in np.ndenumerate(ranges):
                counter = int(idx[0]//msg.increments)
                #rospy.logerr(idx)
            # Extract x, y coordinates and heading (yaw) angle of the turtlebot, 
            # assuming that the turtlebot is on the ground plane.
                sensor_x = msg.poses[counter].position.x
                sensor_y = msg.poses[counter].position.z
            
                [self.grid_x,self.grid_y]= self.PointToVoxel(sensor_x,sensor_y)
        
                if np.isinf(r):
                    continue
                else:    
                    idx_modulo = idx[0]%70
                # Get angle of this ray in fixed frame.

                    angle = 0.7 + idx_modulo*0.025 + angles[counter]


                    rangeinVoxel = round(r//self._x_res,0)

                    scan_x = int(round(self.grid_x + rangeinVoxel*math.cos(angle),0))
                    scan_y = int(round(self.grid_y + rangeinVoxel*math.sin(angle),0))

                    if self.ProbabilityToLogOdds(self._map[scan_x,scan_y]+0.001) < self._occupied_threshold: 
                            for ii in range(-1,2):
                                for jj in range(-1,2):
                                    if self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) <self._occupied_threshold:
                                        self._map[scan_x+ii,scan_y+jj] = self.ProbabilityToLogOdds(self._map[scan_x+ii,scan_y+jj]+0.001) + self._occupied_update
                                        self._map[scan_x+ii,scan_y+jj] = self.LogOddsToProbability(self._map[scan_x+ii,scan_y+jj])

                    pts = self.get_grid_cells_btw(self.grid_x,self.grid_y,scan_x,scan_y)

                    for x in pts:
    
                        if self.ProbabilityToLogOdds(self._map[x[0],x[1]]+0.001) > self._free_threshold:

                            self._map[x[0],x[1]]= self.ProbabilityToLogOdds(self._map[x[0],x[1]]+0.001) + self._free_update
                            self._map[x[0],x[1]] = self.LogOddsToProbability(self._map[x[0],x[1]])


        
            rospy.logerr("fertig")
            # Visualize.
            self.Visualize()

        else:

            self.Visualize()

            
    def remove_np_duplicates(self,data):
        # Perform lex sort and get sorted data
        sorted_idx = np.lexsort(data.T)
        sorted_data =  data[sorted_idx,:]

        # Get unique row mask
        row_mask = np.append([True],np.any(np.diff(sorted_data,axis=0),1))

        # Get unique rows
        out = sorted_data[row_mask]
        return out
    
    def get_grid_cells_btw(self,x1,y1,x2,y2):
        dx = x2-x1 
        dy = y2-y1

        if dx == 0: # will divide by dx later, this will cause err. Catch this case up here
            step = np.sign(dy)
            ys = np.arange(0,dy+step,step)
            xs = np.repeat(x1, ys.shape[0])
        else:
            m = dy/(dx+0.0)
            b = y1 - m * x1 

            step = 1.0/(max(abs(dx),abs(dy))) 
            xs = np.arange(x1, x2, step * np.sign(x2-x1))
            ys = xs * m + b

        xs = np.rint(xs)
        ys = np.rint(ys)
        pts = np.column_stack((xs,ys))
        pts = self.remove_np_duplicates(pts)

        return pts.astype(int)
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
        m.data = list(int_map)
        m.info.map_load_time = rospy.Time.now()
        m.info.height = self._map.shape[0]
        m.info.width = self._map.shape[1]
        m.info.resolution = self._x_res
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
