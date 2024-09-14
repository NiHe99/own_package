import numpy as np
import rospy
import tf2_ros
import tf
import message_filters
from casadi import * 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from own_package.msg import Three_cat

class Casadi_NMPC:
    """Class for a Nonlinear Model Predictive Control law based using Casadi 
    """
    def __init__(self,horizon, dt):
        """Init func
        Args:
            horizon (float): how many steps to look into the future
            input_constraint (np.array): control constraints
            state_constraints (np.array): state contraints
            dt (float): sampling time
        """
        self.N = horizon
        self.len_f = 0
        self.v_max = 0.1
        self.w_max = 0.2

        self.n_actionsMPC = 2
        self.dt = dt
        self.numberState = 3

        self.Q_pos = 10
        self.Q_yaw = 100
        self.R = 1
        self.width = 50
        self.height = 50

        self.initialize_casadi()


    def reset(self,):
        """Every control class should have a reset function
        """
        return


    def initialize_casadi(self):
        """Initialize the casadi optimal control problem
        """

        # Casadi problem formulation ---------------------------------------
        self.opti = Opti() # Optimization problem
        
        # Decision variables ---------------------------------------
        self.X_casadi = self.opti.variable(self.numberState,self.N+1) # state trajectory
        self.x_casadi   = self.X_casadi[0,:]
        self.y_casadi   = self.X_casadi[1,:]
        self.yaw_casadi   = self.X_casadi[2,:]

        self.U_casadi = self.opti.variable(self.n_actionsMPC,self.N)   # control trajectory

        # Initial State Constraint -----------------------------------
        self.x_0 = self.opti.parameter()
        self.y_0 = self.opti.parameter()
        self.yaw_0 = self.opti.parameter()
        self.opti.subject_to(self.x_casadi[0]==self.x_0)
        self.opti.subject_to(self.y_casadi[0]==self.y_0)
        self.opti.subject_to(self.yaw_casadi[0]==self.yaw_0) 

        # State Constraints and Cost Function -------------------------
        self.set_kinematics()
        self.set_constraints()
        self.set_cost_function()
        
        # Solver parameters ---------------------------------------
        p_opts = dict(print_time=False, verbose=False) 
        s_opts = dict(print_level=0)
        self.opti.solver("ipopt",p_opts,s_opts) # set numerical backend


    def set_kinematics(self):
        """Setting the kinematics constraints
        """
        # Kynematic Constraints ---------------------------------------
        for k in range(self.N): # loop over control intervals
            next_x = self.X_casadi[0,k] + self.U_casadi[0,k]*cos(self.X_casadi[2,k])*self.dt
            next_y = self.X_casadi[1,k] + self.U_casadi[0,k]*sin(self.X_casadi[2,k])*self.dt
            next_theta = self.X_casadi[2,k] + self.U_casadi[1,k]*self.dt

            self.opti.subject_to(self.X_casadi[0,k+1]==next_x) # close the gaps
            self.opti.subject_to(self.X_casadi[1,k+1]==next_y) # close the gaps
            self.opti.subject_to(self.X_casadi[2,k+1]==next_theta) # close the gaps   

            
    def set_constraints(self):
        """Setting input constraints
        """
        for k in range(self.N): # loop over control intervals
            #linear velocity
            self.opti.subject_to(self.U_casadi[0,k] <= self.v_max)
            self.opti.subject_to(self.U_casadi[0,k] >= 0)
            #angular velocity
            self.opti.subject_to(self.U_casadi[1,k] <= self.w_max)
            self.opti.subject_to(self.U_casadi[1,k] >= -self.w_max)



    def set_cost_function(self):
        """Setting the cost function
        """

        # Parametric Cost Function -------------------------------------
        pose_error = 0

        self.front_x = self.opti.parameter(1,2500)
        self.front_y = self.opti.parameter(1,2500)
        self.unex_x = self.opti.parameter(1,2500)
        self.unex_y = self.opti.parameter(1,2500)
        self.obb_x = self.opti.parameter(1,2500)
        self.obb_y = self.opti.parameter(1,2500)

        for k in range(1, self.N+1):

            for i in range(2500):

                pose_error += -exp(-(((self.x_casadi[k] - (((self.front_x[i]-25)/5)-self.x_0))@(self.x_casadi[k] - (((self.front_x[i]-25)/5)-self.x_0)).T)+(self.y_casadi[k] - (((self.front_y[i]-25)/5)-self.y_0))@(self.y_casadi[k] - (((self.front_y[i]-25)/5)-self.y_0)).T)/(2*22.5**2))
                pose_error += 10*exp(-(((self.x_casadi[k] - (((self.obb_x[i]-25)/5)-self.x_0))@(self.x_casadi[k] - (((self.obb_x[i]-25)/5)-self.x_0)).T)+(self.y_casadi[k] - (((self.obb_y[i]-25)/5)-self.y_0))@(self.y_casadi[k] - (((self.obb_y[i]-25)/5)-self.y_0)).T)/(2))
                pose_error += -exp(-(((self.x_casadi[k] - (((self.unex_x[i]-25)/5)-self.x_0))@(self.x_casadi[k] - (((self.unex_x[i]-25)/5)-self.x_0)).T)+(self.y_casadi[k] - (((self.unex_y[i]-25)/5)-self.y_0))@(self.y_casadi[k] - (((self.unex_y[i]-25)/5)-self.y_0)).T)/(2*8**2))


        
        self.opti.minimize(pose_error)



    def compute_control(self, initial_state, front_x, front_y,obb_x,obb_y,unex_x,unex_y):
        """Compute the control actions
        Args:
            initial_state (np.array): actual state of the robot
            reference_x (np.array): x reference for the robot
            reference_y (np.array): y reference for the robot
        Returns:
            (np.array): control actions
        """

        # Setting Initial State ---------------------------------------
    
        self.opti.set_value(self.x_0, initial_state[0])
        self.opti.set_value(self.y_0, initial_state[1])
        self.opti.set_value(self.yaw_0, initial_state[2])


        self.opti.set_value(self.front_x, front_x)
        self.opti.set_value(self.front_y, front_y)
        
        self.opti.set_value(self.unex_x, unex_x)
        self.opti.set_value(self.unex_y, unex_y)        
        self.opti.set_value(self.obb_x, obb_x)
        self.opti.set_value(self.obb_y, obb_y)  


        # Compute solution ---------------------------------------

        sol = self.opti.solve()
       
        # Taking just first action ---------------------------------------
        v = sol.value(self.U_casadi)[0]
        w = sol.value(self.U_casadi)[1]
        pose = sol.value(self.X_casadi)
        
        return v[0], w[0], pose
    

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

        if not rospy.has_param("~topics/vis2"):
            return False
        self._vis_topic2 = rospy.get_param("~topics/vis2")

        if not rospy.has_param("~frames/fixed"):
            return False
        self._fixed_frame = rospy.get_param("~frames/fixed")

        self.state_robot = np.array([0.0, 0.0, 0])
        self.front_x = []
        self.front_y = []
        self.unex_x = []
        self.unex_y = []
        self.obb_x = []
        self.obb_y = []
        self.dt = 0.1
        self.horizon = 10
        self.controller = Casadi_NMPC(self.horizon, self.dt)
        self.controller.reset()

        return True

    def RegisterCallbacks(self):
        # Subscriber.
        self._sensor_sub = message_filters.Subscriber(self._sensor_topic,
                                            PoseStamped)
        
        self._sensor_sub2 = message_filters.Subscriber(self._sensor_topic2,
                                           Three_cat)
        
        ts = message_filters.ApproximateTimeSynchronizer([self._sensor_sub, self._sensor_sub2], 2,0.1)

        ts.registerCallback(self.SensorCallback)        
        # Publisher.
        self._vis_pub = rospy.Publisher(self._vis_topic,
                                        Twist,
                                        queue_size=1)
        self._vis_pub2 = rospy.Publisher(self._vis_topic2,
                                        PoseArray,
                                        queue_size=1)

        return True

    # Callback to process sensor measurements.
    
    def SensorCallback(self, pose, map):
        rospy.logerr("1")
        sensor_x = pose.pose.position.x
        sensor_y = pose.pose.position.z
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [pose.pose.orientation.x, pose.pose.orientation.z,
                    -pose.pose.orientation.y, pose.pose.orientation.w])
        state_robot = np.array([sensor_x, sensor_y,  yaw+1.571])
        front_x = map.front_x
        front_y = map.front_y
        obb_x = map.obb_x
        obb_y = map.obb_y
        unex_x = map.unex_x
        unex_y = map.unex_y
        self.v, self.w, self.pose = self.controller.compute_control(state_robot, front_x,front_y,obb_x,obb_y,unex_x,unex_y)
        rospy.logerr("2")
        self.Visualize()

           
    def Visualize(self):
        
        a = PoseArray()
        a.header.stamp = rospy.Time.now()
        a.header.frame_id = self._fixed_frame

        for i in range(self.horizon+1):
            pose = Pose()
            pose.position.x = self.pose[0,i]  
            pose.position.y = self.pose[1,i]  
            pose.position.z = 0  
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose[2,i])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            a.poses.append(pose)

        m = Twist()
        m.linear.x = self.v 
        m.linear.y = 0
        m.linear.z = 0
        m.angular.x = 0 
        m.angular.y = 0 
        m.angular.z = self.w
        self._vis_pub.publish(m)
        self._vis_pub2.publish(a)


if __name__ == "__main__":
    rospy.init_node("mapping_node")

    og = OccupancyGrid2d()
    if not og.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()


