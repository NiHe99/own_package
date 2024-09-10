import rospy
from casadi import *
from std_msgs.msg import String


# Evaluate numerically


pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():

    
    # Symbols/expressions
    x = MX.sym('x')
    y = MX.sym('y')
    z = MX.sym('z')
    f = x**2+100*z**2
    g = z+(1-x)**2-y

    nlp = {}                 # NLP declaration
    nlp['x']= vertcat(x,y,z) # decision vars
    nlp['f'] = f             # objective
    nlp['g'] = g             # constraints

    # Create solver instance
    F = nlpsol('F','ipopt',nlp);

    # Solve the problem using a guess
    F(x0=[2.5,3.0,0.75],ubg=0,lbg=0)

    rospy.logerr(F)
    pub.publish("hello world")
    r.sleep()