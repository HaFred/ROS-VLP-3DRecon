#!/usr/bin/env python
# the shebang here is necessary for running rospy, otherwise import-im6.q16: not authorized `rospy' @ error/constitute.c/WriteImage/1037. error

""" 
 /**
 * @file 
 *
 * @brief This py file is the template for rospy robot moving
 *
 * @date Dec 4, 2021
 *
 * @author HaFred
 **/
"""
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

x = 0.0
y = 0.0 
theta = 0.0

# callback fn
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # define the moving distance delta in a msg var

rospy.init_node("speed_controller")

# the 2nd param for sub and pub is the msg type
sub = rospy.Subscriber("/odom", Odometry, newOdom) # rostopic list to check which to subscribe
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist() # for both linear and angular 

r = rospy.Rate(4)

goal = Point()
goal.x = 1
goal.y = 1

while not rospy.is_shutdown():
    inc_x = goal.x - x # current x
    inc_y = goal.y - y # current y

    angle_to_goal = atan2(inc_y, inc_x)
    rospy.loginfo("\n************")
    rospy.loginfo("theta is : %f" % theta)
    rospy.loginfo("angle_to_goal is : %f" % angle_to_goal)

    if abs(angle_to_goal - theta) > 0.1: # rotate first, yaw angle theta changes, once it comes to angle_to_goal (which is not changed for spinning) it stops spinning
        speed.linear.x = 0.0
        speed.angular.z = 0.1
    else: # then go straight
        speed.linear.x = 0.05
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()   
