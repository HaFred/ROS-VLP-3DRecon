#!/usr/bin/env python
# the shebang here is necessary for running rospy, otherwise import-im6.q16: not authorized `rospy' @ error/constitute.c/WriteImage/1037. error

""" 
 /**
 * @file 
 *
 * @brief This py file do the translation where no data cap happens.
 *
 * @date Dec 4, 2021
 *
 * @author HaFred
 **/
"""
import rospy
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import sqrt
# import argparser

x = 0.0
y = 0.0 
delta = 0.0
curr_translation = 0.0
trans_amnt = 0.0
trans_amnt_last = 0.0
trans_vel_cur = 0.0

local_trans_enable = False # flag whether the first rot has finished

# odom callback for trans
def odomTransCb(msg):
    global trans_amnt
    global trans_amnt_last
    global x
    global y
    global curr_translation
    global trans_vel_cur
    global delta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # cal the trans amount from the default origin
    trans_amnt = sqrt(x**2 + y**2)

    # the accumulated current trans
    delta = trans_amnt - trans_amnt_last
    curr_translation = curr_translation + delta

    # update the trans last buffer
    trans_amnt_last = trans_amnt

    trans_vel_cur = msg.twist.twist.linear.x

# translation ctrl cb
def transCtrlCb(msg):
    global local_trans_enable
    local_trans_enable = msg.data

def shutdownHook():
    rospy.loginfo("shutdown speed_ctrler node to reduce overhead")

def main():
    rospy.init_node("speed_controller")

    # the 2nd param for sub and pub is the msg type
    sub_odom = rospy.Subscriber("/odom", Odometry, odomTransCb)
    sub_rscd = rospy.Subscriber("enable_robot_trans", Bool, transCtrlCb)
    pub_speed = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    pub_2ndRotate = rospy.Publisher("second_rotate", Bool, queue_size = 100)

    speed = Twist() # for both linear and angular 
    flag_2nd_rotate = Bool()
    flag_2nd_rotate.data = False

    r = rospy.Rate(4)

    while not rospy.is_shutdown():

        assigned_dist = 0.45  # there is odom dist shift, 1m here is around 1.5 in reality. todo consider replacing the dist msg from odom into tf

        rospy.loginfo("\n************")
        rospy.loginfo("delta is : %f" % delta)
        rospy.loginfo("curr_trans is: %f" % curr_translation)

        if (assigned_dist - curr_translation > 0.05):
            if local_trans_enable:
                speed.linear.x = 0.03
                speed.angular.z = 0.0
                # flag_2nd_rotate.data = False
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                flag_2nd_rotate.data = False
            rospy.loginfo("%f" % curr_translation)            
        else:
            rospy.loginfo("ready for or doing 2nd rotataion")
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            flag_2nd_rotate.data = True
            # rospy.on_shutdown(shutdownHook)
            
        # when reaches, publish the cap data signal to rscd node 
        pub_2ndRotate.publish(flag_2nd_rotate)
        pub_speed.publish(speed)
        r.sleep()   


if __name__ == "__main__":
    print(sys.version)
    try:
        main()
    except rospy.ROSInternalException:
        pass
