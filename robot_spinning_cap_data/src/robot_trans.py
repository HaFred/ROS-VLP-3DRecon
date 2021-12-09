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
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt

x = 0.0
y = 0.0 
delta = 0.0
curr_translation = 0.0
trans_amnt = 0.0
trans_amnt_last = 0.0
trans_vel_cur = 0.0

# odom callback for trans
def odomTransCb(msg):
    global trans_amnt
    global trans_amnt_last
    global x
    global y
    global curr_translation
    global trans_vel_cur

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

def main():
    rospy.init_node("speed_controller")

    # the 2nd param for sub and pub is the msg type
    sub = rospy.Subscriber("/odom", Odometry, odomTransCb)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    speed = Twist() # for both linear and angular 

    r = rospy.Rate(4)

    while not rospy.is_shutdown():

        assigned_dist = 1.5

        rospy.loginfo("\n************")
        rospy.loginfo("delta is : %f" % delta)
        rospy.loginfo("curr_trans is: %f" % curr_translation)

        if assigned_dist - curr_translation > 0.1:
            speed.linear.x = 0.05
            speed.angular.z = 0.0
            rospy.loginfo("%f" % curr_translation)
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            
            # when reaches, publish the cap data signal to rscd node 

        pub.publish(speed)
        r.sleep()   


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInternalException:
        pass
