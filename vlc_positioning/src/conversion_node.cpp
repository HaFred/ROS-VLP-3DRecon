#include "ros/ros.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/String.h"
#include <sstream>
#define PI 3.14159265

ros::Publisher pub_pose_odom;
ros::Publisher pub_pose_odom_filtered;
ros::Publisher pub_pose_imu;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw * 180/PI;
    pub_pose_odom.publish(pose2d);
}

void odometryFilteredCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw * 180/PI;
    pub_pose_odom_filtered.publish(pose2d);
}

void imuCallback_(const sensor_msgs::Imu::ConstPtr msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = 0;
    pose2d.y = 0;
    
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw * 180/PI;
    pub_pose_imu.publish(pose2d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conversion_node");
    
    ros::NodeHandle nh_;

    ROS_INFO("Conversiond Node");

    ros::Subscriber sub_odom_ = nh_.subscribe("odom", 1, odometryCallback_);
    pub_pose_odom = nh_.advertise<geometry_msgs::Pose2D>("pose2d_odom", 1);

    ros::Subscriber sub_odom_filtered_ = nh_.subscribe("odometry/filtered", 1, odometryFilteredCallback_);
    pub_pose_odom_filtered = nh_.advertise<geometry_msgs::Pose2D>("pose2d_odom_filtered", 1);

    ros::Subscriber sub_imu_ = nh_.subscribe("imu", 1, imuCallback_);
    pub_pose_imu = nh_.advertise<geometry_msgs::Pose2D>("pose2d_imu", 1);
    
    ros::spin();
    
    return 0;
}