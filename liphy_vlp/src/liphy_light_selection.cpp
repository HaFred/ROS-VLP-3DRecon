#include <ros/ros.h>
#include <liphy_vlp/liphylight.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include <typeinfo>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


using namespace std;

#define USE_ODOMETERY_FILETERD 0
#define PI 3.14159265

geometry_msgs::Pose pose_odom;
geometry_msgs::Pose2D pose2d;

ros::Publisher pub;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    
    pose_odom.orientation.x = msg->pose.pose.orientation.x;
    pose_odom.orientation.y = msg->pose.pose.orientation.y;
    pose_odom.orientation.z = msg->pose.pose.orientation.z;
    pose_odom.orientation.w = msg->pose.pose.orientation.w;


    // cout<<"PoseReceivedFromOdom:"<<endl;
    // cout<<"Pose Orietattion.x"<<pose_odom.orientation.x<<endl;
    // cout<<"Pose Orietattion.y"<<pose_odom.orientation.y<<endl;
    // cout<<"Pose Orietattion.z"<<pose_odom.orientation.z<<endl;
    // cout<<"Pose Orietattion.w"<<pose_odom.orientation.w<<endl;

    // pose2d.x = msg->pose.pose.position.x;
    // pose2d.y = msg->pose.pose.position.y;
    
    // tf::Quaternion q(
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    
}

void lightCallback(const liphy_vlp::liphylight msg)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    float pos_x = 0.0;
    float pos_y = 0.0;
    float theta = 0.0;
    YAML::Node lights = YAML::LoadFile("/home/liphy/catkin_ws/src/liphy_vlp/lights/lights.yaml");
    YAML::Node map = YAML::LoadFile("/home/liphy/map/3.yaml");
    float x_cor = map["origin"][0].as<float>();
    float y_cor = map["origin"][1].as<float>();
    // cout << x_cor << "\t" << y_cor << endl;

    for (int i = 1; i <= lights["Lights"]["numLights"].as<int>(); i++)
    {
        // cout << typeid(lights["Lights"]["Light " + to_string(i)]["keypoint"].as<int>()).name() << endl;
        // cout << typeid(stoi(msg.keypoint)).name() << endl;

        // cout << (lights["Lights"]["Light " + to_string(i)]["keypoint"].as<int>() == stoi(msg.keypoint)) << endl;
        //cout << msg.keypoint << endl;
        if(msg.keypoint != "" && lights["Lights"]["Light " + to_string(i)]["keypoint"].as<int>() == stoi(msg.keypoint)){

            pos_x = lights["Lights"]["Light " + to_string(i)]["pos_x"].as<float>();
            pos_y = lights["Lights"]["Light " + to_string(i)]["pos_y"].as<float>();
            theta = msg.theta;

        }
    }
    
    pos_x = x_cor - pos_x;
    pos_y = y_cor - pos_y;

    pos_x += msg.light_x;
    pos_y += msg.light_y;

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    pose.pose.pose.position.x = pos_x;
    pose.pose.pose.position.y = pos_y;
    pose.pose.pose.position.z = 0.0;

    pose.pose.pose.orientation.x = pose_odom.orientation.x;
    pose.pose.pose.orientation.y = pose_odom.orientation.y;
    pose.pose.pose.orientation.z = pose_odom.orientation.z;
    pose.pose.pose.orientation.w = pose_odom.orientation.w;
    pose.pose.covariance = {0.7223, 0.0927, 0.117, 0.0, 0.0, 0.1171, 0.0927, 1.1779, -0.2214, 0.0, 0.0, 0.4157, 0.177, -0.2214, 9.0631, 0.0, 0.0, 0.5067, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1171, 0.4157, 0.5067, 0.0, 0.0, 3.4576};

    
    pub.publish(pose);
    
    
    
    cout << "Keypoint: " << msg.keypoint << endl;
    cout << "Corners: " << msg.corners << endl;
    cout << "Light_x: " << msg.light_x << endl;
    cout << "Light_y: " << msg.light_y << endl;
    cout << "Light_z: " << msg.light_z << endl;
    cout << "Light Theta: " << msg.theta << endl;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "liphy_light_selection");
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slovlp_ekf_info", 1);
    ros::Subscriber lls_subscriber = nh.subscribe("liphy_vlp_info", 1, lightCallback);
    std::cout<<"Liphly light selection node, suscribe to liphy_vlp_info topic"<<std::endl;

    #if (USE_ODOMETERY_FILETERD)
        ros::Subscriber sub_odom_filtered_ = nh.subscribe("odometry/filtered", 1, odometryCallback_);
        std::cout<<"Using Odometery Filtered: True"<<std::endl;
    #else
        ros::Subscriber sub_odom_ = nh.subscribe("odom", 1, odometryCallback_);
        std::cout<<"Using Odometery Filtered: False"<<std::endl;
    #endif
    
    ros::spin();
    return 0;
}