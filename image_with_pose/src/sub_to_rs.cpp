/**
 * @file sub_to_rs.cpp
 *
 * @brief Just a simple sub to rs
 *
 * @date Dec 2, 2021
 *
 * @author HaFred
 **/

#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_with_pose/robot_spinning.h>

// bool ensig_received_status;

void msgCb(const std_msgs::Bool& msg)
{
    // ROS_INFO("In sub_to_rs, receive %s");
    std::cout << "In sub_to_rs, receive " << (msg.data ? "true" : "false") << std::endl;
    // ensig_received_status = msg.data; // if print out this var within while(ok) may leads to segmentation fault, which makes the robot_spinnning node unfunctional
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_to_rs");
    ros::NodeHandle nh;

    ros::Subscriber sub_to_rs = nh.subscribe("ensig_to_image_with_pose", 100, msgCb);


    // if not while() clause here, dont use spinOnce(), coz the sub node will exit with only one-time listenining... But if we make it into spin(), the publisher in the robot_spinning.cpp will be affected,and the robot spin behavior got somehow weirdly affected in the robot_spinning node (why sub affects pub node???). For now we just take the while() clause way
    while (ros::ok())
    {   
        ros::spinOnce();
    }
    
    return 0;
}