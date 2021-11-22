#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "std_msgs/String.h"
#include <sstream>
//#include <conio>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "single_LEDpositioning");

  ros::NodeHandle n;
  ros::Publisher pub_pose;

  pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("slovlp_ekf_info", 1);


  geometry_msgs::PoseWithCovarianceStamped pubPoseWithCovarianceStamped;

  pubPoseWithCovarianceStamped.header.frame_id = "map";
  pubPoseWithCovarianceStamped.header.stamp = ros::Time::now();
    
  pubPoseWithCovarianceStamped.pose.pose.position.x = 0.0;
  pubPoseWithCovarianceStamped.pose.pose.position.y = 0.0;
  pubPoseWithCovarianceStamped.pose.pose.position.z = 0.0;

  pubPoseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
  pubPoseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
  pubPoseWithCovarianceStamped.pose.pose.orientation.z = 0.0;
  pubPoseWithCovarianceStamped.pose.pose.orientation.w = 1.0;

  pubPoseWithCovarianceStamped.pose.covariance = {0.7223, 0.0927, 0.117, 0.0, 0.0, 0.1171, 0.0927, 1.1779, -0.2214, 0.0, 0.0, 0.4157, 0.177, -0.2214, 9.0631, 0.0, 0.0, 0.5067, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1171, 0.4157, 0.5067, 0.0, 0.0, 3.4576};

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  { 
    ROS_INFO("Waiting for key pressed");
    int c = std::cin.get();

    if (c == 'w'){
      pubPoseWithCovarianceStamped.pose.pose.position.y = pubPoseWithCovarianceStamped.pose.pose.position.y + 0.1;
    }else if (c == 'x'){
      pubPoseWithCovarianceStamped.pose.pose.position.y = pubPoseWithCovarianceStamped.pose.pose.position.y - 0.1;
    }

    

    pub_pose.publish(pubPoseWithCovarianceStamped);

    ros::spinOnce();

    //loop_rate.sleep();
    ++count;
  }


  return 0;
}
