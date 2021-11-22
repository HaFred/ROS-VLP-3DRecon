#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
// std
#include <stdio.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;
  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);

    // f: a publisher node in listener, which publishes a topic for as turtle2/cmd_vel for usage of turtle2
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped worldToTurtle1;
    try{
        // target frame turtle2, src frame turtle1, transformedStamped stored the transform of turtle1 in terms of turtle2, and be used for calculating turtle2 ang & vel
        transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                               ros::Time(0));
        // this var is to check whether the value stored in wordToTurtle1 here is the same as tf_echo?
        // worldToTurtle1 = tfBuffer.lookupTransform("turtle1", "world", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ROS_INFO("Just INFO Sanity Test in the Listener, here excpetion happenes...");
      ros::Duration(1.0).sleep();
      continue;
    }

    // ROS_INFO("%f", transformStamped.transform.translation.y);
    // ROS_INFO("Just INFO Sanity Test in the Listener...");
    // cout<<transformStamped.transform.translation.y<<endl;
    // cout<<"**********************worldToTurtle1:"<<endl;
    // cout<<worldToTurtle1<<endl;
    // cout<<"**********************\n"<<endl;

    // vel_msg for turtle2 to chase turtle1
    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();  // f: Goes to sleep according to the loop rate defined above
    
  }
  return 0;
};