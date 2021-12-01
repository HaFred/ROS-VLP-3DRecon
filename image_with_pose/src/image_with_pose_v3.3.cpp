/* v3.3 tries the tf::TransformListener method with slovlp_ekf_info topic subscription together.
        It also saves the pose matrix from vlp and ekf_tf into 2 different folders. Note that 3.3 is not make and tested yet...
 */

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>  // not sure will it be conflicted with tf2?
#include <nav_msgs/Odometry.h>
#include <typeinfo>
#include "yaml-cpp/yaml.h"

// fred, version 3.1 using these addtional headers from tf2 listener template
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace cv;
using namespace std;

#define VERSION_NUMBER "3.3"  // programmed by fred
#define FIXED_Z_VALUE 0.4

#define SAVE_PATH "/home/liphy/catkin_ws/src/image_with_pose/captured_images/"

// // global var latestPose, takes the posewithcovstamped from slovlp_ekf_info at liphy_light_selection.cpp
geometry_msgs::PoseWithCovarianceStamped latestPoseVLP;

// global var latestPose as in tf_listener, however it cannot be as transformstamped type, coz tfBuffer.lookupTransform() below won't be able to accept "world" frame?
// as long as tf is publishing transformstamped type as in the tf_listener tutorial, then doing this with correct tfBuffer.lookupTransform() should work
geometry_msgs::TransformStamped latestPoseEKF;
bool pose_is_updated = false;

// a callback for slovlp_ekf_info topic sub
void poseFromVLP(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    std::cout<<"Pose from VLP received"<<std::endl;
    latestPoseVLP = msg;
    pose_is_updated = true;
}

// using listener so no need
// // a callback for tf topic sub
// void pose_from_tf_(const geometry_msgs::TransformStamped& msg) {
//     std::cout<<"Pose from EKF received"<<std::endl;
//     latestPoseEKF = msg;
//     pose_is_updated = true;
// }

// will be called in imageCallback() thus declared here
void saveCurrentPoseVLP(std::string& current_time_stamp);

// will be called in imageCallback() thus declared here
void saveCurrentPoseEKF(std::string& current_time_stamp);

// todo: f: find out whether there is such msg file built by catkin for every built. And will the such &msg parameter be able to work only with the msg file built? 

void imageCallback(const sensor_msgs::ImageConstPtr& msg) // these msg are setup to receive msg from the topic which is encoded by the publisher 
{
   
    std::cout<<"Image callback received"<<std::endl;

    // **** First Convert the Sensor Msg Image to OpenCV format

    cv_bridge::CvImagePtr cv_ptr;
    try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
    }
            
    if(pose_is_updated){
        std::string current_time_stamp = std::to_string((int)(ros::Time::now().toSec()));

        ROS_ASSERT( cv::imwrite( std::string(SAVE_PATH) + std::string( "image_at_" ) + current_time_stamp + std::string( ".png" ), cv_ptr->image ) );

        //imwrite("/home/Documents/image_with_pose.jpg", cv_ptr->image);
        cout<<"Image saved at "<<current_time_stamp<<endl;

        saveCurrentPoseVLP(current_time_stamp);
        saveCurrentPoseEKF(current_time_stamp);
    }else{
        cout<<"Image was not saved, becasue pose was not updated. Try again."<<endl;
    }
}

// Function to save the current pose with time_stamp in a text file
void saveCurrentPoseEKF(std::string& current_time_stamp){
    
    // Convert Quaternion to Roation Matrix
    tf::Quaternion q(
        latestPoseEKF.transform.rotation.x,
        latestPoseEKF.transform.rotation.y,
        latestPoseEKF.transform.rotation.z,
        latestPoseEKF.transform.rotation.w);
    tf::Matrix3x3 m(q);

    // Create a Matrix to save Pose with last column being the T vector
    cv::Mat pose_matrix = cv::Mat::zeros(3,4,CV_64F);

    pose_matrix.at<double>(1,1) = m.getRow(1).getX();
    pose_matrix.at<double>(1,2) = m.getRow(1).getY();
    pose_matrix.at<double>(1,3) = m.getRow(1).getZ();
    pose_matrix.at<double>(1,4) = latestPoseEKF.transform.translation.x; // f: vlp position x?

    pose_matrix.at<double>(2,1) = m.getRow(2).getX();
    pose_matrix.at<double>(2,2) = m.getRow(2).getY();
    pose_matrix.at<double>(2,3) = m.getRow(2).getZ();
    pose_matrix.at<double>(2,4) = latestPoseEKF.transform.translation.y; // f: vlp position y?

    pose_matrix.at<double>(3,1) = m.getRow(3).getX();
    pose_matrix.at<double>(3,2) = m.getRow(3).getY();
    pose_matrix.at<double>(3,3) = m.getRow(3).getZ();
    pose_matrix.at<double>(3,4) = FIXED_Z_VALUE;

    ofstream pose_ekf_file(std::string(SAVE_PATH) + std::string("pose_at_") + current_time_stamp + std::string(".txt"), ios::out | ios::binary);

    for (int i = 1; i <= pose_matrix.rows; i++){
        for (int j=1; j<= pose_matrix.cols; j++){
            pose_ekf_file << std::to_string(pose_matrix.at<double>(i,j)) + " ";
        }
        pose_ekf_file <<"\n";
    }

    pose_ekf_file.close();
}

void saveCurrentPoseVLP(std::string& current_time_stamp){
    // m is the rot matrix generated from the quat api
    tf::Quaternion q(
        latestPoseVLP.pose.pose.orientation.x,
        latestPoseVLP.pose.pose.orientation.y,
        latestPoseVLP.pose.pose.orientation.z,
        latestPoseVLP.pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);

    // create a matrix to save pose with last column being the T vector
    cv::Mat pose_matrix = cv::Mat::zeros(3, 4, CV_64F);

    pose_matrix.at<double>(1,1) = m.getRow(1).getX();
    pose_matrix.at<double>(1,2) = m.getRow(1).getY();
    pose_matrix.at<double>(1,3) = m.getRow(1).getZ();
    pose_matrix.at<double>(1,4) = latestPoseVLP.pose.pose.position.x;

    pose_matrix.at<double>(2,1) = m.getRow(2).getX();
    pose_matrix.at<double>(2,2) = m.getRow(2).getY();
    pose_matrix.at<double>(2,3) = m.getRow(2).getZ();
    pose_matrix.at<double>(2,4) = latestPoseVLP.pose.pose.position.y;

    pose_matrix.at<double>(3,1) = m.getRow(1).getX();
    pose_matrix.at<double>(3,2) = m.getRow(1).getY();
    pose_matrix.at<double>(3,3) = m.getRow(1).getZ();
    pose_matrix.at<double>(3,4) = FIXED_Z_VALUE;

    ofstream pose_vlp_file(std::string(SAVE_PATH) + std::string("/pose_by_vlp/") + std::string("quad_from_vlp_") + current_time_stamp + std::string(".txt"), ios::out | ios::binary);
    for (int i = 1; i <= pose_matrix.rows; i++){
        for(int j = 1; j <= pose_matrix.cols; j++){
            pose_vlp_file << std::to_string(pose_matrix.at<double>(i, j)) + " ";
        }
        pose_vlp_file << "\n";
    }

    pose_vlp_file.close();
}

int main(int argc, char** argv)
{
    std::cout<<"Image with Pose node is started. Version: "<<VERSION_NUMBER<<std::endl;

    ros::init(argc, argv, "image_with_pose");
    ros::NodeHandle nh;

    // tf2_listener adpatation, using the listener to subscribe tf
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback); // the size of the publisher queue is set to 1

    ros::Subscriber pose_from_vlp = nh.subscribe("slovlp_ekf_info", 1, poseFromVLP);

    while (nh.ok()){
        ROS_INFO("Press a key to capture image");
        int c = std::cin.get();

        // tf2_listener adpatation
        try{
            // fixme: is such lookupTransform really working for the info which is published by the ekf_se node in tf?
            latestPoseEKF = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
            pose_is_updated = true;
            cout<<"Pose from VLP received."<<endl;
            cout<<"Pose updated: "<<pose_is_updated<<endl;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce(); // single threaded spinning, allowing all the callbacks work, only for subscription. However, we have the it subscriber here, so it has to be activated.
    }
    return 0;
}