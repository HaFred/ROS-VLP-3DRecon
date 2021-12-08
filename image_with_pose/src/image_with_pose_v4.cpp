/* v4 the final version of fusing positioning from tf_echo odom -> map (which is in /tf, back and forth processed by the loosely coupled EKF), and the rotation from odom itself (base_footprint -> odom).
        In v4, the data cap signal is subscribed from the topic published by the robot spin node. No tele op node is required in this case.

        It turns out, the data_cap_en cannot turns true coz there is no ros::spin to activate the callback, the ros::spin is in the data_cap_en to avoid it.sub's imageCallback. Thus making data_cap_en stuck at false...
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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

using namespace cv;

#define VERSION_NUMBER "4"  // programmed by fred
#define FIXED_Z_VALUE 0.4

#define SAVE_PATH "/home/liphy/catkin_ws/src/image_with_pose/captured_images/"

// global var latestPose odom (base_footprint->odom), we use its rotation in the transformStamped meg as the pose
geometry_msgs::TransformStamped latestPoseFromOdom;

// global var latestPose EKF (odom->map), we use its positioning in the pose
geometry_msgs::TransformStamped latestPoseFromEKF;

bool data_cap_en;

// this flag is deprecated, previously is used for delay the capturing to adapt the 1Hz rate of VLP pose update
bool pose_is_updated = true;

// save the rotation from odom and position from ekf
void saveCurrentPose(std::string& current_time_stamp){
    ROS_INFO("Overall saving pose");
	
	// saving the rotation matrix in the transform matrix
	tf::Quaternion q(
        latestPoseFromOdom.transform.rotation.x,
        latestPoseFromOdom.transform.rotation.y,
        latestPoseFromOdom.transform.rotation.z,
        latestPoseFromOdom.transform.rotation.w
    );
	tf::Matrix3x3 m(q);
	
	// initialize the pose matrix 
	cv::Mat pose_matrix = cv::Mat::zeros(3, 4, CV_64F);
	
	pose_matrix.at<double>(1, 1) = m.getRow(1).getX();
	pose_matrix.at<double>(1, 2) = m.getRow(1).getY();
	pose_matrix.at<double>(1, 3) = m.getRow(1).getZ();
	pose_matrix.at<double>(1, 4) = latestPoseFromEKF.transform.translation.x;
	
	pose_matrix.at<double>(2, 1) = m.getRow(2).getX();
	pose_matrix.at<double>(2, 2) = m.getRow(2).getY();
	pose_matrix.at<double>(2, 3) = m.getRow(2).getZ();
	pose_matrix.at<double>(2, 4) = latestPoseFromEKF.transform.translation.y;
	
	pose_matrix.at<double>(3, 1) = m.getRow(3).getX();
	pose_matrix.at<double>(3, 2) = m.getRow(3).getY();
	pose_matrix.at<double>(3, 3) = m.getRow(3).getZ();
	pose_matrix.at<double>(3, 4) = FIXED_Z_VALUE;
	
	// saving quat from odom
    std::ofstream quat_odom_file(std::string(SAVE_PATH) + std::string("quat_from_odom_") + current_time_stamp + std::string(".txt"), std::ios::out | std::ios::binary);
    for (int i = 0; i < 4; i++){
        quat_odom_file << std::to_string(q[i]) + " ";
    }
    quat_odom_file << "\n";
    quat_odom_file << std::to_string(latestPoseFromOdom.transform.translation.x) + " " << std::to_string(latestPoseFromOdom.transform.translation.y) + " ";
	
	// saving pose matrix
	std::ofstream pose_file(std::string(SAVE_PATH) + std::string("pose_at_") + current_time_stamp + std::string(".txt"), std::ios::out | std::ios::binary);
	for (int i = 1; i <= pose_matrix.rows; i++) {
		for (int j = 1; j <= pose_matrix.cols; j++){
			pose_file << std::to_string(pose_matrix.at<double>(i, j)) + " ";
		}
		pose_file << "\n";
	}

    quat_odom_file.close();
	pose_file.close();
}

// here if you look at the src of imageTransport::Sub, usb_cam, you will find that they directly utilize:
// it.advertiseCamera("image_raw", 1); rather than using nh.advertise(). With the publish(const sensor_msgs::ImageConstPtr& image, ...) of 
// it.advertiseCamera, the cb of any subs (imageCallback) here can take in ImageConstPtr& image type param without requiring like nh.advertise<> setting up the msg datatype
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
        std::cout<<"Image saved at "<<current_time_stamp<<std::endl;

        saveCurrentPose(current_time_stamp);

        // restore the flag as false since the pose for VLP in the buffer is used
        // make it true for the testing purpose
        pose_is_updated = true;
    }else{
        std::cout<<"Image was not saved, becasue pose was not updated. Try again."<<std::endl;
    }
}

void dataCapEnCb(const std_msgs::Bool& msg)
{
    // if (msg.data){
    //     data_cap_en = true;
    // }
    // else {
    //     data_cap_en = false;
    // }
    // ROS_INFO("In image_with_pose node, I got %s msg from dataCapEnCb", data_cap_en);
    data_cap_en = msg.data;
}

int main(int argc, char** argv)
{
    std::cout<<"Image with Pose node is started. Version: "<<VERSION_NUMBER<<std::endl;

    ros::init(argc, argv, "image_with_pose");
    ros::NodeHandle nh;

    // tf2_listener adpatation, using the listener to subscribe tf
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Subscriber data_cap_en_sub = nh.subscribe("data_cap_en_sub", 1, &dataCapEnCb);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback); // the size of the publisher queue is set to 1

    while (nh.ok()){
        // ROS_INFO("Press a key to capture image");
        // int c = std::cin.get();

        // bool c = data_cap_en;

        if (data_cap_en){
            // ROS_INFO("Robot spin node pass in the enable signal as %s", data_cap_en);
            // tf2_listener adpatation
            try{
                // fixme: is the source odom or map??? No, then it will be no transform, namely, every quat is w=1 and no translation...
                // lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, const ros::Duration timeout=ros::Duration(0.0)) const
                latestPoseFromOdom = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
                std::cout<<"Pose from Odom listener passed."<<std::endl;
                latestPoseFromEKF = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
                std::cout<<"Pose from EKF listener passed."<<std::endl;

                // hardcoded the pose_is_updated as true for now
                std::cout<<"Pose updated: "<<pose_is_updated<<std::endl;
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            ros::spinOnce(); // single threaded spinning, allowing all the callbacks work, only for subscription. However, we have the it subscriber here, so it has to be activated.
        }
        else {
            ROS_INFO("no snap signal, image with pose not activated yet");
        }
    }
    return 0;
}