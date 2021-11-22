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
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <typeinfo>
#include "yaml-cpp/yaml.h"

using namespace cv;
using namespace std;

#define VERSION_NUMBER "2.0"
#define FIXED_Z_VALUE 0.4

#define SAVE_PATH "/home/liphy/catkin_ws/src/image_with_pose/captured_images/"


geometry_msgs::PoseWithCovarianceStamped latestPose;

// image_transport::Publisher pub;
// ros::Publisher info_pub;

bool capture_next_frame = false; // Not used at the moment
bool pose_is_updated = false;

void saveCurrentPose(std::string& current_time_stamp);


void pose_from_vlp_(const geometry_msgs::PoseWithCovarianceStamped& msg) {

    std::cout<<"Pose from VLP received"<<std::endl;

    latestPose = msg;

    pose_is_updated = true;

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   
    std::cout<<"Image callback received"<<std::endl;

    // **** First Convert the Sensor Msg Image to OpenCV format

    cv_bridge::CvImagePtr cv_ptr;
    try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
    catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    
    if(pose_is_updated){
        
        std::string current_time_stamp = std::to_string((int)(ros::Time::now().toSec()));

        capture_next_frame = false;
        ROS_ASSERT( cv::imwrite( std::string(SAVE_PATH) + std::string( "image_at_" ) + current_time_stamp + std::string( ".png" ), cv_ptr->image ) );

        //imwrite("/home/Documents/image_with_pose.jpg", cv_ptr->image);
        cout<<"Image saved at "<<current_time_stamp<<endl;

        saveCurrentPose(current_time_stamp);

        pose_is_updated = false;
        
    }else{
        cout<<"Image was not saved, becasue pose was not updated. Try again."<<endl;
    }


}

// Function to save the current pose with time_stamp in a text file
void saveCurrentPose(std::string& current_time_stamp){
    
    // Convert Quaternion to Roation Matrix
    tf::Quaternion q(
        latestPose.pose.pose.orientation.x,
        latestPose.pose.pose.orientation.y,
        latestPose.pose.pose.orientation.z,
        latestPose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    // Create a Matrix to save Pose with last column being the T vector
    cv::Mat pose_matrix = cv::Mat::zeros(3,4,CV_64F);

    pose_matrix.at<double>(1,1) = m.getRow(1).getX();
    pose_matrix.at<double>(1,2) = m.getRow(1).getY();
    pose_matrix.at<double>(1,3) = m.getRow(1).getZ();
    pose_matrix.at<double>(1,4) = latestPose.pose.pose.position.x;

    pose_matrix.at<double>(2,1) = m.getRow(2).getX();
    pose_matrix.at<double>(2,2) = m.getRow(2).getY();
    pose_matrix.at<double>(2,3) = m.getRow(2).getZ();
    pose_matrix.at<double>(2,4) = latestPose.pose.pose.position.y;

    pose_matrix.at<double>(3,1) = m.getRow(3).getX();
    pose_matrix.at<double>(3,2) = m.getRow(3).getY();
    pose_matrix.at<double>(3,3) = m.getRow(3).getZ();
    pose_matrix.at<double>(3,4) = FIXED_Z_VALUE;

    ofstream myfile(std::string(SAVE_PATH) + std::string("pose_at_") + current_time_stamp + std::string(".txt"), ios::out | ios::binary);

    for (int i = 1; i <= pose_matrix.rows; i++){
        for (int j=1; j<= pose_matrix.cols; j++){
            myfile << std::to_string(pose_matrix.at<double>(i,j))+" ";
        }
        myfile <<"\n";
    }

    myfile.close();

    // cv::FileStorage file(std::string(SAVE_PATH) +"some_name.ext", cv::FileStorage::WRITE);

    // // Write to file!
    // file << "poseMatrix" << pose_matrix;

}

int main(int argc, char** argv)
{
    std::cout<<"Image with Pose node is started. Version: "<<VERSION_NUMBER<<std::endl;

    ros::init(argc, argv, "image_with_pose");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    // pub = it.advertise("liphy_vlp/processed_image", 1);

    //info_pub = nh.advertise<liphy_vlp::liphylight>("image_n_pose_info", 1);
    ros::Subscriber pose_from_vlp = nh.subscribe("slovlp_ekf_info", 1, pose_from_vlp_);

    while (ros::ok())
  { 
    pose_is_updated = false;
    ROS_INFO("Press a key to capture image");
    int c = std::cin.get();
    //capture_next_frame = true;

    ros::spinOnce();

  }
    return 0;
}


