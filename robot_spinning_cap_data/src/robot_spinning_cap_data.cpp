 /**
 * @file /include/image_with_pose/robot_spinning.cpp
 *
 * @brief This node merge image_with_pose and robot_spinning into one single node, to avoid the closed loop of image_with_pose_v4.
 *
 * @date Nov 24, 2021
 *
 * @author HaFred
 **/

// image with pose
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

// robot spinning
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <robot_spinning_cap_data/robot_spinning.h>

#define FIXED_Z_VALUE 0.4

#define SAVE_PATH "/home/liphy/catkin_ws/src/robot_spinning_cap_data/cap_data/"

// global var latestPose odom (base_footprint->odom), we use its rotation in the transformStamped meg as the pose
geometry_msgs::TransformStamped latestPoseFromOdom;

// global var latestPose EKF (odom->map), we use its positioning in the pose
geometry_msgs::TransformStamped latestPoseFromEKF;

namespace robot_spinning
{
    SpinApp::SpinApp() : nh(), priv_nh("~") // make priv_nh priv when access
    {}

    SpinApp::~SpinApp()
    {}

    void SpinApp::init()
    { // direct class access scope resolution operator

        // because of the priv_nh, when calling this srv, need the prefix in the name. And this service deal with parameters from the rosservice call cmd
        srv_start_spin = priv_nh.advertiseService("spin_srv", &SpinApp::doRobotSpinServiceCb, this); // object to call srv_func on

        // the callback for listener and it_cb is in snap()
        // tf2_listener adpatation, using the listener to subscribe tf
        tf2_ros::TransformListener tfListener(tfBuffer);
        image_transport::ImageTransport it(nh);

        // once include header and class, do not put the declaration and definition together as in iwp_v2... It may cause the sub not working
        sub_camera = it.subscribe("usb_cam/image_raw", 1, &SpinApp::imageCb, this); // the size of the publisher queue is set to 1
        
        /****************************
         robot control
        ****************************/
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100); // queue 100, no latch
        sub_odom = nh.subscribe("odom", 100, &SpinApp::odomCb, this); // callback function ptr is passed in as the 3rd arg 

        cmd_vel.linear.x = 0.0f;
        cmd_vel.linear.y = 0.0f;
        cmd_vel.linear.z = 0.0f;
        cmd_vel.angular.x = 0.0f;
        cmd_vel.angular.y = 0.0f;
        cmd_vel.angular.z = 0.0f;
        zero_cmd_vel = cmd_vel;
        is_active = false;
        ang_vel_cur = 0.0;
        given_target_angle = 0.0;
        curr_angle = 0.0;
        last_angle = 0.0;
    }

    void SpinApp::spin()
    {
        ros::Rate loop_rate(10);
        double start_time = 0.0;
        bool take_snapshot = false;

        while(ros::ok())
        {
            if(is_active)
            {
               // robot spinning
                ROS_INFO_STREAM_THROTTLE(1.0, "Degrees to go: " << radians_to_degrees(std::abs(given_target_angle - curr_angle))); // throttle for this output
                if ((given_target_angle - curr_angle) <= 0.0174) // check if ultimate target angle is reached (<1 degree) & finished, then cap images and do the snitching. But here for 3d_recon we just give the en signal for the last frame cap
                {
                    snap();
                    // halt the robot for spinning 180 degree
                    pub_cmd_vel.publish(zero_cmd_vel);
                    log("Finished Semicircle Data Cap");

                    ROS_INFO("Angle: %f", curr_angle); 
                    ROS_INFO("Last Angle: %f", last_angle); 
                    curr_angle=0.0;
                    last_angle=0.0;
                    is_active = false;
                }
                else
                { // not finished yet
                    if (hasReachedAngle())
                    {
                        pub_cmd_vel.publish(zero_cmd_vel); // stop before taking a snapshot when angle reached 
                        ROS_INFO("Reach: hasReachedAngle");
                        take_snapshot = true;
                    }
                    if (take_snapshot) // the take_snapshot signal is true, stop and data cap
                    {
                        if (std::abs(ang_vel_cur) <= 0.01) // wait until robot has stopped
                        {
                            snap(); // ask the process to handle the callback with the spinning once and consume 1 sec
                            ROS_INFO("take_snapshot: take_snapshot");
                            take_snapshot = false; // because the robot only stops when the rot target is achieved, by then stop taking snapshot
                        }
                        else
                        {
                            std::stringstream ss;
                            std::string str;
                            ss << "Waiting for the robot to stop (speed = " << ang_vel_cur << ")...";
                            str = ss.str(); // turn stream into str?
                            log(str);
                        }
                    }
                    else // haven't meet the requirement, rotate
                    {
                        rotate();
                    }
                }
            }
            else
            {
                // ROS_INFO("In the spin(), but is_active false");
            }
            // if not active, still needs to activate the callback to make the process flowing by single-threaded spin
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /****************************
     priviate api for the app 
    ****************************/    

    // used when spinning reaches its interval angle, ask the process to handle the callback of tflistener and it_sub photo, with the single-threaded spin and consume 1 sec
    void SpinApp::snap()
    {
        log("snap");
        store_image = true;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // the whole motion function, rotate and translate
    void SpinApp::rotate()
    {
        log("rotate");
        pub_cmd_vel.publish(cmd_vel);
    }

    // the rotation reaches a certain angle interval, true for data capturing
    bool SpinApp::hasReachedAngle()
    {
        if (curr_angle > last_angle + degrees_to_radians(snap_interval))
        {
            last_angle = curr_angle;
            return true;
        } 
        else
        {
            return false;
        }

    }

    void SpinApp::odomCb(const nav_msgs::OdometryConstPtr& msg)
    {
        // ROS_INFO("odom cb is activated");
        static double heading_last = 0.0f; // static initialized only once, and accu
        double heading = 0.0f;

        Eigen::AngleAxisf angle_axis(Eigen::Quaternionf(msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z));

        Eigen::Vector3f axis = angle_axis.axis(); // a read-write reference to the stored rotation axis, is it xyz?

        // force the axis z or y to be positive? why?
        if (axis(2) > 0.0)
        {
            heading = angle_axis.angle();
            // ROS_INFO("F: Cur heading is %f", heading);
        }
        else if (axis(2) < 0.0)
        {
            heading = -1.0 * angle_axis.angle();
            // ROS_INFO("F: Cur heading is %f", heading);
        }

        curr_angle += std::abs(wrap_angle(heading - heading_last));
        heading_last = heading;
        ang_vel_cur = msg->twist.twist.angular.z;
    }

    // here if you look at the src of imageTransport::Sub, usb_cam, you will find that they directly utilize: it.advertiseCamera("image_raw", 1); rather than using nh.advertise(). With the publish(const sensor_msgs::ImageConstPtr& image, ...) of it.advertiseCamera, the cb of any subs (imageCb) here can take in ImageConstPtr& image type param without requiring like nh.advertise<> setting up the msg datatype
    void SpinApp::imageCb(const sensor_msgs::ImageConstPtr& msg) // these msg are setup to receive msg from the topic which is encoded by the publisher 
    {
        ROS_INFO("camera image cb is activated ********");
        if (store_image)
        {
            ROS_INFO("STORE IMAGE true, Image callback received");

            // **** First Convert the Sensor Msg Image to OpenCV format

            cv_bridge::CvImagePtr cv_ptr;
            try{
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
            }

            // tf listener
            try{
                // fixme: is the source odom or map??? No, then it will be no transform, namely, every quat is w=1 and no translation...
                // lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, const ros::Duration timeout=ros::Duration(0.0)) const
                latestPoseFromOdom = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
                std::cout<<"Pose from Odom listener passed."<<std::endl;
                latestPoseFromEKF = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
                std::cout<<"Pose from EKF listener passed."<<std::endl;
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                // continue; // coz no while here
            }
            
            std::string current_time_stamp = std::to_string((int)(ros::Time::now().toSec()));

            ROS_ASSERT( cv::imwrite( std::string(SAVE_PATH) + std::string( "image_at_" ) + current_time_stamp + std::string( ".png" ), cv_ptr->image ) );

            //imwrite("/home/Documents/image_with_pose.jpg", cv_ptr->image);
            std::cout<<"Image saved at "<<current_time_stamp<<std::endl;

            SpinApp::saveCurrentPose(current_time_stamp);
            store_image = false;
        }
        else
        {
            ROS_INFO("STORE IMAGE is false, the imageCb not activated");
        }
    }

    // the msgs md5sum security parity pairs are hard to deal with, thus we borrow the request and response msgs from TakePanorama here
    bool SpinApp::doRobotSpinServiceCb(turtlebot3_applications_msgs::TakePanorama::Request& request, turtlebot3_applications_msgs::TakePanorama::Response& response)
    {
        if (is_active && (request.mode == request.SNAPANDROTATE))
        {
            log("Panorama creation already in progress.");
            response.status = request.IN_PROGRESS;
        }
        else if (is_active && (request.mode == request.STOP))
        {
            is_active = false;
            log("Robot spin stopped");
            response.status = request.STOPPED;
            return true;
        }
        else if (!is_active && (request.mode == request.STOP))
        {
            log("No spin and data cap in progress");
            response.status = request.STOPPED;
            return true;
        }
        else
        {
            // physical exceptions below
            if (request.pano_angle <= 0.0)
            {
                log("Specified spin angle is zero or negative! Robot spin aborted");
                return true;
            }
            else if (request.snap_interval <= 0.0)
            {
                log("Specified snapshot interval is zero or negative! Robot spin aborted");
                return true;
            }
            else if (request.rot_vel == 0.0) // it should not be stopped but accidentally stopped
            {
                log("Specified rotating speed is zero! Robot spin aborted");
                return true;
            }
            // everything is ok, keep on assigning vel vars with params from srv request
            else
            {
                given_target_angle = degrees_to_radians(request.pano_angle);
                snap_interval = request.snap_interval;
                cmd_vel.angular.z = request.rot_vel;
            }
            log("Starting robot spin and data capturing, turning is_active as true");
            // startPanoAction();
            is_active = true;
            response.status = request.STARTED; 
        }
        return true;
    }

    // save the rotation from odom and position from ekf
    void SpinApp::saveCurrentPose(std::string& current_time_stamp){
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

    //*************
    // Logging takes str message arg only
    //*************
    void SpinApp::log(std::string log)
    {
        std_msgs::String msg;
        msg.data = log;
        ROS_INFO_STREAM(log);
    }
} // namespace robot_spinning 

 int main(int argc, char **argv)
{
    std::cout<<"Robot spin with cap data node is started." <<std::endl;
    ros::init(argc, argv, "robot_spinning_cap_data");
    robot_spinning::SpinApp spin;
    spin.log("Robot rotating for data capturing starting...");
    spin.init();
    spin.log("Rotating initialized");
    spin.spin();
    spin.log("Rotating spin done");
    // spin.trans();
    // spin.log("Rotating translation done, end");

    return 0;
}