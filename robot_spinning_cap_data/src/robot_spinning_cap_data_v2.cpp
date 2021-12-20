 /**
 * @file 
 *
 * @brief 2.0 tries to embed the fin signal and publish to robot_trans node. Meanwhile the srv not stopped, still listening for the trans_fin singal from robot_trans node.
 *        2. For the data cap, no longer using ofstream, instead we try rosbag (done at v2.1).
 *        3. Replace all the tf with tf2
 * 
 *        2.0 is a minimum workable template for image cap and data pose saving.
 *
 * @date Dec 15, 2021
 *
 * @author HaFred
 **/

// from image with pose
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
#include <nav_msgs/Odometry.h>
#include <typeinfo>
#include "yaml-cpp/yaml.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

// for rosbag
// #include <rosbag/bag.h>

// from robot spinning
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <robot_spinning_cap_data/robot_spinning_cap_data.h>

#define FIXED_Z_VALUE 0.4
#define VERSION "2.0"

#define SAVE_PATH "/home/liphy/catkin_ws/src/robot_spinning_cap_data/cap_data/"

using namespace cv;

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
        srv_start_spin = priv_nh.advertiseService("spin_srv", &SpinApp::doRobotSpinServiceCb, this); // this: object to call srv_func on

        image_transport::ImageTransport it(nh);

        // once include header and class, do not put the declaration and definition together as in iwp_v2... It may cause the sub not working
        sub_camera = it.subscribe("usb_cam/image_raw", 1, &SpinApp::imageCb, this); // the size of the publisher queue is set to 1
        
        // robot control
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100); // queue 100, no latch

        pub_to_rbt_trans = nh.advertise<std_msgs::Bool>("enable_robot_trans", 100);

        sub_odom = nh.subscribe("odom", 100, &SpinApp::odomCb, this); // callback function ptr is passed in as the 3rd arg, tracked object is this

        // sub_speed_ctrller = nh.subscribe("second_rotate", 100, &SpinApp::secondRotateCb, this);

        cmd_vel.linear.x = 0.0f;
        cmd_vel.linear.y = 0.0f;
        cmd_vel.linear.z = 0.0f;
        cmd_vel.angular.x = 0.0f;
        cmd_vel.angular.y = 0.0f;
        cmd_vel.angular.z = 0.0f;
        zero_cmd_vel = cmd_vel;
        
        // const msg for pub_to_rbt_trans publishment
        trans_enable_sign.data = true;
        trans_not_yet_enabled_sign.data = false;

        is_active = false;
        first_spin_done = false;
        translation_done = false;
        store_image = false;
        ang_vel_cur = 0.0;
        given_target_angle = 0.0;
        curr_angle = 0.0;
        last_angle = 0.0;
        snap_cnt = 0;
        // start_time = std::to_string(ros::Time::now().toSec());

        // to make rosbag contiuously saving
        // pose_bag.open("pose_" + start_time + ".bag", rosbag::bagmode::Write);
        // pose_bag.open(std::string(SAVE_PATH) + "pose_" + std::to_string(ros::Time::now().toSec()) + ".bag", rosbag::bagmode::Write);
    }

    void SpinApp::spin()
    {
        ros::Rate loop_rate(10); // read rate at 10hz
        double start_time = 0.0;
        bool take_snapshot = false;

        while(ros::ok())
        {
            if(is_active)  // only in the srv to make it true, so when the srv is done, it won't go into this clause
            {
               // robot spinning
                ROS_INFO_STREAM_THROTTLE(1.0, "Degrees to go: " << radians_to_degrees(std::abs(given_target_angle - curr_angle))); // throttle for this output
                if ((given_target_angle - curr_angle) <= 0.0174) // check if ultimate target angle is reached (<1 degree) & finished, then cap images and do the snitching. But here for 3d_recon we just give the en signal for the last frame cap
                {
                    snap();
                    // halt the robot for spinning 180 degree
                    pub_cmd_vel.publish(zero_cmd_vel);
                    log("Finished Semicircle Data Cap");
                    curr_angle=0.0;
                    last_angle=0.0;
                    is_active = false;
                    first_spin_done = true;
                    // translation_done = false; // avoid the is_active turns true in the major else clause

                    log("Finished Semicircle Data Cap"); 
                    ROS_INFO("translation_done: %s", translation_done ? "true":"false"); 
                    ros::shutdown();

                    // if (translation_done) // second spin is also done
                    // {
                    //     ROS_INFO("DEBUG: ##################### second spin done...");
                    //     // pose_bag.close();
                    //     ros::shutdown();
                    //     ROS_INFO("SHUTDOWN AND QUIT rscd node...");
                    // }
                }
                else
                {   // not finished yet
                    pub_to_rbt_trans.publish(trans_not_yet_enabled_sign);
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
                if(translation_done){
                    ROS_INFO("DEBUG: ##################### translation done...");
                    is_active = true;
                }
            }

            if (first_spin_done) {
                // publish a topic to robot_trans node
                pub_to_rbt_trans.publish(trans_enable_sign);
            }
            // if not active, still needs to activate the callback to make the process flowing by single-threaded spin
            ros::spinOnce();
            loop_rate.sleep(); // works with rate(10)
        }
    }

    /****************************
     priviate api for the app 
    ****************************/    

    // used when spinning reaches its interval angle, ask the process to handle the callback of tflistener and it_sub photo, with the single-threaded spin and consume 1 sec
    void SpinApp::snap()
    {
        log("snap");
        ++snap_cnt;
        ROS_INFO("with the snap count: %d", snap_cnt);
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

    void SpinApp::secondRotateCb(const std_msgs::BoolConstPtr& msg)
    {
        if (msg->data)
        {
            // ROS_INFO("DEBUG: second rotate activated");
            cmd_vel.angular.z = -request_rot_vel;
            translation_done = true;
        }
        else
        {
            // ROS_INFO("DEBUG: second rotate not activated")
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

    // here if you look at the src of ima.eTransport::Sub, usb_cam, you will find that they directly utilize: it.advertiseCamera("image_raw", 1); rather than using nh.advertise(). With the publish(const sensor_msgs::ImageConstPtr& image, ...) of it.advertiseCamera, the cb of any subs (imageCb) here can take in ImageConstPtr& image type param without requiring like nh.advertise<> setting up the msg datatype
    void SpinApp::imageCb(const sensor_msgs::ImageConstPtr& msg) // these msg are setup to receive msg from the topic which is encoded by the publisher 
    {
        // ROS_INFO("DEBUG: camera image cb is activated ********");
        if (store_image)
        {
            /* note that lookupTransform here must be updated (accessed) before every snap(). Otherwise, the pose saved won't be updated (in dec20 1st commit, I put it in main() and cannot be accessed with the ok() in spin())...

            Previously we make robot spin and data cap as separated node, robot_spinning & image_with_pose, that's why back then the poses are updated. Now we combine these two nodes into one, if we do not go through lookupTransform() every time, the dated-pose issue occurs.
            
            Since for spin(), we stuck in the ros::ok() and cannot get the updated lookupTransform() in main() outside spin(). We mannually access it here...*/
            try{
                // lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, const ros::Duration timeout=ros::Duration(0.0)) const
                latestPoseFromOdom = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
                std::cout<<"Pose from Odom listener passed."<<std::endl;
                latestPoseFromEKF = tfBuffer.lookupTransform("map", "odom", ros::Time(0));
                std::cout<<"Pose from EKF listener passed."<<std::endl;
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                return;
                // continue; // coz no while here
            }

            // **** First Convert the Sensor Msg Image to OpenCV format

            cv_bridge::CvImagePtr cv_ptr;
            try{
                    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
            }
            
            std::string current_time_stamp = std::to_string((int)(ros::Time::now().toSec()));

            ROS_ASSERT( cv::imwrite( std::string(SAVE_PATH) + std::string( "image_" ) + current_time_stamp + std::string( ".png" ), cv_ptr->image ) );

            //imwrite("/home/Documents/image_with_pose.jpg", cv_ptr->image);
            std::cout<<"Image saved at "<<current_time_stamp<<std::endl;

            SpinApp::saveCurrentPose(current_time_stamp);
            store_image = false; // in the callback, everytime the data cap, reset store flag
            ROS_INFO("DEBUG: Image cb received and data saved++++++++");
        }
        else
        {
            // ROS_INFO("DEBUG: STORE IMAGE is false, the imageCb not received --------");
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
                request_rot_vel = request.rot_vel;
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
        tf2::Quaternion q(
            latestPoseFromOdom.transform.rotation.x,
            latestPoseFromOdom.transform.rotation.y,
            latestPoseFromOdom.transform.rotation.z,
            latestPoseFromOdom.transform.rotation.w
        );
        // q.normalize();
        tf2::Matrix3x3 rotate(q);
        // geometry_msgs::Quaternion msg_quat;
        // msg_quat = tf2::toMsg(q);
        // tf2::Vector3 translation(latestPoseFromEKF.transform.translation.x,
        //                     latestPoseFromEKF.transform.translation.y,
        //                     FIXED_Z_VALUE);
        // tf2::Transform transform(rotate, translation);
        // geometry_msgs::TransformStamped msg_transform;
        // msg_transform.transform = tf2::toMsg(transform);
        
        // initialize the pose matrix 
        cv::Mat pose_matrix = cv::Mat::zeros(3, 4, CV_64F);
        
        pose_matrix.at<double>(1, 1) = rotate.getRow(1).getX();
        pose_matrix.at<double>(1, 2) = rotate.getRow(1).getY();
        pose_matrix.at<double>(1, 3) = rotate.getRow(1).getZ();
        pose_matrix.at<double>(1, 4) = latestPoseFromEKF.transform.translation.x;
        
        pose_matrix.at<double>(2, 1) = rotate.getRow(2).getX();
        pose_matrix.at<double>(2, 2) = rotate.getRow(2).getY();
        pose_matrix.at<double>(2, 3) = rotate.getRow(2).getZ();
        pose_matrix.at<double>(2, 4) = latestPoseFromEKF.transform.translation.y;
        
        pose_matrix.at<double>(3, 1) = rotate.getRow(3).getX();
        pose_matrix.at<double>(3, 2) = rotate.getRow(3).getY();
        pose_matrix.at<double>(3, 3) = rotate.getRow(3).getZ();
        pose_matrix.at<double>(3, 4) = FIXED_Z_VALUE;
        
        // saving quat from odom, comment it for reliving comp burdens on rspi
        // std::ofstream quat_odom_file(std::string(SAVE_PATH) + std::string("quat_from_odom_") + current_time_stamp + std::string(".txt"), std::ios::out | std::ios::binary);
        // for (int i = 0; i < 4; i++){
        //     quat_odom_file << std::to_string(q[i]) + " ";
        // }
        // quat_odom_file << "\n";
        // quat_odom_file << std::to_string(latestPoseFromOdom.transform.translation.x) + " " << std::to_string(latestPoseFromOdom.transform.translation.y) + " ";
        
        // saving pose matrix
        std::ofstream pose_file(std::string(SAVE_PATH) + std::string("pose_") + current_time_stamp + std::string(".txt"), std::ios::out | std::ios::binary);
        for (int i = 1; i <= pose_matrix.rows; i++) {
            for (int j = 1; j <= pose_matrix.cols; j++){
                pose_file << std::to_string(pose_matrix.at<double>(i, j)) + " ";
            }
            pose_file << "\n";
        }

        // quat_odom_file.close();
        pose_file.close();

        // pose_bag.write("bag_quat", ros::Time::now(), msg_quat);
        // pose_bag.write("bag_rotmat", ros::Time::now(), msg_transform);
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
    std::cout<<"Robot spin with cap data node is started. Version: "<<VERSION<<std::endl;
    ros::init(argc, argv, "robot_spinning_cap_data");
    robot_spinning::SpinApp spin;

    // the callback for listener and it_cb is in snap()
    // tf2_listener adpatation, using the listener to subscribe tf
    tf2_ros::TransformListener tfListener(spin.tfBuffer);

    spin.init();
    spin.log("Rotating initialized");
    
    /***************************** 
     global var for tfbuffer lookupTransform
    **/
    while (ros::ok()){
        // tf listener, for src/target frame, refer to iwp_3.4
        // try-catch clause is for lookupTransform to continue, because it cannot be put inside the cb fn (refer to my ros answers upvote). Making it class member var and put outside, it works
        
        try{
            latestPoseFromOdom = spin.tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
            std::cout<<"Pose from Odom listener passed."<<std::endl;
            latestPoseFromEKF = spin.tfBuffer.lookupTransform("map", "odom", ros::Time(0));
            std::cout<<"Pose from EKF listener passed."<<std::endl;
            spin.log("Robot rotating for data capturing starting...");
            spin.spin();
            spin.log("Rotating spin done");
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue; // doing the rest other than exception, if no while just comment
        }
    }
    return 0;
}