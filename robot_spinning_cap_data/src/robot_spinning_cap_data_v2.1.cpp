 /**
 * @file 
 *
 * @brief 2.1 tries to embed the fin signal and publish to robot_trans node. Meanwhile the srv not stopped, still listening for the trans_fin singal from robot_trans node.
 *          The robot spins at postion A, then translate to postion B and spin again. Take data during the spin.
 * 
 * @date Dec 20, 2021
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
#include "geometry_msgs/Pose2D.h"
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <typeinfo>
#include "yaml-cpp/yaml.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

// for rosbag and data saving
#include <rosbag/bag.h>
#include <sys/types.h>
#include <sys/stat.h> // https://www.gnu.org/software/libc/manual/html_node/Permission-Bits.html
#include <errno.h>
#include <string.h>
#include <boost/filesystem.hpp>

// from robot spinning
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <robot_spinning_cap_data/robot_spinning_cap_data.h>

#define TURTLE_Z_VALUE 0.435 // measured turtlebot3 web camera centroid height 
#define TROLLEY_HEIGHT 0.77
#define MV_CAM_HEIGHT 0.165
#define LIGHT_HEIGHT 2.7
#define PI 3.14159265
#define VERSION "2.1"

#define SAVE_PATH "/home/liphy/catkin_ws/src/robot_spinning_cap_data/cap_data"

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
        // before rect, it's usb_cam/image_raw
        sub_camera = it.subscribe("usb_cam/image_rect_color", 1, &SpinApp::imageCb, this);
        // sub_camera = it.subscribe("usb_cam/image_raw", 1, &SpinApp::imageCb, this);  
        // the size of the publisher queue is set to 1
        
        // robot control
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100); // queue 100, no latch

        pub_to_rbt_trans = nh.advertise<std_msgs::Bool>("enable_robot_trans", 100);

        sub_odom = nh.subscribe("odom", 100, &SpinApp::odomCb, this); // callback function ptr is passed in as the 3rd arg, tracked object is this

        sub_speed_ctrller = nh.subscribe("second_rotate", 100, &SpinApp::secondRotateCb, this);

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
        store_image = true;
        ang_vel_cur = 0.0;
        given_target_angle = 0.0;
        curr_angle = 0.0;
        last_angle = 0.0;
        snap_cnt = 0;

        // to make rosbag contiuously saving
        init_time_path = std::string(SAVE_PATH) + "/" + std::to_string(int(ros::Time::now().toSec()));

        // do the os.mkdir as in python fashion
        // struct stat info;
        // if (stat(init_time_path, &info) != 0)
        //     printf( "cannot access %s\n", init_time_path );
        // else if (info.st_mode & S_IFDIR)
        //     printf("the data path already exists, do nothing")
        // else
        // mkdir(init_time_path);
        // printf("make dir new init time path done");

        // only works for cpp17
        // try{
        //     if(fs::create_directory(init_time_path))
        //         std::cout << "create the init time path done\n";
        //     else
        //         std::cerr << "failed to create the dir\n";
        // } catch (const std::exception& e){
        //     std::cerr << e.what << "\n";
        // }

        // if (mkdir(init_time_path, S_IRWXU | S_IRWXG | S_IRWXO) == -1) { // if mkdir fails
        //     printf("Error: %s\n", strerror(errno));
        // }

        boost::filesystem::path dir(init_time_path + "/color");
        // if (boost::filesystem::create_directory(dir)){
        //     std::cerr << "Directory Created" << init_time_path << std::endl;
        // }
        if (boost::filesystem::create_directories(dir)){ // create_dirs goes to those path even not exist
            std::cerr << "Directory Created" << init_time_path << std::endl;
        }

        pose_bag.open(init_time_path + "/pose.bag", rosbag::bagmode::Write);
    }

    void SpinApp::spin()
    {
        ros::Rate loop_rate(10); // read rate at 10hz
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

                    is_active = false;
                    first_spin_done = true;
                    log("Finished semicircle data cap");

                    // log("Finished Semicircle Data Cap"); 
                    // ROS_INFO("translation_done: %s", translation_done ? "true":"false"); 
                    // ros::shutdown();

                    if (translation_done) // second spin is also done
                    {
                        // sleep for .5 sec to enactivate the last snap
                        ros::Duration(0.5).sleep();
                        ROS_INFO("DEBUG: ##################### second spin done...");
                        pose_bag.close();
                        ROS_INFO("SHUTDOWN AND QUIT rscd node...");
                        ros::shutdown();
                    }
                    else
                    {
                        ROS_INFO("DEBUG: ##################### Start the translation");
                    }
                }
                else
                {   // not finished yet
                    if (!translation_done)
                    {
                        pub_to_rbt_trans.publish(trans_not_yet_enabled_sign);
                        msg_translate = trans_not_yet_enabled_sign;
                    }
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
                    // reset
                    curr_angle=0.0;
                    last_angle=0.0;

                    // now the translation is at position B, thus msg_translate turns into 1
                    msg_translate = trans_enable_sign;
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

            // keep it as inverse clockwise comparing to 1st rotate, coz it helps determine/validate msg_translate
            cmd_vel.angular.z = - request_rot_vel;
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

    // here if you look at the src of imageTransport::Sub, usb_cam, you will find that they directly utilize: it.advertiseCamera("image_raw", 1); rather than using nh.advertise(). With the publish(const sensor_msgs::ImageConstPtr& image, ...) of it.advertiseCamera, the cb of any subs (imageCb) here can take in ImageConstPtr& image type param without requiring like nh.advertise<> setting up the msg datatype
    void SpinApp::imageCb(const sensor_msgs::ImageConstPtr& msg) // these msg are setup to receive msg from the topic which is encoded by the publisher 
    {
        // ROS_INFO("DEBUG: camera image cb is activated ********");
        if (store_image)
        {
            // **** First Convert the Sensor Msg Image to OpenCV format

            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            /* note that lookupTransform here must be updated (accessed) before every snap. Otherwise, the pose saved won't be updated (in dec20 1st commit, I put it in main() and cannot be accessed with the ok() in spin())...

            Previously we make robot spin and data cap as separated node, robot_spinning & image_with_pose, that's why back then the poses are updated. Now we combine these two nodes into one, if we do not go through lookupTransform() every time, the dated-pose issue occurs.
            
            Since for spin(), we stuck in the ros::ok() and cannot get the updated lookupTransform() in main() outside spin(). We mannually access it here...*/
            try{
                // lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, const ros::Duration timeout=ros::Duration(0.0)) const
                latestPoseFromOdom = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
                // std::cout<<"Pose from Odom listener passed."<<std::endl;
                latestPoseFromEKF = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
                // std::cout<<"Pose from EKF listener passed."<<std::endl;
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                return;
                // continue; // coz no while here
            }

            ros::Time current_time;
            current_time = ros::Time::now();

            std::string current_time_int_stamp = std::to_string((int)(current_time.toSec()));

            ROS_ASSERT( cv::imwrite(init_time_path + "/color/" + current_time_int_stamp + std::string( ".jpg" ), cv_ptr->image ));

            //imwrite("/home/Documents/image_with_pose.jpg", cv_ptr->image);
            std::cout<<"Image saved at "<<current_time_int_stamp<<std::endl;

            SpinApp::saveCurrentPose(current_time);
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
            is_active = true;
            response.status = request.STARTED; 
        }
        return true;
    }

    // save the rotation from odom and position from ekf
    void SpinApp::saveCurrentPose(ros::Time& current_time){
        // ROS_INFO("Overall saving pose");
        
        // saving the rotation matrix in the transform matrix
        tf2::Quaternion q(
            latestPoseFromOdom.transform.rotation.x,
            latestPoseFromOdom.transform.rotation.y,
            latestPoseFromOdom.transform.rotation.z,
            latestPoseFromOdom.transform.rotation.w
        );
        // q.normalize();
        tf2::Matrix3x3 rotate(q);
        double roll, pitch, yaw;
        rotate.getRPY(roll, pitch, yaw);
        geometry_msgs::Pose2D msg_pose2d;
        msg_pose2d.theta = -1*(yaw * 180/PI + 180);
        msg_pose2d.x = latestPoseFromEKF.transform.translation.x;
        msg_pose2d.y = latestPoseFromEKF.transform.translation.y;

        geometry_msgs::Quaternion msg_quat;
        msg_quat = tf2::toMsg(q);
        tf2::Vector3 translation(latestPoseFromEKF.transform.translation.x,
                            latestPoseFromEKF.transform.translation.y,
                            TURTLE_Z_VALUE);
        tf2::Transform transform(rotate, translation);

        geometry_msgs::TransformStamped msg_transform;
        msg_transform.transform = tf2::toMsg(transform);

        pose_bag.write("bag_pose2d", current_time, msg_pose2d);
        pose_bag.write("bag_quat", current_time, msg_quat);
        pose_bag.write("bag_rotmat", current_time, msg_transform);
        pose_bag.write("bag_tran_bool", current_time, msg_translate);

        std::cout<<"saving pose2d x at this frame as: "<<msg_pose2d.x<<std::endl;
        std::cout<<"saving pose2d y at this frame as: "<<msg_pose2d.y<<std::endl;
        std::cout<<"saving theta at this frame as: "<<msg_pose2d.theta<<std::endl;
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
        try{
            // these lookupTransform here is for the init frame captured before spinning
            latestPoseFromOdom = spin.tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
            std::cout<<"Pose from Odom listener passed."<<std::endl;
            latestPoseFromEKF = spin.tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            std::cout<<"Pose from EKF listener passed."<<std::endl;
            spin.log("Robot rotating for data capturing starting...");
            spin.log("Spinnning function activated");
            spin.spin();
            spin.log("Rotating spin done");
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            spin.log("The slovlp is not activated yet, the very first few frames pose coming from slovlp is not correct, plz restart the program...");
            ros::Duration(1.0).sleep();
            continue; // doing the rest other than exception, if no while just comment
        }
    }
    return 0;
}