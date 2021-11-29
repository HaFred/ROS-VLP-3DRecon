/**
 * @file /include/image_with_pose/robot_spinning.cpp
 *
 * @brief Robot spinning and navigation to a certain point for data capturing, which will be further used in 3D reconstruction system. The spin in the context is general, means robot motion for data capturing.
 *
 * @date Nov 24, 2021
 *
 * @author HaFred
 **/

#ifndef SPINNING_H_
#define SPINNING_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h> 		
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>		
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>		
// #include <image_transport/image_transport.h>	
#include <turtlebot3_applications_msgs/TakePanorama.h>

#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace robot_spinning
{
    /*****************************************************************************
    ** Interfaces
    *****************************************************************************/
    template<typename T>
    T degrees_to_radians(const T &degrees)
    {
        static const double degs_to_rads = M_PI / 180.0;
        return degrees * degs_to_rads;
    }

    template<typename T>
    T radians_to_degrees(const T &radians)
    {
        static const double rads_to_degs = 180.0 / M_PI;
        return radians * rads_to_degs;
    }

    template<typename T>
    T wrap_angle(const T &angle)
    {
        float wrapped;
        if ((angle <= M_PI) && (angle >= -M_PI))
        {
        wrapped = angle;
        }
        else if (angle < 0.0)
        {
        wrapped = fmodf(angle - M_PI, 2.0 * M_PI) + M_PI;
        }
        else
        {
        wrapped = fmodf(angle + M_PI, 2.0 * M_PI) - M_PI;
        }
        return wrapped;
    }

    class SpinApp
    {
        public:
            SpinApp();
            ~SpinApp();

            void init();
            void spin();
            void trans(); // the translation function 

            /**
            * Additionally sends out logging information on a ROS topic
            * @param msg logging information
            */
            void log(std::string msg);

        private:
            ros::NodeHandle nh;
            ros::NodeHandle priv_nh;
            std::map<std::string, std::string> params;

            geometry_msgs::Twist cmd_vel, zero_cmd_vel;
            double snap_interval;
            double curr_angle, last_angle, given_target_angle, ang_vel_cur;
            
            // these are placeholders for the translation part
            // double trans, last_angle, given_angle, ang_vel_cur;

            bool continuous; // f: is it a flag?
            
            image_transport::Publisher pub_image_with_pose;
            image_transport::Subscriber sub_camera;

            ros::ServiceServer srv_start_spin;

            // for publishing robot motion
            ros::Publisher pub_cmd_vel;

            // info publisher to image_with_pose node
            ros::Publisher pub_to_image_with_pose;

            // for retrieving the odometry of robot
            ros::Subscriber sub_odom;

            std::vector<cv::Mat> images_;

            /**
            * turns true, when the spin_ros action goal goes active
            */
            bool is_active;
            std_msgs::Bool data_cap_en;

            /**
            * Tells the spin_ros feedback callback to set is_active to true (starts rotating the robot)
            * This is necessary in order to capture the first picture at the start,
            * since it takes a while to get the first pciture from the webcam.
            */
            bool go_active;
            /**
            * Default spinning mode used for interaction via rostopic
            */
            int default_mode;
            /**
            * Default spinning angle used for interaction via rostopic
            */
            double default_spin_angle;
            double default_trans_speed;

            double default_snap_interval;
            double default_rotation_velocity;

            bool store_image;

            // callback used for the service, no msgs input required for data recording, but a service file is needed
            /**
            * @param request specify the details for panorama creation
            * @param response the current state of the app (started, in progress, stopped)
            * @return true, if service call was successful
            */
            bool DoRobotSpinServiceCb(turtlebot3_applications_msgs::TakePanorama::Request& request, turtlebot3_applications_msgs::TakePanorama::Response& response);

            void snap();
            void rotate();
            void translate();
            bool hasReachedAngle();
            bool hasReachedTranslation();
            void odomCb(const nav_msgs::OdometryConstPtr& msg);
            void startSpinAction();
            void cameraImageCb(const sensor_msgs::ImageConstPtr& msg);
    };
} // namespace robot_spinning

#endif /* SPINNING_H_ */
