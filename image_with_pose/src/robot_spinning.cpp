/**
 * @file /include/image_with_pose/robot_spinning.cpp
 *
 * @brief Robot spinning and navigation to a certain point for data capturing, which will be further used in 3D reconstruction system. The spin in the context is general, means robot motion for data capturing.
 *
 * @date Nov 24, 2021
 *
 * @author HaFred
 **/

 #include <cmath>
 #include <iostream>
 #include <eigen3/Eigen/Core>
 #include <eigen3/Eigen/Geometry>
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>

 // to be deleted
//  #include <turtlebot3_panorama/panorama.h>

 #include <image_with_pose/robot_spinning.h>

 namespace robot_spinning
 {

    SpinApp::SpinApp() : nh(), priv_nh("~") // make it priv
    {}

    SpinApp::~SpinApp()
    {}

    void SpinApp::init()
    { // direct class access scope resolution operator
        
        /****************************
         public api for the app 
        ****************************/
        // because of the priv_nh, when calling this srv, need the prefix in the name. And this service deal with parameters from the rosservice call cmd
        srv_start_spin = priv_nh.advertiseService("spin_srv", &SpinApp::DoRobotSpinServiceCb, this); // object to call srv_func on

        pub_to_image_with_pose = nh.advertise<std_msgs::Bool>("pub_to_image_with_pose", 1, true); // queue 1, true for latch, no subscriber for this topic is required

        // should not it be sub to image_with_pose node? No, I would suggest this ro_spin node just publishes an enable signal to the image_with_pose node, to record data. And this ro_spin node does not subscribe to usb_cam anymore (as in the panorama node)
        // image_transport::ImageTransport it(nh);
        // sub_camera = ?

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
        data_cap_en.data = false;
        is_active = false;
        continuous = false;
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
                ROS_INFO_STREAM_THROTTLE(1.0, "Degrees to go: " << radians_to_degrees(std::abs(given_target_angle - curr_angle))); // throttle for this output
                if ((given_target_angle - curr_angle) <= 0.0174) // check if ultimate target angle is reached (<1 degree) & finished, then cap images and do the snitching. But here for 3d_recon we just give the en signal for the last frame cap
                {
                    snap();
                    // halt the robot for spinning 180 degree
                    pub_cmd_vel.publish(zero_cmd_vel);
                    data_cap_en.data = true;
                    // pub_to_image_with_pose.publish(data_cap_en.data);
                    ROS_INFO("F: Publishing the data capturing enabling signal to image_with_pose node");
                    log("Finished Semicircle Data Cap");

                    ROS_INFO("Angle: %f", curr_angle); 
                    ROS_INFO("Last Angle: %f", last_angle); 
                    curr_angle=0.0;
                    last_angle=0.0;
                    is_active = false;
                }
                else
                { // not finished yet
                    if(continuous) // if the request mode turns into continuous, then record the duration in snap_interval
                    {
                        log("Continuous for rotating");
                        rotate();
                        ros::Duration(snap_interval).sleep();
                        snap();
                        ROS_INFO("Angle Continuous: %f", curr_angle);
                        ROS_INFO("Target Angle Given: %f", given_target_angle);
                    }
                    else // if not in the continuous request mode of the service
                    {
                        if (hasReachedAngle())
                        {
                            pub_cmd_vel.publish(zero_cmd_vel); // stop before taking a snapshot when angle reached 
                            data_cap_en.data = true;
                            // pub_to_image_with_pose.publish(data_cap_en.data);
                            ROS_INFO("F: Publishing the data capturing enabling signal to image_with_pose node");
                            take_snapshot = true;
                        }
                        if (take_snapshot) // the take_snapshot signal is true, stop and data cap
                        {
                            if (std::abs(ang_vel_cur) <= 0.01) // wait until robot has stopped
                            {
                                snap(); // ask the process to handle the callback with the spinning once and consume 1 sec
                                data_cap_en.data = true;
                                // pub_image_with_pose.publish(data_cap_en.data);
                                ROS_INFO("F: Publishing the data capturing enabling signal to image_with_pose node");
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
            }
            // if not active, still needs to activate the callback to make the process flowing by single-threaded spin
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    /****************************
     priviate api for the app 
    ****************************/    

    // ask the process to handle the callback with the single-threaded spin and consume 1 sec, so as to save an image
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

    // the msgs md5sum security parity pairs are hard to deal with, thus we borrow the request and response msgs from TakePanorama here
    bool SpinApp::DoRobotSpinServiceCb(turtlebot3_applications_msgs::TakePanorama::Request& request, turtlebot3_applications_msgs::TakePanorama::Response& response)
    {
        if (is_active && (request.mode == request.STOP))
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

            log("Starting robot spin and data capturing");
            // startPanoAction();
            is_active = true;
            response.status = request.STARTED; 
        }
        return true;
    }

    // this is for image_transport sub cb, used for panorama but not here
    // void SpinApp::cameraImageCb(const sensor_msgs::ImageConstPtr& msg)
    // {

    // }

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
    ros::init(argc, argv, "robot_motion_datacap");
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
