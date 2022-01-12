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
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <lightfly.hpp>
#include <liphy_vlp/liphylight.h>
#include <typeinfo>
#include "yaml-cpp/yaml.h"


#define USE_FIXED_OUTPUT_RATE 1
#define OUTPUT_RATE 0.5 // Hz


#define PI 3.14159265
#define USE_ODOMETERY_FILETERD 0
#define LIGHT_DIAMETER 17.38f


using namespace cv;
using namespace std;

geometry_msgs::Pose pose_odom;
geometry_msgs::Pose2D pose2d;

image_transport::Publisher pub;
ros::Publisher info_pub;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {

    pose_odom.orientation.x = msg->pose.pose.orientation.x;
    pose_odom.orientation.y = msg->pose.pose.orientation.y;
    pose_odom.orientation.z = msg->pose.pose.orientation.z;
    pose_odom.orientation.w = msg->pose.pose.orientation.w;
    
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = -1*(yaw * 180/PI + 180);

    cout<<"PoseReceivedFromOdom:"<<endl;    
    cout<<"Pose Orietattion.x: "<<pose_odom.orientation.x<<endl;
    cout<<"Pose Orietattion.y: "<<pose_odom.orientation.y<<endl;
    cout<<"Pose Orietattion.z: "<<pose_odom.orientation.z<<endl;
    cout<<"Pose Orietattion.w: "<<pose_odom.orientation.w<<endl;  
    cout<<"*******************************************"<<endl;
    cout<<"odom theta at this frame is: "<<pose2d.theta<<endl;
    cout<<"pose2d x at this frame is: "<<pose2d.x<<endl;
    cout<<"pose2d y at this frame is: "<<pose2d.y<<endl;
    cout<<"roll at this frame is: "<<roll<<endl;
    cout<<"pitch at this frame is: "<<pitch<<endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // no need to scale the dia, dia is used to calculate the x, y in lightFlyManager
    // float light_diameter = LIGHT_DIAMETER / (LIGHT_HEIGHT - TROLLEY_HEIGHT - MV_CAM_HEIGHT) * (LIGHT_HEIGHT - MV_CAM_HEIGHT);

    float light_diameter = LIGHT_DIAMETER;

    bool sameKeypoint = false;

    //std::cout<<"Image callback received"<<std::endl;

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


    // ********************** LightFly SDK Test Script *************************//
    // ===========================================================================
    LightFly lightFlyManager;

    LightFly::Parameters params;
    
    //cout<<"\nVersion Number is:"<<lightFlyManager.getVersionNumber()<<endl;

    params.fx = 1281.47f;
    params.fy = 1281.47f;
    params.cx = 654.0f;
    params.cy = 426.0f;

    params.isFront = true;
    params.showKeyPoints = true;

    vector<LEDKeyPoint> ledKeyPoints {};

    //std::cout<<"Image Converted Successfully"<<std::endl;
    
    lightFlyManager.processImage(cv_ptr->image,params,ledKeyPoints,0,0,pose2d.theta);
    YAML::Node lights = YAML::LoadFile("/home/liphy/catkin_ws/src/liphy_vlp/lights/lights.yaml");
    liphy_vlp::liphylight ll;
    for (int i = 0; i < ledKeyPoints.size(); i++){
        ll.keypoint = ledKeyPoints.at(i).id;
        ll.corners = ledKeyPoints.at(i).corners;
        ll.light_x = ledKeyPoints.at(i).y;
        ll.light_y = ledKeyPoints.at(i).x;
        ll.light_z = ledKeyPoints.at(i).z;
        ll.theta = pose2d.theta;

    }

    sameKeypoint = false;
    for(int i = 1; i <= lights["Lights"]["numLights"].as<int>() && sameKeypoint == false; i++) {

        if(ll.keypoint == lights["Lights"]["Light " + to_string(i)]["keypoint"].as<string>()) {

            sameKeypoint = true;
            light_diameter = lights["Lights"]["Light " + to_string(i)]["diameter"].as<float>();

        } else {
            sameKeypoint = false;
        }
    }

    ll.light_x = ll.light_x*light_diameter/100;
    ll.light_y = ll.light_y*(-light_diameter/100);
    ll.light_z = ll.light_z*(light_diameter/100);

        //*LIGHT_DIAMETER)/100
        //*(-LIGHT_DIAMETER))/100
        //*LIGHT_DIAMETER)/100
        // for (int i = 0; i < ledKeyPoints.size(); i++){
        //     cout<<"\nKeyPointFound:"<<ledKeyPoints.at(i).id<<endl;
        //     cout<<"\nNo of corners are:"<<ledKeyPoints.at(i).corners<<endl;
        //     cout<<"\n( X, Y, Z): "<<ledKeyPoints.at(i).y*LIGHT_DIAMETER<<", "<<ledKeyPoints.at(i).x*(-LIGHT_DIAMETER)<<", "<<ledKeyPoints.at(i).z*LIGHT_DIAMETER<<endl;

        //     cout<<"\n(Center X, Center Y, size): "<<ledKeyPoints.at(i).centerX<<", "<<ledKeyPoints.at(i).centerY<<", "<<ledKeyPoints.at(i).size<<endl;
        //     cout<<"\n current theta is: "<<pose2d.theta<<endl;
        // }
        // for (int j = 0; j<ledKeyPoints.at(i).polygonCornerVec.size(); j++){
        //     cout<<"\nCorner X:"<<ledKeyPoints.at(i).polygonCornerVec.at(j).x<<" Y:"<<ledKeyPoints.at(i).polygonCornerVec.at(j).y<<endl;
        
        // }

   
            
    //std::cout<<"LiPHY Process Image Completed"<<std::endl;


    // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", srcImage).toImageMsg();
    // pub.publish(*msg);
    //
    pub.publish(cv_ptr->toImageMsg());

    if (ll.keypoint != "" && sameKeypoint){

        info_pub.publish(ll);

    }
    
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}

int main(int argc, char** argv)
{
    std::cout<<"LiPHY VLP node is started"<<std::endl;

    ros::init(argc, argv, "liphy_vlp");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("mvcam/image", 1, imageCallback);
    pub = it.advertise("liphy_vlp/processed_image", 1);

    info_pub = nh.advertise<liphy_vlp::liphylight>("liphy_vlp_info", 1);
    std::cout<<"Liphly vlp node, publish liphy_vlp_info topic"<<std::endl;    

    #if (USE_ODOMETERY_FILETERD)
        ros::Subscriber sub_odom_filtered_ = nh.subscribe("odometry/filtered", 1, odometryCallback_);
        std::cout<<"Using Odometery Filtered: True"<<std::endl;
    #else
        ros::Subscriber sub_odom_ = nh.subscribe("odom", 1, odometryCallback_);
        std::cout<<"Using Odometery Filtered: False"<<std::endl;
    #endif
    
    
    #if (USE_FIXED_OUTPUT_RATE)

    ros::Rate r(OUTPUT_RATE);
    
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    #else
    
    ros::spin();
    
    #endif

    


    return 0;
}


