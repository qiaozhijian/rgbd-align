//
// Created by qzj on 2021/2/23.
//
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<opencv2/core/core.hpp>

cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    if(img_msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if(cv_ptr->image.type()==16)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }else if(img_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        if(cv_ptr->image.type()==0)
        {
            return cv_ptr->image.clone();
        }
        else
        {
            std::cout << "Error type" << std::endl;
            return cv_ptr->image.clone();
        }
    }
}

void GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
//    cv::Mat im = GetImage(img_msg);
    double tImLeft = img_msg->header.stamp.toSec();
    ROS_INFO("GrabImageDepth");
}

void GrabImageRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
//    cv::Mat im = GetImage(img_msg);
    double tImLeft = img_msg->header.stamp.toSec();
    ROS_INFO("GrabImageRGB");
}

int main(int argc, char **argv) {


    ros::init(argc, argv, "rgbd_node");
    ros::NodeHandle n("~");
    ROS_INFO("rgbd_node start.");

    ros::Subscriber sub_img_left = n.subscribe("/camera/depth/image_rect", 1, &GrabImageDepth);
    ros::Subscriber sub_img_right = n.subscribe("/camera/rgb/image_rect_color", 1, &GrabImageRGB);

    ros::Rate rate(200);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}