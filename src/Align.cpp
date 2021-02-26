//
// Created by qzj on 2021/2/26.
//
#include "../include/Align.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

using namespace std;
Align::Align(ros::NodeHandle nh):nh(nh)
{
    // ORB: /camera/rgb/image_raw  /camera/depth_registered/image_raw
    rgb_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera/rgb/image_rect_color", 100);
    depth_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera/depth/image_rect", 100);
    sync_ptr = std::make_shared<message_filters::Synchronizer <sync_pol>>(sync_pol(10), *rgb_sub_ptr, *depth_sub_ptr);
    sync_ptr->registerCallback(boost::bind(&Align::GrabRGBD, this, _1, _2));

}

void Align::Run()
{
    ros::spinOnce();
}

void Align::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat rgbImg = cv_ptrRGB->image;
    cv::Mat depthImg = cv_ptrD->image;
    double t = cv_ptrRGB->header.stamp.toSec();

    ROS_INFO("Receive images.");
}
