//
// Created by qzj on 2021/2/26.
//

#include "../include/Align.h"


Align::Align(ros::NodeHandle nh):nh(nh)
{

    sub_img_left = nh.subscribe("/camera/depth/image_rect", 1, &Align::GrabImageDepth, this);
    sub_img_right = nh.subscribe("/camera/rgb/image_rect_color", 1, &Align::GrabImageRGB, this);

}

void Align::Run()
{

}

cv::Mat Align::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
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

void Align::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
//    cv::Mat im = GetImage(img_msg);
    double tImLeft = img_msg->header.stamp.toSec();
    ROS_INFO("GrabImageDepth");
}

void Align::GrabImageRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
//    cv::Mat im = GetImage(img_msg);
    double tImLeft = img_msg->header.stamp.toSec();
    ROS_INFO("GrabImageRGB");
}
