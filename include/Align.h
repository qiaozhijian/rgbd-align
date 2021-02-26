//
// Created by qzj on 2021/2/26.
//

#ifndef SRC_ALIGN_H
#define SRC_ALIGN_H

#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>

#include<ros/ros.h>

class Align {

    public:
    Align(ros::NodeHandle nh);

    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg);

    void GrabImageRGB(const sensor_msgs::ImageConstPtr &img_msg);

    void Run();

    private:

    ros::NodeHandle nh;
    ros::Subscriber sub_img_left;
    ros::Subscriber sub_img_right;
};


#endif //SRC_ALIGN_H
