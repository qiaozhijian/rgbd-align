//
// Created by qzj on 2021/2/26.
//

#ifndef SRC_ALIGN_H
#define SRC_ALIGN_H

#include<cv_bridge/cv_bridge.h>
#include<opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<ros/ros.h>

typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Image> sync_pol;

class Align {

    public:
    Align(ros::NodeHandle &nh);

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void Run();

    private:

    ros::NodeHandle _nh;
//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> rgb_sub_ptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_ptr;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_ptr;
};


#endif //SRC_ALIGN_H
