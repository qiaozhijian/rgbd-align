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
#include "LoopDetector.h"


int main(int argc, char **argv) {


    ros::init(argc, argv, "rgbd_node");
    ros::NodeHandle n("~");

    LoopDetector detector(n);
    ROS_INFO("rgbd_node start.");

    ros::Rate rate(200);
    while (ros::ok()) {
        detector.Run();
        rate.sleep();
    }
}