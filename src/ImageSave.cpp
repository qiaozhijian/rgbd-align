//
// Created by qzj on 2021/2/26.
//

#include "ImageSave.h"
#include <opencv2/highgui/highgui.hpp>
#include "global_defination/global_defination.h"
#include "utility.h"

ImageSave::ImageSave(const std::string &topic) {
    nh_ptr = make_shared<ros::NodeHandle>("~");
    sub = nh_ptr->subscribe<sensor_msgs::Image>(topic, 1, &ImageSave::GrabImage, this);

    InitSaveImg();

    rgbBuffer.clear();
    rgbBuffer.reserve(1000);
    timestampsBuf.clear();
    timestampsBuf.reserve(1000);
}

void ImageSave::GrabImage(const sensor_msgs::ImageConstPtr &msgRGB) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rgbBuffer.push_back(cv_ptrRGB->image);
    timestampsBuf.push_back(cv_ptrRGB->header.stamp.toSec());
//    ROS_INFO("Receive images.");
}


void ImageSave::InitSaveImg() {
    time_t now_time = time(NULL);
    tm *T_tm = localtime(&now_time);
    //转换为年月日星期时分秒结果，如图：
    string timeDetail = asctime(T_tm);
    timeDetail.pop_back();

    saveDir = cmake_template::WORK_SPACE_PATH + "/dataset/" + timeDetail + "-img/";
    createDirectory(saveDir);

    thread getKey(ImageSave::GetKey, std::ref(key));
    getKey.detach();
}

void ImageSave::Run() {
    static uint32_t spaceCnt = 0;
    //32对应空格
    while (!rgbBuffer.empty()) {
        cv::Mat rgb = rgbBuffer.back();
        double timestamp = timestampsBuf.back();

        if (key == 32) {
            key = 0;
            string image_path = saveDir + to_string(uint64_t(timestamp * 1e9)) + ".jpg";
            cv::imwrite(image_path, rgb);
//                        SaveCameraTime(dir + "cameraStamps.txt", now*1e9);
            spaceCnt++;
            ROS_INFO("save space img %d", spaceCnt);
        }

        rgbBuffer.pop_back();
        timestampsBuf.pop_back();
    }


}