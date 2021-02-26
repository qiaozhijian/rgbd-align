//
// Created by qzj on 2021/2/26.
//

#ifndef SRC_IMAGESAVE_H
#define SRC_IMAGESAVE_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "thread"
#include <termio.h>

using namespace std;

class ImageSave {
public:

    ImageSave(const std::string &topic);

    void GrabImage(const sensor_msgs::ImageConstPtr &msgRGB);

    void Run();

    void InitSaveImg();

    static void GetKey(char &key) {
        while (1) {
            struct termios new_settings;
            struct termios stored_settings;
            tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
            new_settings = stored_settings;           //
            new_settings.c_lflag &= (~ICANON);        //
            new_settings.c_cc[VTIME] = 0;
            tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
            new_settings.c_cc[VMIN] = 1;
            tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //
            key = getchar();
            tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
        }
    }

private:
    shared_ptr<ros::NodeHandle> nh_ptr;
    ros::Subscriber sub;

    char key;
    std::string saveDir;

    std::vector<cv::Mat> rgbBuffer;
    std::vector<double> timestampsBuf;
};


#endif //SRC_IMAGESAVE_H
