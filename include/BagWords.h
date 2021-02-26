//
// Created by qzj on 2021/2/26.
//

#ifndef SRC_BAGWORDS_H
#define SRC_BAGWORDS_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "Thirdparty/DBow3/src/DBoW3.h"
#include <memory>

class BagWords {

public:
    BagWords(std::string voc_file);

    cv::Mat DetectORB(cv::Mat &image);

    void addImage(cv::Mat &image, uint32_t &idx);

private:

    std::vector<cv::Mat> pdescs;
    cv::Ptr<cv::Feature2D> detector;

    std::shared_ptr<DBoW3::Vocabulary> vocab_ptr;
    std::shared_ptr<DBoW3::Database> db_ptr;

    uint32_t imgIdx;
    std::vector<cv::Mat> imageDB;
    std::vector<cv::Mat> descDB;
    std::vector<uint32_t> idxDB;

};


#endif //SRC_BAGWORDS_H
