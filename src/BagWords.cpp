//
// Created by qzj on 2021/2/26.
//

#include "BagWords.h"
#include "KeyFrameDatabase.h"
using namespace std;
using namespace cv;

BagWords::BagWords(std::string voc_file) {
    vocab_ptr = std::make_shared<DBoW3::Vocabulary>(voc_file);

    if (vocab_ptr->empty()) {
        cerr << "Vocabulary does not exist." << endl;
    }
    db_ptr = std::make_shared<DBoW3::Database>(*vocab_ptr, false, 0);
    imageDB.clear();
    descDB.clear();
    idxDB.clear();
}


Mat BagWords::DetectORB(cv::Mat &image) {
    Mat pdesc;
    vector<KeyPoint> keypoints;
    detector->detectAndCompute(image, Mat(), keypoints, pdesc);
}

void BagWords::addImage(cv::Mat &image, uint32_t &idx) {
    idxDB.push_back(idx);
    imageDB.push_back(image);
    descDB.push_back(DetectORB(image));

    uint32_t size = imageDB.size();
    uint32_t ignore = 20;
    if (size < ignore)
        return;

    uint32_t candi_num = 3;
    db_ptr->add(descDB[size - ignore]);

    DBoW3::QueryResults ret;
    db_ptr->query(descDB.back(), ret, candi_num);      // max result=4
    for (int j = 1; j < ret.size(); j++) {
        idxDB[ret[j].Id];
    }
}

