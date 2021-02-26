//
// Created by qzj on 2021/2/26.
//

#ifndef SRC_LOOPDETECTOR_H
#define SRC_LOOPDETECTOR_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "KeyFrame.h"
#include <set>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
typedef std::pair<std::set<KeyFrame*>,int> ConsistentGroup;

class LoopDetector {

public:
    LoopDetector(ros::NodeHandle &nh);

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    void Run();

    void InitSaveImg();

    void LoadVoc(const std::string& strVocFile);

private:

    ros::NodeHandle _nh;
//    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> rgb_sub_ptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_ptr;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_ptr;

    std::vector<KeyFrame*> kfBuffer;
    std::vector<cv::Mat> rgbBuffer;
    std::vector<cv::Mat> depthBuffer;
    std::vector<double> timestampsBuf;

    ORBextractor* mpORBextractor;

    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    // Loop detector parameters
    float mnCovisibilityConsistencyTh;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;

    char key;
    std::string saveDir;
    uint32_t mLastLoopKFid;

    uint32_t imgIdx;
    ORBVocabulary* mpORBvocabulary;
    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDB;

    void UpdateConnections(int cur_idx);

    void UpdateConnections(KeyFrame *pKF, int cur_idx);

    bool DetectLoop();
};


#endif //SRC_LOOPDETECTOR_H
