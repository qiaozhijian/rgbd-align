//
// Created by qzj on 2021/2/26.
//
#include "../include/LoopDetector.h"
#include<chrono>

using namespace std;

LoopDetector::LoopDetector(ros::NodeHandle &nh) : _nh(nh),imgIdx(0),mLastLoopKFid(0) {
    // ORB: /camera/rgb/image_raw  /camera/depth_registered/image_raw
    rgb_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(_nh, "/camera/rgb/image_rect_color",
                                                                                    100);
    depth_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(_nh, "/camera/depth/image_rect",
                                                                                      100);
    sync_ptr = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *rgb_sub_ptr, *depth_sub_ptr);
    sync_ptr->registerCallback(boost::bind(&LoopDetector::GrabRGBD, this, _1, _2));

    string voc_file = "";
    nh.param<std::string>("VOC", voc_file, "");
    ROS_INFO("load %s", voc_file.c_str());
    LoadVoc(voc_file);
    mpKeyFrameDB = new KeyFrameDatabase(*mpORBvocabulary);
    mnCovisibilityConsistencyTh = 3;
    mpORBextractor = new ORBextractor(400,1.2,5,20,7);


//    imgIdx = 0;
    kfBuffer.clear();
    kfBuffer.reserve(1000);
    rgbBuffer.clear();
    rgbBuffer.reserve(1000);
    depthBuffer.clear();
    depthBuffer.reserve(1000);
    timestampsBuf.clear();
    timestampsBuf.reserve(1000);
}

void LoopDetector::LoadVoc(const std::string& strVocFile)
{
#if defined USE_FBOW
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    cout << "open at: " << strVocFile << endl;
    mpORBvocabulary = new ORBVocabulary();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mpORBvocabulary->readFromFile(strVocFile);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t_loadVoc = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    cout << "Vocabulary loaded! Spend "<<t_loadVoc<<" seconds. K:"<<mpORBvocabulary->getK() << " L:"<<mpORBvocabulary->getL() << endl;
    cout<<"Vocabulary. leafSize: "<<mpORBvocabulary->leafSize()<<" blockSize: "<<mpORBvocabulary->blockSize()
        <<" DescSize: "<<mpORBvocabulary->getDescSize()<<endl << endl;
#elif defined USE_DBOW3
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mpORBvocabulary = new ORBVocabulary();
    mpORBvocabulary->load(strVocFile);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t_loadVoc = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if(mpORBvocabulary->empty())
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", t_loadVoc);
#elif defined USE_DBOW2
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mpORBvocabulary = new ORBVocabulary();
    // bool bVocLoad = mpORBvocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpORBvocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpORBvocabulary->loadFromBinaryFile(strVocFile);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t_loadVoc = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    printf("Vocabulary loaded in %.2fs\n", t_loadVoc);
#endif
}

void LoopDetector::Run() {
    while (!rgbBuffer.empty()) {
        cv::Mat rgb = rgbBuffer.back();
        cv::Mat depth = depthBuffer.back();
        double timestamp = timestampsBuf.back();

//        计算词带向量
        KeyFrame* pKF = new KeyFrame(mpKeyFrameDB, rgb, mpORBvocabulary, mpORBextractor);
        kfBuffer.push_back(pKF);
        UpdateConnections(pKF, imgIdx);

        if(DetectLoop())
        {
            mvpEnoughConsistentCandidates;
            ROS_INFO("DetectLoop!");
        }

        imgIdx++;

        rgbBuffer.pop_back();
        depthBuffer.pop_back();
        timestampsBuf.pop_back();
    }
    ros::spinOnce();
}

bool LoopDetector::DetectLoop() {

    mpCurrentKF = kfBuffer.back();

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
//        mpCurrentKF->SetErase();
        return false;
    }
    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const BowVec &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
//        if(pKF->isBad())
//            continue;
        const BowVec &BowVec = pKF->mBowVec;

        float score = mpORBvocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
//        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
//        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }
}

void LoopDetector::UpdateConnections(KeyFrame* pKF, int cur_idx)
{
    if(cur_idx < 3)
        return;
    for(int i=0;i<3;i++)
        pKF->AddConnection(kfBuffer[cur_idx-i], 3-i);
}

void LoopDetector::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
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

    rgbBuffer.push_back(cv_ptrRGB->image);
    depthBuffer.push_back(cv_ptrD->image);
    timestampsBuf.push_back(cv_ptrRGB->header.stamp.toSec());
//    ROS_INFO("Receive images.");
}
