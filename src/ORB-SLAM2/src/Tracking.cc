/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Converter.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "Map.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "Tracking.h"

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, fbow::Vocabulary* pFbowVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, bool bReuseMap):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false),
    mpKeyFrameDB(pKFDB), mpFBOWVocabulary(pFbowVoc), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    mpOptimizer = new Optimizer(strSettingPath);
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = fSettings["Tracking.minFrames"];
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    int _nPatchSize = fSettings["ORBextractor.patchSize"];
    nPatchSize = (_nPatchSize == 0) ? 31 : _nPatchSize;
    int _nHalfPatchSize = fSettings["ORBextractor.halfPatchSize"];
    nHalfPatchSize = (_nHalfPatchSize == 0) ? 15 : _nHalfPatchSize;
    int _nEdgeThreshold = fSettings["ORBextractor.edgeThreshold"];
    nEdgeThreshold = (_nEdgeThreshold == 0) ? 19 : _nEdgeThreshold;

    leftExtractorCPU = fSettings["ORBextractor.leftCPU"];
    rightExtractorCPU = fSettings["ORBextractor.rightCPU"];
    if (leftExtractorCPU == rightExtractorCPU && leftExtractorCPU != -1) {
      leftExtractorCPU = 0;
      rightExtractorCPU = 1;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, nPatchSize, nHalfPatchSize, nEdgeThreshold);

    if (sensor == System::STEREO) {
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, nPatchSize, nHalfPatchSize, nEdgeThreshold);

        // Initialize semaphores
        if (sem_init(&leftSem1, 0, 0) || sem_init(&leftSem2, 0, 0) ||
            sem_init(&rightSem1, 0, 0) || sem_init(&rightSem2, 0, 0)) {
            throw std::runtime_error("Tracking::Tracking: Could not initialize feature extractors semaphores");
        }

        stopping = false;

        // Create left feature extractor thread
        leftExtractorThread = new std::thread(
            &Tracking::ExtractorThread,
            this,
            mpORBextractorLeft,
            &leftImage,
            &leftKeyPoints,
            &leftDescriptors,
            &leftSem2,
            &leftSem1);
        if (leftExtractorCPU != -1) {
            cpu_set_t left_extractor_set;
            CPU_ZERO(&left_extractor_set);
            CPU_SET(leftExtractorCPU, &left_extractor_set);
            if (pthread_setaffinity_np(leftExtractorThread->native_handle(), sizeof(cpu_set_t), &left_extractor_set)) {
                char err_msg_buf[100] = {};
                char * err_msg = strerror_r(errno, err_msg_buf, 100);
                throw std::runtime_error(
                    "Tracking::Tracking: Failed to configure left feature extractor thread: " +
                    std::string(err_msg));
            }
        }
        cout << "Left feature extractor thread created" << endl;

        // Create right feature extractor thread
        rightExtractorThread = new std::thread(
            &Tracking::ExtractorThread,
            this,
            mpORBextractorRight,
            &rightImage,
            &rightKeyPoints,
            &rightDescriptors,
            &rightSem2,
            &rightSem1);
        if (rightExtractorCPU != -1) {
            cpu_set_t right_extractor_set;
            CPU_ZERO(&right_extractor_set);
            CPU_SET(rightExtractorCPU, &right_extractor_set);
            if (pthread_setaffinity_np(rightExtractorThread->native_handle(), sizeof(cpu_set_t), &right_extractor_set)) {
                char err_msg_buf[100] = {};
                char * err_msg = strerror_r(errno, err_msg_buf, 100);
                throw std::runtime_error(
                    "Tracking::Tracking: Failed to configure right feature extractor thread: " +
                    std::string(err_msg));
            }
        }
        cout << "Right feature extractor thread created" << endl;
    } else {
        mpORBextractorRight = nullptr;
    }

    if (sensor == System::MONOCULAR) {
        mpIniORBextractor = new ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST, nPatchSize, nHalfPatchSize, nEdgeThreshold);
    } else {
        mpIniORBextractor = nullptr;
    }

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    cout << "- Patch size: " << nPatchSize << endl;
    cout << "- Half patch size: " << nHalfPatchSize << endl;
    cout << "- Edge Threshold: " << nEdgeThreshold << endl;
    cout << "- LeftExtractorCPU: " << leftExtractorCPU << endl;
    cout << "- RightExtractorCPU: " << rightExtractorCPU << endl;

    if (sensor == System::STEREO || sensor == System::RGBD)
    {
        mThDepth = mbf * float(fSettings["ThDepth"]) / fx;
        cout << endl << "Depth Threshold (close/far Points): " << mThDepth << endl;
    }

    if (sensor == System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }
    if (bReuseMap)
        mState = LOST;

    float _mReferenceKeyframeNnRatioOrbMatcher = fSettings["Tracking.referenceKeyframeNnRatioOrbMatcher"];
    mReferenceKeyframeNnRatioOrbMatcher = (_mReferenceKeyframeNnRatioOrbMatcher == 0.0) ? 0.7 : _mReferenceKeyframeNnRatioOrbMatcher;
    float _mMotionModelNnRatioOrbMatcher = fSettings["Tracking.motionModelNnRatioOrbMatcher"];
    mMotionModelNnRatioOrbMatcher = (_mMotionModelNnRatioOrbMatcher == 0.0) ? 0.9 : _mMotionModelNnRatioOrbMatcher;
    float _mCosineDelta = fSettings["Tracking.cosineDelta"];
    mCosineDelta = (_mCosineDelta == 0.0) ? 0.5 : _mCosineDelta;
    float _mSearchLocalPointsNnRatioOrbMatcher = fSettings["Tracking.searchLocalPointsNnRatioOrbMatcher"];
    mSearchLocalPointsNnRatioOrbMatcher = (_mSearchLocalPointsNnRatioOrbMatcher == 0.0) ? 0.8 : _mSearchLocalPointsNnRatioOrbMatcher;
    float _mRelocalizationNnRatioOrbMatcher = fSettings["Tracking.relocalizationNnRatioOrbMatcher"];
    mRelocalizationNnRatioOrbMatcher = (_mRelocalizationNnRatioOrbMatcher == 0.0) ? 0.75 : _mRelocalizationNnRatioOrbMatcher;
    float _mPnpSolverRansacProbability = fSettings["Tracking.pnpSolverRansacProbability"];
    mPnpSolverRansacProbability = (_mPnpSolverRansacProbability == 0.0) ? 0.99 : _mPnpSolverRansacProbability;
    float _mPnpSolverRansacEpsilon = fSettings["Tracking.pnpSolverRansacEpsilon"];
    mPnpSolverRansacEpsilon = (_mPnpSolverRansacEpsilon == 0.0) ? 0.5 : _mPnpSolverRansacEpsilon;
    float _mPnpSolverRansacTh2 = fSettings["Tracking.pnpSolverRansacTh2"];
    mPnpSolverRansacTh2 = (_mPnpSolverRansacTh2 == 0.0) ? 5.991 : _mPnpSolverRansacTh2;
    float _mP4pRelocalizationNnRatioOrbMatcher = fSettings["Tracking.p4pRelocalizationNnRatioOrbMatcher"];
    mP4pRelocalizationNnRatioOrbMatcher = (_mP4pRelocalizationNnRatioOrbMatcher == 0.0) ? 0.9 : _mP4pRelocalizationNnRatioOrbMatcher;
    int _mMinimumMatchesRefKeyframe = fSettings["Tracking.minimumMatchesRefKeyframe"];
    mMinimumMatchesRefKeyframe = (_mMinimumMatchesRefKeyframe == 0) ? 15 : _mMinimumMatchesRefKeyframe;
    int _mKeyframeTrackingThreshold = fSettings["Tracking.keyframeTrackingThreshold"];
    mKeyframeTrackingThreshold = (_mKeyframeTrackingThreshold == 0) ? 10 : _mKeyframeTrackingThreshold;
    int _mPointsCloserThreshold = fSettings["Tracking.pointsCloserThreshold"];
    mPointsCloserThreshold = (_mPointsCloserThreshold == 0) ? 100 : _mPointsCloserThreshold;
    int _mStereoSearchingRadius = fSettings["Tracking.stereoSearchingRadius"];
    mStereoSearchingRadius = (_mStereoSearchingRadius == 0) ? 15 : _mStereoSearchingRadius;
    int _mSearchingRadius = fSettings["Tracking.searchingRadius"];
    mSearchingRadius = (_mSearchingRadius == 0) ? 7 : _mSearchingRadius;
    int _mSpeedupMatchesThreshold = fSettings["Tracking.speedupMatchesThreshold"];
    mSpeedupMatchesThreshold = (_mSpeedupMatchesThreshold == 0) ? 20 : _mSpeedupMatchesThreshold;
    int _mSpeedupMatchesThreshold2 = fSettings["Tracking.speedupMatchesThreshold2"];
    mSpeedupMatchesThreshold2 = (_mSpeedupMatchesThreshold2 == 0) ? 20 : _mSpeedupMatchesThreshold2;
    int _mMotionModelThreshold = fSettings["Tracking.motionModelThreshold"];
    mMotionModelThreshold = (_mMotionModelThreshold == 0) ? 10 : _mMotionModelThreshold;
    int _mLocalMapTrackingThreshold = fSettings["Tracking.localMapTrackingThreshold"];
    mLocalMapTrackingThreshold = (_mLocalMapTrackingThreshold == 0) ? 30 : _mLocalMapTrackingThreshold;
    int _mLocalMapTrackingThreshold2 = fSettings["Tracking.localMapTrackingThreshold2"];
    mLocalMapTrackingThreshold2 = (_mLocalMapTrackingThreshold2 == 0) ? 50 : _mLocalMapTrackingThreshold2;
    int _mNewKeyframeThreshold = fSettings["Tracking.newKeyframeThreshold"];
    mNewKeyframeThreshold = (_mNewKeyframeThreshold == 0) ? 100 : _mNewKeyframeThreshold;
    int _mRGBDSearchingRadiusThreshold = fSettings["Tracking.RGBDSearchingRadiusThreshold"];
    mRGBDSearchingRadiusThreshold = (_mRGBDSearchingRadiusThreshold == 0) ? 3 : _mRGBDSearchingRadiusThreshold;
    int _mSearchingByProjectionThreshold = fSettings["Tracking.searchingByProjectionThreshold"];
    mSearchingByProjectionThreshold = (_mSearchingByProjectionThreshold == 0) ? 5 : _mSearchingByProjectionThreshold;
    int _mKeyframesLimit = fSettings["Tracking.keyframesLimit"];
    mKeyframesLimit = (_mKeyframesLimit == 0) ? 80 : (size_t)_mKeyframesLimit;
    int _mKeyframeCandidateThreshold = fSettings["Tracking.keyframeCandidateThreshold"];
    mKeyframeCandidateThreshold = (_mKeyframeCandidateThreshold == 0) ? 15 : _mKeyframeCandidateThreshold;
    int _mPnpSolverRansacMinInliers = fSettings["Tracking.pnpSolverRansacMinInliers"];
    mPnpSolverRansacMinInliers = (_mPnpSolverRansacMinInliers == 0) ? 10 : _mPnpSolverRansacMinInliers;
    int _mPnpSolverRansacMaxIterations = fSettings["Tracking.pnpSolverRansacMaxIterations"];
    mPnpSolverRansacMaxIterations = (_mPnpSolverRansacMaxIterations == 0) ? 300 : _mPnpSolverRansacMaxIterations;
    int _mPnpSolverRansacMinSet = fSettings["Tracking.pnpSolverRansacMinSet"];
    mPnpSolverRansacMinSet = (_mPnpSolverRansacMinSet == 0) ? 4 : _mPnpSolverRansacMinSet;
    int _mRansacIterationsRelocalization = fSettings["Tracking.ransacIterationsRelocalization"];
    mRansacIterationsRelocalization = (_mRansacIterationsRelocalization == 0) ? 5 : _mRansacIterationsRelocalization;

    cout << endl << "Tracking parameters:" << endl;
    cout << "- " << "ReferenceKeyframeNnRatioOrbMatcher: " << mReferenceKeyframeNnRatioOrbMatcher << endl;
    cout << "- " << "MotionModelNnRatioOrbMatcher: " << mMotionModelNnRatioOrbMatcher << endl;
    cout << "- " << "CosineDelta: " << mCosineDelta << endl;
    cout << "- " << "SearchLocalPointsNnRatioOrbMatcher: " << mSearchLocalPointsNnRatioOrbMatcher << endl;
    cout << "- " << "RelocalizationNnRatioOrbMatcher: " << mRelocalizationNnRatioOrbMatcher << endl;
    cout << "- " << "PnpSolverRansacProbability: " << mPnpSolverRansacProbability << endl;
    cout << "- " << "PnpSolverRansacEpsilon: " << mPnpSolverRansacEpsilon << endl;
    cout << "- " << "PnpSolverRansacTh2: " << mPnpSolverRansacTh2 << endl;
    cout << "- " << "P4pRelocalizationNnRatioOrbMatcher: " << mP4pRelocalizationNnRatioOrbMatcher << endl;
    cout << "- " << "MinimumMatchesRefKeyframe: " << mMinimumMatchesRefKeyframe << endl;
    cout << "- " << "KeyframeTrackingThreshold: " << mKeyframeTrackingThreshold << endl;
    cout << "- " << "PointsCloserThreshold: " << mPointsCloserThreshold << endl;
    cout << "- " << "StereoSearchingRadius: " << mStereoSearchingRadius << endl;
    cout << "- " << "SearchingRadius: " << mSearchingRadius << endl;
    cout << "- " << "SpeedupMatchesThreshold: " << mSpeedupMatchesThreshold << endl;
    cout << "- " << "SpeedupMatchesThreshold2: " << mSpeedupMatchesThreshold2 << endl;
    cout << "- " << "MotionModelThreshold: " << mMotionModelThreshold << endl;
    cout << "- " << "LocalMapTrackingThreshold: " << mLocalMapTrackingThreshold << endl;
    cout << "- " << "LocalMapTrackingThreshold2: " << mLocalMapTrackingThreshold2 << endl;
    cout << "- " << "NewKeyframeThreshold: " << mNewKeyframeThreshold << endl;
    cout << "- " << "RGBDSearchingRadiusThreshold: " << mRGBDSearchingRadiusThreshold << endl;
    cout << "- " << "SearchingByProjectionThreshold: " << mSearchingByProjectionThreshold << endl;
    cout << "- " << "KeyframesLimit: " << mKeyframesLimit << endl;
    cout << "- " << "KeyframeCandidateThreshold: " << mKeyframeCandidateThreshold << endl;
    cout << "- " << "PnpSolverRansacMinInliers: " << mPnpSolverRansacMinInliers << endl;
    cout << "- " << "PnpSolverRansacMaxIterations: " << mPnpSolverRansacMaxIterations << endl;
    cout << "- " << "PnpSolverRansacMinSet: " << mPnpSolverRansacMinSet << endl;
    cout << "- " << "RansacIterationsRelocalization: " << mRansacIterationsRelocalization << endl;
}

Tracking::~Tracking()
{
    if (mSensor == System::STEREO) {
        stopping = true;
        sem_post(&leftSem1);
        sem_post(&rightSem1);
        sem_post(&leftSem2);
        sem_post(&rightSem2);

        leftExtractorThread->join();
        cout << "Left feature extractor terminated" << endl;
        rightExtractorThread->join();
        cout << "Right feature extractor terminated" << endl;

        delete leftExtractorThread;
        delete rightExtractorThread;

        sem_destroy(&leftSem1);
        sem_destroy(&leftSem2);
        sem_destroy(&rightSem1);
        sem_destroy(&rightSem2);
    }

    delete mpOptimizer;

    if (mpORBextractorLeft)
      delete mpORBextractorLeft;

    if (mpORBextractorRight)
      delete mpORBextractorRight;

    if (mpIniORBextractor)
      delete mpIniORBextractor;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(
        mImGray, imGrayRight, timestamp,
        mpORBextractorLeft, mpORBextractorRight,
        mpFBOWVocabulary, mK,
        mDistCoef, mbf, mThDepth,
        &leftSem1, &leftSem2,
        &rightSem1, &rightSem2,
        &leftImage, &rightImage,
        &leftKeyPoints, &rightKeyPoints,
        &leftDescriptors, &rightDescriptors);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpFBOWVocabulary, mK, mDistCoef, mbf, mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if(mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    {
        mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpFBOWVocabulary, mK, mDistCoef, mbf, mThDepth);
    }
    else
    {
        mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpFBOWVocabulary, mK, mDistCoef, mbf, mThDepth);
    }

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        mLoops = mpSystem->GetLoopCount();

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;

            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera gets lost soon after initialization
        if (mState == LOST)
        {
            if (mpMap->KeyFramesInMap() <= 5)
            {
                cout << "Track lost soon after initialisation, resetting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        if (!mlRelativeFramePoses.empty())
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }
}

void Tracking::ExtractorThread(
    ORB_SLAM2::ORBextractor * extractor,
    cv::Mat ** image,
    std::vector<cv::KeyPoint> ** keyPoints,
    cv::Mat ** descriptors,
    sem_t * sem2,
    sem_t * sem1)
{
    while (true) {
        sem_wait(sem2);

        if (stopping) {
            break;
        }

        (*extractor)(**image, cv::Mat(), **keyPoints, **descriptors);

        sem_post(sem1);
    }
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and associate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{
    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

    pKFini->ComputeFboW();
    pKFcur->ComputeFboW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and associate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        // Create MapPoint
        cv::Mat worldPos(mvIniP3D[i]);
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        // Fill current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        // Add to map
        mpMap->AddMapPoint(pMP);
    }

    // Update connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Do first Bundle Adjustment
    cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;
    mpOptimizer->GlobalBundleAdjustemnt(mpMap);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
    {
        cout << "Wrong initialization, resetting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute FBag of Words vector
    mCurrentFrame.ComputeFboW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(mReferenceKeyframeNnRatioOrbMatcher, true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches;

    nmatches = matcher.SearchByFboW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if(nmatches < mMinimumMatchesRefKeyframe)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    mpOptimizer->PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap >= mKeyframeTrackingThreshold;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first > mThDepth && nPoints > mPointsCloserThreshold)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(mMotionModelNnRatioOrbMatcher, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
      th = mStereoSearchingRadius;
    else
      th = mSearchingRadius;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches < mSpeedupMatchesThreshold)
    {
      fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
      nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches < mSpeedupMatchesThreshold2)
      return false;

    // Optimize frame pose with all matches
    mpOptimizer->PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= mMotionModelThreshold;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    mpOptimizer->PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < mLocalMapTrackingThreshold2)
        return false;

    if(mnMatchesInliers < mLocalMapTrackingThreshold)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor != System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor
        // We create all those MapPoints whose depth < mThDepth
        // If there are too less we take the closest ones
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first > mThDepth && nPoints > mNewKeyframeThreshold)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP, mCosineDelta))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch > 0)
    {
      ORBmatcher matcher(mSearchLocalPointsNnRatioOrbMatcher); 
      int th = 1;
      if(mSensor == System::RGBD)
        th = mRGBDSearchingRadiusThreshold;
      // If the camera has been relocalised recently, perform a coarser search
      if(mCurrentFrame.mnId < mnLastRelocFrameId+2)
        th = mSearchingByProjectionThreshold;
      matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size() > mKeyframesLimit)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    vector<KeyFrame*> vpCandidateKFs;

    // Compute FBag of Words Vector
    mCurrentFrame.ComputeFboW();
    vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(mRelocalizationNnRatioOrbMatcher, true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches;
            nmatches = matcher.SearchByFboW(pKF, mCurrentFrame, vvpMapPointMatches[i]);

            if(nmatches < mKeyframeCandidateThreshold)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(mPnpSolverRansacProbability, mPnpSolverRansacMinInliers, mPnpSolverRansacMaxIterations, mPnpSolverRansacMinSet, mPnpSolverRansacEpsilon, mPnpSolverRansacTh2);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(mP4pRelocalizationNnRatioOrbMatcher, true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform mRansacIterationsRelocalization Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(mRansacIterationsRelocalization, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = mpOptimizer->PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = mpOptimizer->PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = mpOptimizer->PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        mCurrentFrame.mTcw = cv::Mat::zeros(0, 0, CV_32F); // set mTcw back to empty if relocation is failed
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System resetting" << endl;
    if (mpViewer)
    {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(3000));
        }
    }

    // Reset Local Mapping
    mpLocalMapper->RequestReset();
    cout << "Local Mapping reset" << endl;

    // Reset Loop Closing
    mpLoopClosing->RequestReset();
    cout << "Loop Closing reset" << endl;

    // Clear BoW Database
    mpKeyFrameDB->clear();
    cout << "KeyFrame Database reset" << endl;

    // Clear Map (this erases MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

} // namespace ORB_SLAM2
