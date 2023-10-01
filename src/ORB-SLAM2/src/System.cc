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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <thread>

#include <pangolin/pangolin.h>

#include "Converter.h"
#include "System.h"

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, bool is_save_map_, bool replayer_):mSensor(sensor), is_save_map(is_save_map_), replayer(replayer_), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),
        mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mCurrCameraPose(cv::Mat::eye(4, 4, CV_32F)), bActive(true)
{
    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    int _mloopClosingCPU = fSettings["LoopClosing.CPU"];
    int _mlocalMappingCPU = fSettings["LocalMapping.CPU"];

    cout << "System mode: ";

    if (mSensor == MONOCULAR)
        cout << "MONOCULAR" << endl;
    else if (mSensor == STEREO)
        cout << "STEREO" << endl;
    else if (mSensor == RGBD)
        cout << "RGBD" << endl;

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file: " << strSettingsFile << endl;
       throw std::runtime_error("Failed to open settings file: " + strSettingsFile);
    }

    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty()) {
        mapfile = std::string(mapfilen);
    } else if (is_save_map) {
        cerr << "ORB_SLAM2::System::System: No map file has been specified in configuration file" << endl;
        throw std::runtime_error("ORB_SLAM2::System::System: No map file has been specified in configuration file");
    }

    // Load FBoW Vocabulary
    cout << endl << "Loading FBoW vocabulary..." << std::flush;
    mpFBOWVocabulary = new fbow::Vocabulary();
    mpFBOWVocabulary->readFromFile(strVocFile);
    cout << " done" << endl;

    // Set camera parameters
    mCameraFx = fsSettings["Camera.fx"];
    mCameraFy = fsSettings["Camera.fy"];

    // Create KeyFrame Database and Map
    bool loaded_map = false;
    if (!mapfile.empty()) {
        if (LoadMap(mapfile)) {
            bReuseMap = true;
            loaded_map = true;
        } else {
            mpKeyFrameDatabase = new KeyFrameDatabase(mpFBOWVocabulary);
            mpMap = new Map();
        }
    } else {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpFBOWVocabulary);
        mpMap = new Map();
    }

    // Create Drawers, used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // Initialize the Tracking module
    mpTracker = new Tracking(this, mpFBOWVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);

    // Initialize the Local Mapping module
    mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, strSettingsFile);

    // Initialize the Loop Closing module
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpFBOWVocabulary, mSensor != MONOCULAR, strSettingsFile);
    if (loaded_map) {
        mpLoopCloser->SetLoopCount(initLoopCount);
    }

    // Initialize the Viewer module
    if (bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, bReuseMap);
        mpTracker->SetViewer(mpViewer);
    }

    // Set pointers between modules
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // Start Local Mapping thread
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    if (_mlocalMappingCPU != -1) {
        std::cout << "Starting Local Mapping on CPU " << _mlocalMappingCPU << "..." << std::endl;
        cpu_set_t local_mapping_cpu_set;
        CPU_ZERO(&local_mapping_cpu_set);
        CPU_SET(_mlocalMappingCPU, &local_mapping_cpu_set);
        if (pthread_setaffinity_np(mptLocalMapping->native_handle(), sizeof(cpu_set_t), &local_mapping_cpu_set)) {
            char err_msg_buf[100] = {};
            char * err_msg = strerror_r(errno, err_msg_buf, 100);
            throw std::runtime_error(
                  "ORB_SLAM2::System::System: Failed to configure Local Mapping thread: " +
                  std::string(err_msg));
        }
    } else {
        std::cout << "Starting Local Mapping..." << std::endl;
    }

    // Start Loop Closing thread
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    if (_mloopClosingCPU != -1) {
        std::cout << "Starting Loop Closing on CPU " << _mloopClosingCPU << "..." << std::endl;
        cpu_set_t loop_closing_cpu_set;
        CPU_ZERO(&loop_closing_cpu_set);
        CPU_SET(_mloopClosingCPU, &loop_closing_cpu_set);
        if (pthread_setaffinity_np(mptLoopClosing->native_handle(), sizeof(cpu_set_t), &loop_closing_cpu_set)) {
            char err_msg_buf[100] = {};
            char * err_msg = strerror_r(errno, err_msg_buf, 100);
            throw std::runtime_error(
                    "ORB_SLAM2::System::System: Failed to configure Loop Closing thread: " +
                    std::string(err_msg));
        }
    } else {
        std::cout << "Starting Loop Closing..." << std::endl;
    }

    // Start Viewer thread
    if (bUseViewer) {
        std::cout << "Starting Viewer..." << std::endl;
        mptViewer = new thread(&Viewer::Run, mpViewer);
    }
}

System::~System()
{
    // Shutdown ORB-SLAM2 system
    if (bActive) {
        Shutdown();
    }

    // Destroy modules in reverse order
    if (mpLoopCloser) {
        delete mpLoopCloser;
    }
    if (mpLocalMapper) {
        delete mpLocalMapper;
    }
    if (mpTracker) {
        delete mpTracker;
    }
    if (mpViewer) {
        delete mpViewer;
    }
    if (mpMapDrawer) {
        delete mpMapDrawer;
    }
    if (mpFrameDrawer) {
        delete mpFrameDrawer;
    }

    // Destroy local data
    if (mpKeyFrameDatabase) {
        delete mpKeyFrameDatabase;
    }
    if (mpMap) {
        delete mpMap;
    }
    if (mpFBOWVocabulary) {
        delete mpFBOWVocabulary;
    }
}

HPose System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if (mSensor != STEREO)
    {
        cerr << "ORB_SLAM2::System::TrackStereo: mode not set to STEREO" << endl;
        throw std::runtime_error("ORB_SLAM2::System::TrackStereo: mode not set to STEREO");
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    // Fixes to cope with replays
    // Wait until LoopCloser has effectively finished and LocalMapper
    // has been released. This allows not to get new frames from the
    // tracker (it actually freezes the images grabbing).
    if (replayer) {
      while(!mpLoopCloser->isFinishedGBA())
      {
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }

      while(mpLocalMapper->isStopped())
      {
          std::this_thread::sleep_for(std::chrono::microseconds(1000));
      }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);
    mCurrCameraPose = Tcw;
    HPose pose{};
    pose_mat_to_hpose(Tcw, pose);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return pose;
}

HPose System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if (mSensor!=RGBD)
    {
        cerr << "ORB_SLAM2::System::TrackRGBD: Mode not set to RGBD" << endl;
        throw std::runtime_error("ORB_SLAM2::System::TrackRGBD: Mode not set to RGBD");
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);
    mCurrCameraPose = Tcw;
    HPose pose{};
    pose_mat_to_hpose(Tcw, pose);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return pose;
}

// Overload of the track RGBD function
HPose System::TrackIRD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if (mSensor != RGBD)
    {
        // IR-D mode is a variation of RGB-D mode
        cerr << "ORB_SLAM2::System::TrackIRD: Mode not set to RGBD" << endl;
        throw std::runtime_error("ORB_SLAM2::System::TrackIRD: Mode not set to RGBD");
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);
    mCurrCameraPose = Tcw;
    HPose pose{};
    pose_mat_to_hpose(Tcw, pose);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return pose;
}

HPose System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if (mSensor != MONOCULAR)
    {
        cerr << "ORB_SLAM2::System::TrackMonocular: Mode not set to MONOCULAR" << endl;
        throw std::runtime_error("ORB_SLAM2::System::TrackMonocular: Mode not set to MONOCULAR");
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
    mCurrCameraPose = Tcw;
    HPose pose{};
    pose_mat_to_hpose(Tcw, pose);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return pose;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
  // Wait for a Global Bundle Adjustment to finish
  while (mpLoopCloser->isRunningGBA()) {
    this_thread::sleep_for(chrono::milliseconds(500));
  }

  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }
}

void System::Shutdown()
{
    std::cout << "Shutting down VSLAM system" << std::endl;
    mpLocalMapper->RequestFinish();
    std::cout << "Stopping Local Mapping..." << std::endl;
    mpLoopCloser->RequestFinish();
    std::cout << "Stopping Loop Closing..." << std::endl;
    if (mpViewer)
    {
        mpViewer->RequestFinish();
        std::cout << "Stopping Viewer..." << std::endl;
        while(!mpViewer->isFinished())
        {
            std::cerr << "Waiting for Viewer to terminate..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "Viewer terminated" << std::endl;
    }

    // Wait until all thread have effectively stopped
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cerr << "Waiting for threads to terminate..." << std::endl;
    }

    if (is_save_map)
    {
        SaveMap(mapfile);
    }

    // Destroy threads
    if (mptLoopClosing) {
      mptLoopClosing->join();
      delete mptLoopClosing;
    }
    if (mptLocalMapping) {
      mptLocalMapping->join();
      delete mptLocalMapping;
    }
    if (mptViewer) {
      mptViewer->join();
      delete mptViewer;
    }

    bActive = false;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "Trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while (pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "Trajectory saved!" << endl;
}

void System::SaveTrajectory(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "Trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectory(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if (pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "Trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

// Computing covariance matrix for VSLAM pose as per
// Appendix F in "Asynchronous Multi-Sensor Fusion for 3D
// Mapping and Localization", P. Geneva, K. Eckenhoff, G. Huang
// Converts it into a NWU coordinate system
cv::Mat System::GetCurrentCovarianceMatrix(bool rotationInverse)
{
  float fx = mCameraFx;
  float fy = mCameraFy;

  // Computing Visual odometry covariance matrix
  if (this->GetTrackingState() == Tracking::OK)
  {
    vector<MapPoint*> trackedPoints = this->GetTrackedMapPoints();
    std::vector<cv::Mat> featPos;
    for (size_t i = 0; i < trackedPoints.size(); i++)
    {
      MapPoint* pMP = trackedPoints[i];
      if (pMP && !pMP->isBad())
        featPos.push_back(pMP->GetWorldPos());
    }

    // Get current camera pose
    cv::Mat cameraPose = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Twc = GetCurrentCameraPose();
    cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
    cv::Mat Pwc = -Rwc.t() * Twc.rowRange(0, 3).col(3);
    Rwc.copyTo(cameraPose.rowRange(0, 3).colRange(0, 3));
    Pwc.copyTo(cameraPose.rowRange(0, 3).col(3));

    // Initialize algorithm data
    cv::Mat factor = cv::Mat::zeros(6, 6, CV_32F); // Iterative solution
    cv::Mat lambda = cv::Mat::zeros(2, 2, CV_32F); // Noise matrix
    lambda.at<float>(0, 0) = 1.0f / (fx*fx);
    lambda.at<float>(1, 1) = 1.0f / (fy*fy);
    cv::Mat Rcg = cv::Mat::zeros(3, 3, CV_32F); // Rotation from Camera pose to Global frame
    cameraPose.rowRange(0, 3).colRange(0, 3).copyTo(Rcg);
    if (rotationInverse) {
      Rcg = Rcg.inv();
    }

    for (size_t j = 0; j < featPos.size(); j++)
    {
      cv::Mat H1 = cv::Mat::zeros(2, 3, CV_32F);
      cv::Mat H2 = cv::Mat::zeros(3, 6, CV_32F);
      cv::Mat Pcf = cv::Mat::zeros(3, 1, CV_32F); // Pose of feature w.r.t. Camera frame
      cv::Mat Pgc = cv::Mat::zeros(3, 1, CV_32F); // Pose of camera w.r.t. Global frame
      float a, b;
      Pgc.at<float>(0) = cameraPose.at<float>(0, 3);
      Pgc.at<float>(1) = cameraPose.at<float>(1, 3);
      Pgc.at<float>(2) = cameraPose.at<float>(2, 3);

      Pcf = Rcg * (featPos[j] - Pgc);

      // Prevent ill-conditioned divisions
      if (abs(Pcf.at<float>(2)) < 1e-6f) {
        float sign = Pcf.at<float>(2) < 0.0f ? -1.0f : 1.0f;
        a = sign * 1000000.0f;
        b = sign * 1000000000000.0f;
      } else {
        a = 1.0f / Pcf.at<float>(2);
        b = 1.0f / (Pcf.at<float>(2) * Pcf.at<float>(2));
      }

      H1.at<float>(0, 0) = a;
      H1.at<float>(0, 2) = -Pcf.at<float>(0) * b;
      H1.at<float>(1, 1) = a;
      H1.at<float>(1, 2) = -Pcf.at<float>(1) * b;
      H2.at<float>(0, 1) = -Pcf.at<float>(2);
      H2.at<float>(0, 2) = Pcf.at<float>(1);
      H2.at<float>(0, 3) = -Rcg.at<float>(0, 0);
      H2.at<float>(0, 4) = -Rcg.at<float>(0, 1);
      H2.at<float>(0, 5) = -Rcg.at<float>(0, 2);
      H2.at<float>(1, 0) = Pcf.at<float>(2);
      H2.at<float>(1, 2) = -Pcf.at<float>(0);
      H2.at<float>(1, 3) = -Rcg.at<float>(1, 0);
      H2.at<float>(1, 4) = -Rcg.at<float>(1, 1);
      H2.at<float>(1, 5) = -Rcg.at<float>(1, 2);
      H2.at<float>(2, 0) = -Pcf.at<float>(1);
      H2.at<float>(2, 1) = Pcf.at<float>(0);
      H2.at<float>(2, 3) = -Rcg.at<float>(2, 0);
      H2.at<float>(2, 4) = -Rcg.at<float>(2, 1);
      H2.at<float>(2, 5) = -Rcg.at<float>(2, 2);

      cv::Mat tmp = H1 * H2;
      factor = factor + tmp.t() * lambda.inv() * tmp;
    }

    cv::Mat swap_rows = cv::Mat::zeros(6, 6, CV_32F);
    cv::Mat swap_cols = cv::Mat::zeros(6, 6, CV_32F);

    // Swap position information
    swap_rows.at<float>(0, 2) = -1.0f;
    swap_rows.at<float>(1, 0) = 1.0f;
    swap_rows.at<float>(2, 1) = 1.0f;
    swap_cols.at<float>(2, 0) = -1.0f;
    swap_cols.at<float>(0, 1) = 1.0f;
    swap_cols.at<float>(1, 2) = 1.0f;

    // Swap rotation information
    swap_rows.at<float>(3, 5) = -1.0f;
    swap_rows.at<float>(4, 3) = 1.0f;
    swap_rows.at<float>(5, 4) = 1.0f;
    swap_cols.at<float>(5, 3) = -1.0f;
    swap_cols.at<float>(3, 4) = 1.0f;
    swap_cols.at<float>(4, 5) = 1.0f;

    cv::Mat cov_mat_spurious_cv = swap_rows * factor.inv() * swap_cols;

    // We need to purge the spurious negative eigenvalues: only consider variances,
    // and set to a small value the negative ones (it is taken as the default in the
    // robot_localization EKF implementation)
    Eigen::Matrix<float, 6, 6> cov_mat_spurious = cov_cv_to_eigen(cov_mat_spurious_cv);
    Eigen::Matrix<float, 6, 6> L = cov_mat_spurious.diagonal().asDiagonal();
    for (int i = 0; i < 6; i++) {
      L(i, i) = std::fmaxf(L(i, i), float(1e-6f));
    }
    return cov_eigen_to_cv(L);
  } else {
    return cv::Mat::eye(6, 6, CV_32F);
  }
}

// Returns the current FrameDrawer frame
cv::Mat System::GetFrameDrawerFrame()
{
  return mpFrameDrawer->DrawFrame();
}

// Returns the current loops count
long unsigned int System::GetLoopCount()
{
  return mpLoopCloser->GetLoopCount();
}

// Returns the currently stored map: each element is a 3D-point coordinates vector
std::shared_ptr<Eigen::MatrixXf> System::GetMap(bool wait_gba)
{
    // If a Global Bundle Adjustment is running, abort this or wait for it
    while (mpLoopCloser->isRunningGBA()) {
        if (wait_gba) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            break;
        }
    }

    // Other threads must not update the map while this reads it
    unique_lock<mutex> mapLock(mpMap->mMutexMapUpdate);

    // Get all valid map points
    vector<MapPoint *> mapPoints = mpMap->GetAllMapPoints();
    mapPoints.erase(
        remove_if(
            mapPoints.begin(),
            mapPoints.end(),
            [] (MapPoint * p) { return p->isBad(); }),
            mapPoints.end());

    // Fill a matrix with their homogeneous coordinates ([X Y Z 1]w = [Z -X -Y 1]o)
    map_eigen = std::make_shared<Eigen::MatrixXf>(Eigen::MatrixXf(4, mapPoints.size()));
    for (unsigned long int i = 0; i < mapPoints.size(); i++) {
        cv::Mat pPos = mapPoints[i]->GetWorldPos();
        map_eigen->block<4, 1>(0, i) = Eigen::Vector4f(
            pPos.at<float>(2),
            -pPos.at<float>(0),
            -pPos.at<float>(1),
            1.0f);
    }

  return map_eigen;
}

void System::SaveMap(const string & filename)
{
    std::ofstream out(filename, std::ios_base::binary | std::ios_base::trunc);
    if (!out)
    {
        cerr << "ORB_SLAM2::System::SaveMap: Cannot write to file: " << filename << std::endl;
        throw std::runtime_error("ORB_SLAM2::System::SaveMap: Cannot write to file: " + filename);
    }
    cout << "Saving map to file: " << filename << " ..." << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    oa << mpLoopCloser->GetLoopCount();
    out.close();
    cout << " done" << std::endl;
}

bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot open map file: " << mapfile << std::endl;
        return false;
    }
    cout << "Loading map from file: " << mapfile << " ..." << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    ia >> initLoopCount;
    mpKeyFrameDatabase->SetFBOWvocabulary(mpFBOWVocabulary);
    cout << " done" << std::endl;
    cout << "Rebuilding map from archive..." << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetFBOWvocabulary(mpFBOWVocabulary);
        it->ComputeFboW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " done" << endl;
    in.close();
    return true;
}

// Converts an OpenCV isometry matrix to an HPose, converting into a NWU coordinate system
void System::pose_mat_to_hpose(const cv::Mat & mat, HPose & pose)
{
  if (!mat.empty()) {
    cv::Mat Rwc = mat.rowRange(0, 3).colRange(0, 3);
    cv::Mat Twc = -Rwc.t() * mat.rowRange(0, 3).col(3);

    Eigen::Matrix3f orMat;
    orMat(0, 0) = Rwc.at<float>(0, 0);
    orMat(0, 1) = Rwc.at<float>(0, 1);
    orMat(0, 2) = Rwc.at<float>(0, 2);
    orMat(1, 0) = Rwc.at<float>(1, 0);
    orMat(1, 1) = Rwc.at<float>(1, 1);
    orMat(1, 2) = Rwc.at<float>(1, 2);
    orMat(2, 0) = Rwc.at<float>(2, 0);
    orMat(2, 1) = Rwc.at<float>(2, 1);
    orMat(2, 2) = Rwc.at<float>(2, 2);
    Eigen::Quaternionf q_orb_tmp(orMat);

    cv::Vec3f p_orb(Twc.at<float>(2), -Twc.at<float>(0), -Twc.at<float>(1));
    cv::Vec4f q_orb(q_orb_tmp.w(), -q_orb_tmp.z(), q_orb_tmp.x(), q_orb_tmp.y());

    pose.SetPosition(p_orb);
    pose.SetRotation(q_orb);
  }
}

Eigen::Matrix<float, 6, 6> System::cov_cv_to_eigen(const cv::Mat & cov_cv)
{
  Eigen::Matrix<float, 6, 6> cov_eigen;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov_eigen(i, j) = cov_cv.at<float>(i, j);
    }
  }

  return cov_eigen;
}

cv::Mat System::cov_eigen_to_cv(const Eigen::Matrix<float, 6, 6> & cov_eigen)
{
  cv::Mat cov_cv(6, 6, CV_32F);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov_cv.at<float>(i, j) = cov_eigen(i, j);
    }
  }

  return cov_cv;
}

} // namespace ORB_SLAM2
