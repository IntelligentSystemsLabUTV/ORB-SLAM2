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


#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

#include <sched.h>
#include <pthread.h>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "Viewer.h"
#include "HPose.h"

#include "BoostArchiver.h"
// for map file io
#include <fstream>

#include <fbow/fbow.h>

using namespace std;

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    enum eSensor {
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, bool is_save_map_ = false, bool replayer = false);

    ~System();

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    HPose TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    HPose TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Process the given ird frame. Depthmap must be registered to the IR frame.
    // referenced to world reference frame.
    // Input image: IR (CV_8UC3) or grayscale (CV_8U).
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    HPose TrackIRD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    HPose TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // Save camera trajectory (for stereo and RGB-D)
    void SaveTrajectory(const string &filename);

    // Save keyframe poses (for all sensor inputs)
    void SaveKeyFrameTrajectory(const string &filename);

    // Information from most recent processed frame
    // You can call this right after Track functions
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    inline const cv::Mat & GetCurrentCameraPose() const
    {
      return mCurrCameraPose;
    }
    inline void SetCurrentCameraPose(const cv::Mat & pose)
    {
      mCurrCameraPose = pose.clone();
    }

    cv::Mat GetCurrentCovarianceMatrix(bool);

    // Returns the currently stored map: each column is a 3D-point coordinates vector
    std::shared_ptr<Eigen::MatrixXf> GetMap(bool wait_gba = false);

    // Returns the current FrameDrawer frame
    cv::Mat GetFrameDrawerFrame();

    // Returns the current loops count
    long unsigned int GetLoopCount();

private:
    // Save/Load functions
    void SaveMap(const string &filename);
    bool LoadMap(const string &filename);

    // Converts an OpenCV isometry matrix to an HPose, converting into a NWU coordinate system
    void pose_mat_to_hpose(const cv::Mat & mat, HPose & hpose);

    Eigen::Matrix<float, 6, 6> cov_cv_to_eigen(const cv::Mat & cov_cv);
    cv::Mat cov_eigen_to_cv(const Eigen::Matrix<float, 6, 6> & cov_eigen);

private:

    // Input sensor
    eSensor mSensor;

    // FBOW vocabulary used for place recognition and feature matching.
    fbow::Vocabulary* mpFBOWVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    string mapfile;
    bool is_save_map;
    long unsigned int initLoopCount;
    bool replayer;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping = nullptr;
    std::thread* mptLoopClosing = nullptr;
    std::thread* mptViewer = nullptr;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // Current camera pose
    cv::Mat mCurrCameraPose;

    // Latest requested map
    std::shared_ptr<Eigen::MatrixXf> map_eigen = nullptr;

    // Camera parameters
    float mCameraFx;
    float mCameraFy;

    // Status flag
    bool bActive;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
