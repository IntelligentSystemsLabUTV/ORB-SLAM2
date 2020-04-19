#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <sys/types.h>
#include <dirent.h>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

void LoadImages(const string &sequenceDir, vector<string> &imageFilenamesIR, vector<string> &imageFilenamesD, vector<double> &timestamps);

int main(int argc, char **argv)
{
  if(argc != 4)
  {
    cerr << endl << "Usage: ./Test path_to_vocabulary path_to_settings path_to_sequence" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> imageFilenamesIR;
  vector<string> imageFilenamesD;
  vector<double> timestamps;
  string sequenceDir = string(argv[3]);
  LoadImages(sequenceDir, imageFilenamesIR, imageFilenamesD, timestamps);

  // Check consistency in the number of images and depthmaps
  int nImages = imageFilenamesIR.size();
  if(imageFilenamesIR.empty())
  {
    cerr << endl << "No images found in provided path." << endl;
    return 1;
  }
  else if(imageFilenamesD.size() != imageFilenamesIR.size())
  {
    cerr << endl << "Different number of images for ir and depth." << endl;
    return 1;
  }

  try {
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::RGBD, true, true);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat imIR, imD;
    // Main loop
    for(int ni = 0; ni < nImages; ni++)
    {
      // Read image and depthmap from file
      imIR = cv::imread(string(argv[3])+"/infrared/"+imageFilenamesIR[ni], cv::IMREAD_UNCHANGED);
      imD  = cv::imread(string(argv[3])+"/depth/"+imageFilenamesD[ni], cv::IMREAD_UNCHANGED);
      imD.convertTo(imD, CV_16SC1, 256.0 / 15.0);
      double tframe = timestamps[ni];
      SLAM.TrackRGBD(imIR, imD, tframe);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.dat");
    // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

void ReadDirectory(const string& name, vector<string> &v)
{
  DIR* dirp = opendir(name.c_str());
  struct dirent * dp;
  while ((dp = readdir(dirp)) != NULL) {
    v.push_back(dp->d_name);
  }
  closedir(dirp);
}

void LoadImages(const string &sequenceDir, vector<string> &imageFilenamesIR, vector<string> &imageFilenamesD, vector<double> &timestamps)
{
  ReadDirectory(sequenceDir + "/infrared", imageFilenamesIR);
  imageFilenamesIR.erase(imageFilenamesIR.begin());
  imageFilenamesIR.erase(imageFilenamesIR.begin());
  sort(imageFilenamesIR.begin(), imageFilenamesIR.end());

  ReadDirectory(sequenceDir + "/depth", imageFilenamesD);
  imageFilenamesD.erase(imageFilenamesD.begin());
  imageFilenamesD.erase(imageFilenamesD.begin());
  sort(imageFilenamesD.begin(), imageFilenamesD.end());

  for (auto x : imageFilenamesD)
  {
    size_t sPos = x.find("depth_");
    x.erase(sPos, 6);
    sPos = x.find(".jpg");
    x.erase(sPos, 4);
    timestamps.push_back(stod(x));
  }
}