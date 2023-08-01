/**
 * First step of creating a vocabulary is extracting features from a set of images.
 * We save them to a file for next step.
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <fbow/fbow.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif

#include "cmdline_parser.hpp"
#include "dirreader.h"

using namespace fbow;
using namespace std;

// ----------------------------------------------------------------------------
void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}


std::vector<cv::Mat> loadFeatures(std::vector<string> path_to_images, string descriptor = "")
{
  // Select detector
  cv::Ptr<cv::Feature2D> fdetector;
  if (descriptor == "orb") {fdetector = cv::ORB::create(2000);} else if (descriptor == "brisk") {
    fdetector = cv::BRISK::create();
  }
#ifdef OPENCV_VERSION_3
  else if (descriptor == "akaze") {
    fdetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4);
  }
#endif
#ifdef USE_CONTRIB
  else if (descriptor == "surf") {fdetector = cv::xfeatures2d::SURF::create(15, 4, 2);}
#endif

  else {throw std::runtime_error("Invalid descriptor");}
  assert(!descriptor.empty());
  std::vector<cv::Mat> features;

  cout << "STARTING FEATURE EXTRACTION" << endl;
  for (size_t i = 0; i < path_to_images.size(); ++i) {
    if (path_to_images[i][path_to_images[i].size() - 1] == '.') {
      continue;
    }
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    std::cout << "Reading image: " << path_to_images[i] << std::endl;
    cv::Mat image = cv::imread(path_to_images[i], cv::ImreadModes::IMREAD_GRAYSCALE);
    if (image.empty()) {
      std::cerr << "Could not open image: " << path_to_images[i] << std::endl;
      continue;
    }
    fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
    std::cout << "Extracted features: " << keypoints.size() << "\n" << std::endl;
    features.push_back(descriptors);
  }
  return features;
}

// ----------------------------------------------------------------------------
void saveToFile(
  std::string filename,
  const vector<cv::Mat> & features,
  std::string desc_name,
  bool rewrite = true)
{

  // Test it can be written
  if (!rewrite) {
    std::fstream ifile(filename, std::ios::binary);
    if (ifile.is_open()) { // Read size and rewrite
      std::runtime_error("Output file " + filename + " already exists!");
    }
  }
  std::ofstream ofile(filename, std::ios::binary);
  if (!ofile.is_open()) {
    std::cerr << "Could not open output file" << std::endl;
    exit(EXIT_FAILURE);
  }

  char _desc_name[20];
  desc_name.resize(min(size_t(19), desc_name.size()));
  strcpy(_desc_name, desc_name.c_str());
  ofile.write(_desc_name, 20);

  uint32_t size = features.size();
  ofile.write((char *)&size, sizeof(size));
  for (auto & f:features) {
    if (!f.isContinuous()) {
      throw std::runtime_error("Matrices should be continuous");
    }
    uint32_t aux = f.cols; ofile.write( (char *)&aux, sizeof(aux));
    aux = f.rows; ofile.write( (char *)&aux, sizeof(aux));
    aux = f.type(); ofile.write( (char *)&aux, sizeof(aux));
    ofile.write( (char *)f.ptr<uchar>(0), f.total() * f.elemSize());
  }
}

// ----------------------------------------------------------------------------
int main(int argc, char ** argv)
{

  try {
    CmdLineParser cml(argc, argv);
    if (cml["-h"] || argc < 4) {
      std::cerr
        << "Usage:\n"
        << "\tfbow_create_voc_step0 descriptor_name output dir_with_images\n"
        << "\tdescriptors: brisk, surf, orb (default), akaze (only if using opencv 3)"
        << std::endl;
      exit(EXIT_FAILURE);
    }

    string descriptor = argv[1];
    string output = argv[2];

    auto images = DirReader::read(argv[3]);
    std::vector<cv::Mat> features = loadFeatures(images, descriptor);

    // Save features to file
    std::cerr << "Saving to " << argv[2] << std::endl;
    saveToFile(argv[2], features, descriptor);


  } catch (std::exception & ex) {
    cerr << ex.what() << endl;
  }

  exit(EXIT_SUCCESS);
}
