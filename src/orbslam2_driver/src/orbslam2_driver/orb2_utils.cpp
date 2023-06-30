/**
 * ORB-SLAM2 driver node utility routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 1, 2023
 */

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Converts an HPose to a PoseKit::Pose.
 *
 * @param hpose The HPose to convert.
 * @param frame_id The frame ID to set in the PoseKit::Pose.
 * @param ts The timestamp to set in the PoseKit::Pose.
 * @param cov The covariance to set in the PoseKit::Pose.
 *
 * @return The converted PoseKit::Pose.
 *
 * @throws RuntimeError if the covariance matrix is not 6x6.
 */
PoseKit::Pose hpose_to_pose(
  const ORB_SLAM2::HPose & hpose,
  std::string && frame_id,
  const Time & ts,
  const cv::Mat & cov)
{
  if (cov.rows != 6 || cov.cols != 6) {
    throw std::runtime_error("ORB_SLAM2DriverNode::hpose_to_pose: Covariance matrix must be 6x6");
  }

  Eigen::Vector3d p(
    hpose.GetTranslation()[0],
    hpose.GetTranslation()[1],
    hpose.GetTranslation()[2]);
  Eigen::Quaterniond q(
    hpose.GetRotation()[0],
    hpose.GetRotation()[1],
    hpose.GetRotation()[2],
    hpose.GetRotation()[3]);

  Header header;
  header.set__frame_id(frame_id);
  header.set__stamp(ts);

  std::array<double, 36> cov_array;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      cov_array[i * 6 + j] = double(cov.at<float>(i, j));
    }
  }

  return PoseKit::Pose(p, q, header, cov_array);
}

} // namespace ORB_SLAM2Driver
