/**
 * ORB-SLAM2 driver node topic subscription callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 1, 2023
 */

#include <cmath>

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Gets the mounting correction transform.
 *
 * @param msg Imu message to parse.
 */
void ORB_SLAM2DriverNode::camera_imu_callback(const Imu::SharedPtr msg)
{
  // Check if orientation info is valid
  if (msg->orientation_covariance[0] == -1.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::camera_imu_callback: Invalid orientation");
    return;
  }

  Eigen::Quaterniond init_q(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z);

  PoseKit::Pose pose{};
  pose.set_attitude(init_q);

  init_pose_lock_.lock();
  init_pose_ = pose;
  init_pose_lock_.unlock();

  RCLCPP_INFO(
    this->get_logger(),
    "Mounting correction: [%.6f, %.6f, %.6f, %.6f] -> (R: %.4f°, P: %.4f°)",
    init_q.w(),
    init_q.x(),
    init_q.y(),
    init_q.z(),
    pose.get_rpy().alpha() * 180.0 / M_PI,
    pose.get_rpy().beta() * 180.0 / M_PI);

  camera_imu_sub_.reset();
}

/**
 * @brief Gets two stereoscopic frames and forwards them to the ORB-SLAM2 thread.
 *
 * @param camera_1_msg Camera 1 frame.
 * @param camera_2_msg Camera 2 frame.
 */
void ORB_SLAM2DriverNode::stereo_callback(
  const Image::ConstSharedPtr & camera_1_msg,
  const Image::ConstSharedPtr & camera_2_msg)
{
  // Parse frames
  cv::Mat frame_1 = image_to_cv_mat(camera_1_msg);
  cv::Mat frame_2 = image_to_cv_mat(camera_2_msg);

  if (camera_1_msg->encoding != camera_2_msg->encoding) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::stereo_callback: Left/Right frames have different encodings: %s, %s",
      camera_1_msg->encoding.c_str(),
      camera_2_msg->encoding.c_str());
    return;
  }

  cv::Mat frame_1_input, frame_2_input;
  if (preconvert_frames_) {
    cv::ColorConversionCodes conversion_code = cv::COLOR_BGR2GRAY;
    if (camera_1_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      conversion_code = cv::COLOR_BGR2GRAY;
    } else if (camera_1_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      conversion_code = cv::COLOR_RGB2GRAY;
    } else if (camera_1_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
      conversion_code = cv::COLOR_BGRA2GRAY;
    } else if (camera_1_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
      conversion_code = cv::COLOR_RGBA2GRAY;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "ORB_SLAM2DriverNode::stereo_callback: Unsupported image encoding: %s",
        camera_1_msg->encoding.c_str());
    }

    cv::cvtColor(frame_1, frame_1_input, conversion_code);
    cv::cvtColor(frame_2, frame_2_input, conversion_code);
  } else {
    frame_1_input = frame_1;
    frame_2_input = frame_2;
  }

  // Forward frames to tracking thread
  sem_wait(&tracking_sem_1_);
  camera_1_frame_ = frame_1_input.clone();
  camera_2_frame_ = frame_2_input.clone();
  frame_ts_ = camera_1_msg->header.stamp;
  sem_post(&tracking_sem_2_);
}

} // namespace ORB_SLAM2Driver
