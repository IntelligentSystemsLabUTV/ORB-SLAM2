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
    "Mounting correction: [%.4f, %.4f, %.4f, %.4f] (R: %.4f°, P: %.4f°, Y: %.4f°)",
    init_q.w(),
    init_q.x(),
    init_q.y(),
    init_q.z(),
    pose.get_rpy().alpha() * 180.0 / M_PI,
    pose.get_rpy().beta() * 180.0 / M_PI,
    pose.get_rpy().gamma() * 180.0 / M_PI);

  camera_imu_sub_.reset();
}

/**
 * @brief Gets two stereoscopic frames and forwards them to the ORB-SLAM2 thread.
 *
 * @param camera_1_msg Camera 1 frame.
 * @param camera_2_msg Camera 2 frame.
 *
 * @throws RuntimeError if system time cannot be retrieved.
 */
void ORB_SLAM2DriverNode::stereo_callback(
  const Image::ConstSharedPtr & camera_1_msg,
  const Image::ConstSharedPtr & camera_2_msg)
{
  struct timespec timeout;
  if (clock_gettime(CLOCK_REALTIME, &timeout) == -1) {
    RCLCPP_FATAL(
      this->get_logger(),
      "ORB_SLAM2DriverNode::stereo_callback: clock_gettime failed");
    throw std::runtime_error(
            "ORB_SLAM2DriverNode::stereo_callback: clock_gettime failed");
  }
  timeout.tv_sec += 1;

  // Try to forward new frames to the tracking thread
  if (sem_timedwait(&orb2_thread_sem_1_, &timeout) == 0) {
    camera_1_frame_ = image_to_cv_mat(camera_1_msg);
    camera_2_frame_ = image_to_cv_mat(camera_2_msg);
    sem_post(&orb2_thread_sem_2_);
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::stereo_callback: Tracking thread busy, missed frames");
  }
}

} // namespace ORB_SLAM2Driver
