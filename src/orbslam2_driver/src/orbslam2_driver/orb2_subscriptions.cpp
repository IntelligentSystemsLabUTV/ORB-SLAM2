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

} // namespace ORB_SLAM2Driver
