/**
 * ORB-SLAM2 driver node thread routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 2, 2023
 */

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Runs the ORB-SLAM2 tracking loop.
 *
 * @throws RuntimeError if something fails.
 */
void ORB_SLAM2DriverNode::orb2_thread_routine()
{
  cv::Mat frame_1, frame_2;
  Time frame_ts_ros;
  double frame_ts = 0.0;

  while (running_.load(std::memory_order_acquire)) {
    struct timespec timeout;
    if (clock_gettime(CLOCK_REALTIME, &timeout) == -1) {
      RCLCPP_FATAL(
        this->get_logger(),
        "ORB_SLAM2DriverNode::orb2_thread_routine: clock_gettime failed");
      throw std::runtime_error(
              "ORB_SLAM2DriverNode::orb2_thread_routine: clock_gettime failed");
    }
    timeout.tv_sec += 1;

    if (sem_timedwait(&orb2_thread_sem_2_, &timeout) == 0) {
      // Get frames
      frame_1 = camera_1_frame_;
      frame_2 = camera_2_frame_;
      frame_ts_ros = frame_ts_;
      frame_ts =
        double(frame_ts_.sec) * 1e9 + double(frame_ts_.nanosec);
      sem_post(&orb2_thread_sem_1_);

      // Execute tracking
      PoseKit::Pose orb2_pose{};
      if (mode_str_ == "STEREO") {
        ORB_SLAM2::HPose orb2_hpose = orb2_->TrackStereo(frame_1, frame_2, frame_ts);
        cv::Mat cov_mat = covariance_scaling_factor_ * orb2_->GetCurrentCovarianceMatrix(true);
        orb2_pose = hpose_to_pose(
          orb2_hpose,
          link_namespace_ + "orb2_odom",
          frame_ts_ros,
          cov_mat);
      } else {
        // Should never happen
        RCLCPP_FATAL(
          this->get_logger(),
          "ORB_SLAM2DriverNode::orb2_thread_routine: Invalid system mode stored: '%s'",
          mode_str_.c_str());
        throw std::runtime_error(
                "ORB_SLAM2DriverNode::orb2_thread_routine: Invalid system mode stored: " +
                mode_str_);
      }

      // Check tracking state
      if (orb2_->GetTrackingState() != ORB_SLAM2::Tracking::OK) {
        RCLCPP_ERROR(this->get_logger(), "Tracking lost");
        continue;
      }

      // Apply initial transformation
      Eigen::Isometry3d orb2_iso = orb2_pose.get_isometry();
      init_pose_lock_.lock();
      Eigen::EulerAnglesXYZd init_rpy(
        init_pose_.get_rpy().alpha(),
        init_pose_.get_rpy().beta(),
        0.0);
      init_pose_lock_.unlock();
      Eigen::Isometry3d init_iso = Eigen::Isometry3d::Identity();
      init_iso.rotate(init_rpy.toRotationMatrix());
      Eigen::Isometry3d orb2_corrected_iso = init_iso * orb2_iso;
      orb2_pose.set_position(orb2_corrected_iso.translation());
      orb2_pose.set_attitude(Eigen::Quaterniond(orb2_corrected_iso.rotation()));

      // Get base_link pose
      PoseKit::Pose base_link_pose = orb2_pose;
      try {
        tf_lock_.lock();
        base_link_pose.track_parent(odom_to_camera_odom_);
        tf_lock_.unlock();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ORB_SLAM2DriverNode::orb2_thread_routine: base_link_pose::track_parent: %s",
          e.what());
        tf_lock_.unlock();
      }

      // Publish pose messages
      base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
      pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());
      rviz_base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
      rviz_pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());
    }
  }
}

} // namespace ORB_SLAM2Driver
