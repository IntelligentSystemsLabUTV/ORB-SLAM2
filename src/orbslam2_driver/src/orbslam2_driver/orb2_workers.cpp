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
 */
void ORB_SLAM2DriverNode::tracking_thread_routine()
{
  cv::Mat frame_1, frame_2;
  Time frame_ts_ros{};
  double frame_ts = 0.0;
  ORB_SLAM2::Tracking::eTrackingState last_tracking_state = ORB_SLAM2::Tracking::OK;
  uint32_t loop_cnt = 0;
  bool first_run = true;

  while (true) {
    // Get input data from the subscriber
    sem_wait(&tracking_sem_2_);
    frame_1 = camera_1_frame_.clone();
    frame_2 = camera_2_frame_.clone();
    frame_ts_ros = frame_ts_;
    sem_post(&tracking_sem_1_);
    frame_ts = double(frame_ts_ros.sec) * 1e9 + double(frame_ts_ros.nanosec);

    // Check if the system is still running
    if (!running_.load(std::memory_order_acquire)) {
      break;
    }

    // Execute tracking
    PoseKit::Pose orb2_pose{};
    if (mode_str_ == "STEREO") {
      ORB_SLAM2::HPose orb2_hpose = orb2_->TrackStereo(frame_1, frame_2, frame_ts);
      cv::Mat cov_mat = covariance_scaling_factor_ * orb2_->GetCurrentCovarianceMatrix(true);
      orb2_pose = hpose_to_pose(
        orb2_hpose,
        global_frame_id_.empty() ? orb2_odom_frame_ : orb2_map_frame_,
        frame_ts_ros,
        cov_mat);
    } else {
      // Should never happen
      RCLCPP_FATAL(
        this->get_logger(),
        "ORB_SLAM2DriverNode::tracking_thread_routine: Invalid system mode stored: '%s'",
        mode_str_.c_str());
      continue;
    }

    // Check tracking state
    switch (orb2_->GetTrackingState()) {
      case ORB_SLAM2::Tracking::SYSTEM_NOT_READY:
        RCLCPP_WARN(this->get_logger(), "System not ready");
        continue;
      case ORB_SLAM2::Tracking::NO_IMAGES_YET:
        RCLCPP_WARN(this->get_logger(), "No images yet");
        continue;
      case ORB_SLAM2::Tracking::NOT_INITIALIZED:
        RCLCPP_WARN(this->get_logger(), "Not initialized");
        continue;
      case ORB_SLAM2::Tracking::OK:
        if (last_tracking_state != ORB_SLAM2::Tracking::OK) {
          last_tracking_state = ORB_SLAM2::Tracking::OK;
          RCLCPP_WARN(this->get_logger(), "Relocalized");
        }
        break;
      case ORB_SLAM2::Tracking::LOST:
        if (last_tracking_state != ORB_SLAM2::Tracking::LOST) {
          last_tracking_state = ORB_SLAM2::Tracking::LOST;
          RCLCPP_ERROR(this->get_logger(), "Tracking lost");
        }
        continue;
      default:
        // Should never happen
        RCLCPP_FATAL(
          this->get_logger(),
          "ORB_SLAM2DriverNode::tracking_thread_routine: Invalid tracking state");
        break;
    }

    // Apply initial transformation, or global_frame -> orb2_map
    if (global_frame_id_.empty()) {
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
    } else {
      Eigen::Isometry3d orb2_iso = orb2_pose.get_isometry();
      tf_lock_.lock();
      Eigen::Isometry3d global_to_orb2_map_iso = tf2::transformToEigen(global_to_orb2_map_);
      tf_lock_.unlock();
      Eigen::Isometry3d orb2_corrected_iso = global_to_orb2_map_iso * orb2_iso;
      orb2_pose.set_position(orb2_corrected_iso.translation());
      orb2_pose.set_attitude(Eigen::Quaterniond(orb2_corrected_iso.rotation()));
      orb2_pose.set_frame_id(global_frame_id_);
    }

    // Get base_link pose
    PoseKit::Pose base_link_pose = orb2_pose;
    if (global_frame_id_.empty()) {
      try {
        tf_lock_.lock();
        base_link_pose.track_parent(odom_to_camera_odom_);
        tf_lock_.unlock();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ORB_SLAM2DriverNode::tracking_thread_routine: base_link_pose::track_parent: %s",
          e.what());
        tf_lock_.unlock();
      }
    } else {
      // For a moment, we pretend that the 'frame_id' fields denote the frame id of the frame that the pose
      // tracks, and not the one w.r.t. they are expressed, just to be able to apply track_parent
      base_link_pose.set_frame_id(link_namespace_ + "orb2_link");
      try {
        tf_lock_.lock();
        base_link_pose.track_parent(base_link_to_camera_);
        tf_lock_.unlock();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ORB_SLAM2DriverNode::tracking_thread_routine: base_link_pose::track_parent: %s",
          e.what());
        tf_lock_.unlock();
      }
      base_link_pose.set_frame_id(global_frame_id_);
    }

    // Publish pose messages
    base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
    pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());
    rviz_base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
    rviz_pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());

    // Publish tf local/global frame -> base_link
    if (publish_tf_) {
      TransformStamped tf_msg{};
      tf_msg.header.set__frame_id(global_frame_id_.empty() ? orb2_odom_frame_ : global_frame_id_);
      tf_msg.header.set__stamp(base_link_pose.get_header().stamp);

      tf_msg.set__child_frame_id(link_namespace_ + "base_link");

      tf_msg.transform.translation.set__x(base_link_pose.get_position().x());
      tf_msg.transform.translation.set__y(base_link_pose.get_position().y());
      tf_msg.transform.translation.set__z(base_link_pose.get_position().z());

      tf_msg.transform.rotation.set__x(base_link_pose.get_attitude().x());
      tf_msg.transform.rotation.set__y(base_link_pose.get_attitude().y());
      tf_msg.transform.rotation.set__z(base_link_pose.get_attitude().z());
      tf_msg.transform.rotation.set__w(base_link_pose.get_attitude().w());

      tf_broadcaster_->sendTransform(tf_msg);
    }

    // Publish loops count
    uint64_t curr_loops = orb2_->GetLoopCount();
    if (curr_loops > loop_cnt) {
      if (first_run) {
        RCLCPP_INFO(this->get_logger(), "Loaded map with %ld loops closed", curr_loops);
      } else {
        RCLCPP_WARN(this->get_logger(), "Closed loop: %ld", curr_loops);
      }

      loop_cnt = curr_loops;
      UInt64 loops_msg{};
      loops_msg.set__data(curr_loops);
      loops_pub_->publish(loops_msg);
    }

    // Publish FrameDrawer frame
    if (frame_view_ && frame_drawer_pub_->getNumSubscribers() > 0) {
      cv::Mat frame_drawer_frame = orb2_->GetFrameDrawerFrame();
      Image::SharedPtr frame_drawer_msg = frame_to_msg(frame_drawer_frame);
      frame_drawer_msg->header.set__stamp(orb2_pose.get_header().stamp);
      frame_drawer_pub_->publish(frame_drawer_msg);
    }

    first_run = false;
  }
}

} // namespace ORB_SLAM2Driver
