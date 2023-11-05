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
next:
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

    // Apply mounting correction if required, or global_frame -> orb2_map
    if (global_frame_id_.empty()) {
      if (set_gravity_as_origin_) {
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
      }
    } else {
      TransformStamped global_to_orb2_map{};
      rclcpp::Time tf_time(frame_ts_ros);
      while (true) {
        try {
          global_to_orb2_map = tf_buffer_->lookupTransform(
            global_frame_id_,
            orb2_map_frame_,
            tf_time,
            tf2::durationFromSec(0.1));
          break;
        } catch (const tf2::ExtrapolationException & e) {
          // Just get the latest
          tf_time = rclcpp::Time{};
        } catch (const tf2::TransformException & e) {
          RCLCPP_ERROR(
            this->get_logger(),
            "ORB_SLAM2DriverNode::tracking_thread_routine: TF exception: %s",
            e.what());
          goto next;
        }
      }
      Eigen::Isometry3d orb2_iso = orb2_pose.get_isometry();
      Eigen::Isometry3d global_to_orb2_map_iso = tf2::transformToEigen(global_to_orb2_map);
      Eigen::Isometry3d orb2_corrected_iso = global_to_orb2_map_iso * orb2_iso;
      orb2_pose.set_position(orb2_corrected_iso.translation());
      orb2_pose.set_attitude(Eigen::Quaterniond(orb2_corrected_iso.rotation()));
      orb2_pose.set_frame_id(global_frame_id_);
    }

    // Get body_frame -> orb2_link tf
    TransformStamped base_link_to_orb2{};
    rclcpp::Time tf_time(frame_ts_ros);
    while (true) {
      try {
        base_link_to_orb2 = tf_buffer_->lookupTransform(
          body_frame_,
          orb2_frame_,
          tf_time,
          tf2::durationFromSec(0.1));
        break;
      } catch (const tf2::ExtrapolationException & e) {
        // Just get the latest
        tf_time = rclcpp::Time{};
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "ORB_SLAM2DriverNode::tracking_thread_routine: TF exception: %s",
          e.what());
        goto next;
      }
    }

    // Get body pose
    PoseKit::Pose base_link_pose = orb2_pose;
    std::string new_frame_id;
    if (global_frame_id_.empty()) {
      new_frame_id = odom_frame_;
    } else {
      new_frame_id = global_frame_id_;
    }
    base_link_pose.rigid_transform(base_link_to_orb2, new_frame_id);

    // Publish pose messages
    base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
    pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());
    rviz_base_link_pose_pub_->publish(base_link_pose.to_pose_with_covariance_stamped());
    rviz_pose_pub_->publish(orb2_pose.to_pose_with_covariance_stamped());

    // Publish tf local/global frame -> base_link
    if (publish_tf_) {
      TransformStamped tf_msg{};
      tf_msg.header.set__frame_id(global_frame_id_.empty() ? odom_frame_ : global_frame_id_);
      tf_msg.header.set__stamp(base_link_pose.get_header().stamp);
      tf_msg.set__child_frame_id(body_frame_);

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

    // Publish map
    if (publish_map_) {
      // Retrieve the map from the system
      std::shared_ptr<Eigen::MatrixXf> map_points = orb2_->GetMap();
      rclcpp::Time pc_ts = this->get_clock()->now();

      // Fill and publish point cloud message
      PointCloud2 pc_msg{};
      sensor_msgs::PointCloud2Modifier pc_modifier(pc_msg);
      pc_msg.header.set__frame_id(global_frame_id_.empty() ? orb2_odom_frame_ : orb2_map_frame_);
      pc_msg.header.set__stamp(pc_ts);
      pc_msg.set__height(1);
      pc_msg.set__width(map_points->cols());
      pc_msg.set__is_bigendian(false);
      pc_msg.set__is_dense(true);
      pc_modifier.setPointCloud2Fields(
        3,
        "x", 1, PointField::FLOAT32,
        "y", 1, PointField::FLOAT32,
        "z", 1, PointField::FLOAT32);
      sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
      for (Eigen::Index i = 0; i < map_points->cols(); i++) {
        if (!global_frame_id_.empty() || !set_gravity_as_origin_) {
          *iter_x = (*map_points)(0, i);
          *iter_y = (*map_points)(1, i);
          *iter_z = (*map_points)(2, i);
        } else {
          Eigen::Isometry3d point_iso = Eigen::Isometry3d::Identity();
          point_iso.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(
            (*map_points)(0, i),
            (*map_points)(1, i),
            (*map_points)(2, i));

          init_pose_lock_.lock();
          Eigen::Isometry3d corrected_point_iso = init_pose_.get_isometry() * point_iso;
          init_pose_lock_.unlock();

          *iter_x = corrected_point_iso.translation().x();
          *iter_y = corrected_point_iso.translation().y();
          *iter_z = corrected_point_iso.translation().z();
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      rviz_map_pub_->publish(pc_msg);
    }

    first_run = false;
  }
}

} // namespace ORB_SLAM2Driver
