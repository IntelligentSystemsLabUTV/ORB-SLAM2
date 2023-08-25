/**
 * ORB-SLAM2 driver node timers callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 1, 2023
 */

#define NOOP ((void)0)

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Updates tf2 transforms.
 */
void ORB_SLAM2DriverNode::tf_timer_callback()
{
  TransformStamped odom_to_camera_odom{}, map_to_camera_odom{}, global_to_orb2_map{};

  // Start listening
  // odom -> orb2_odom, base_link -> orb2_link (it's rigid)
  try {
    odom_to_camera_odom = tf_buffer_->lookupTransform(
      odom_frame_,
      orb2_odom_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    odom_to_camera_odom_ = odom_to_camera_odom;
    base_link_to_camera_ = odom_to_camera_odom;
    base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
    base_link_to_camera_.set__child_frame_id(link_namespace_ + "orb2_link");
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }

  // map -> orb2_odom
  try {
    map_to_camera_odom = tf_buffer_->lookupTransform(
      map_frame_,
      orb2_odom_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));

    tf_lock_.lock();
    map_to_camera_odom_ = map_to_camera_odom;
    tf_lock_.unlock();
  } catch (const tf2::TimeoutException & e) {
    NOOP;
  } catch (const tf2::TransformException & e) {
    RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
  }

  // global_frame -> orb2_map
  if (!global_frame_id_.empty()) {
    try {
      global_to_orb2_map = tf_buffer_->lookupTransform(
        global_frame_id_,
        orb2_map_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));

      tf_lock_.lock();
      global_to_orb2_map_ = global_to_orb2_map;
      tf_lock_.unlock();
    } catch (const tf2::TimeoutException & e) {
      NOOP;
    } catch (const tf2::TransformException & e) {
      RCLCPP_INFO(this->get_logger(), "TF exception: %s", e.what());
    }
  }
}

} // namespace ORB_SLAM2Driver
