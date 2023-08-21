/**
 * ORB-SLAM2 driver node service callbacks.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * July 1, 2023
 */

#define UNUSED(arg) (void)(arg)

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Enable service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ORB_SLAM2DriverNode::enable_callback(
  const SetBool::Request::SharedPtr req,
  const SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = false;
    if (running_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      if (init_orbslam2()) {
        resp->set__success(true);
        resp->set__message("");
      } else {
        resp->set__success(false);
        resp->set__message("Failed to initialize ORB-SLAM2 system.");
      }
    }
  } else {
    bool expected = true;
    if (running_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      fini_orbslam2();
      resp->set__success(true);
      resp->set__message("");
    }
  }
}

/**
 * @brief Localization service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ORB_SLAM2DriverNode::localization_callback(
  const SetBool::Request::SharedPtr req,
  const SetBool::Response::SharedPtr resp)
{
  if (req->data) {
    bool expected = false;
    if (localization_.compare_exchange_strong(
        expected,
        true,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      orb2_->ActivateLocalizationMode();
      resp->set__success(true);
      resp->set__message("");
      RCLCPP_INFO(this->get_logger(), "Localization mode activated");
    }
  } else {
    bool expected = true;
    if (localization_.compare_exchange_strong(
        expected,
        false,
        std::memory_order_release,
        std::memory_order_acquire))
    {
      orb2_->DeactivateLocalizationMode();
      resp->set__success(true);
      resp->set__message("");
      RCLCPP_INFO(this->get_logger(), "Localization mode deactivated");
    }
  }
}

/**
 * @brief Reset service callback.
 *
 * @param req Service request.
 * @param resp Service response.
 */
void ORB_SLAM2DriverNode::reset_callback(
  const Trigger::Request::SharedPtr req,
  const Trigger::Response::SharedPtr resp)
{
  UNUSED(req);

  if (running_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(this->get_logger(), "ORB-SLAM2 system reset requested");
    orb2_->Reset();
    resp->set__success(true);
    resp->set__message("");
  } else {
    resp->set__success(false);
    resp->set__message("ORB-SLAM2 system not running");
  }
}

} // namespace ORB_SLAM2Driver
