/**
 * ORB-SLAM2 driver node service callbacks.
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
      init_orbslam2();
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
    }
  }
  resp->set__success(true);
  resp->set__message("");
}

} // namespace ORB_SLAM2Driver
