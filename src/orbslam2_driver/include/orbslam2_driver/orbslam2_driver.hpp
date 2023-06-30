/**
 * ORB-SLAM2 Driver node definition.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2023
 */

#ifndef ORBSLAM2_DRIVER_HPP
#define ORBSLAM2_DRIVER_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <semaphore.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <ORB_SLAM2/HPose.h>
#include <ORB_SLAM2/System.h>

#include <rclcpp/rclcpp.hpp>

#include <pose_kit/pose.hpp>

#include <dua_node/dua_node.hpp>
#include <dua_qos/dua_qos.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dua_interfaces/msg/point_cloud2_with_roi.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace dua_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace visualization_msgs::msg;

using namespace std_srvs::srv;

typedef message_filters::sync_policies::ExactTime<Image, Image> ImageSyncPolicy;
typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSynchronizer;

namespace ORB_SLAM2Driver
{

/**
 * Embeds and manages the ORB-SLAM2 algorithm.
 */
class ORB_SLAM2DriverNode : public DUANode::NodeBase
{
public:
  ORB_SLAM2DriverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());
  virtual ~ORB_SLAM2DriverNode();

private:
  /* Node initialization routines. */
  void init_atomics();
  void init_sync_primitives();
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_tf_listeners();
  void init_subscriptions();

  /* Node parameters. */
  bool autostart_;
  std::string camera_topic_1_;
  std::string camera_topic_2_;
  std::string link_namespace_;
  std::string orb2_config_path_;
  std::string vocabulary_path_;

  /* Worker thread, routine, and data. */
  std::thread orb2_thread_;
  void orb2_thread_routine();

  /* Internal state variables. */
  std::atomic<bool> is_running_;

  /* ORB-SLAM2 data. */

  /* Auxiliary routines. */
  void init_orbslam2();
  void fini_orbslam2();
  void hpose_to_pose(const ORB_SLAM2::HPose & hpose, PoseKit::Pose & pose);
};

} // namespace ORB_SLAM2Driver

#endif // ORBSLAM2_DRIVER_HPP
