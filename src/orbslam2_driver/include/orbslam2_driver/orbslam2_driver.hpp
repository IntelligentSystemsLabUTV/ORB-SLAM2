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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sched.h>
#include <semaphore.h>

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

#include <theora_wrappers/publisher.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace builtin_interfaces::msg;
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
  void init_parameters();
  void init_publishers();
  void init_services();
  void init_tf2();

  /* TF listener, broadcaster, and related data. */
  std::string body_frame_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string orb2_frame_;
  std::string orb2_odom_frame_;
  std::string orb2_map_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Topic subscriptions. */
  rclcpp::Subscription<Imu>::SharedPtr camera_imu_sub_;

  /* Topic subscription callbacks. */
  void camera_imu_callback(const Imu::SharedPtr msg);

  /* Topic publishers. */
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr base_link_pose_pub_;
  rclcpp::Publisher<UInt64>::SharedPtr loops_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr rviz_map_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr rviz_pose_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr rviz_base_link_pose_pub_;

  /* FrameDrawer publisher */
  std::shared_ptr<TheoraWrappers::Publisher> frame_drawer_pub_;

  /* Service servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<SetBool>::SharedPtr localization_srv_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;

  /* Service callbacks. */
  void enable_callback(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr res);
  void localization_callback(
    SetBool::Request::SharedPtr req,
    SetBool::Response::SharedPtr res);
  void reset_callback(
    Trigger::Request::SharedPtr req,
    Trigger::Response::SharedPtr res);

  /* image_transport subscribers and synchronizers. */
  std::shared_ptr<image_transport::SubscriberFilter> camera_1_sub_;
  std::shared_ptr<image_transport::SubscriberFilter> camera_2_sub_;
  std::shared_ptr<ImageSynchronizer> stereo_sync_;

  /* Camera callbacks. */
  void stereo_callback(
    const Image::ConstSharedPtr & camera_1_msg,
    const Image::ConstSharedPtr & camera_2_msg);

  /* Node parameters. */
  bool autostart_;
  std::string body_frame_id_;
  std::string camera_orientation_topic_;
  std::string camera_topic_1_;
  std::string camera_topic_2_;
  double covariance_scaling_factor_;
  bool display_;
  bool frame_view_;
  std::string global_frame_id_;
  std::string link_namespace_;
  ORB_SLAM2::System::eSensor mode_ = ORB_SLAM2::System::eSensor::STEREO;
  std::string mode_str_;
  bool preconvert_frames_;
  bool publish_map_;
  bool publish_tf_;
  std::string orb2_config_path_;
  bool save_map_;
  bool set_gravity_as_origin_;
  bool start_localization_;
  int64_t tracking_cpu_;
  std::string transport_;
  std::string vocabulary_path_;
  bool verbose_;

  /* Node parameters validators. */
  bool validate_frame_view(const rclcpp::Parameter & p);
  bool validate_global_frame_id(const rclcpp::Parameter & p);
  bool validate_mode(const rclcpp::Parameter & p);
  bool validate_save_map(const rclcpp::Parameter & p);
  bool validate_transport(const rclcpp::Parameter & p);

  /* Worker thread, routine, and data. */
  std::thread tracking_thread_;
  void tracking_thread_routine();
  sem_t tracking_sem_1_;
  sem_t tracking_sem_2_;
  cv::Mat camera_1_frame_;
  cv::Mat camera_2_frame_;
  Time frame_ts_;

  /* Internal state variables. */
  std::atomic<bool> running_;
  PoseKit::Pose init_pose_{};
  std::mutex init_pose_lock_;
  std::atomic<bool> localization_;

  /* ORB-SLAM2 system data. */
  std::shared_ptr<ORB_SLAM2::System> orb2_ = nullptr;

  /* Auxiliary routines. */
  bool init_orbslam2();
  void fini_orbslam2();
  cv::Mat image_to_cv_mat(const Image::ConstSharedPtr & msg);
  PoseKit::Pose hpose_to_pose(
    const ORB_SLAM2::HPose & hpose,
    std::string & frame_id,
    const Time & ts,
    const cv::Mat & cov = cv::Mat::zeros(6, 6, CV_32F));
  Image::SharedPtr frame_to_msg(cv::Mat & frame);
};

} // namespace ORB_SLAM2Driver

#endif // ORBSLAM2_DRIVER_HPP
