/**
 * ORB-SLAM2 driver node initialization routines.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 30, 2023
 */

#include <orbslam2_driver/orbslam2_driver.hpp>

namespace ORB_SLAM2Driver
{

/**
 * @brief Node constructor.
 */
ORB_SLAM2DriverNode::ORB_SLAM2DriverNode(const rclcpp::NodeOptions & opts)
: NodeBase("orbslam2_driver", opts, true)
{
  init_atomics();
  init_parameters();
  init_publishers();
  init_services();
  init_tf2();

  if (start_localization_) {
    localization_.store(true, std::memory_order_release);
  } else {
    localization_.store(false, std::memory_order_release);
  }

  if (autostart_) {
    init_orbslam2();
  }

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Node destructor.
 */
ORB_SLAM2DriverNode::~ORB_SLAM2DriverNode()
{
  if (running_.load(std::memory_order_acquire)) {
    fini_orbslam2();
    running_.store(false, std::memory_order_release);
  }

  stereo_sync_.reset();
  camera_1_sub_.reset();
  camera_2_sub_.reset();
}

/**
 * @brief Routine to initialize atomic members.
 */
void ORB_SLAM2DriverNode::init_atomics()
{
  running_.store(false, std::memory_order_release);
}

/**
 * @brief Routine to initialize publishers.
 */
void ORB_SLAM2DriverNode::init_publishers()
{
  // base_link_pose
  base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/base_link_pose",
    DUAQoS::get_datum_qos());

  // loops
  loops_pub_ = this->create_publisher<UInt64>(
    "~/loops",
    DUAQoS::get_datum_qos());

  // pose
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/pose",
    DUAQoS::get_datum_qos());

  // rviz/map
  rviz_map_pub_ = this->create_publisher<PointCloud2>(
    "~/rviz/map",
    DUAQoS::Visualization::get_scan_qos());

  // rviz/base_link_pose
  rviz_base_link_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/base_link_pose",
    DUAQoS::Visualization::get_datum_qos());

  // rviz/pose
  rviz_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/rviz/pose",
    DUAQoS::Visualization::get_datum_qos());
}

/**
 * @brief Routine to initialize services.
 */
void ORB_SLAM2DriverNode::init_services()
{
  // enable
  enable_srv_ = this->create_service<SetBool>(
    "~/enable",
    std::bind(
      &ORB_SLAM2DriverNode::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  // localization
  localization_srv_ = this->create_service<SetBool>(
    "~/localization_mode",
    std::bind(
      &ORB_SLAM2DriverNode::localization_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  // reset
  reset_srv_ = this->create_service<Trigger>(
    "~/reset",
    std::bind(
      &ORB_SLAM2DriverNode::reset_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

/**
 * @brief Initializes TF listeners and broadcasters.
 */
void ORB_SLAM2DriverNode::init_tf2()
{
  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize local data
  body_frame_ = link_namespace_ + body_frame_id_;
  global_frame_ = "map";
  odom_frame_ = link_namespace_ + "odom";
  orb2_frame_ = link_namespace_ + "orb2_link";
  orb2_odom_frame_ = link_namespace_ + "orb2_odom";
  orb2_map_frame_ = link_namespace_ + "orb2_map";
}

} // namespace ORB_SLAM2Driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ORB_SLAM2Driver::ORB_SLAM2DriverNode)
