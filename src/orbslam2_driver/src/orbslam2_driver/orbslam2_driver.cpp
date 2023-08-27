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
  map_frame_ = "map";
  odom_frame_ = link_namespace_ + "odom";
  orb2_odom_frame_ = link_namespace_ + "orb2_odom";
  orb2_map_frame_ = link_namespace_ + "orb2_map";
  odom_to_camera_odom_.header.set__frame_id(odom_frame_);
  odom_to_camera_odom_.set__child_frame_id(orb2_odom_frame_);
  map_to_camera_odom_.header.set__frame_id(map_frame_);
  map_to_camera_odom_.set__child_frame_id(orb2_odom_frame_);
  base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
  base_link_to_camera_.set__child_frame_id(link_namespace_ + "orb2_link");
  global_to_orb2_map_.header.set__frame_id(global_frame_id_);
  global_to_orb2_map_.set__child_frame_id(orb2_map_frame_);

  // Initlaize TF timer
  tf_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(
      &ORB_SLAM2DriverNode::tf_timer_callback,
      this));
}

} // namespace ORB_SLAM2Driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ORB_SLAM2Driver::ORB_SLAM2DriverNode)
