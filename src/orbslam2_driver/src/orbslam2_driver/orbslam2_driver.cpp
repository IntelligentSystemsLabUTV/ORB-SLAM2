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
  init_sync_primitives();
  init_parameters();
  init_publishers();
  init_services();
  init_tf_listeners();

  if (autostart_) {
    init_orbslam2();
    running_.store(true, std::memory_order_release);
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

  sem_destroy(&orb2_thread_sem_1_);
  sem_destroy(&orb2_thread_sem_2_);
}

/**
 * @brief Routine to initialize atomic members.
 */
void ORB_SLAM2DriverNode::init_atomics()
{
  running_.store(false, std::memory_order_release);
}

/**
 * @brief Routine to initialize synchronization primitives.
 *
 * @throws RuntimeError if some initialization fails.
 */
void ORB_SLAM2DriverNode::init_sync_primitives()
{
  if (sem_init(&orb2_thread_sem_1_, 0, 1) ||
    sem_init(&orb2_thread_sem_2_, 0, 0))
  {
    perror("sem_init");
    throw std::runtime_error(
            "ORB_SLAM2DriverNode::init_sync_primitives: Failed to initialize semaphores.");
  }
}

/**
 * @brief Routine to initialize publishers.
 */
void ORB_SLAM2DriverNode::init_publishers()
{
  // pose
  pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "~/pose",
    DUAQoS::get_datum_qos());

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
}

/**
 * @brief Initializes TF listeners and their timer.
 */
void ORB_SLAM2DriverNode::init_tf_listeners()
{
  // Initialize TF buffers and listeners
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize local data
  map_frame_ = "map";
  odom_frame_ = link_namespace_ + "odom";
  orb2_odom_frame_ = link_namespace_ + "orb2_odom";
  odom_to_camera_odom_.header.set__frame_id(odom_frame_);
  odom_to_camera_odom_.set__child_frame_id(orb2_odom_frame_);
  map_to_camera_odom_.header.set__frame_id(map_frame_);
  map_to_camera_odom_.set__child_frame_id(orb2_odom_frame_);
  base_link_to_camera_.header.set__frame_id(link_namespace_ + "base_link");
  base_link_to_camera_.set__child_frame_id(link_namespace_ + "orb2_link");

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
