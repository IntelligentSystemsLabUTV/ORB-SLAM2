/**
 * ORB-SLAM2 driver node utility routines.
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
 * @brief Initializes the ORB-SLAM2 system instance.
 *
 * @return True if initialization succeeded, false otherwise.
 *
 * @throws RuntimeError if something fails.
 */
bool ORB_SLAM2DriverNode::init_orbslam2()
{
  // Subscribe to camera orientation topic
  if (global_frame_id_.empty() && set_gravity_as_origin_) {
    camera_imu_sub_ = this->create_subscription<Imu>(
      camera_orientation_topic_,
      DUAQoS::get_datum_qos(),
      std::bind(
        &ORB_SLAM2DriverNode::camera_imu_callback,
        this,
        std::placeholders::_1));
  }

  // Initialize synchronization primitives
  if (sem_init(&tracking_sem_1_, 0, 1) ||
    sem_init(&tracking_sem_2_, 0, 0))
  {
    char err_msg_buf[100] = {};
    char * err_msg = strerror_r(errno, err_msg_buf, 100);
    throw std::runtime_error(
            "ORB_SLAM2DriverNode::init_orbslam2: Failed to initialize semaphores: " +
            std::string(err_msg));
  }

  // Subscribe to camera topics
  int64_t depth = this->get_parameter("subscriber_depth").as_int();
  if (mode_str_ == "STEREO") {
    camera_1_sub_ = std::make_shared<image_transport::SubscriberFilter>();
    camera_2_sub_ = std::make_shared<image_transport::SubscriberFilter>();

    camera_1_sub_->subscribe(
      this,
      camera_topic_1_,
      transport_,
      DUAQoS::get_image_qos(depth).get_rmw_qos_profile());
    camera_2_sub_->subscribe(
      this,
      camera_topic_2_,
      transport_,
      DUAQoS::get_image_qos(depth).get_rmw_qos_profile());

    stereo_sync_ = std::make_shared<ImageSynchronizer>(
      ImageSyncPolicy(depth),
      *camera_1_sub_,
      *camera_2_sub_);
    stereo_sync_->registerCallback(
      std::bind(
        &ORB_SLAM2DriverNode::stereo_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::init_orbslam2: Invalid system mode stored: '%s'",
      mode_str_.c_str());
    return false;
  }

  // Initialize frame_viewer publisher
  if (frame_view_) {
    frame_drawer_pub_ = std::make_shared<TheoraWrappers::Publisher>(
      this,
      "~/frame_viewer",
      DUAQoS::Visualization::get_image_qos().get_rmw_qos_profile());
  }

  // Initialize ORB-SLAM2 system
  orb2_ = std::make_shared<ORB_SLAM2::System>(
    vocabulary_path_,
    orb2_config_path_,
    mode_,
    display_,
    save_map_,
    false);
  if (localization_.load(std::memory_order_acquire)) {
    orb2_->ActivateLocalizationMode();
  }

  // Spawn tracking thread
  running_.store(true, std::memory_order_release);
  tracking_thread_ = std::thread(
    &ORB_SLAM2DriverNode::tracking_thread_routine,
    this);
  if (tracking_cpu_ != -1) {
    cpu_set_t tracking_cpu_set;
    CPU_ZERO(&tracking_cpu_set);
    CPU_SET(tracking_cpu_, &tracking_cpu_set);
    if (pthread_setaffinity_np(
        tracking_thread_.native_handle(),
        sizeof(cpu_set_t),
        &tracking_cpu_set))
    {
      char err_msg_buf[100] = {};
      char * err_msg = strerror_r(errno, err_msg_buf, 100);
      throw std::runtime_error(
              "ORB_SLAM2DriverNode::init_orbslam2: Failed to configure tracking thread: " +
              std::string(err_msg));
    }
  }

  RCLCPP_WARN(this->get_logger(), "Tracking thread started");

  return true;
}

/**
 * @brief Stops the ORB-SLAM2 thread and system.
 */
void ORB_SLAM2DriverNode::fini_orbslam2()
{
  stereo_sync_.reset();
  camera_1_sub_.reset();
  camera_2_sub_.reset();
  camera_imu_sub_.reset();

  running_.store(false, std::memory_order_release);
  sem_post(&tracking_sem_1_);
  sem_post(&tracking_sem_2_);
  tracking_thread_.join();

  orb2_->Shutdown();
  orb2_.reset();

  frame_drawer_pub_.reset();

  sem_destroy(&tracking_sem_1_);
  sem_destroy(&tracking_sem_2_);

  RCLCPP_WARN(this->get_logger(), "Tracking thread stopped");
}

/**
 * @brief Converts an Image message to a cv::Mat.
 *
 * @param msg The Image message to convert.
 * @return The converted cv::Mat.
 */
cv::Mat ORB_SLAM2DriverNode::image_to_cv_mat(const Image::ConstSharedPtr & msg)
{
  cv::Mat new_frame(
    msg->height,
    msg->width,
    CV_8UC3);
  std::memcpy(new_frame.data, msg->data.data(), msg->data.size());

  return new_frame;
}

/**
 * @brief Converts an HPose to a PoseKit::Pose.
 *
 * @param hpose The HPose to convert.
 * @param frame_id The frame ID to set in the PoseKit::Pose.
 * @param ts The timestamp to set in the PoseKit::Pose.
 * @param cov The covariance to set in the PoseKit::Pose.
 *
 * @return The converted PoseKit::Pose.
 *
 * @throws RuntimeError if the covariance matrix is not 6x6.
 */
PoseKit::Pose ORB_SLAM2DriverNode::hpose_to_pose(
  const ORB_SLAM2::HPose & hpose,
  std::string & frame_id,
  const Time & ts,
  const cv::Mat & cov)
{
  if (cov.rows != 6 || cov.cols != 6) {
    throw std::runtime_error("ORB_SLAM2DriverNode::hpose_to_pose: Covariance matrix must be 6x6");
  }

  Eigen::Vector3d p(
    hpose.GetTranslation()[0],
    hpose.GetTranslation()[1],
    hpose.GetTranslation()[2]);
  Eigen::Quaterniond q(
    hpose.GetRotation()[0],
    hpose.GetRotation()[1],
    hpose.GetRotation()[2],
    hpose.GetRotation()[3]);

  Header header;
  header.set__frame_id(frame_id);
  header.set__stamp(ts);

  std::array<double, 36> cov_array;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      cov_array[i * 6 + j] = double(cov.at<float>(i, j));
    }
  }

  return PoseKit::Pose(p, q, header, cov_array);
}

/**
 * @brief Validates the frame_view parameter.
 *
 * @param p The parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ORB_SLAM2DriverNode::validate_frame_view(const rclcpp::Parameter & p)
{
  if (running_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::validate_frame_view: Cannot change frame_view while the system is running");
    return false;
  }

  frame_view_ = p.as_bool();
  return true;
}

/**
 * @brief Validates the global_frame_id parameter.
 *
 * @param p The parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ORB_SLAM2DriverNode::validate_global_frame_id(const rclcpp::Parameter & p) {
  if (running_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::validate_global_frame_id: Cannot change global_frame_id while the system is running");
    return false;
  }

  global_frame_id_ = p.as_string();
  return true;
}

/**
 * @brief Validates the mode parameter.
 *
 * @param p The parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ORB_SLAM2DriverNode::validate_mode(const rclcpp::Parameter & p)
{
  std::string mode_str = p.as_string();

  if (mode_str == "MONOCULAR") {
    mode_ = ORB_SLAM2::System::eSensor::MONOCULAR;
  } else if (mode_str == "STEREO") {
    mode_ = ORB_SLAM2::System::eSensor::STEREO;
  } else if (mode_str == "RGBD") {
    mode_ = ORB_SLAM2::System::eSensor::RGBD;
  } else if (mode_str == "IRD") {
    mode_ = ORB_SLAM2::System::eSensor::RGBD;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::validate_mode: Invalid mode '%s'",
      mode_str.c_str());
    return false;
  }

  mode_str_ = mode_str;
  return true;
}

/**
 * @brief Validates the save_map parameter.
 *
 * @param p The parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ORB_SLAM2DriverNode::validate_save_map(const rclcpp::Parameter & p)
{
  if (running_.load(std::memory_order_acquire)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::validate_save_map: Cannot change save_map while the system is running");
    return false;
  }

  save_map_ = p.as_bool();
  return true;
}

/**
 * @brief Validates the transport parameter.
 *
 * @param p The parameter to validate.
 * @return True if the parameter is valid, false otherwise.
 */
bool ORB_SLAM2DriverNode::validate_transport(const rclcpp::Parameter & p)
{
  std::string transport_str = p.as_string();

  if (transport_str == "raw" || transport_str == "compressed") {
    transport_ = transport_str;
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "ORB_SLAM2DriverNode::validate_transport: Invalid transport '%s'",
      transport_str.c_str());
    return false;
  }

  return true;
}

/**
 * @brief Converts a frame into an Image message.
 *
 * @param frame cv::Mat storing the frame.
 * @return Shared pointer to a new Image message.
 */
Image::SharedPtr ORB_SLAM2DriverNode::frame_to_msg(cv::Mat & frame)
{
  auto ros_image = std::make_shared<Image>();

  // Set frame-relevant fields
  ros_image->set__width(frame.cols);
  ros_image->set__height(frame.rows);
  ros_image->set__step(frame.cols * frame.elemSize());
  ros_image->set__is_bigendian(false);
  ros_image->set__encoding(sensor_msgs::image_encodings::BGR8);
  ros_image->header.set__frame_id(link_namespace_ + "orb2_link");

  // Copy frame data
  size_t size = ros_image->step * frame.rows;
  ros_image->data.resize(size);
  memcpy(ros_image->data.data(), frame.data, size);

  return ros_image;
}

} // namespace ORB_SLAM2Driver
