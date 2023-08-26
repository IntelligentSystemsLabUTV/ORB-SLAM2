cmake_minimum_required(VERSION 3.8)
project(orbslam2_driver)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(orbslam2 REQUIRED)
find_package(params_manager REQUIRED)
find_package(pose_kit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(theora_wrappers REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/orbslam2_driver/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure ORB-SLAM2 Driver node source files
file(GLOB ORB2_DRIVER_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/orbslam2_driver/*.cpp")
list(APPEND ORB2_DRIVER_NODE_SRCS init_parameters.cpp)

# ORB-SLAM2 Driver node
add_library(orbslam2_driver SHARED ${ORB2_DRIVER_NODE_SRCS})
target_compile_definitions(orbslam2_driver PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(orbslam2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(orbslam2_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(orbslam2_driver
  Eigen3::Eigen
  ${OpenCV_LIBS}
  Threads::Threads)
ament_target_dependencies(orbslam2_driver
  builtin_interfaces
  dua_node
  dua_qos
  geometry_msgs
  image_transport
  message_filters
  orbslam2
  params_manager
  pose_kit
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_ros
  theora_wrappers
  visualization_msgs)
rclcpp_components_register_nodes(orbslam2_driver "ORB_SLAM2Driver::ORB_SLAM2DriverNode")

# ORB-SLAM2 Driver standalone application
add_executable(orbslam2_driver_app
  src/orbslam2_driver_app.cpp)
target_include_directories(orbslam2_driver_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(orbslam2_driver_app orbslam2_driver)
ament_target_dependencies(orbslam2_driver_app
  dua_app_management
  rclcpp)

# Install ORB-SLAM2 Driver component
install(TARGETS orbslam2_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install ORB-SLAM2 Driver app
install(TARGETS orbslam2_driver_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
