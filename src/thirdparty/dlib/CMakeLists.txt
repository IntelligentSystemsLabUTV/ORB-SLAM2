cmake_minimum_required(VERSION 3.8)
project(dlib)

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
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# dlib library configuration
add_library(dlib SHARED
  src/Random.cpp
  src/Timestamp.cpp)
target_compile_features(dlib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(dlib PRIVATE "dlib_BUILDING_LIBRARY")
target_include_directories(dlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Library installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS dlib
  EXPORT dlibTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(dlibTargets HAS_LIBRARY_TARGET)

ament_package()
