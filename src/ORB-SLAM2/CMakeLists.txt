cmake_minimum_required(VERSION 3.8)
project(orbslam2)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Build options
option(JETSON_BUILD "Enables optimizations for Jetson platforms" OFF)
if(JETSON_BUILD)
  # Activates specific code paths in thirdparty dependencies
  add_definitions(-D__ANDROID__)
endif(JETSON_BUILD)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -D EIGEN_NO_DEPRECATED_WARNING -Wno-ignored-attributes -Wno-unused-result")
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Check C++14, C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14) # Added to let ORB-SLAM2 compile in Ubuntu 22.04
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14/C++11 support. Please use a different compiler.")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(dlib REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
find_package(fbow REQUIRED)
find_package(g2o REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Threads REQUIRED)

find_library(BOOST_SERIALIZATION boost_serialization)

find_package(OpenCV 4.2 QUIET)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.2 not found.")
endif()

# ORB-SLAM2 library configuration
add_library(orbslam2 SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/HPose.cpp)
target_include_directories(orbslam2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
target_link_libraries(orbslam2
  ${BOOST_SERIALIZATION}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  Threads::Threads)
ament_target_dependencies(orbslam2
  dlib
  fbow
  g2o)

# Build testers for datasets and specific configurations
# TODO

# Library installation
install(
  DIRECTORY include/
  DESTINATION include/ORB_SLAM2)
install(
  TARGETS orbslam2
  EXPORT orbslam2Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/ORB_SLAM2)

# Uncompress FBOW vocabulary
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/orb_mur.fbow")
  message(STATUS "Uncompressing FBOW vocabulary...")
  execute_process(
    COMMAND tar -xf "orb_mur.fbow.tar.gz"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary")
endif()

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

ament_export_targets(orbslam2Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  dlib
  Eigen3
  fbow
  g2o
  OpenCV
  Pangolin
  Threads)

ament_package()
