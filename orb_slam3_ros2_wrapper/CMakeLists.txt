cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2_wrapper)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)

# ------------------------
# RGBD Executable
# ------------------------
add_executable(rgbd
  src/type_conversion.cpp
  src/orb_slam3_interface.cpp
  src/rgbd/rgbd-slam-node.cpp
  src/rgbd/rgbd.cpp
)

ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin tf2_ros tf2_eigen slam_msgs pcl_ros pcl_conversions PCL nav_msgs)
target_link_libraries(rgbd ${PCL_LIBRARIES})

install(TARGETS rgbd
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------
# Install launch and params
# ------------------------
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------
# Map Merger Executable
# ------------------------
# Build main executable
add_executable(map_merger
  src/map_merger/map_merger.cpp         # Implementation of map_merger class
  src/map_merger/map_merger_main.cpp    # Main file to run the map_merger
)

# Include directories
include_directories(
  include
  src/map_merger # Add this line to include the map_merger directory
  ${ORB_SLAM3_ROOT_DIR}/include #Add the orb slam 3 include directory.
  ${PCL_INCLUDE_DIRS} #Add the PCL include directory.
)

ament_target_dependencies(map_merger
                          rclcpp
                          sensor_msgs
                          cv_bridge
                          message_filters
                          ORB_SLAM3
                          Pangolin
                          tf2_ros
                          tf2_eigen
                          slam_msgs
                          pcl_ros
                          pcl_conversions
                          PCL
                          nav_msgs
)

target_link_libraries(map_merger
  ${PCL_LIBRARIES}
  ${ORB_SLAM3_LIBRARIES}
)

# Install targets
# Install the map_merger executable
install(TARGETS map_merger
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()