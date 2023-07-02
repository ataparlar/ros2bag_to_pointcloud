cmake_minimum_required(VERSION 3.8)
project(rosbag_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_DIR "/usr/local/share/pcl-1.13/PCLConfig.cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS})

add_executable(rosbag_to_pointcloud_exe
        src/rosbag_to_pointcloud.cpp
        include/rosbag_to_pointcloud/rosbag_to_pointcloud.hpp)
ament_target_dependencies(rosbag_to_pointcloud_exe rclcpp sensor_msgs PCL pcl_ros
        tf2 tf2_ros message_filters GeographicLib geometry_msgs Eigen3)

target_link_libraries(${PROJECT_NAME}_exe
        ${GeographicLib_LIBRARIES}
        Eigen3::Eigen
        ${PCL_LIBRARIES})

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

install(TARGETS
        rosbag_to_pointcloud_exe
        DESTINATION lib/${PROJECT_NAME})

ament_package()
