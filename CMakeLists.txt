cmake_minimum_required(VERSION 3.8)
project(pointcloud_merge_and_kitti)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs	REQUIRED)
find_package(geometry_msgs	REQUIRED)
find_package(tf2_geometry_msgs	REQUIRED)
find_package(pcl_msgs	REQUIRED)
find_package(tf2	REQUIRED)
find_package(tf2_msgs	REQUIRED)
find_package(tf2_sensor_msgs	REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions	REQUIRED)
find_package(pcl_ros	REQUIRED)
find_package(tf2_ros	REQUIRED)
find_package(tf2_eigen	REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

ament_package()
add_executable(merge_and_kitti src/merge_and_kitti.cpp)

ament_target_dependencies(merge_and_kitti rclcpp std_msgs PCL pcl_conversions pcl_ros pcl_msgs sensor_msgs tf2 tf2_ros tf2_msgs tf2_sensor_msgs geometry_msgs tf2_geometry_msgs tf2_eigen)

install(TARGETS
  merge_and_kitti
  DESTINATION lib/${PROJECT_NAME}
)