cmake_minimum_required(VERSION 3.8)
project(interceptor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vehicle_odometry_subscriber src/vehicle_odometry_subscriber.cpp)
ament_target_dependencies(vehicle_odometry_subscriber
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_msgs
  sensor_msgs
)

add_executable(interceptor_tf2_odometry src/interceptor_tf2_odometry.cpp)
ament_target_dependencies(interceptor_tf2_odometry
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_msgs
  px4_ros_com
  sensor_msgs
)

add_executable(target_tf2_odometry src/target_tf2_odometry.cpp)
ament_target_dependencies(target_tf2_odometry
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_msgs
  px4_ros_com
  px4_ros2_cpp
  sensor_msgs
)

add_executable(tf2_listener src/tf2_listener.cpp)
ament_target_dependencies(tf2_listener
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_msgs
  sensor_msgs
  px4_ros_com
)

add_executable(pursuit_mode src/pursuit_mode.cpp)
ament_target_dependencies(pursuit_mode
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_ros_com
  px4_ros2_cpp
)

add_executable(PN_mode src/PN_mode.cpp)
ament_target_dependencies(PN_mode
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  px4_ros_com
  px4_ros2_cpp
)

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
  vehicle_odometry_subscriber
  target_tf2_odometry
  interceptor_tf2_odometry
  tf2_listener
  pursuit_mode
  PN_mode
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
