cmake_minimum_required(VERSION 3.8)
project(bt_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(
  THIS_PACKAGE_DEPS
  rclcpp
  behaviortree_ros2
  std_msgs
  std_srvs
  btcpp_ros2_interfaces
)

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node ${THIS_PACKAGE_DEPS})

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node ${THIS_PACKAGE_DEPS})

add_executable(servo_node src/servo.cpp)
ament_target_dependencies(servo_node ${THIS_PACKAGE_DEPS})

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

install(
  TARGETS
  publisher_node
  subscriber_node
  servo_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
