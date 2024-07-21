cmake_minimum_required(VERSION 3.8)
project(bt_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# 実行追加
add_executable(publisher_node src/publisher_node.cpp)
add_executable(sample_no01 src/sample_no01.cpp)
add_executable(sample_no02 src/sample_no02.cpp)
add_executable(sample_no03 src/sample_no03.cpp)
add_executable(subscriber_node src/subscriber_node.cpp)

# 依存関係
ament_target_dependencies(publisher_node rclcpp std_msgs)
ament_target_dependencies(sample_no01 rclcpp std_msgs behaviortree_cpp)
ament_target_dependencies(sample_no02 rclcpp std_msgs behaviortree_cpp)
ament_target_dependencies(sample_no03 rclcpp std_msgs behaviortree_cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

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
  sample_no01
  sample_no02
  sample_no03
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
