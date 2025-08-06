cmake_minimum_required(VERSION 3.8)
project(plan_manage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(test_node src/test_node.cpp)
add_executable(global_path_node src/global_path_node.cpp)
add_executable(plan_manage_node src/plan_manage_node.cpp)
ament_target_dependencies(test_node rclcpp rclcpp_lifecycle lifecycle_msgs nav_msgs geometry_msgs nav2_util tf2_ros)
ament_target_dependencies(global_path_node rclcpp rclcpp_lifecycle rclcpp_action lifecycle_msgs nav_msgs geometry_msgs nav2_msgs)
ament_target_dependencies(plan_manage_node rclcpp rclcpp_lifecycle rclcpp_action lifecycle_msgs nav_msgs geometry_msgs nav2_msgs)

install(TARGETS
  test_node global_path_node plan_manage_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
