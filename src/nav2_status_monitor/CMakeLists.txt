cmake_minimum_required(VERSION 3.8)
project(nav2_status_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Nav2Status.msg"
  DEPENDENCIES std_msgs
)

add_executable(nav2_status_monitor_node src/nav2_status_monitor.cpp)
target_compile_features(nav2_status_monitor_node PUBLIC cxx_std_17)
ament_target_dependencies(nav2_status_monitor_node
  rclcpp
  std_msgs
  action_msgs
  nav2_msgs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nav2_status_monitor_node "${cpp_typesupport_target}")

install(TARGETS nav2_status_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
