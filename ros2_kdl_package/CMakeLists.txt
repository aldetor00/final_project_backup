cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======== Find dependencies ========
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)

# ======== Generazione action ========
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ======== Nodo principale (Server) ========
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ros2_kdl_node
  orocos_kdl
  rclcpp
  rclcpp_action
  action_msgs
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  tf2_ros
)
# FIX WARNING 1
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ros2_kdl_node "${cpp_typesupport_target}")


# ======== Client Dual Marker ========
add_executable(dual_marker_client 
  src/dual_marker_client.cpp
)
ament_target_dependencies(dual_marker_client
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  aruco_msgs
)
# FIX WARNING 2
target_link_libraries(dual_marker_client "${cpp_typesupport_target}")


# ======== Client IIWA1 Pick&Place (CORRETTO) ========
add_executable(iiwa1_pick_place_client 
  src/iiwa1_pick_place_client.cpp
)
ament_target_dependencies(iiwa1_pick_place_client
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
)
# FIX WARNING 3
target_link_libraries(iiwa1_pick_place_client "${cpp_typesupport_target}")


# ======== Install ========
install(TARGETS 
  ros2_kdl_node 
  dual_marker_client
  iiwa1_pick_place_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
