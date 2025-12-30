cmake_minimum_required(VERSION 3.5)
project(turn_on_rai_robot)
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rai_robot_msg REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
 )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
include_directories(
  include
)
set(rai_robot_node_SRCS
  src/rai_robot.cpp
  src/Quaternion_Solution.cpp
)
add_executable(rai_robot_node src/rai_robot.cpp src/Quaternion_Solution.cpp)
ament_target_dependencies(rai_robot_node tf2_ros tf2 tf2_geometry_msgs rclcpp std_msgs nav_msgs std_srvs sensor_msgs serial rai_robot_msg turtlesim)
add_executable(camera_info_node src/camera_info_node.cpp)
ament_target_dependencies(camera_info_node rclcpp  sensor_msgs  )
install(TARGETS
rai_robot_node
camera_info_node
DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
