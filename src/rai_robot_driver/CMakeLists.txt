cmake_minimum_required(VERSION 3.8)
project(rai_robot_driver)

# Đặt chuẩn C++14 (hoặc C++17 nếu muốn mới hơn)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. TÌM KIẾM DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)         # Cần cho tf2/LinearMath
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Cần cho toMsg(q)
find_package(serial REQUIRED)

# --- 2. EXECUTABLE VÀ INCLUDE ---

# Thêm tất cả file nguồn C++ vào executable
add_executable(rai_robot_driver_node 
  src/rai_robot_driver.cpp 
)

# Thêm include directories của package này (để tìm rai_robot_driver/rai_robot_driver.h)
target_include_directories(rai_robot_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Thêm thư mục include của Serial (sử dụng biến nếu cần thiết, hoặc phụ thuộc vào ament_target_dependencies)
  ${SERIAL_INCLUDE_DIRS}
)

# THIẾT LẬP DEPENDENCIES VÀ LIÊN KẾT (CÁCH CHUẨN AMENT)
# Lệnh này xử lý cả include directories và cờ biên dịch từ các gói ROS, 
# bao gồm cả việc liên kết với thư viện 'serial' (như Wheeltec đã làm)
ament_target_dependencies(rai_robot_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  # Dùng tên package 'serial' ở đây để liên kết/include
  serial 
)

# Lưu ý: Loại bỏ target_link_libraries cũ vì ament_target_dependencies đã thay thế vai trò linking cho các gói Ament.
# Nếu 'serial' vẫn không được liên kết, hãy uncomment target_link_libraries dưới đây.
# target_link_libraries(rai_robot_driver_node serial) 

# --- 3. CÀI ĐẶT ---
install(TARGETS
  rai_robot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

# --- 4. EXPORT DEPENDENCIES ---
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
)

# --- 5. KIỂM THỬ VÀ ĐÓNG GÓI ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()