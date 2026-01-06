# rai_robot_slam

Dự án này chứa các gói phần mềm để thực hiện SLAM (Simultaneous Localization and Mapping) sử dụng giải pháp GMapping cho ROS 2.

Thư mục bao gồm 2 package chính:
1.  **openslam_gmapping**: Thư viện lõi (Core) chứa thuật toán Grid-based FastSLAM.
2.  **slam_gmapping**: ROS 2 Wrapper node để sử dụng thuật toán GMapping trong môi trường ROS 2.

## 1. openslam_gmapping
Đây là thư viện thuật toán gốc của OpenSLAM GMapping, đã được port (chuyển đổi) để tương thích với hệ thống build `ament_cmake` của ROS 2. Nó cung cấp các class và hàm xử lý scan matching, particle filter và map update.
- **Nguồn**: Được fork và port từ `ros-perception/openslam_gmapping` (phiên bản ROS 2 từ Project-MANAS).

## 2. slam_gmapping
Gói này chứa node ROS 2 (`slam_gmapping`) thực hiện việc đăng ký (subscribe) dữ liệu laser và odometry, sau đó tính toán vị trí robot và xây dựng bản đồ.
- **Node**: `slam_gmapping`
- **Chức năng**:
    - Nhận dữ liệu từ topic scan (`sensor_msgs/msg/LaserScan`).
    - Nhận transforms (TF) từ `odom` -> `base_link` (hoặc frame tương đương).
    - Tính toán và phát transform từ `map` -> `odom`.
    - Công bố bản đồ (OccupancyGrid) trên topic `map`.

## Cách sử dụng (Usage)

### Yêu cầu (Prerequisites)
Đảm bảo robot của bạn đã publish:
- Dữ liệu rplidar hoặc laser scan trên topic `/scan`.
- Transform TF hợp lệ từ frame `odom` tới frame của laser (thường là `base_link` -> `laser`).

### Build
Tại thư mục gốc của workspace (`~/rai_ws`):
```bash
colcon build --packages-select openslam_gmapping slam_gmapping
source install/setup.bash
```

### Chạy Node (Run)
Chạy node với các tham số mặc định:
```bash
ros2 run slam_gmapping slam_gmapping
```

### Các Topic quan trọng
- **Subscribed Topics**:
    - `scan` (`sensor_msgs/msg/LaserScan`): Dữ liệu quét laser.
    - `/tf`: Dữ liệu biến đổi tọa độ (cần có `odom` -> `base_link`).
- **Published Topics**:
    - `map` (`nav_msgs/msg/OccupancyGrid`): Bản đồ 2D.
    - `map_metadata` (`nav_msgs/msg/MapMetaData`): Thông tin về bản đồ.
    - `entropy` (`std_msgs/msg/Float64`): Độ hỗn loạn của phân bố vị trí (độ không chắc chắn).

### Tham số cấu hình (Parameters)
Bạn có thể cấu hình các tham số qua dòng lệnh hoặc file yaml. Một số tham số chính:
- `base_frame` (default: `base_link`): Frame gắn liền với robot.
- `odom_frame` (default: `odom`): Frame odometry.
- `map_frame` (default: `map`): Frame bản đồ.
- `map_update_interval` (default: `5.0`): Thời gian cập nhật bản đồ (giây).
- `maxUrange` (default: `80.0`): Khoảng cách tối đa sử dụng của laser cho việc tính toán.

**Ví dụ chạy với tham số:**
```bash
ros2 run slam_gmapping slam_gmapping --ros-args -p base_frame:=base_footprint -p map_update_interval:=1.0
```
