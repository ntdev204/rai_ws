# rai_robot_nav2

Package này cung cấp cấu hình và launch files để chạy Navigation2 (Nav2) cho robot RAI (dựa trên nền tảng Wheeltec).

## Cấu trúc thư mục
- `launch/`: Chứa các file khởi động.
    - `rai_nav2.launch.py`: File launch chính để khởi động toàn bộ hệ thống (Robot driver, Lidar, Nav2).
    - `bringup_launch.py`: Wrapper cho `nav2_bringup`, quản lý các node của Nav2 (AMCL, Planner, Controller, v.v.).
    - `save_map.launch.py`: Lưu bản đồ từ topic `/map`.
- `map/`: Chứa các file bản đồ (`.yaml`, `.pgm`).
- `param/`: Chứa các file cấu hình (`.yaml`) cho Nav2 và các loại robot khác nhau (AKM, Mecanum, Omni, Diff...).
- `rviz/`: Chứa cấu hình hiển thị RViz.

## Hướng dẫn sử dụng

### 1. Khởi động Navigation
Để bắt đầu điều hướng (Navigation) với bản đồ đã có:

```bash
ros2 launch rai_robot_nav2 rai_nav2.launch.py map:=/path/to/your/map.yaml
```

**Tham số:**
- `map`: Đường dẫn tuyệt đối tới file bản đồ `.yaml`. Mặc định: `src/rai_robot_nav2/map/WHEELTEC.yaml`.
- `params`: Đường dẫn tới file cấu hình tham số Nav2. Mặc định: `src/rai_robot_nav2/param/rai_params/param_mini_akm.yaml`.
    - Thay đổi `params` phù hợp với loại robot của bạn (ví dụ: `param_mini_diff.yaml`, `param_mini_mec.yaml`).

**Ví dụ:**
```bash
ros2 launch rai_robot_nav2 rai_nav2.launch.py \
    map:=/home/ntdev204/rai_ws/src/rai_robot_nav2/map/my_map.yaml \
    params:=/home/ntdev204/rai_ws/src/rai_robot_nav2/param/rai_params/param_mini_diff.yaml
```

### 2. Lưu bản đồ (Map Saver)
Sau khi chạy SLAM (ví dụ dùng `rai_robot_slam`), để lưu bản đồ:

```bash
ros2 launch rai_robot_nav2 save_map.launch.py
```
*Lưu ý: Kiểm tra nội dung file `save_map.launch.py` để biết đường dẫn lưu mặc định hoặc thay đổi nó.*

### 3. Quy trình hoạt động
1.  **Launch**: Chạy lệnh `rai_nav2.launch.py`.
    - Robot driver sẽ khởi động.
    - Lidar sẽ quay.
    - Nav2 stack sẽ load bản đồ và khởi động AMCL (định vị).
2.  **Initial Pose**: Sử dụng RViz, chọn "2D Pose Estimate" để xác định vị trí ban đầu của robot trên bản đồ.
3.  **Navigation**: Sử dụng "Nav2 Goal" trên RViz để ra lệnh robot di chuyển tới điểm đích.

## Lưu ý quan trọng
- Đảm bảo bạn đã `colcon build` và `source install/setup.bash`.
- Chọn đúng file tham số (`params`) khớp với cấu hình động học (kinematics) của robot (Mecanum, Ackermann, Differential).
