import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Định nghĩa thư mục chia sẻ của package driver
    # Điều này cần thiết để Node có thể tìm kiếm các tham số, nếu có
    rai_driver_pkg_dir = get_package_share_directory('rai_robot_driver')

    # 2. Định nghĩa các tham số mặc định cho Node Driver
    # Đây là các tham số sẽ ghi đè lên giá trị mặc định trong Constructor C++
    driver_params = {
        'serial_baud_rate': 115200,
        'usart_port_name': '/dev/ttyACM0',  # Quan trọng: Dùng tên cố định UDEV
        'odom_frame_id': 'odom',
        'robot_frame_id': 'base_footprint',
        'gyro_frame_id': 'gyro_link',
        'odom_x_scale': 1.0,
        'odom_y_scale': 1.0,
        'odom_z_scale_positive': 1.0,
        'odom_z_scale_negative': 1.0,
    }

    # 3. Khởi tạo Node Driver C++
    driver_node = Node(
        package='rai_robot_driver',           # Tên package đã biên dịch C++
        executable='rai_robot_driver_node',   # Tên executable (từ add_executable)
        name='rai_robot_driver_node',
        output='screen',
        parameters=[driver_params],
        # Thêm remapping nếu cần thay đổi topic (ví dụ: '/cmd_vel' -> '/rai/cmd_vel')
    )
    
    # 4. Tạo mô tả Launch
    return LaunchDescription([
        driver_node,
        # TODO: Thêm các Node khác sau này (như LiDAR, Camera, EKF) vào đây
    ])