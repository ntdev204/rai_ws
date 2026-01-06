Gắn kết NFS:
sudo mount -t nfs 192.168.0.100:/home/rai/rai_ros2 /mnt
Hủy gắn NFS:
sudo umount -t nfs 192.168.0.100:/home/rai/rai_ros2 /mnt


1. Khởi động khung gầm robot
ros2 launch turn_on_rai_robot turn_on_rai_robot.launch.py

2. Khởi động điều khiển khung gầm
ros2 launch turn_on_rai_robot turn_on_rai_robot.launch.py

3. Khởi động Camera
ros2 launch turn_on_rai_robot rai_camera.launch.py

4. Khởi động Lidar
ros2 launch turn_on_rai_robot rai_lidar.launch.py

Khởi động Lidar, Camera và Khung gầm:
ros2 launch turn_on_rai_robot rai_sensors.launch.py

5. Khởi động điều khiển bàn phím
ros2 run rai_robot_keyboard rai_keyboard 

6. Chức năng đi theo đơn giản
① Đi theo bằng Lidar
ros2 launch simple_follower_ros2 laser_follower.launch.py

② Dò đường bằng hình ảnh (Visual Line Follow)
ros2 launch simple_follower_ros2 line_follower.launch.py

③ Theo dõi đối tượng bằng hình ảnh (Visual Tracking)
ros2 launch simple_follower_ros2 visual_follower.launch.py

③ Đi theo KCF
ros2 launch rai_robot_kcf rai_robot_kcf.launch.py

7. Tạo bản đồ 2D
① Sử dụng gmapping để tạo bản đồ
ros2 launch slam_gmapping slam_gmapping.launch.py

② Sử dụng slam_toolbox để tạo bản đồ
ros2 launch rai_slam_toolbox online_async_launch.py

③ Sử dụng cartographer để tạo bản đồ
ros2 launch rai_cartographer cartographer.launch.py

Lưu bản đồ
ros2 launch rai_nav2 save_map.launch.py

8. Điều hướng 2D [Bao gồm điều hướng đa điểm]
ros2 launch rai_nav2 rai_nav2.launch.py

10. Tạo bản đồ RTAB-MAP
ros2 launch rai_robot_rtab rai_slam_rtab.launch.py
Lưu bản đồ
ros2 launch rai_nav2 save_map.launch.py

11. Điều hướng RTAB-MAP
Bước 1:
ros2 launch rai_robot_rtab rtabmap_localization.launch.py
Bước 2:
ros2 launch rai_robot_rtab rai_nav2_rtab.launch.py

12. ORB Slam
ros2 launch orb_slam2_ros orb_slam2_Astra_rgbd_launch.py

13. Tạo bản đồ tự động khám phá RRT [Đăng 4 điểm theo chiều/ngược chiều kim đồng hồ, điểm cuối cùng nằm trong bản đồ đã biết]
Bước 1: ros2 launch rai_slam_toolbox online_async_launch.py
Bước 2: ros2 launch rai_robot_rrt rai_rrt_slam.launch.py

14. Bám theo đường dẫn (Path tracking): [Xem hướng dẫn chức năng để biết thao tác chi tiết]
Thiết kế đường dẫn và lưu:
Bước 1: Chạy chức năng điều hướng
ros2 launch rai_nav2 rai_nav2.launch.py
Bước 2: Lưu đường dẫn
ros2 launch rai_path_follow save_path.launch.py
Bước 3: Điều khiển bàn phím để thiết kế đường dẫn
ros2 run rai_robot_keyboard rai_keyboard 
Bật bám theo đường dẫn:
Bước 1: Chạy chức năng điều hướng
ros2 launch rai_nav2 rai_nav2.launch.py
Bước 2: Bật bám theo đường dẫn
ros2 launch rai_path_follow follow_path.launch.py

15. Đội hình nhiều robot [Xem hướng dẫn chức năng để biết thao tác chi tiết]

Bước 1: Chạy chương trình điều hướng đội hình trên xe chủ (master)
ros2 launch rai_multi navigation.launch.py
Bước 2: Chạy chương trình đội hình trên các xe tớ (slave) tương ứng
ros2 launch rai_multi rai_slave.launch.py
Bước 3: Mở node điều khiển bàn phím trên xe chủ để điều khiển chuyển động của xe chủ
ros2 run rai_robot_keyboard rai_keyboard 

16. Chức năng ROS2 QT
ros2 launch qt_ros_test qt_ros_test.launch.py 

17. Chức năng chuyển văn bản thành giọng nói (TTS)
(Nội dung văn bản chuyển thành âm thanh nằm trong file tts_make.launch.py)
ros2 launch tts tts_make.launch.py

18. Điều khiển bằng giọng nói
// Khởi động node khởi tạo mảng micro
ros2 launch rai_mic_ros2 mic_init.launch.py
// Khởi động node khởi tạo chức năng xe
ros2 launch rai_mic_ros2 base.launch.py

19. Nhận diện khung xương
① Điều khiển tư thế (lệnh trong chương điều khiển tư thế, chương điều khiển định hướng nhiều người, chương kết hợp RGB ghi nhớ nhân vật)
ros2 launch  bodyreader bodyinteraction.launch.py
② Đi theo khung xương người (Chương theo dõi cơ thể người)
ros2 launch bodyreader bodyfollow.launch.py
③ Kết hợp điều khiển tư thế và đi theo —— Mặc định khi khởi động là chế độ điều khiển tư thế, đan chéo hai tay trước ngực để chuyển đổi chế độ
ros2 launch bodyreader final.launch.py

20. Hiển thị camera trên trình duyệt WEB
Bước 1: Khởi động camera
ros2 launch turn_on_rai_robot rai_camera.launch.py
Bước 2:
ros2 run web_video_server web_video_server
Bước 3: Nhập địa chỉ vào trình duyệt để xem giám sát video thời gian thực
http://192.168.0.100:8080/

21. Điều khiển bằng tay cầm USB
ros2 launch rai_joy rai_joy.launch.py

22. Chức năng tự động sạc lại
Sửa đổi loại xe và dung lượng pin
Mở file auto_recharge_ros2/robot_info.yaml, chọn loại xe và dung lượng pin khớp với xe hiện tại

Sử dụng chức năng tự động sạc lại
① Tạo bản đồ trước và lưu lại
ros2 launch slam_gmapping slam_gmapping.launch.py
ros2 launch rai_nav2 save_map.launch.py
② Bật chức năng điều hướng:
ros2 launch rai_nav2 rai_nav2.launch.py
③ Bật chức năng tự động sạc lại:
ros2 run auto_recharge_ros2 auto_recharge
Trên rviz, sử dụng topic “charger_position_update” để định vị trí trạm sạc. Sử dụng theo nhắc nhở trên terminal

23. Nhận diện và đi theo thẻ AR
// Nhận diện thẻ AR
ros2 launch aruco_ros aruco_recognize.launch.py
// Đi theo thẻ AR
ros2 launch simple_follower_ros2 aruco_follower.launch.py

Các lệnh thường dùng khác

1. Biên dịch gói chức năng riêng lẻ
Ví dụ chỉ biên dịch turn_on_rai_robot
colcon build --packages-select turn_on_rai_robot
Biên dịch tất cả các gói chức năng
colcon build
Lưu ý: Sau khi người dùng sửa đổi nội dung file launch, cần biên dịch lại, để có hiệu lực.

2. Đệ quy sửa đổi thời gian sửa đổi của các file trong thư mục hiện tại (terminal):
find ./* -exec touch {} \;

3. Chạy trong không gian làm việc, cài đặt tất cả các phụ thuộc của gói chức năng ROS (đã cấu hình rosdep trong image):
rosdep install --from-paths src --ignore-src -r -y

4. Sửa đổi thời gian hệ thống:
sudo date -s "2022-06-15 09:00:00"

5. Sử dụng nguồn Douban để cài đặt pip (tốc độ mạng sẽ nhanh hơn nhiều):
pip install -i https://pypi.doubanio.com/simple/ tên_gói_python

6. Đăng nhập SSH:
ssh -Y rai@192.168.0.100

7. Cấp quyền thực thi cho tất cả các file trong thư mục:
sudo chmod -R 777 thư_mục

8. Mở đường dẫn bản đồ:
cd /home/rai/rai_ros2/src/rai_robot_nav2/map
Lưu bản đồ thủ công:
ros2 run nav2_map_server map_saver_cli -f ~/map

Sử dụng công cụ rqt để xem topic hình ảnh:
rqt_image_view

Xem mối quan hệ giữa nút và topic
rqt_graph

Tạo pdf cây TF tại đường dẫn hiển thị trong terminal
ros2 run tf2_tools view_frames

Điều chỉnh độ phân giải VNC
xrandr --fb 1024x768
