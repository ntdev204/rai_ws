#include "rai_robot_driver/rai_robot_driver.h"

#include <cmath> 
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// Khai báo biến IMU toàn cục (theo cách của Wheeltec)
sensor_msgs::msg::Imu Mpu6050; 

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

// ====================================================================
// HÀM CHÍNH (MAIN FUNCTION)
// ====================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Sử dụng make_shared thay vì new để quản lý bộ nhớ tốt hơn
  auto node = std::make_shared<RaiRobotDriver>();
  node->ControlLoop();
  rclcpp::shutdown();
  return 0;  
} 

// ====================================================================
// HÀM TẠO (CONSTRUCTOR) - ĐƯỢC CHẠY MỘT LẦN
// ====================================================================
RaiRobotDriver::RaiRobotDriver() : rclcpp::Node("rai_robot_driver")
{
    // 1. KHỞI TẠO BIẾN
    Sampling_Time = 0.0f;
    Power_voltage = 0.0f;
    
    // Xóa bộ nhớ cấu trúc dữ liệu
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data)); 
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

    // 2. KHAI BÁO VÀ LẤY THAM SỐ ROS 2
    this->declare_parameter<int>("serial_baud_rate", 115200);
    this->declare_parameter<std::string>("usart_port_name", "/dev/wheeltec_controller");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
    this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
    this->declare_parameter<double>("odom_x_scale", 1.0);
    this->declare_parameter<double>("odom_y_scale", 1.0);
    this->declare_parameter<double>("odom_z_scale_positive", 1.0);
    this->declare_parameter<double>("odom_z_scale_negative", 1.0);

    this->get_parameter("serial_baud_rate", serial_baud_rate);
    this->get_parameter("usart_port_name", usart_port_name);
    this->get_parameter("odom_frame_id", odom_frame_id);
    this->get_parameter("robot_frame_id", robot_frame_id);
    this->get_parameter("gyro_frame_id", gyro_frame_id);
    this->get_parameter("odom_x_scale", odom_x_scale);
    this->get_parameter("odom_y_scale", odom_y_scale);
    this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
    this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);

    // 3. KHỞI TẠO PUBLISHERS VÀ SUBSCRIBERS
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);
    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2);
    voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);

    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 2, std::bind(&RaiRobotDriver::Cmd_Vel_Callback, this, _1));
    
    // 4. MỞ CỔNG SERIAL
    try { 
        Stm32_Serial.setPort(usart_port_name);
        Stm32_Serial.setBaudrate(serial_baud_rate);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open();
        Stm32_Serial.flushInput();
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully: %s @ %d", usart_port_name.c_str(), serial_baud_rate);
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port! Please check the serial cable and udev rules! Error: %s", e.what());
    }
}

// ====================================================================
// HÀM HỦY (DESTRUCTOR)
// ====================================================================
RaiRobotDriver::~RaiRobotDriver()
{
    // Gửi lệnh dừng 0 tốc độ trước khi hủy
    Send_Data.tx[0]=FRAME_HEADER; Send_Data.tx[1] = 0; Send_Data.tx[2] = 0; 
    Send_Data.tx[4] = 0; Send_Data.tx[3] = 0; Send_Data.tx[6] = 0; Send_Data.tx[5] = 0;
    Send_Data.tx[8] = 0; Send_Data.tx[7] = 0; 
    Send_Data.tx[9]=Check_Sum(9, 1); 
    Send_Data.tx[10]=FRAME_TAIL; 
    
    try { Stm32_Serial.write(Send_Data.tx, sizeof (Send_Data.tx)); }
    catch (serial::IOException& e) {}
    
    Stm32_Serial.close();
    RCLCPP_INFO(this->get_logger(), "Shutting down Rai Robot Driver.");
}

// ====================================================================
// HÀM PHỤ TRỢ (HELPER FUNCTIONS)
// ====================================================================

// BCC Checksum (XOR)
unsigned char RaiRobotDriver::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;
  
  if(mode == 0) // Receive data mode
  {
    for(k = 0; k < Count_Number; k++)
      check_sum = check_sum ^ Receive_Data.rx[k]; 
  }
  if(mode == 1) // Send data mode
  {
    for(k = 0; k < Count_Number; k++)
      check_sum = check_sum ^ Send_Data.tx[k]; 
  }
  return check_sum;
}

// Chuyển đổi 2 bytes thành short có dấu
short RaiRobotDriver::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = Data_High << 8;   
  transition_16 |= Data_Low;
  return transition_16;   
}

// Chuyển đổi 2 bytes (mm/s short) thành float (m/s)
float RaiRobotDriver::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16 = IMU_Trans(Data_High, Data_Low); 
  return (float)transition_16 / 1000.0f; 
}


// ====================================================================
// HÀM CALLBACK VÀ GỬI LỆNH (SEND COMMAND)
// ====================================================================
void RaiRobotDriver::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short transition;

    Send_Data.tx[0]=FRAME_HEADER; // 0x7B
    Send_Data.tx[1] = 0;          // AutoRecharge (default 0)
    Send_Data.tx[2] = 0;          // Dự trữ

    // Vx (m/s -> short mm/s)
    transition = static_cast<short>(twist_aux->linear.x * 1000);
    Send_Data.tx[4] = transition;     // Low byte
    Send_Data.tx[3] = transition >> 8; // High byte

    // Vy
    transition = static_cast<short>(twist_aux->linear.y * 1000);
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition >> 8;

    // Vz (Angular Z)
    transition = static_cast<short>(twist_aux->angular.z * 1000);
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition >> 8;

    Send_Data.tx[9]=Check_Sum(9, 1); // Checksum (XOR của byte 0-8)
    Send_Data.tx[10]=FRAME_TAIL; // 0x7D

    try {
        Stm32_Serial.write(Send_Data.tx, sizeof (Send_Data.tx));
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port: %s", e.what());
    }
}

// ====================================================================
// HÀM XỬ LÝ DỮ LIỆU SERIAL (RECEIVE DATA)
// Tương đương Get_Sensor_Data_New() của Wheeltec
// ====================================================================
bool RaiRobotDriver::Get_Sensor_Data_New()
{
    // 
    // Đây là nơi thực hiện logic đọc và giải mã gói tin 24 bytes (Odom + IMU + Voltage)
    
    // 1. Kiểm tra bộ đệm Serial
    if (Stm32_Serial.available() < RECEIVE_DATA_SIZE)
        return false;

    // 2. Tìm kiếm và đọc gói tin (Logic này phức tạp, ta chỉ dùng read cố định để qua lỗi liên kết)
    // Tốt nhất là dùng loop tìm header và tail trong vòng lặp ControlLoop
    
    // Tạm thời đọc 24 bytes (Dễ bị lỗi nếu gói tin không căn chỉnh)
    size_t bytes_read = Stm32_Serial.read(Receive_Data.rx, RECEIVE_DATA_SIZE);
    if (bytes_read != RECEIVE_DATA_SIZE)
        return false;

    // 3. Kiểm tra Header/Tail
    if (Receive_Data.rx[0] != FRAME_HEADER || Receive_Data.rx[23] != FRAME_TAIL)
        return false;

    // 4. Kiểm tra Checksum (XOR của byte 0 đến 21, Checksum ở byte 22)
    unsigned char check_sum_received = Receive_Data.rx[22];
    if (check_sum_received != Check_Sum(22, 0)) // Checksum cho 22 bytes đầu
        return false;

    // 5. Giải mã và Cập nhật Biến Trạng thái (Thành công)
    
    // Vận tốc (Bytes 2-7, 3 giá trị short)
    Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
    Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
    Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);

    // IMU (Bytes 8-19, 6 giá trị short)
    Mpu6050.linear_acceleration.x = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]) / ACCEl_RATIO;
    Mpu6050.angular_velocity.z = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]) * GYROSCOPE_RATIO; 
    // Tương tự cho các trục Y, Z khác...

    // Điện áp (Bytes 20-21, 1 giá trị short mV)
    Power_voltage = Odom_Trans(Receive_Data.rx[20], Receive_Data.rx[21]);

    return true;
}

// ====================================================================
// HÀM PUBLISH TOPICS
// ====================================================================

// Publish Odometry
void RaiRobotDriver::Publish_Odom()
{
    // 1. Chuyển đổi góc Z (Euler) sang Quaternion
    // Note: Nếu bạn sử dụng IMU để tính góc Z (Robot_Pos.Z), hãy dùng nó.
    // Nếu không, góc Z là kết quả tích phân từ Robot_Vel.Z.
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z); 
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    // 2. Điền dữ liệu Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now(); 
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = robot_frame_id; 

    // Vị trí (Tích phân)
    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.orientation = odom_quat;

    // Vận tốc
    odom.twist.twist.linear.x = Robot_Vel.X;
    odom.twist.twist.linear.y = Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;

    // TODO: Đặt Covariance Matrix (sử dụng các hằng số từ file .h)
    
    odom_publisher->publish(odom);
}

// Publish IMU Sensor (Dữ liệu sau khi chuyển đổi đơn vị)
void RaiRobotDriver::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub; 
    Imu_Data_Pub.header.stamp = this->get_clock()->now(); 
    Imu_Data_Pub.header.frame_id = gyro_frame_id; 

    // Góc Quaternion (Thường được tính toán trong Quaternion_Solution.cpp, ta giả định)
    // Nếu bạn không dùng Kalman Filter/Madgwick, bạn phải gán Quaternion đã tính
    // Imu_Data_Pub.orientation = Mpu6050.orientation; 
    
    // Vận tốc góc và Gia tốc (đã được tính trong Get_Sensor_Data_New)
    Imu_Data_Pub.angular_velocity = Mpu6050.angular_velocity;
    Imu_Data_Pub.linear_acceleration = Mpu6050.linear_acceleration;

    // TODO: Đặt Covariance Matrix
    
    imu_publisher->publish(Imu_Data_Pub); 
}

// Publish Voltage
void RaiRobotDriver::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; 
    static float Count_Voltage_Pub = 0;
    
    // Giống Wheeltec, chỉ publish sau 10 lần (để giảm tải)
    if(Count_Voltage_Pub++ > 10)
    {
      Count_Voltage_Pub = 0;
      voltage_msgs.data = Power_voltage;
      voltage_publisher->publish(voltage_msgs); 
    }
}


// ====================================================================
// VÒNG LẶP ĐIỀU KHIỂN CHÍNH (CONTROL LOOP)
// ====================================================================
void RaiRobotDriver::ControlLoop()
{
    _Last_Time = this->get_clock()->now();
    
    // Vòng lặp chính ROS 2
    while(rclcpp::ok())
    {
        _Now = this->get_clock()->now();
        // Tính thời gian lấy mẫu (Sampling Time)
        Sampling_Time = (_Now - _Last_Time).seconds(); 
        
        // 1. Đọc và Giải mã dữ liệu Serial
        if (true == Get_Sensor_Data_New()) 
        {
            // 2. Áp dụng Hiệu chỉnh Odometry (nếu có tham số)
            Robot_Vel.X = Robot_Vel.X * odom_x_scale;
            Robot_Vel.Y = Robot_Vel.Y * odom_y_scale;
            if (Robot_Vel.Z >= 0)
                Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_positive;
            else
                Robot_Vel.Z = Robot_Vel.Z * odom_z_scale_negative;

            // 3. TÍCH PHÂN TÍNH VỊ TRÍ (Odometry Integration)
            // Tính toán vị trí mới (Robot_Pos)
            Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; 
            Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; 
            Robot_Pos.Z += Robot_Vel.Z * Sampling_Time; 
            
            // 4. Cập nhật góc Quaternion IMU (Nếu có file Quaternion_Solution.cpp)
            // Quaternion_Solution(Mpu6050.angular_velocity.x, ..., Mpu6050.linear_acceleration.z);
            // Bạn cần điền mã gọi hàm từ Quaternion_Solution.cpp ở đây.

            // 5. Publish Topics
            Publish_Odom(); 
            Publish_ImuSensor(); 
            Publish_Voltage(); 
            
            _Last_Time = _Now; 
        }

        // Đảm bảo các callbacks (như Cmd_Vel_Callback) được xử lý
        rclcpp::spin_some(this->get_node_base_interface());
    }
}