#ifndef __RAI_ROBOT_DRIVER_H_
#define __RAI_ROBOT_DRIVER_H_

#include <iostream>
#include <string> 
#include <math.h> 
#include <unistd.h> 

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h> // Thư viện Serial
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_msgs/msg/float32.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define FRAME_HEADER        0X7B
#define FRAME_TAIL          0X7D
#define RECEIVE_DATA_SIZE   24
#define SEND_DATA_SIZE      11

#define GYROSCOPE_RATIO     0.00026644f
#define ACCEl_RATIO         1671.84f

typedef struct __Vel_Pos_Data_
{
  float X;
  float Y;
  float Z;
}Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
  short accele_x_data; 
  short accele_y_data;  
  short accele_z_data; 
  short gyros_x_data; 
  short gyros_y_data;  
  short gyros_z_data; 
}MPU6050_DATA;

typedef struct _SEND_DATA_  
{
  uint8_t tx[SEND_DATA_SIZE];
  float X_speed;          
  float Y_speed;            
  float Z_speed;          
  unsigned char Frame_Tail; 
}SEND_DATA;

typedef struct _RECEIVE_DATA_     
{
  uint8_t rx[RECEIVE_DATA_SIZE];
  uint8_t Flag_Stop;
  unsigned char Frame_Header;
  float X_speed;
  float Y_speed;
  float Z_speed;
  float Power_Voltage;
  unsigned char Frame_Tail;
}RECEIVE_DATA;

class RaiRobotDriver : public rclcpp::Node
{
  public:
    RaiRobotDriver();  // Constructor
    ~RaiRobotDriver(); // Destructor
    void ControlLoop();  // Vòng lặp chính (nhận/gửi data, pub topics)
    
  private:
    serial::Serial Stm32_Serial; // Đối tượng serial
    
    rclcpp::Time _Now, _Last_Time;  // Thời gian
    float Sampling_Time;          // Thời gian lấy mẫu
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;      
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;        
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;

    // Callbacks
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    
    // Logic Giao tiếp & Xử lý
    void Publish_Odom();        // Pub topic Odom
    void Publish_ImuSensor();   // Pub topic IMU
    void Publish_Voltage();     // Pub topic Voltage
    bool Get_Sensor_Data_New(); // Đọc và kiểm tra dữ liệu Serial
    
    // Hàm phụ trợ
    unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
    short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
    float Odom_Trans(uint8_t Data_High,uint8_t Data_Low);
    
    // Biến Tham số
    std::string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id;
    int serial_baud_rate;
    
    // Biến Trạng thái
    RECEIVE_DATA Receive_Data;
    SEND_DATA Send_Data;
    Vel_Pos_Data Robot_Pos;
    Vel_Pos_Data Robot_Vel;
    MPU6050_DATA Mpu6050_Data;
    float Power_voltage;
    
    // Tham số hiệu chỉnh Odometry (Lấy từ tham số ROS 2)
    float odom_x_scale,odom_y_scale,odom_z_scale_positive,odom_z_scale_negative; 
};

#endif // __RAI_ROBOT_DRIVER_H_