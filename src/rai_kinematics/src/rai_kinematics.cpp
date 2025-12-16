#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rai_robot_msg/msg/data.hpp"
#include "rai_robot_msg/msg/supersonic.hpp"

// Khai báo một lớp Kinematics Node
class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("rai_kinematics_node")
    {
        // Khởi tạo Publisher cho dữ liệu siêu âm đã xử lý (nếu cần)
        supersonic_pub_ = this->create_publisher<rai_robot_msg::msg::Supersonic>("supersonic_data", 10);
        
        // Khởi tạo Publisher cho dữ liệu Kinematics đã tính toán (ví dụ: tư thế)
        kinematics_pub_ = this->create_publisher<rai_robot_msg::msg::Data>("kinematics_pose", 10);

        // Khởi tạo Subscriber để nhận dữ liệu từ driver (ví dụ: odom thô)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&KinematicsNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 1. Nhận dữ liệu Odometry (Vị trí và Vận tốc)
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        
        // 2. Thực hiện tính toán Động học (Nếu cần thiết, bạn có thể thực hiện
        // Kinematics xuôi/nghịch hoặc chuyển đổi hệ tọa độ tại đây)
        
        // 3. Xuất bản Dữ liệu đã xử lý bằng message tùy chỉnh 'data.msg'
        auto kinematics_msg = rai_robot_msg::msg::Data();
        kinematics_msg.x = current_x;
        kinematics_msg.y = current_y;
        kinematics_msg.z = 0.0f; // Góc Yaw (hoặc giá trị khác từ Odometry)
        
        kinematics_pub_->publish(kinematics_msg);
        
        // **(Lưu ý: Bạn sẽ cần logic để đọc dữ liệu siêu âm và publish message Supersonic)**
        // Supersonic data thường đến từ một node khác hoặc được xử lý ở đây.
        
    }

    rclcpp::Publisher<rai_robot_msg::msg::Data>::SharedPtr kinematics_pub_;
    rclcpp::Publisher<rai_robot_msg::msg::Supersonic>::SharedPtr supersonic_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

// Hàm chính
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}