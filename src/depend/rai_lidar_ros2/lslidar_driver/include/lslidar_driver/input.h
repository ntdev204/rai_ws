#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"
#include <lslidar_msgs/msg/lslidar_packet.hpp>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>

namespace lslidar_driver
{
  static uint16_t MSOP_DATA_PORT_NUMBER = 2368;

  class Input
  {
public:
    Input(rclcpp::Node * private_nh, uint16_t port);

    virtual ~Input()
    {
    }

    virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr & packet) = 0;

    int getRpm(void);
    int getReturnMode(void);
    bool getUpdateFlag(void);
    void clearUpdateFlag(void);
    void UDP_order(const std_msgs::msg::Int8 msg);
    void UDP_difop();

protected:
    rclcpp::Node * private_nh_;
    uint16_t port_;
    std::string devip_str_;
    std::string lidar_name;
    int cur_rpm_;
    int return_mode_;
    bool npkt_update_flag_;
    bool add_multicast;
    std::string group_ip;
    int UDP_PORT_NUMBER_DIFOP;
    int socket_id_difop;
    int sockfd_;
    std::string devip_str_difop;
  };

  class InputSocket: public Input
  {
public:
    InputSocket(rclcpp::Node * private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

    virtual ~InputSocket();

    virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr & packet);

private:

private:
    in_addr devip_;
    in_addr devip_difop;
};
  class InputPCAP: public Input
  {
public:
    InputPCAP(
      rclcpp::Node * private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
      std::string filename = "");
    virtual ~InputPCAP();
    virtual int getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr & pkt);

private:
    rclcpp::Rate packet_rate_;
    std::string filename_;
    pcap_t * pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
  };
}

#endif
