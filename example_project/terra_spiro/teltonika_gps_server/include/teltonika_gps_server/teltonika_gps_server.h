#ifndef TELTONIKA_GPS_SERVER_H
#define TELTONIKA_GPS_SERVER_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <cstring>
#include <memory>
#include <string>

constexpr unsigned int MAX_NMEA_LENGTH = 1024;
constexpr unsigned short TCP_PORT = 10001;
constexpr unsigned short UDP_PORT = 10002;

namespace asio = boost::asio;
using asio::ip::tcp;
using asio::ip::udp;

inline bool parse_nmea_and_fill_fix(const std::string& nmea_str, sensor_msgs::msg::NavSatFix& fix);

class NmeaServerNode : public rclcpp::Node
{
public:
  NmeaServerNode();

  void publish_nmea_message(sensor_msgs::msg::NavSatFix& msg) const
  {
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = gps_frame_id_;
    pub_gps_fix_->publish(msg);
  };

  void runAsio();
  void stopAsio();
  void startAccept();
  void handleAccept(std::shared_ptr<tcp::socket> socket, const boost::system::error_code& ec);
  void startRead(std::shared_ptr<tcp::socket> socket);
  void handleRead(std::shared_ptr<tcp::socket> socket, std::shared_ptr<std::array<char, MAX_NMEA_LENGTH>> buffer,
                   const boost::system::error_code& ec, std::size_t bytes_transferred);

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_fix_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_accept_;
  std::string protocol_, gps_frame_id_;
  int port_;
  double gps_fix_pub_rate_;

  asio::io_context io_context_;
  tcp::acceptor acceptor_;
};
#endif // TELTONIKA_GPS_SERVER_H
