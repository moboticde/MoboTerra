#ifndef SICK_FLEXISOFT_ETH_COMM_H_
#define SICK_FLEXISOFT_ETH_COMM_H_
#include "rclcpp/rclcpp.hpp"
#include "terraspiro_sick_interface/msg/sick_info.hpp"
#include "std_msgs/msg/string.hpp"
#include <bitset>
#include <boost/asio.hpp>
#include <memory>

using boost::asio::ip::tcp;

class SickEthernetSocketCommunication : public rclcpp::Node
{
public:
  SickEthernetSocketCommunication();

  inline void run_asio();
  inline void stop_asio();

private:
  void start_accept();
  void handle_accept(std::shared_ptr<tcp::socket> socket, const boost::system::error_code& ec);

  void start_read(std::shared_ptr<tcp::socket> socket);

  void handle_read(std::shared_ptr<tcp::socket> socket, std::shared_ptr<std::array<char, 256>> buffer,
                   const boost::system::error_code& ec, std::size_t bytes_transferred);

  rclcpp::Publisher<terraspiro_sick_interface::msg::SickInfo>::SharedPtr pub_sick_status_;

  boost::asio::io_context io_context_;
  tcp::acceptor acceptor_;

  // Parameters
  int port_{0};
};
#endif // SICK_FLEXISOFT_ETH_COMM_H_
