#include "sick_flexisoft_eth_comm/sick_flexisoft_eth_comm.h"

SickEthernetSocketCommunication::SickEthernetSocketCommunication()
    : Node("sick_ethernet_socket_node"),
      // The acceptor listens on a specific endpoint
      acceptor_(io_context_) // Port set later
{
  // 1. Declare and get parameters
  this->declare_parameter<int>("port", 0);
  this->get_parameter("port", port_);

  // 2. Setup the ROS 2 publisher
  pub_sick_status_ = this->create_publisher<terraspiro_sick_interface::msg::SickInfo>("sick_info", rclcpp::QoS(1));

  // 3. Configure the acceptor and start listening
  tcp::endpoint endpoint(tcp::v4(), port_);
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen();

  RCLCPP_INFO_STREAM(this->get_logger(), "Server started. Listening on port " << port_);

  // 4. Start the first accept operation
  start_accept();
}

inline void SickEthernetSocketCommunication::run_asio() { io_context_.run(); }

inline void SickEthernetSocketCommunication::stop_asio() { io_context_.stop(); }

void SickEthernetSocketCommunication::start_accept()
{
  auto new_socket = std::make_shared<tcp::socket>(io_context_);

  acceptor_.async_accept(*new_socket,
                         [this, new_socket](const boost::system::error_code& ec) { handle_accept(new_socket, ec); });
}

void SickEthernetSocketCommunication::handle_accept(std::shared_ptr<tcp::socket> socket,
                                                    const boost::system::error_code& ec)
{
  if (!ec)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Client connected from: " << socket->remote_endpoint().address().to_string());

    start_read(socket);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Accept error: " << ec.message().c_str());
  }

  start_accept();
}

void SickEthernetSocketCommunication::start_read(std::shared_ptr<tcp::socket> socket)
{
  auto buffer = std::make_shared<std::array<char, 256>>();

  socket->async_read_some(boost::asio::buffer(*buffer),
                          [this, socket, buffer](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                            handle_read(socket, buffer, ec, bytes_transferred);
                          });
}

void SickEthernetSocketCommunication::handle_read(std::shared_ptr<tcp::socket> socket,
                                                  std::shared_ptr<std::array<char, 256>> buffer,
                                                  const boost::system::error_code& ec, std::size_t bytes_transferred)
{
  if (ec && ec == boost::asio::error::eof)
  {
    RCLCPP_WARN(this->get_logger(), "Client disconnected cleanly.");
    return;
  }

  if (ec && ec != boost::asio::error::eof)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Client error: " << ec.message());
    return;
  }

  if (bytes_transferred < 15)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Received less then 15 bytes required for reading the desired SICK status. This could indicate a problem.");
    return;
  }

  std::string data_str(buffer->data(), bytes_transferred);
  std::vector<char> char_vector(data_str.begin(), data_str.end());

  // Expected data layout
  // byte 1 - 2 | byte 3 - 10 | ... | byte 15 | ...
  // command | command_params (length of datasets) | ... | module 1 input values | ...

  short command = (char_vector[0] << 8) | char_vector[1];
  short command_params[4];
  for (int i = 2, counter = 0; i < 10; i += 2)
  {
    command_params[i / 2 - 1] = (char_vector[i] << 8) | char_vector[i + 1];
    counter++;
  }

  char sick_status = char_vector[14];

  terraspiro_sick_interface::msg::SickInfo sick_msg;
  sick_msg.header.stamp = this->now();

  // Safety zones are breached if both OSSD pairs are 0
  sick_msg.front_safety_zone_breached = (sick_status & 0b00000011) == 0;
  sick_msg.back_safety_zone_breached = (sick_status & 0b00001100) >> 2 == 0;

  sick_msg.estop_active = sick_status & 0b00010000;
  sick_msg.safety_override_active = sick_status & 0b00100000;
  sick_msg.front_warning_zone_breached = sick_status & 0b10000000;
  sick_msg.back_warning_zone_breached = sick_status & 0b01000000;
  pub_sick_status_->publish(sick_msg);

  start_read(socket);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto server_node = std::make_shared<SickEthernetSocketCommunication>();

  std::thread asio_thread([&]() { server_node->run_asio(); });

  rclcpp::spin(server_node);

  server_node->stop_asio();
  if (asio_thread.joinable())
  {
    asio_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
