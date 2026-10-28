#include <teltonika_gps_server/teltonika_gps_server.h>

#include <nmea/message/gga.hpp>
#include <nmea/sentence.hpp>

inline bool parse_nmea_and_fill_fix(const std::string& nmea_str, sensor_msgs::msg::NavSatFix& fix)
{
  // --- Real NMEA Parsing Logic Goes Here ---
  nmea::sentence nmea_sentence(nmea_str);

  // Check if the NMEA message type is GGA.
  if (nmea_sentence.type() == "GGA")
  {
    // Parse GGA message from NMEA sentence.
    nmea::gga gga(nmea_sentence);

    if (!gga.longitude.exists() || !gga.latitude.exists() || !gga.altitude.exists() || gga.talker.empty())
    {
      return false;
    }

    // Service mappings from NMEA to sensor_msgs::msg::NavSatStatus service
    if (gga.talker == "GP")
    {
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    }
    else if (gga.talker == "GL")
    {
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    }
    else if (gga.talker == "GA")
    {
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    }
    else
    {
      fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
    }

    // Fix mapping from NMEA to sensor_msgs::msg::NavSatStatus
    if (gga.fix.get() == nmea::gga::fix_type::GPS)
    {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    }
    else if (gga.fix.get() == nmea::gga::fix_type::DGPS)
    {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    }
    else if (gga.fix.get() == nmea::gga::fix_type::RTK_FLOAT || gga.fix.get() == nmea::gga::fix_type::RTK)
    {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    }
    else if (gga.fix.get() == nmea::gga::fix_type::ESTIMATED || gga.fix.get() == nmea::gga::fix_type::NONE)
    {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    else
    {
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_UNKNOWN;
    }

    fix.altitude = gga.altitude.get();
    fix.longitude = gga.longitude.get();
    fix.latitude = gga.latitude.get();
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  return true;
}

NmeaServerNode::NmeaServerNode() : Node("nmea_socket_server"), acceptor_(io_context_) // Port set later
{
  // 1. Declare and Get Parameters
  this->declare_parameter<std::string>("protocol");
  this->declare_parameter<int>("port");
  this->declare_parameter<double>("gps_fix_pub_rate");
  this->declare_parameter<std::string>("gps_frame_id");

  protocol_ = this->get_parameter("protocol").as_string();
  gps_frame_id_ = this->get_parameter("gps_frame_id").as_string();
  port_ = this->get_parameter("port").as_int();
  gps_fix_pub_rate_ = this->get_parameter("gps_fix_pub_rate").as_double();

  std::transform(protocol_.begin(), protocol_.end(), protocol_.begin(), ::tolower);

  tcp::endpoint endpoint(tcp::v4(), port_);
  acceptor_.open(endpoint.protocol());
  acceptor_.set_option(boost::asio::socket_base::reuse_address(true));
  acceptor_.bind(endpoint);
  acceptor_.listen();

  RCLCPP_INFO(this->get_logger(), "Configured to use protocol: %s on port: %d", protocol_.c_str(), port_);

  // 2. Setup Publisher
  pub_gps_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_fix", 1);

  srv_start_accept_ = this->create_service<std_srvs::srv::Trigger>(
      "~/start_accept", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        startAccept();
        response->success = true;
      });

  startAccept();
}

inline void NmeaServerNode::runAsio() { io_context_.run(); }

inline void NmeaServerNode::stopAsio() { io_context_.stop(); }

void NmeaServerNode::startAccept()
{
  auto new_socket = std::make_shared<tcp::socket>(io_context_);

  acceptor_.async_accept(*new_socket,
                         [this, new_socket](const boost::system::error_code& ec) { handleAccept(new_socket, ec); });
}

void NmeaServerNode::handleAccept(std::shared_ptr<tcp::socket> socket, const boost::system::error_code& ec)
{
  if (!ec)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Client connected from: " << socket->remote_endpoint().address().to_string());

    startRead(socket);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Accept error: " << ec.message().c_str());
  }

  startAccept();
}

void NmeaServerNode::startRead(std::shared_ptr<tcp::socket> socket)
{
  auto buffer = std::make_shared<std::array<char, MAX_NMEA_LENGTH>>();

  socket->async_read_some(boost::asio::buffer(*buffer),
                          [this, socket, buffer](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                            handleRead(socket, buffer, ec, bytes_transferred);
                          });
}

void NmeaServerNode::handleRead(std::shared_ptr<tcp::socket> socket,
                                 std::shared_ptr<std::array<char, MAX_NMEA_LENGTH>> buffer,
                                 const boost::system::error_code& ec, std::size_t bytes_transferred)
{
  if (ec && ec == boost::asio::error::eof)
  {
    return;
  }

  if (ec && ec != boost::asio::error::eof)
  {
    return;
  }

  std::string data_str(buffer->data(), bytes_transferred);
  std::istringstream iss(data_str);

  std::string nmea_sentence;
  std::getline(iss, nmea_sentence);

  sensor_msgs::msg::NavSatFix msg_fix;

  if (!nmea_sentence.empty() && parse_nmea_and_fill_fix(nmea_sentence, msg_fix))
  {
    msg_fix.header.stamp = this->get_clock()->now();
    msg_fix.header.frame_id = gps_frame_id_;
    pub_gps_fix_->publish(msg_fix);
  }
  startRead(socket);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto server_node = std::make_shared<NmeaServerNode>();

  std::thread asio_thread([&]() { server_node->runAsio(); });

  rclcpp::spin(server_node);

  server_node->stopAsio();
  if (asio_thread.joinable())
  {
    asio_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
