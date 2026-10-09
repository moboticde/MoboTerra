#include <mobotic_driver/virtual_mobotic_drive.h>
#include <chrono>

VirtualMoboticDriveCanCommunication::VirtualMoboticDriveCanCommunication(const rclcpp::NodeOptions& options)
    : Node("virtual_mobotic_drive", options)
{
  const auto id = declare_parameter<std::string>("can_node_id", "0x01");
  const auto role = declare_parameter<std::string>("drive_type", "steering");
  const auto resolution = declare_parameter<double>("encoder_resolution", 4096.0);
  const auto gearing = declare_parameter<double>("gear_ratio", role == "traction" ? 16.0 : 121.0);
  if (role != "steering" && role != "traction") { throw std::invalid_argument("drive_type must be steering or traction"); }
  std::size_t consumed = 0;
  const auto node_id = std::stoul(id, &consumed, 16);
  if (consumed != id.size() || node_id == 0 || node_id > 127) { throw std::invalid_argument("can_node_id must be 0x01..0x7F"); }
  protocol_ = std::make_unique<VirtualDriveProtocol>(node_id, role == "traction", resolution, gearing);
  last_update_ = now();
  // Every simulator receives all eight units' bootstrap configuration frames.
  pub_can_frames_ = create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(1000));
  sub_can_frames_ = create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(1000), [this](const can_msgs::msg::Frame& frame) { canFrameCallback(frame); });
  motion_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { advanceMotion(); });
  RCLCPP_WARN(get_logger(), "Virtual drive: synthetic protocol/kinematics only; no physical safety or motor model.");
}

void VirtualMoboticDriveCanCommunication::advanceMotion()
{
  const auto current = now();
  const double dt = (current - last_update_).seconds();
  last_update_ = current;
  protocol_->advance(std::clamp(dt, 0.0, 0.1));
}

void VirtualMoboticDriveCanCommunication::canFrameCallback(const can_msgs::msg::Frame& request)
{
  advanceMotion();
  for (auto reply : protocol_->handle(request))
  {
    reply.header.frame_id = "can";
    reply.header.stamp = now();
    pub_can_frames_->publish(reply);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualMoboticDriveCanCommunication>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
