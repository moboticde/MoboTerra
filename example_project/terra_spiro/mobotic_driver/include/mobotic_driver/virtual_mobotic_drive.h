#ifndef VIRTUAL_MOBOTIC_DRIVE_H
#define VIRTUAL_MOBOTIC_DRIVE_H
#include "communication.hpp"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

class VirtualMoboticDriveCanCommunication final : public rclcpp::Node
{
public:
  explicit VirtualMoboticDriveCanCommunication(const rclcpp::NodeOptions& options);
  void canFrameCallback(const can_msgs::msg::Frame& req_frame);

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_frames_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_frames_;
  DeviceMode mode_{DeviceMode::UNKNOWN};
  unsigned can_node_id_{};
  int velocity_{}, position_{}, current_{};
  bool enabled_{false};
};
#endif // VIRTUAL_MOBOTIC_DRIVE_H
