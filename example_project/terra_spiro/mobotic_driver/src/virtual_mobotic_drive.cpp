#include <iomanip>
#include <iostream>
#include <mobotic_driver/frame_operations.h>
#include <mobotic_driver/virtual_mobotic_drive.h>

VirtualMoboticDriveCanCommunication::VirtualMoboticDriveCanCommunication(const rclcpp::NodeOptions& options)
    : Node("virtual_mobotic_drive", options)
{

  this->declare_parameter("can_node_id", "0x00");

  bool params{true};
  std::string can_node_id_str;
  params &= this->get_parameter("can_node_id", can_node_id_str);

  can_node_id_ = std::stoi(can_node_id_str, nullptr, 16);

  if (!params)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "[mobotic_driver] Failed to load parameters from ROS parameter server. Required parameters are: "
                 "can_node_id.");
    exit(1);
  }

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));
  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()), [this](const can_msgs::msg::Frame& frame) { canFrameCallback(frame); });
}

void VirtualMoboticDriveCanCommunication::canFrameCallback(const can_msgs::msg::Frame& req_frame)
{

  can_msgs::msg::Frame res_frame;
  res_frame.id = 0x580 + can_node_id_;
  res_frame.dlc = 8;
  res_frame.is_rtr = false;
  res_frame.is_extended = false;
  res_frame.is_error = false;
  std::fill(res_frame.data.begin(), res_frame.data.end(), 0);

  if (req_frame.id != (0x600 + can_node_id_))
  {
    return;
  }

  if (req_frame.data[0] == 0x23 && req_frame.data[1] == 0x04 && req_frame.data[2] == 0x30)
  {
    RCLCPP_INFO(this->get_logger(), "Enable/disable device request received.");
    enabled_ = req_frame.data[4];
    return;
  }

  if (req_frame.data[0] == 0x23 && req_frame.data[1] == 0x03 && req_frame.data[2] == 0x30) // Set device mode
  {
    mode_ = static_cast<DeviceMode>(req_frame.data[4]);
    return;
  }

  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x01 && req_frame.data[2] == 0x30)
  // Read error register request
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x01;
    res_frame.data[2] = 0x30;
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    pub_can_frames_->publish(res_frame);
  }
  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x03 && req_frame.data[2] == 0x30) // Read device mode
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x03;
    res_frame.data[2] = 0x30;
    res_frame.data[4] = mode_;
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    pub_can_frames_->publish(res_frame);
  }
  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x04 && req_frame.data[2] == 0x30) // Read device state
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x04;
    res_frame.data[2] = 0x30;
    res_frame.data[4] = enabled_ ? 0x01 : 0x00;
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    pub_can_frames_->publish(res_frame);
  }

  if (!enabled_)
  {
    return;
  }

  if (req_frame.data[0] == 0x23 && req_frame.data[1] == 0x00 && req_frame.data[2] == 0x32)
  {
    // Set current
    current_ = intFromFrame(req_frame, 4);
    return;
  }

  if (req_frame.data[0] == 0x23 && req_frame.data[1] == 0x00 &&
      (req_frame.data[2] == 0x35 || req_frame.data[2] == 0x33))
  {
    // Set velocity
    velocity_ = intFromFrame(req_frame, 4);
    return;
  }

  if (req_frame.data[0] == 0x23 && req_frame.data[1] == 0x90 && req_frame.data[2] == 0x37)
  {
    // Set position
    position_ = intFromFrame(req_frame, 4);
    return;
  }
  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x62 && req_frame.data[2] == 0x32) // Read current feedback
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x62;
    res_frame.data[2] = 0x32;
    intToFrame(res_frame, 4, current_);
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    pub_can_frames_->publish(res_frame);
  }
  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x04 && req_frame.data[2] == 0x3A && req_frame.data[3] == 0x01)
  // Read velocity feedback
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x04;
    res_frame.data[2] = 0x3A;
    res_frame.data[3] = 0x01;
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    intToFrame(res_frame, 4, velocity_);
    pub_can_frames_->publish(res_frame);
  }
  if (req_frame.data[0] == 0x40 && req_frame.data[1] == 0x62 && req_frame.data[2] == 0x37) // Read position feedback
  {
    res_frame.data[0] = 0x43;
    res_frame.data[1] = 0x62;
    res_frame.data[2] = 0x37;
    res_frame.data[3] = 0x00;
    res_frame.header.frame_id = "can";
    res_frame.header.stamp = this->now();
    intToFrame(res_frame, 4, position_);
    pub_can_frames_->publish(res_frame);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  VirtualMoboticDriveCanCommunication drive{rclcpp::NodeOptions()};
  rclcpp::spin(drive.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}