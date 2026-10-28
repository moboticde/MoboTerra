#include <iomanip>
#include <iostream>
#include <vanguard_battery_driver/communication.h>
#include <vanguard_battery_driver/frame_operations.h>
#include <vanguard_battery_driver/virtual_vanguard_battery.h>

VirtualVanguardBattery::VirtualVanguardBattery(const rclcpp::NodeOptions& options)
    : Node("virtual_vanguard_battery", options)
{

  this->declare_parameter("battery_id", "");
  this->declare_parameter("source_address", "");
  this->declare_parameter("status_pgn", "");
  this->declare_parameter("controller_address", "");

  bool params{true};
  std::string source_address_str, status_pgn_str, controller_address_str;
  params &= this->get_parameter("battery_id", battery_id_);
  params &= this->get_parameter("source_address", source_address_str);
  params &= this->get_parameter("status_pgn", status_pgn_str);
  params &= this->get_parameter("controller_address", controller_address_str);

  status_pgn_ = std::stoi(status_pgn_str, nullptr, 16);
  source_address_ = std::stoi(source_address_str, nullptr, 16);
  controller_address_ = std::stoi(controller_address_str, nullptr, 16);
  operational_status_ = OperationalStatus::INITIALIZATION_IN_PROGRESS;
  high_voltage_status_ = HighVoltageStatus::DISCONNECTED;

  if (!params)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "[mobotic_driver] Failed to load parameters from ROS parameter server. Required parameters are: "
                 "battery_id, source_address, status_pgn.");
    exit(1);
  }

  pub_can_frames_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()));
  sub_can_frames_ = this->create_subscription<can_msgs::msg::Frame>(
      "can_rx", rclcpp::QoS(rclcpp::KeepAll()), [this](const can_msgs::msg::Frame& frame) { canFrameCallback(frame); });

  timer_feedback_ = this->create_timer(std::chrono::duration<double>(0.02), [this]() {
    can_msgs::msg::Frame frame = createFrameAndFillHeader(source_address_, 3, status_pgn_);
    frame.data[5] = static_cast<uint8_t>(operational_status_) << 4 | static_cast<uint8_t>(high_voltage_status_);
    frame.header.stamp = this->now();
    frame.header.frame_id = battery_id_;
    pub_can_frames_->publish(frame);
  });
}

void VirtualVanguardBattery::canFrameCallback(const can_msgs::msg::Frame& req_frame)
{

  uint8_t source_address, priority;
  uint32_t status_pgn;
  extract_arb_id_elements(req_frame.id, priority, status_pgn, source_address);

  can_msgs::msg::Frame res_frame;
  if (status_pgn != 7155 || source_address != controller_address_)
  {
    return;
  }

  if (req_frame.data[1] == 0xFF)
  {
    if (!seen_op_consent_3)
    {
      seen_op_consent_3 = true;
      time_first_op_consent_3 = this->get_clock()->now();
    }
    if (seen_op_consent_3)
    {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[virtual_vanguard_battery] Battery "
                                                 << battery_id_ << " has seen op_consent=3 for "
                                                 << (this->get_clock()->now() - time_first_op_consent_3).seconds()
                                                 << " seconds.");
    }
  }

  if (req_frame.data[1] == 0xDF)
  {
    if (!seen_op_consent_3)
    {
      RCLCPP_FATAL_STREAM_ONCE(this->get_logger(), "[virtual_vanguard_battery] Battery "
                                                  << battery_id_ << " has not seen op_consent=3 upon startup!");
      exit(1);
    }
    if (!seen_op_consent_1)
    {
      seen_op_consent_1 = true;
      time_first_op_consent_1 = this->get_clock()->now();
    }
    if (seen_op_consent_1)
    {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[virtual_vanguard_battery] Battery "
                                                 << battery_id_ << " has seen op_consent=1 for "
                                                 << (this->get_clock()->now() - time_first_op_consent_1).seconds()
                                                 << " seconds.");
      operational_status_ = OperationalStatus::OPERATIONAL;
    }
  }

  if (req_frame.data[0] == 0xFD)
  {
    if (!seen_op_consent_1 || !seen_op_consent_3)
    {
      RCLCPP_FATAL_STREAM_ONCE(this->get_logger(), "[virtual_vanguard_battery] Battery "
                                                  << battery_id_
                                                  << " has been instructed to connect to high-voltage bus without "
                                                     "seeing the required op_consent variables!");
      exit(1);
    }
    if (!seen_high_voltage_1)
    {
      time_first_high_voltage_1 = this->get_clock()->now();
      seen_high_voltage_1 = true;
    }
    if (seen_high_voltage_1)
    {
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[virtual_vanguard_battery] Battery "
                                                 << battery_id_ << " has seen high voltage command for "
                                                 << (this->get_clock()->now() - time_first_high_voltage_1).seconds()
                                                 << " seconds.");
      high_voltage_status_ = HighVoltageStatus::CONNECTED;
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions());
  VirtualVanguardBattery drive{rclcpp::NodeOptions()};
  rclcpp::spin(drive.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}