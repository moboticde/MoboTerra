#pragma once
// Test-only message-shaped adapter for compiling the real CAN decoder without
// ROS. Never add this directory to ROS build targets: they use actual can_msgs.
#include <array>
#include <cstdint>
#include <string>

namespace can_msgs { namespace msg {
struct Frame
{
  struct Header
  {
    struct Stamp { int32_t sec{0}; uint32_t nanosec{0}; } stamp;
    std::string frame_id;
  } header;
  uint32_t id{0};
  bool is_rtr{false}, is_extended{false}, is_error{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};
}} // namespace can_msgs::msg
