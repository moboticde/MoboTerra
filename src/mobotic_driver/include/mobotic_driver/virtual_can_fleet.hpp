#pragma once
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <map>

// A virtual bus only: each CAN node retains an independent controller model.
// Dispatch locally instead of delivering every request to eight DDS subscribers.
class VirtualCanFleet
{
public:
  void add(unsigned node, bool traction, double resolution, double gearing,
           const std::string& controller)
  {
    if (models_.count(node)) { throw std::invalid_argument("duplicate virtual CAN node"); }
    models_.emplace(node, VirtualDriveProtocol(node, traction, resolution, gearing, controller));
  }
  void advance(double seconds)
  { for (auto& item : models_) { item.second.advance(seconds); } }
  std::vector<can_msgs::msg::Frame> handle(const can_msgs::msg::Frame& frame)
  {
    if (frame.id <= 0x600 || frame.id > 0x67f) { return {}; }
    const auto model = models_.find(frame.id - 0x600);
    return model == models_.end() ? std::vector<can_msgs::msg::Frame>{} : model->second.handle(frame);
  }
private:
  std::map<unsigned, VirtualDriveProtocol> models_;
};
