#pragma once
#include <mobotic_driver/communication.hpp>
#include <optional>

// Deterministic protocol/kinematic test double, not a motor or safety model.
class VirtualDriveProtocol
{
public:
  using Frame = can_msgs::msg::Frame;
  VirtualDriveProtocol(unsigned node, bool traction, double resolution = 4096.0,
                       double gearing = 121.0, const std::string& controller_type = "")
      : node_(node), traction_joint_(traction), resolution_(resolution), gearing_(gearing)
  {
    if (!controller_type.empty() && controller_type != "micontrol")
    { throw std::invalid_argument("only micontrol controllers are supported"); }
    if (node == 0 || node > 127 || !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(gearing) || gearing <= 0)
    { throw std::invalid_argument("invalid virtual drive node/scaling"); }
  }
  bool enabled() const { return enabled_; }
  // Test fixture setup only; no CAN command or hardware encoder write.
  void setInitialPosition(int32_t value)
  {
    if (enabled_) { throw std::logic_error("initial position requires a disabled model"); }
    position_ = value; target_position_ = value;
  }
  // Model only the requested/actual enable distinction needed by status tests.
  void setHardwareEnable(bool value)
  {
    hardware_enable_ = value;
    setEnabled(software_enable_requested_ && hardware_enable_ && !error_);
  }
  void setFault(uint32_t error)
  {
    error_ = error;
    if (error_) { setEnabled(false); }
  }
  void advance(double dt)
  {
    if (!std::isfinite(dt) || dt <= 0 || !enabled_) { return; }
    if (mode_ == VELOCITY || mode_ == S_VELOCITY)
    { position_ += static_cast<double>(velocity_) * resolution_ * dt / (60.0 * (traction_joint_ ? 1.0 : gearing_)); return; }
    const double previous = position_;
    if (mode_ == POSITION)
    {
      const double step = std::abs(static_cast<double>(profile_rpm_)) * resolution_ * dt / (60.0 * gearing_);
      position_ += std::clamp(static_cast<double>(target_position_) - position_, -step, step);
      const double rpm = (position_ - previous) * 60.0 * gearing_ / (resolution_ * dt);
      velocity_ = checkedCanPosition(rpm).value_or(0);
    }
  }
  std::vector<Frame> handle(const Frame& frame)
  {
    if (frame.is_error || frame.is_extended || frame.is_rtr || frame.dlc > 8) { return {}; }
    if (frame.id != 0x600 + node_ || frame.dlc != 8) { return {}; }
    const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1)), sub = frame.data[3];
    if (frame.data[0] == READ_FROM_DEVICE)
    {
      const auto object = read(index, sub);
      if (!object) { return {abort(frame, 0x06020000)}; }
      auto reply = response(frame, static_cast<uint8_t>(0x43 + (4 - object->second) * 4));
      uint32ToFrame(reply, 4, object->first);
      return {reply};
    }
    unsigned size = frame.data[0] == WRITE_TO_DEVICE_1B ? 1 :
                    frame.data[0] == WRITE_TO_DEVICE_2B ? 2 :
                    frame.data[0] == WRITE_TO_DEVICE_4B ? 4 : 0;
    if (!size) { return {abort(frame, 0x05040001)}; }
    uint32_t value = 0;
    for (unsigned byte = 0; byte < size; ++byte) { value |= uint32_t(frame.data[4 + byte]) << (8 * byte); }
    const auto error = write(index, sub, size, value);
    return {error ? abort(frame, error) : response(frame, WRITE_REPLY)};
  }
private:
  Frame response(const Frame& request, uint8_t command) const
  {
    Frame reply; reply.id = 0x580 + node_; reply.dlc = 8;
    reply.data[0] = command;
    for (unsigned i = 1; i < 4; ++i) { reply.data[i] = request.data[i]; }
    return reply;
  }
  Frame abort(const Frame& request, uint32_t code) const
  { auto reply = response(request, ABORT_REPLY); uint32ToFrame(reply, 4, code); return reply; }
  void setEnabled(bool value)
  {
    if (!value || !enabled_)
    { velocity_ = current_ = profile_rpm_ = pending_profile_rpm_ = 0; target_position_ = checkedCanPosition(position_).value(); }
    enabled_ = value;
  }

  std::optional<std::pair<uint32_t, unsigned>> read(unsigned index, unsigned sub) const
  {
    const auto result = [](int64_t value, unsigned size) {
      return std::make_pair(static_cast<uint32_t>(value), size);
    };

    if (sub == 0)
    {
      switch (index)
      {
      case 0x3001: return result(error_, 4);
      case 0x3002: return result((enabled_ ? 1u : 0u) | (error_ ? 2u : 0u), 4);
      case 0x3003: return result(mode_, 4);
      case 0x3004: return result(software_enable_requested_ ? 1 : 0, 4);
      case 0x3762: return result(checkedCanPosition(position_).value(), 4);
      case 0x3111: return result(48000, 4); // Synthetic 48 V.
      case 0x3114: return result(25, 4);
      case 0x3262: return result(std::abs(static_cast<int64_t>(current_)), 4);
      default: break;
      }
    }
    if (sub == 1 && index == 0x3A04) { return result(velocity_, 4); }
    if (sub == 1 && index == 0x3262) { return result(current_, 4); }

    // In particular: never fabricate decoded STO/digital-input telemetry.
    return std::nullopt;
  }
  uint32_t write(unsigned index, unsigned sub, unsigned size, uint32_t value)
  {
    if (sub != 0) { return 0x06090011; }
    unsigned expected = 0;
    if (index == 0x3000 || index == 0x3004 || index == 0x3003 || index == 0x3300 ||
        index == 0x3500 || index == 0x3790 || index == 0x3200)
    { expected = 4; }
    if (!expected) { return read(index, sub) ? 0x06010002 : 0x06020000; }
    if (size != expected) { return 0x06070010; }
    const auto signed_value = static_cast<int32_t>(value);
    if (index == 0x3000)
    {
      if (value == 1) { error_ = 0; }
      else if (value == 5 && mode_ == POSITION) { profile_rpm_ = pending_profile_rpm_; }
    }
    else if (index == 0x3004)
    {
      software_enable_requested_ = value != 0;
      setEnabled(software_enable_requested_ && hardware_enable_ && !error_);
    }
    else if (index == 0x3003)
    {
      if (value != POSITION && value != VELOCITY && value != S_VELOCITY && value != CURRENT) { return 0x06090030; }
      if (mode_ != static_cast<DeviceMode>(value)) { setEnabled(false); }
      mode_ = static_cast<DeviceMode>(value);
    }
    else if (index == 0x3300)
    {
      if (mode_ == POSITION) { pending_profile_rpm_ = signed_value; }
      else if (mode_ == VELOCITY || !enabled_) { velocity_ = enabled_ ? signed_value : 0; }
      else { return 0x06090030; }
    }
    else if (index == 0x3790) { if (enabled_) { target_position_ = signed_value; profile_rpm_ = pending_profile_rpm_; } }
    else if (index == 0x3500) { velocity_ = enabled_ ? signed_value : 0; }
    else if (index == 0x3200)
    {
      current_ = enabled_ ? signed_value : 0;
    }
    return 0;
  }

  unsigned node_;
  bool enabled_{false};
  bool software_enable_requested_{false}, hardware_enable_{true};
  bool traction_joint_;
  double resolution_, gearing_, position_{0};
  int32_t target_position_{0}, profile_rpm_{0}, pending_profile_rpm_{0}, velocity_{0}, current_{0};
  DeviceMode mode_{UNKNOWN};
  uint32_t error_{0};
};
