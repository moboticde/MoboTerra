#pragma once
#include <mobotic_driver/communication.hpp>
#include <optional>

// Deterministic protocol/kinematic test double, not a motor or safety model.
class VirtualDriveProtocol
{
public:
  using Frame = can_msgs::msg::Frame;
  VirtualDriveProtocol(unsigned node, bool traction, double resolution = 4096.0,
                       double gearing = 121.0)
      : node_(node), traction_(traction), resolution_(resolution), gearing_(gearing)
  {
    if (node == 0 || node > 127 || !std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(gearing) || gearing <= 0)
    { throw std::invalid_argument("invalid virtual drive node/scaling"); }
  }
  bool enabled() const { return enabled_; }
  void setFault(uint8_t error, uint16_t code)
  {
    error_ = error; error_code_ = code;
    if (error || code) { setEnabled(false); statusword_ = 0x0008; }
  }
  void advance(double dt)
  {
    if (!std::isfinite(dt) || dt <= 0 || !enabled_ || traction_) { return; }
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
    if (frame.id == 0 && frame.dlc == 2 && (frame.data[1] == 0 || frame.data[1] == node_))
    {
      if (frame.data[0] == 1) { operational_ = true; sync_count_ = 0; }
      else if (frame.data[0] == 0x80 || frame.data[0] == 2)
      { operational_ = false; pending_rpdo_.reset(); setEnabled(false); }
      else if (frame.data[0] == 0x81 || frame.data[0] == 0x82) { reset(); }
      return {};
    }
    if (frame.id == 0x80 && frame.dlc == 0) { return sync(); }
    const auto rpdo_cob = objects_.find(key(0x1400, 1));
    if (traction_ && operational_ && rpdo_cob != objects_.end() &&
        frame.id == (rpdo_cob->second & 0x7FFu) &&
        !(rpdo_cob->second & CANOPEN_PDO_COB_ID_INVALID_BIT) &&
        objects_[key(0x1600, 0)] == 2 && frame.dlc == 6 &&
        objects_[key(0x1600, 1)] == EC_TRACTION_RPDO_CONTROLWORD_MAPPING &&
        objects_[key(0x1600, 2)] == EC_TRACTION_RPDO_TARGET_VELOCITY_MAPPING)
    { pending_rpdo_ = frame; return {}; }
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
  static unsigned key(unsigned index, unsigned sub) { return index * 256 + sub; }
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
    { velocity_ = current_ = profile_rpm_ = 0; target_position_ = checkedCanPosition(position_).value(); }
    if (!value) { pending_rpdo_.reset(); }
    enabled_ = value;
  }
  void controlword(uint16_t value)
  {
    if (value & 0x80) { error_ = 0; error_code_ = 0; }
    if (error_ || error_code_) { setEnabled(false); statusword_ = 8; return; }
    if ((value & 0x8F) == 0x0F) { statusword_ = 0x27; setEnabled(true); }
    else
    {
      setEnabled(false);
      statusword_ = (value & 0x8F) == 7 ? 0x23 : (value & 0x87) == 6 ? 0x21 : 0x40;
    }
  }
  void reset()
  {
    operational_ = false; setEnabled(false); mode_ = UNKNOWN;
    error_ = 0; error_code_ = 0; statusword_ = 0x40;
    objects_.clear(); pending_rpdo_.reset(); sync_count_ = 0;
  }
  std::optional<std::pair<uint32_t, unsigned>> read(unsigned index, unsigned sub) const
  {
    const auto result = [](int64_t value, unsigned size) {
      return std::make_pair(static_cast<uint32_t>(value), size);
    };
    if (traction_ && sub == 0)
    {
      switch (index)
      {
      case 0x1001: return result(error_, 1);
      case 0x603F: return result(error_code_, 2);
      case 0x6041: return result(statusword_, 2);
      case 0x6060: case 0x6061: return result(mode_, 1);
      case 0x606C: return result(velocity_, 4);
      case 0x6078: return result(current_, 2);
      case 0x22A2: return result(25, 2); // Synthetic temperature, no thermal model.
      default: break;
      }
    }
    if (!traction_)
    {
      if (sub == 0)
      {
        switch (index)
        {
        case 0x3001: return result(error_, 4);
        case 0x3003: return result(mode_, 4);
        case 0x3004: return result(enabled_ ? 1 : 0, 2);
        case 0x3762: return result(checkedCanPosition(position_).value(), 4);
        case 0x3111: return result(48000, 4); // Synthetic 48 V.
        case 0x3114: return result(25, 4);
        case 0x3262: return result(std::abs(static_cast<int64_t>(current_)), 4);
        default: break;
        }
      }
      if (sub == 1 && index == 0x3A04) { return result(velocity_, 4); }
      if (sub == 1 && index == 0x3262) { return result(current_, 4); }
    }
    const auto found = objects_.find(key(index, sub));
    if (found != objects_.end())
    {
      const bool comm = index == 0x1400 || (index >= 0x1800 && index <= 0x1802);
      const bool mapping = index == 0x1600 || (index >= 0x1A00 && index <= 0x1A02);
      return result(found->second, (comm && sub == 2) || (mapping && sub == 0) ? 1 : 4);
    }
    // In particular: never fabricate decoded STO/digital-input telemetry.
    return std::nullopt;
  }
  uint32_t write(unsigned index, unsigned sub, unsigned size, uint32_t value)
  {
    const bool comm = (index >= 0x1800 && index <= 0x1802) || (traction_ && index == 0x1400);
    const bool mapping = (index >= 0x1A00 && index <= 0x1A02) || (traction_ && index == 0x1600);
    if (comm || mapping)
    {
      const unsigned expected = comm ? (sub == 1 ? 4 : sub == 2 ? 1 : 0) : (sub == 0 ? 1 : sub <= 8 ? 4 : 0);
      if (!expected) { return 0x06090011; }
      if (size != expected) { return 0x06070010; }
      if (comm && sub == 1 && (value & ~0x800007FFu)) { return 0x06090030; }
      if (index == 0x1400 && sub == 1) { pending_rpdo_.reset(); }
      if (mapping && sub > 0 && objects_[key(index, 0)] != 0) { return 0x06010000; }
      if (mapping && sub == 0)
      {
        unsigned bits = 0;
        if (value > 8) { return 0x06040042; }
        for (unsigned i = 1; i <= value; ++i)
        {
          auto entry = objects_.find(key(index, i));
          if (entry == objects_.end()) { return 0x06040041; }
          const auto descriptor = entry->second;
          const auto object = read(descriptor >> 16, (descriptor >> 8) & 0xFF);
          const bool rpdo = index == 0x1600;
          const bool writable = descriptor == EC_TRACTION_RPDO_CONTROLWORD_MAPPING ||
                                descriptor == EC_TRACTION_RPDO_TARGET_VELOCITY_MAPPING;
          if (rpdo ? !writable : (!object || object->second * 8 != (descriptor & 0xFF))) { return 0x06040041; }
          bits += descriptor & 0xFF;
        }
        if (bits > 64) { return 0x06040042; }
      }
      objects_[key(index, sub)] = value; return 0;
    }
    if (sub != 0) { return 0x06090011; }
    unsigned expected = 0;
    if (traction_)
    {
      if (index == 0x6040) { expected = 2; }
      else if (index == 0x6060) { expected = 1; }
      else if (index == 0x60FF || index == 0x3200 || index == 0x607F || index == 0x6083 ||
               index == 0x6084 || index == 0x60C5 || index == 0x60C6) { expected = 4; }
    }
    else
    {
      if (index == 0x3000 || index == 0x3004) { expected = 2; }
      else if (index == 0x3003 || index == 0x3300 || index == 0x3500 || index == 0x3790 || index == 0x3200)
      { expected = 4; }
    }
    if (!expected) { return read(index, sub) ? 0x06010002 : 0x06020000; }
    if (size != expected) { return 0x06070010; }
    const auto signed_value = static_cast<int32_t>(value);
    if (traction_ && index == 0x6040) { controlword(static_cast<uint16_t>(value)); }
    else if (index == 0x3000) { if (value) { error_ = 0; error_code_ = 0; } }
    else if (index == 0x3004) { setEnabled(value != 0 && !error_); }
    else if (index == 0x3003 || index == 0x6060)
    {
      if ((traction_ && value != VELOCITY && value != CURRENT) ||
          (!traction_ && value != POSITION && value != S_VELOCITY && value != CURRENT)) { return 0x06090030; }
      if (mode_ != static_cast<DeviceMode>(value)) { setEnabled(false); }
      mode_ = static_cast<DeviceMode>(value);
    }
    else if (index == 0x3300) { profile_rpm_ = enabled_ ? signed_value : 0; }
    else if (index == 0x3790) { if (enabled_) { target_position_ = signed_value; } }
    else if (index == 0x60FF || index == 0x3500) { velocity_ = enabled_ ? signed_value : 0; }
    else if (index == 0x3200)
    {
      if (traction_ && (signed_value < -32768 || signed_value > 32767)) { return 0x06090030; }
      current_ = enabled_ ? signed_value : 0;
    }
    else { objects_[key(index, sub)] = value; }
    return 0;
  }
  std::vector<Frame> sync()
  {
    if (!operational_) { return {}; }
    if (pending_rpdo_)
    {
      const auto frame = *pending_rpdo_; pending_rpdo_.reset();
      controlword(static_cast<uint16_t>(shortFromFrame(frame, 0)));
      velocity_ = enabled_ && mode_ == VELOCITY ? intFromFrame(frame, 2) : 0;
    }
    ++sync_count_;
    std::vector<Frame> frames;
    for (unsigned pdo = 0; pdo < 3; ++pdo)
    {
      const auto cob = objects_.find(key(0x1800 + pdo, 1));
      const auto divider = objects_.find(key(0x1800 + pdo, 2));
      const auto count = objects_.find(key(0x1A00 + pdo, 0));
      if (cob == objects_.end() || cob->second & CANOPEN_PDO_COB_ID_INVALID_BIT ||
          divider == objects_.end() || divider->second == 0 || divider->second > 240 ||
          sync_count_ % divider->second != 0 || count == objects_.end() || count->second == 0) { continue; }
      Frame frame; frame.id = cob->second & 0x7FF; unsigned offset = 0;
      for (unsigned i = 1; i <= count->second; ++i)
      {
        const auto descriptor = objects_.at(key(0x1A00 + pdo, i));
        const auto object = read(descriptor >> 16, (descriptor >> 8) & 0xFF);
        if (!object || offset + object->second > 8) { offset = 0; break; }
        for (unsigned byte = 0; byte < object->second; ++byte)
        { frame.data[offset++] = (object->first >> (8 * byte)) & 0xFF; }
      }
      frame.dlc = offset;
      if (offset) { frames.push_back(frame); }
    }
    return frames;
  }
  unsigned node_;
  bool traction_, operational_{false}, enabled_{false};
  double resolution_, gearing_, position_{0};
  int32_t target_position_{0}, profile_rpm_{0}, velocity_{0}, current_{0};
  DeviceMode mode_{UNKNOWN};
  uint8_t error_{0}; uint16_t error_code_{0}, statusword_{0x40};
  uint64_t sync_count_{0};
  std::map<unsigned, uint32_t> objects_;
  std::optional<Frame> pending_rpdo_;
};
