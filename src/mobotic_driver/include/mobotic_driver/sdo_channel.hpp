#pragma once
#include <mobotic_driver/communication.hpp>
#include <algorithm>
#include <chrono>
#include <deque>
#include <optional>

// One outstanding SDO per physical node. Polling never changes controller
// configuration. Operational writes are acknowledged and expire before they
// can become a delayed command after a lost supervisor/watchdog lease.
class SdoChannel
{
public:
  using Clock = std::chrono::steady_clock;
  void push(const can_msgs::msg::Frame& frame, Clock::time_point now = Clock::now())
  {
    const bool read = frame.data[0] == READ_FROM_DEVICE;
    const auto same = [&frame](const Entry& entry) {
      return entry.frame.id == frame.id && entry.frame.data == frame.data;
    };
    if ((read && pending_ && same(*pending_)) || std::any_of(queue_.begin(), queue_.end(), same)) { return; }
    // Keep only the newest queued motion target for a given object. State and
    // explicit fault-reset writes retain their order.
    const unsigned index = static_cast<unsigned>(frame.data[1]) | (static_cast<unsigned>(frame.data[2]) << 8);
    if (!read && index != EC_CONTROLWORD && index != MICONTROL_DEVICE_STATE && index != MICONTROL_CLEAR_ERROR_STATUS)
    {
      queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&frame](const Entry& e) {
        return e.frame.data[0] != READ_FROM_DEVICE && e.frame.data[1] == frame.data[1] &&
               e.frame.data[2] == frame.data[2] && e.frame.data[3] == frame.data[3];
      }), queue_.end());
    }
    queue_.push_back({frame, now});
  }

  std::optional<can_msgs::msg::Frame> next(double write_timeout, Clock::time_point now = Clock::now(), int64_t stamp_ns = 0)
  {
    if (pending_ && std::chrono::duration<double>(now - sent_).count() > 0.1) { pending_.reset(); }
    if (pending_) { return std::nullopt; }
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [now, write_timeout](const Entry& e) {
      return e.frame.data[0] != READ_FROM_DEVICE && !stopCommand(e.frame) &&
          std::chrono::duration<double>(now - e.queued).count() > write_timeout;
    }), queue_.end());
    if (queue_.empty()) { return std::nullopt; }
    auto selected = queue_.begin();
    // Alternate writes with reads under load so motion traffic cannot starve
    // the status/fault observations that authorize it.
    const auto disable = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return disableCommand(e.frame); });
    const auto stop = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return stopCommand(e.frame); });
    if (disable != queue_.end()) { selected = disable; }
    else if (stop != queue_.end() && !last_was_stop_) { selected = stop; }
    else if (!last_was_write_)
    {
      const auto write = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return e.frame.data[0] != READ_FROM_DEVICE; });
      if (write != queue_.end()) { selected = write; }
    }
    else
    {
      const auto read = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return e.frame.data[0] == READ_FROM_DEVICE; });
      if (read != queue_.end()) { selected = read; }
    }
    pending_ = *selected; queue_.erase(selected);
    last_was_write_ = pending_->frame.data[0] != READ_FROM_DEVICE;
    last_was_stop_ = stopCommand(pending_->frame);
    sent_ = now; sent_stamp_ns_ = stamp_ns;
    return pending_->frame;
  }

  bool receive(const can_msgs::msg::Frame& frame)
  {
    if (!pending_ || frame.is_error || frame.is_rtr || frame.is_extended || frame.dlc != 8) { return false; }
    const auto& request = pending_->frame;
    const auto reply_stamp_ns = sourceStampNanoseconds(frame);
    if (frame.id != request.id - 0x600 + 0x580 || frame.data[1] != request.data[1] ||
        frame.data[2] != request.data[2] || frame.data[3] != request.data[3] ||
        reply_stamp_ns < 0 || (sent_stamp_ns_ > 0 && reply_stamp_ns > 0 && reply_stamp_ns < sent_stamp_ns_)) { return false; }
    const bool read = request.data[0] == READ_FROM_DEVICE;
    if (frame.data[0] != ABORT_REPLY && !(read ? isReadReplyFromDevice(frame) : frame.data[0] == WRITE_REPLY)) { return false; }
    pending_.reset();
    return true;
  }

  void cancelWrites(bool preserve_reset = false)
  {
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [preserve_reset](const Entry& e) {
      if (e.frame.data[0] == READ_FROM_DEVICE) { return false; }
      const unsigned index = static_cast<unsigned>(e.frame.data[1]) | (static_cast<unsigned>(e.frame.data[2]) << 8);
      const bool reset = index == MICONTROL_CLEAR_ERROR_STATUS ||
          (index == EC_CONTROLWORD && (static_cast<unsigned short>(shortFromFrame(e.frame, 4)) & 0x0080));
      return !(preserve_reset && reset);
    }), queue_.end());
  }
  void cancelMotionTargets()
  {
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [](const Entry& e) {
      const unsigned index = static_cast<unsigned>(e.frame.data[1]) | (static_cast<unsigned>(e.frame.data[2]) << 8);
      return e.frame.data[0] != READ_FROM_DEVICE && index != EC_CONTROLWORD &&
             index != MICONTROL_DEVICE_STATE && index != MICONTROL_CLEAR_ERROR_STATUS;
    }), queue_.end());
  }
  void clear() { queue_.clear(); pending_.reset(); last_was_stop_ = false; }

private:
  static bool disableCommand(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE || frame.data[3] != 0) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return (index == EC_CONTROLWORD || index == MICONTROL_DEVICE_STATE) && shortFromFrame(frame, 4) == 0;
  }
  static bool stopCommand(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE || frame.data[3] != 0) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    const bool state = index == EC_CONTROLWORD || index == MICONTROL_DEVICE_STATE;
    const bool target = index == EC_TRACTION_VELOCITY_INPUT || index == STEERING_PROFILE_VELOCITY_INPUT;
    return (state && shortFromFrame(frame, 4) == 0) || (target && intFromFrame(frame, 4) == 0);
  }
  struct Entry { can_msgs::msg::Frame frame; Clock::time_point queued; };
  std::deque<Entry> queue_;
  std::optional<Entry> pending_;
  Clock::time_point sent_{};
  int64_t sent_stamp_ns_{0};
  bool last_was_write_{false};
  bool last_was_stop_{false};
};
