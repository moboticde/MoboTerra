#pragma once
#include <mobotic_driver/communication.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <limits>
#include <optional>
#include <vector>

// Shared timing, per-node transactions, bus arbitration, and final sender queue.
// Define the minimum spacing once; both admission points use this value.
namespace can_scheduling
{
inline constexpr auto minimum_gap = std::chrono::milliseconds(2);
inline constexpr auto response_timeout = std::chrono::milliseconds(100);
}

// -----------------------------------------------------------------------------
// Per-drive acknowledged transactions
// -----------------------------------------------------------------------------
// One outstanding SDO per physical node. Polling never changes controller
// configuration. Operational writes are acknowledged and expire before they
// can become a delayed command after a lost supervisor/watchdog lease.
class SdoChannel
{
public:
  using Clock = std::chrono::steady_clock;
  void push(const can_msgs::msg::Frame& frame, Clock::time_point now = Clock::now(), double command_watchdog = 0.1)
  {
    const bool read = frame.data[0] == READ_FROM_DEVICE;
    const auto same = [&frame](const Entry& entry) {
      return entry.frame.id == frame.id && entry.frame.data == frame.data;
    };
    if (std::any_of(queue_.begin(), queue_.end(), same)) { return; }
    if (profileUpdate(frame))
    {
      if (pending_ && same(*pending_)) { return; }
      const auto proof = acknowledged_targets_.find(MICONTROL_CLEAR_ERROR_STATUS);
      if (proof != acknowledged_targets_.end() && same(proof->second)) { return; }
    }
    // A newer admitted move replaces an unsent profile stop. Never let its
    // positive speed be latched into an old target by a delayed stop Update.
    if (!read && shortFromFrame(frame, 1) == STEERING_PROFILE_VELOCITY_INPUT && intFromFrame(frame, 4) != 0)
    { queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [](const Entry& e) {
        return profileUpdate(e.frame);
      }), queue_.end()); }
    // Keep only the newest queued motion target for a given object. State and
    // explicit fault-reset writes retain their order.
    const unsigned index = static_cast<unsigned>(frame.data[1]) | (static_cast<unsigned>(frame.data[2]) << 8);
    if (!read && index != MICONTROL_DEVICE_STATE && index != MICONTROL_CLEAR_ERROR_STATUS)
    {
      queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&frame](const Entry& e) {
        return e.frame.data[0] != READ_FROM_DEVICE && e.frame.data[1] == frame.data[1] &&
               e.frame.data[2] == frame.data[2] && e.frame.data[3] == frame.data[3];
      }), queue_.end());
      // Retain the newest target above, then avoid duplicating an in-flight
      // identical target or an acknowledged target during its refresh window.
      if (pending_ && same(*pending_)) { return; }
      const auto acknowledged = acknowledged_targets_.find(index);
      // A proven zero is already stopped. Heartbeats retain controller
      // liveness; a new motion/state write invalidates this zero proof.
      const double refresh = zeroMotionCommand(frame) ? std::numeric_limits<double>::infinity() :
          std::min(0.05, command_watchdog / 2.0);
      if (motionTarget(frame) && acknowledged != acknowledged_targets_.end() && same(acknowledged->second) &&
          std::chrono::duration<double>(now - acknowledged->second.queued).count() < refresh) { return; }
    }
    if ((read || disableCommand(frame)) && pending_ && same(*pending_)) { return; }
    queue_.push_back({frame, now});
  }

  bool motionReadQueued() const
  { return std::any_of(queue_.begin(), queue_.end(), [](const Entry& e) { return motionRead(e.frame); }); }
  bool motionReadOutstanding() const
  { return motionReadQueued() || (pending_ && motionRead(pending_->frame)); }
  static bool requiredFaultRead(const can_msgs::msg::Frame& frame)
  {
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return frame.data[0] == READ_FROM_DEVICE &&
        index == MICONTROL_ERROR_REGISTER;
  }
  bool faultReadOutstanding() const
  {
    return (pending_ && requiredFaultRead(pending_->frame)) ||
        std::any_of(queue_.begin(), queue_.end(), [](const Entry& e) { return requiredFaultRead(e.frame); });
  }
  bool backgroundReadOutstanding() const
  {
    const auto background = [](const Entry& e) {
      return e.frame.data[0] == READ_FROM_DEVICE && !motionRead(e.frame) && !nativeRead(e.frame) && !requiredFaultRead(e.frame);
    };
    return (pending_ && background(*pending_)) || std::any_of(queue_.begin(), queue_.end(), background);
  }

  bool nativeReadOutstanding() const
  {
    return (pending_ && nativeRead(pending_->frame)) ||
        std::any_of(queue_.begin(), queue_.end(), [](const Entry& e) { return nativeRead(e.frame); });
  }

  std::optional<can_msgs::msg::Frame> next(double write_timeout, Clock::time_point now = Clock::now(),
      int64_t stamp_ns = 0, bool motion_phase = false, bool native_enable_fresh = true)
  {
    if (pending_ && now - sent_ > can_scheduling::response_timeout) { pending_.reset(); }
    if (pending_) { return std::nullopt; }
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [now, write_timeout, native_enable_fresh](const Entry& e) {
      return (!native_enable_fresh && enableCommand(e.frame)) ||
          (e.frame.data[0] != READ_FROM_DEVICE && !stopCommand(e.frame) &&
           std::chrono::duration<double>(now - e.queued).count() > write_timeout);
    }), queue_.end());
    if (queue_.empty()) { return std::nullopt; }
    auto selected = queue_.begin();
    // Alternate writes with reads under load so motion traffic cannot starve
    // the status/fault observations that authorize it.
    const auto disable = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return disableCommand(e.frame); });
    const auto stop = std::find_if(queue_.begin(), queue_.end(), [this](const Entry& e) {
      if (!stopCommand(e.frame)) { return false; }
      if (disableCommand(e.frame)) { return true; }
      const unsigned index = e.frame.data[1] | (unsigned(e.frame.data[2]) << 8);
      const auto ack = acknowledged_targets_.find(index);
      // A confirmed zero remains zero until a new motion write invalidates
      // its proof. Its periodic refresh shares ordinary service slots;
      // a new/unconfirmed stop and every disable still interrupt immediately.
      return ack == acknowledged_targets_.end() || ack->second.frame.data != e.frame.data;
    });
    const auto fault = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return requiredFaultRead(e.frame); });
    const auto native = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return nativeRead(e.frame); });
    const auto management = std::find_if(queue_.begin(), queue_.end(), [motion_phase](const Entry& e) {
      if (e.frame.data[0] == READ_FROM_DEVICE) { return false; }
      const unsigned index = e.frame.data[1] | (unsigned(e.frame.data[2]) << 8);
      const bool reset = index == MICONTROL_CLEAR_ERROR_STATUS && !profileUpdate(e.frame);
      return reset || (!motion_phase && enableCommand(e.frame));
    });
    if (disable != queue_.end()) { selected = disable; }
    else if (stop != queue_.end() && !last_was_stop_) { selected = stop; }
    else if (stop == queue_.end() && management != queue_.end()) { selected = management; }
    // Complete a bounded motion phase, then service the state/mode evidence
    // before optional telemetry and ordinary targets. The batch admission
    // barrier prevents the next motion generation from overtaking these reads.
    else if (!motion_phase && fault != queue_.end()) { selected = fault; }
    else if (!motion_phase && native != queue_.end()) { selected = native; }
    else if (motion_phase)
    {
      const auto motion = std::find_if(queue_.begin(), queue_.end(), [](const Entry& e) { return motionRead(e.frame); });
      if (motion != queue_.end()) { selected = motion; }
      else if (stop != queue_.end()) { selected = stop; }
      else { return std::nullopt; }
    }
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
    if (profileUpdate(selected->frame))
    {
      const auto proof = acknowledged_targets_.find(STEERING_PROFILE_VELOCITY_INPUT);
      if (proof == acknowledged_targets_.end() || !zeroMotionCommand(proof->second.frame))
      {
        auto zero = selected->frame;
        shortToFrame(zero, 1, STEERING_PROFILE_VELOCITY_INPUT); intToFrame(zero, 4, 0);
        const auto queued_zero = std::find_if(queue_.begin(), queue_.end(), [&zero](const Entry& e) {
          return e.frame.data == zero.data;
        });
        if (queued_zero != queue_.end()) { selected = queued_zero; }
        else { queue_.push_front({zero, now}); selected = queue_.begin(); }
      }
    }
    pending_ = *selected; queue_.erase(selected);
    const bool selected_write = pending_->frame.data[0] != READ_FROM_DEVICE;
    // Native/motion phases must not restart ordinary write/read alternation:
    // otherwise the first service slot always writes and faults never refresh.
    if (!nativeRead(pending_->frame) && !motionRead(pending_->frame))
    { last_was_write_ = selected_write; }
    if (selected_write)
    {
      const auto& frame = pending_->frame;
      if (!motionTarget(frame)) { acknowledged_targets_.clear(); }
      else
      {
        const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
        acknowledged_targets_.erase(index);
        // Any potentially moving write invalidates previous zero-stop proof,
        // including a new position target paired with a profile-velocity stop.
        if (!zeroMotionCommand(frame))
        {
          for (auto it = acknowledged_targets_.begin(); it != acknowledged_targets_.end();)
          { if (zeroMotionCommand(it->second.frame)) { it = acknowledged_targets_.erase(it); } else { ++it; } }
        }
      }
    }
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
        reply_stamp_ns < 0 || (sent_stamp_ns_ > 0 && reply_stamp_ns < sent_stamp_ns_)) { return false; }
    const bool read = request.data[0] == READ_FROM_DEVICE;
    if (frame.data[0] != ABORT_REPLY && !(read ? isReadReplyFromDevice(frame) : frame.data[0] == WRITE_REPLY)) { return false; }
    if (read && frame.data[0] != ABORT_REPLY && nativeRead(request))
    {
      const unsigned index = request.data[1] | (unsigned(request.data[2]) << 8);
      const auto value = intFromFrame(frame, 4);
      const auto previous = native_values_.find(index);
      if (previous == native_values_.end() || previous->second != value)
      { acknowledged_targets_.clear(); native_values_[index] = value; }
    }
    if (!read && frame.data[0] == WRITE_REPLY && motionTarget(request))
    { acknowledged_targets_[request.data[1] | (unsigned(request.data[2]) << 8)] = Entry{request, sent_}; }
    pending_.reset();
    return true;
  }

  void cancelWrites(bool preserve_reset = false, bool preserve_stop = false)
  {
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [preserve_reset, preserve_stop](const Entry& e) {
      if (e.frame.data[0] == READ_FROM_DEVICE) { return false; }
      if (preserve_stop && stopCommand(e.frame)) { return false; }
      const unsigned index = static_cast<unsigned>(e.frame.data[1]) | (static_cast<unsigned>(e.frame.data[2]) << 8);
      const bool reset = index == MICONTROL_CLEAR_ERROR_STATUS && !profileUpdate(e.frame);
      return !(preserve_reset && reset);
    }), queue_.end());
  }
  void cancelMotionTargets()
  {
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [](const Entry& e) {
      const unsigned index = static_cast<unsigned>(e.frame.data[1]) | (static_cast<unsigned>(e.frame.data[2]) << 8);
      return e.frame.data[0] != READ_FROM_DEVICE && index != MICONTROL_DEVICE_STATE && index != MICONTROL_CLEAR_ERROR_STATUS;
    }), queue_.end());
    for (auto it = acknowledged_targets_.begin(); it != acknowledged_targets_.end();)
    { if (!zeroMotionCommand(it->second.frame)) { it = acknowledged_targets_.erase(it); } else { ++it; } }
  }
  void clear() { queue_.clear(); pending_.reset(); acknowledged_targets_.clear(); native_values_.clear(); last_was_stop_ = false; }

  bool faultResetPending() const
  {
    const auto reset = [](const Entry& entry) {
      const auto& frame = entry.frame;
      if (frame.data[0] == READ_FROM_DEVICE || frame.data[3] != 0) { return false; }
      const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
      return (index == MICONTROL_CLEAR_ERROR_STATUS && shortFromFrame(frame, 4) == 1);
    };
    return (pending_ && reset(*pending_)) || std::any_of(queue_.begin(), queue_.end(), reset);
  }

private:
  static bool profileUpdate(const can_msgs::msg::Frame& frame)
  {
    return frame.data[0] != READ_FROM_DEVICE && frame.data[3] == 0 &&
        shortFromFrame(frame, 1) == MICONTROL_CLEAR_ERROR_STATUS && intFromFrame(frame, 4) == 5;
  }
  static bool enableCommand(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return index == MICONTROL_DEVICE_STATE && intFromFrame(frame, 4) != 0;
  }
  static bool nativeRead(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] != READ_FROM_DEVICE) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return index == MICONTROL_STATUS_REGISTER || index == MICONTROL_DEVICE_MODE;
  }
  static bool motionRead(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] != READ_FROM_DEVICE) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return index == POSITION_FEEDBACK || index == MICONTROL_VELOCITY_FEEDBACK;
  }
  static bool motionTarget(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return profileUpdate(frame) || index == STEERING_PROFILE_VELOCITY_INPUT ||
        index == S_VELOCITY_INPUT || index == CURRENT_INPUT || index == ABSOLUTE_POSITION_INPUT;
  }
  static bool zeroMotionCommand(const can_msgs::msg::Frame& frame)
  { return stopCommand(frame) && !disableCommand(frame); }
  static bool disableCommand(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE || frame.data[3] != 0) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    return (index == MICONTROL_DEVICE_STATE) && shortFromFrame(frame, 4) == 0;
  }
  static bool stopCommand(const can_msgs::msg::Frame& frame)
  {
    if (frame.data[0] == READ_FROM_DEVICE || frame.data[3] != 0) { return false; }
    const unsigned index = frame.data[1] | (unsigned(frame.data[2]) << 8);
    const bool state = index == MICONTROL_DEVICE_STATE;
    const bool target = index == STEERING_PROFILE_VELOCITY_INPUT ||
                        index == S_VELOCITY_INPUT || index == CURRENT_INPUT;
    return profileUpdate(frame) || (state && shortFromFrame(frame, 4) == 0) || (target && intFromFrame(frame, 4) == 0);
  }
  struct Entry { can_msgs::msg::Frame frame; Clock::time_point queued; };
  std::deque<Entry> queue_;
  std::optional<Entry> pending_;
  std::map<unsigned, Entry> acknowledged_targets_;
  std::map<unsigned, int32_t> native_values_;
  Clock::time_point sent_{};
  int64_t sent_stamp_ns_{0};
  bool last_was_write_{false};
  bool last_was_stop_{false};
};

// -----------------------------------------------------------------------------
// Shared round-robin drive admission
// -----------------------------------------------------------------------------
// Shared physical-bus admission, above the per-node acknowledged SDO queues.
// A late executor invocation grants one transmission, never catch-up credits.
class SdoBusScheduler
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr auto minimum_gap = can_scheduling::minimum_gap;
  void useFinalSenderPacing() { final_sender_paces_ = true; }

  template<class Select>
  std::optional<can_msgs::msg::Frame> next(const std::vector<unsigned>& nodes,
      Select&& select, Clock::time_point now = Clock::now())
  {
    if (nodes.empty() || (!final_sender_paces_ && last_sent_ && now - *last_sent_ < minimum_gap)) { return std::nullopt; }
    for (std::size_t offset = 0; offset < nodes.size(); ++offset)
    {
      const auto index = (cursor_ + offset) % nodes.size();
      if (auto frame = select(nodes[index]))
      {
        cursor_ = (index + 1) % nodes.size();
        last_sent_ = now;
        return frame;
      }
    }
    return std::nullopt;
  }

  // Anchor the next grant after publication, rather than before selection work.
  void published(Clock::time_point now = Clock::now()) { last_sent_ = now; }

private:
  bool final_sender_paces_{false};
  std::size_t cursor_{0};
  std::optional<Clock::time_point> last_sent_;
};

// -----------------------------------------------------------------------------
// Final bounded transmit queue
// -----------------------------------------------------------------------------
// Final, single-owner admission for an SDO-only physical bus. All producers
// (driver SDOs, PAIL SDO/NMT and heartbeat) share the same monotonic slot.
// The caller completes exactly one nonblocking socket attempt, then anchors
// the next slot after that attempt. A delayed executor earns no catch-up slots.
class CanTxQueue
{
public:
  using Frame = can_msgs::msg::Frame;
  using Clock = std::chrono::steady_clock;
  static constexpr auto minimum_gap = can_scheduling::minimum_gap;
  explicit CanTxQueue(int64_t max_age_ns = 50'000'000) : max_age_ns_(max_age_ns) {}
  static int64_t stamp(const Frame& f)
  { return int64_t(f.header.stamp.sec) * 1'000'000'000 + f.header.stamp.nanosec; }
  static unsigned object(const Frame& f) { return f.data[1] | (unsigned(f.data[2]) << 8); }
  static bool sdo(const Frame& f) { return f.id > 0x600 && f.id <= 0x67f && f.dlc == 8; }
  static bool write(const Frame& f) { return sdo(f) && (f.data[0] & 0xe0) == 0x20; }
  int64_t maximumAge(const Frame& f) const
  {
    // Uploads observe state; they cannot execute a delayed motion command.
    // Use the same bounded window as the driver's reply tracker.
    return sdo(f) && f.data[0] == 0x40 ?
        std::chrono::duration_cast<std::chrono::nanoseconds>(can_scheduling::response_timeout).count() : max_age_ns_;
  }
  static bool stop(const Frame& f)
  {
    if (!write(f) || f.data[3] != 0) { return false; }
    const unsigned index = object(f);
    if (index == 0x3000 && intFromFrame(f, 4) == 5) { return true; }
    if (index != 0x3004 && index != 0x3300 &&
        index != 0x3500 && index != 0x3200) { return false; }
    // Canonical stop writes use zero for every transmitted value byte.
    return std::all_of(f.data.begin() + 4, f.data.end(), [](auto b) { return b == 0; });
  }
  static bool heartbeat(const Frame& f) { return f.id > 0x700 && f.id <= 0x77f && f.dlc == 1; }
  static bool disable(const Frame& f) { return stop(f) && (object(f) == 0x3004); }
  static bool target(const Frame& f)
  {
    const unsigned index = object(f);
    return write(f) && (index == 0x3300 || index == 0x3500 ||
                       index == 0x3200 || index == 0x3790);
  }

  bool push(const Frame& frame, int64_t ros_now_ns)
  {
    expire(ros_now_ns);
    if (frame.is_error || frame.is_rtr || frame.is_extended || frame.id > 0x7ff || frame.dlc > 8 ||
        (!sdo(frame) && !heartbeat(frame) && !(frame.id == 0 && frame.dlc == 2)) ||
        frame.header.stamp.sec < 0 || frame.header.stamp.nanosec >= 1'000'000'000u ||
        stamp(frame) <= 0 || stamp(frame) > ros_now_ns)
    { ++dropped_; ++invalid_; return false; }
    if (!stop(frame) && ros_now_ns - stamp(frame) > maximumAge(frame))
    { ++dropped_; ++expired_; return false; }
    if (write(frame) && !stop(frame) &&
        (stamp(frame) <= disable_cutoff_[frame.id] || (target(frame) && stamp(frame) <= stop_cutoff_[frame.id])))
    { ++dropped_; return false; }
    if (stop(frame))
    {
      auto& cutoff = stop_cutoff_[frame.id]; cutoff = std::max(cutoff, stamp(frame));
      if (disable(frame)) { disable_cutoff_[frame.id] = std::max(disable_cutoff_[frame.id], stamp(frame)); }
      // Stop admission revokes older queued writes before any further send.
      // Reads and explicit reset pulses retain their diagnostic/recovery role.
      queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&](const Frame& queued) {
        const bool reset = object(queued) == 0x3000;
        if (queued.id == frame.id && write(queued) && !stop(queued) && !reset &&
            (disable(frame) || target(queued)) && stamp(queued) <= cutoff)
        { ++revoked_; return true; }
        return false;
      }), queue_.end());
    }
    for (auto& queued : queue_)
    {
      if (queued.id != frame.id) { continue; }
      if (heartbeat(frame) && heartbeat(queued))
      {
        if (stamp(frame) > stamp(queued)) { queued = frame; }
        ++coalesced_; return true;
      }
      if (queued.data == frame.data && queued.dlc == frame.dlc &&
          (stop(frame) || stamp(queued) == stamp(frame)))
      { ++coalesced_; return true; }
    }
    if (queue_.size() >= capacity)
    {
      if (!stop(frame)) { ++dropped_; ++capacity_drops_; return false; }
      const auto old = std::find_if(queue_.begin(), queue_.end(), [](const Frame& f) { return !stop(f); });
      if (old == queue_.end()) { ++dropped_; ++capacity_drops_; return false; }
      queue_.erase(old); ++dropped_; ++capacity_drops_;
    }
    queue_.push_back(frame);
    return true;
  }

  std::optional<Frame> next(int64_t ros_now_ns, Clock::time_point now = Clock::now())
  {
    expire(ros_now_ns);
    selected_.reset();
    if (queue_.empty() || (last_attempt_ && now - *last_attempt_ < minimum_gap)) { return std::nullopt; }
    auto chosen = std::find_if(queue_.begin(), queue_.end(), [](const Frame& f) { return stop(f); });
    if (chosen == queue_.end())
    { chosen = std::find_if(queue_.begin(), queue_.end(), [](const Frame& f) { return heartbeat(f); }); }
    if (chosen == queue_.end())
    {
      chosen = std::min_element(queue_.begin(), queue_.end(), [this](const Frame& a, const Frame& b) {
        return stamp(a) + maximumAge(a) < stamp(b) + maximumAge(b);
      });
    }
    selected_ = static_cast<std::size_t>(chosen - queue_.begin());
    return *chosen;
  }
  void complete(bool success, Clock::time_point after_attempt = Clock::now())
  {
    if (!selected_) { return; }
    last_attempt_ = after_attempt;
    if (success) { queue_.erase(queue_.begin() + *selected_); ++sent_; }
    else { ++failures_; } // Keep the frame for a bounded, paced retry.
    selected_.reset();
  }
  std::size_t size() const { return queue_.size(); }
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
  uint64_t invalid() const { return invalid_; }
  uint64_t expired() const { return expired_; }
  uint64_t capacityDrops() const { return capacity_drops_; }
  uint64_t coalesced() const { return coalesced_; }
  uint64_t revoked() const { return revoked_; }
  uint64_t failures() const { return failures_.load(std::memory_order_relaxed); }
  uint64_t sent() const { return sent_; }
  static constexpr std::size_t capacity = 64;
private:
  void expire(int64_t ros_now_ns)
  {
    if (last_ros_now_ && ros_now_ns < *last_ros_now_)
    {
      dropped_ += queue_.size(); queue_.clear();
      stop_cutoff_.clear(); disable_cutoff_.clear(); selected_.reset();
    }
    last_ros_now_ = ros_now_ns;
    queue_.erase(std::remove_if(queue_.begin(), queue_.end(), [&](const Frame& f) {
      if (stamp(f) > ros_now_ns || (!stop(f) && ros_now_ns - stamp(f) > maximumAge(f)))
      { ++dropped_; ++expired_; return true; }
      return false;
    }), queue_.end());
  }
  int64_t max_age_ns_;
  std::deque<Frame> queue_;
  std::map<unsigned, int64_t> stop_cutoff_;
  std::map<unsigned, int64_t> disable_cutoff_;
  std::optional<int64_t> last_ros_now_;
  std::optional<std::size_t> selected_;
  std::optional<Clock::time_point> last_attempt_;
  std::atomic<uint64_t> dropped_{0}, failures_{0};
  uint64_t coalesced_{0}, revoked_{0}, sent_{0}, invalid_{0}, expired_{0}, capacity_drops_{0};
};

// -----------------------------------------------------------------------------
// ROS and SocketCAN sender
// -----------------------------------------------------------------------------
class PacedSocketCanSender : public rclcpp::Node
{
public:
  using Transport = std::function<void(const can_msgs::msg::Frame&)>;
  // A transport injection is available only to C++ controller-model tests;
  // production has no parameter that substitutes a fake physical sender.
  explicit PacedSocketCanSender(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), Transport transport = {});
  ~PacedSocketCanSender() override;
  uint64_t droppedFrames() const { return queue_.dropped(); }
  uint64_t sendFailures() const { return queue_.failures(); }
private:
  void drain();
  void openSocket(const std::string& interface);
  int socket_{-1};
  Transport transport_;
  CanTxQueue queue_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr accepted_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
  std::optional<CanTxQueue::Clock::time_point> last_success_;
  double min_gap_ms_{0}, max_queue_age_ms_{0};
};
