#pragma once
#include <mobotic_driver/communication.hpp>
#include <chrono>
#include <deque>
#include <optional>

// One SDO at a time per node. Every write is acknowledged then read back.
class PdoConfiguration
{
public:
  using Clock = std::chrono::steady_clock;
  void start(std::vector<can_msgs::msg::Frame> frames)
  { queue_ = std::deque<can_msgs::msg::Frame>(frames.begin(), frames.end()); failed_ = waiting_ = reading_ = false; }
  bool ready() const { return !failed_ && queue_.empty() && !waiting_; }
  bool failed() const { return failed_; }
  void invalidate() { queue_.clear(); waiting_ = reading_ = false; failed_ = true; }
  std::optional<can_msgs::msg::Frame> next(Clock::time_point now = Clock::now(), int64_t source_stamp_ns = 0)
  {
    if (waiting_ && now - sent_ > std::chrono::seconds(2)) { failed_ = true; }
    if (failed_ || waiting_ || queue_.empty()) { return std::nullopt; }
    auto frame = queue_.front();
    if (frame.id == 0) { queue_.pop_front(); return frame; }
    if (reading_) { frame.data[0] = READ_FROM_DEVICE; }
    waiting_ = true; sent_ = now; sent_source_stamp_ns_ = source_stamp_ns;
    return frame;
  }
  void receive(const can_msgs::msg::Frame& frame)
  {
    if (!waiting_ || queue_.empty() || frame.is_error || frame.is_rtr || frame.is_extended || frame.dlc != 8) { return; }
    const auto reply_stamp = sourceStampNanoseconds(frame);
    if (sent_source_stamp_ns_ > 0 && reply_stamp < sent_source_stamp_ns_) { return; }
    const auto& request = queue_.front();
    if (frame.id != request.id - 0x600 + 0x580 || frame.data[1] != request.data[1] ||
        frame.data[2] != request.data[2] || frame.data[3] != request.data[3]) { return; }
    if (frame.data[0] == ABORT_REPLY) { failed_ = true; waiting_ = false; return; }
    if (!reading_ && frame.data[0] == WRITE_REPLY) { waiting_ = false; reading_ = true; return; }
    if (!reading_ || !isReadReplyFromDevice(frame)) { return; }
    const unsigned size = request.data[0] == WRITE_TO_DEVICE_1B ? 1 : request.data[0] == WRITE_TO_DEVICE_2B ? 2 : 4;
    if (4u - ((frame.data[0] >> 2) & 3u) != size) { failed_ = true; }
    for (unsigned i = 0; i < size; ++i) { if (frame.data[4+i] != request.data[4+i]) { failed_ = true; } }
    waiting_ = reading_ = false; queue_.pop_front();
  }
private:
  std::deque<can_msgs::msg::Frame> queue_;
  bool failed_{false}, waiting_{false}, reading_{false};
  Clock::time_point sent_{};
  int64_t sent_source_stamp_ns_{0};
};
