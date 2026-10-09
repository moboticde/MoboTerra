#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>

// Only the decoder of a validated response may refresh its corresponding field.
// Heartbeats, current telemetry, write acknowledgements and unrelated SDOs do not
// prove that motion/status/fault feedback is fresh.
// Keep the exact source watermark separately from the backdated steady time:
// ROS/steady clock conversion jitter must not make a duplicate look newer.
class OrderedFeedbackSample
{
public:
  using Clock = std::chrono::steady_clock;
  bool record(Clock::time_point acquired_at, int64_t source_stamp_ns = 0)
  {
    if (source_stamp_ns < 0) { return false; }
    if (source_stamp_ns > 0 && last_source_stamp_ns_ > 0)
    {
      if (source_stamp_ns <= last_source_stamp_ns_) { return false; }
    }
    else if (received_ && acquired_at <= acquired_at_) { return false; }
    acquired_at_ = acquired_at;
    if (source_stamp_ns > 0) { last_source_stamp_ns_ = source_stamp_ns; }
    received_ = true;
    ++generation_;
    return true;
  }
  uint64_t generation() const { return generation_; }
  double age(Clock::time_point now = Clock::now()) const
  {
    return received_ ? std::chrono::duration<double>(now - acquired_at_).count()
                     : std::numeric_limits<double>::infinity();
  }
private:
  bool received_{false};
  Clock::time_point acquired_at_{};
  int64_t last_source_stamp_ns_{0};
  uint64_t generation_{0};
};

class FeedbackFreshness
{
public:
  using Clock = OrderedFeedbackSample::Clock;
  enum Field : std::size_t { STATUS, VELOCITY, POSITION, CURRENT, ERROR_REGISTER, ERROR_CODE, CONTROL_MODE, FIELD_COUNT };

  // The decoder must commit the value/side effects only when this returns true.
  bool record(Field field, Clock::time_point acquired_at, int64_t source_stamp_ns = 0)
  {
    return samples_[field].record(acquired_at, source_stamp_ns);
  }
  uint64_t generation(Field field) const { return samples_[field].generation(); }
  double age(Field field, Clock::time_point now = Clock::now()) const
  {
    return samples_[field].age(now);
  }
  bool fresh(Field field, double timeout, Clock::time_point now = Clock::now()) const
  {
    const auto elapsed = age(field, now);
    return elapsed >= 0.0 && elapsed <= timeout;
  }
  bool motionFresh(bool steering, double timeout, Clock::time_point now = Clock::now()) const
  {
    return fresh(VELOCITY, timeout, now) && (!steering || fresh(POSITION, timeout, now));
  }
  bool ready(bool steering, double motion_timeout, double status_timeout, double fault_timeout,
             Clock::time_point now = Clock::now()) const
  {
    return motionFresh(steering, motion_timeout, now) && fresh(STATUS, status_timeout, now) &&
           fresh(ERROR_REGISTER, fault_timeout, now) && (steering || fresh(ERROR_CODE, fault_timeout, now));
  }
private:
  std::array<OrderedFeedbackSample, FIELD_COUNT> samples_{};
};
