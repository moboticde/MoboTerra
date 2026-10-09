#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

// Driver-side routing authority, independent of ROS message types. During a
// transition ONLY the old mode's stop/ramp path remains selected. This is not STO.
class MotionSourceGate
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Source { Kinematics, SupervisorDirect };

  bool update(uint8_t mode, bool transitioning, int64_t stamp_ns, int64_t ros_now_ns,
              double timeout, Clock::time_point now = Clock::now())
  {
    const double age = static_cast<double>(ros_now_ns - stamp_ns) * 1e-9;
    if (mode > 2 || stamp_ns <= 0 || stamp_ns <= last_stamp_ns_ ||
        !std::isfinite(timeout) || timeout <= 0 || age < -0.1 || age > timeout)
    {
      return false; // Replayed/invalid states cannot renew the authority lease.
    }
    const bool recovery = !fresh(ros_now_ns, timeout, now);
    if (recovery || mode != mode_ || transitioning != transitioning_)
    {
      boundary_ns_ = std::max(boundary_ns_, recovery ? std::max(stamp_ns, ros_now_ns) : stamp_ns);
      ++generation_;
    }
    mode_ = mode;
    transitioning_ = transitioning;
    last_stamp_ns_ = stamp_ns;
    received_at_ = now - std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(age, 0.0)));
    received_ = true;
    return true;
  }

  bool fresh(int64_t ros_now_ns, double timeout, Clock::time_point now = Clock::now()) const
  {
    const double steady_age = std::chrono::duration<double>(now - received_at_).count();
    const double ros_age = static_cast<double>(ros_now_ns - last_stamp_ns_) * 1e-9;
    return received_ && steady_age >= 0 && steady_age <= timeout &&
           ros_age >= -0.1 && ros_age <= timeout;
  }

  bool permits(Source source, int64_t command_stamp_ns, int64_t ros_now_ns,
               double timeout, Clock::time_point now = Clock::now()) const
  {
    return fresh(ros_now_ns, timeout, now) && command_stamp_ns > 0 &&
           command_stamp_ns >= boundary_ns_ &&
           (source == Source::SupervisorDirect ? mode_ == 2 : mode_ < 2);
  }

  uint8_t mode() const { return mode_; }
  bool transitioning() const { return transitioning_; }
  uint64_t generation() const { return generation_; }
  int64_t boundary() const { return boundary_ns_; }

private:
  bool received_{false}, transitioning_{false};
  uint8_t mode_{255};
  uint64_t generation_{0};
  int64_t last_stamp_ns_{0}, boundary_ns_{0};
  Clock::time_point received_at_{};
};
