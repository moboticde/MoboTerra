#pragma once
#include <chrono>

// Independent of ROS time and the joint-command watchdog. Not a safety-rated STO.
class ManagementLease
{
public:
  using Clock = std::chrono::steady_clock;
  void refresh(bool enable, Clock::time_point now = Clock::now())
  {
    received_ = true;
    enable_ = enable;
    received_at_ = now;
  }
  bool fresh(double timeout, Clock::time_point now = Clock::now()) const
  {
    const auto age = std::chrono::duration<double>(now - received_at_).count();
    return received_ && age >= 0.0 && age <= timeout;
  }
  bool permitsEnable(double timeout, Clock::time_point now = Clock::now()) const
  {
    return enable_ && fresh(timeout, now);
  }
private:
  bool received_{false}, enable_{false};
  Clock::time_point received_at_{};
};
