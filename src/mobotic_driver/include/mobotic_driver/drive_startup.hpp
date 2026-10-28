#pragma once
#include <chrono>

// One supervised startup attempt per explicit recovery cycle. Motion feedback is an
// outcome of enable, never a prerequisite for starting this bounded attempt.
class DriveStartup
{
public:
  using Clock = std::chrono::steady_clock;
  bool begin(Clock::time_point now = Clock::now())
  {
    if (started_) { return false; }
    started_ = true; started_at_ = now; return true;
  }
  bool active() const { return started_ && !complete_ && !failed_; }
  bool started() const { return started_; }
  bool complete() const { return complete_; }
  bool failed() const { return failed_; }
  bool expired(double timeout, Clock::time_point now = Clock::now()) const
  { return active() && std::chrono::duration<double>(now - started_at_).count() >= timeout; }
  void finish() { if (active()) { complete_ = true; } }
  void fail() { if (started_) { failed_ = true; complete_ = false; } }
  // Only an explicitly authorized fault-clear transaction may rearm startup.
  void rearmAfterReset() { if (failed_) { started_ = complete_ = failed_ = false; } }
private:
  bool started_{false}, complete_{false}, failed_{false};
  Clock::time_point started_at_{};
};
