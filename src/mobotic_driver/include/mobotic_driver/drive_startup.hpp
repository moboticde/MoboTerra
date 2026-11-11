#pragma once
#include <chrono>
#include <cstdint>

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
  bool rearmAfterReset(uint64_t reset_id)
  {
    if (!failed_ || reset_id == 0 || reset_id <= last_recovery_id_) { return false; }
    last_recovery_id_ = reset_id;
    started_ = complete_ = failed_ = false;
    return true;
  }
  void newSupervisor() { last_recovery_id_ = 0; }
  // A healthy completed startup may begin one acknowledged operating-mode
  // change. The caller must first authorize standstill and disable the drives.
  // This cannot clear a failed startup or bypass explicit fault recovery.
  bool prepareModeChange()
  {
    if (!complete_ || failed_) { return false; }
    started_ = complete_ = false;
    return true;
  }
private:
  bool started_{false}, complete_{false}, failed_{false};
  uint64_t last_recovery_id_{0};
  Clock::time_point started_at_{};
};
