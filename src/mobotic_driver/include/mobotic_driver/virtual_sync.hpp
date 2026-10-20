#pragma once
#include <algorithm>
#include <cstdint>

inline bool virtualSyncPending(bool virtual_source, int64_t sync_stamp_ns,
                               int64_t published_cycle_ns, int64_t now_ns,
                               double motion_feedback_timeout)
{
  if (!virtual_source || sync_stamp_ns <= published_cycle_ns) { return false; }
  const double age = static_cast<double>(now_ns - sync_stamp_ns) * 1e-9;
  return age >= 0.0 && age < std::min(0.1, motion_feedback_timeout / 2.0);
}
