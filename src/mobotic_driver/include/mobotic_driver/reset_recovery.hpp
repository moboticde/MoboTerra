#pragma once
#include <cmath>
#include <vector>

struct ResetDriveObservation
{
  bool motion_fresh;
  bool velocity_fresh;
  double velocity;
  bool status_fresh;
  bool enabled;
};

// Admission to fault clearing only, never proof of standstill or motion
// permission. Disabled controllers may withhold position/velocity. Their fresh
// native disabled states allow clearing, followed by separately authorized
// zero-target enable and fresh sustained standstill verification.
inline bool mayClearDriveFaults(const std::vector<ResetDriveObservation>& drives)
{
  if (drives.empty()) { return false; }
  bool stationary = true, disabled = true;
  for (const auto& drive : drives)
  {
    if (drive.velocity_fresh && (!std::isfinite(drive.velocity) || std::abs(drive.velocity) > 0.1)) { return false; }
    stationary = stationary && drive.motion_fresh && drive.velocity_fresh &&
        std::isfinite(drive.velocity) && std::abs(drive.velocity) <= 0.1;
    disabled = disabled && drive.status_fresh && !drive.enabled;
  }
  return stationary || disabled;
}
