#include <mobotic_driver/reset_recovery.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }

int main()
{
  try
  {
    std::vector<ResetDriveObservation> drives(4, {false, false, 0.0, true, false});
    require(mayClearDriveFaults(drives), "freshly disabled controllers may clear faults without motion data");
    require(!mayClearDriveFaults({}), "empty topology is not disabled confirmation");
    for (std::size_t i = 0; i < drives.size(); ++i)
    {
      auto bad = drives;
      bad[i].status_fresh = false;
      require(!mayClearDriveFaults(bad), "each controller needs its own fresh native status");
      bad = drives; bad[i].enabled = true;
      require(!mayClearDriveFaults(bad), "one enabled drive blocks telemetry-free fault clearing");
      for (double velocity : {0.11, -0.11, std::numeric_limits<double>::quiet_NaN()})
      {
        bad = drives; bad[i].velocity_fresh = true; bad[i].velocity = velocity;
        require(!mayClearDriveFaults(bad), "fresh movement or invalid velocity cannot be hidden by missing position");
      }
    }
    for (auto& drive : drives) { drive.motion_fresh = drive.velocity_fresh = drive.enabled = true; drive.velocity = 0.1; }
    require(mayClearDriveFaults(drives), "enabled controllers retain measured standstill admission");
    drives[0].motion_fresh = false;
    require(!mayClearDriveFaults(drives), "mixed enabled/missing feedback cannot use disabled admission");
    std::cout << "PASS: reset admission from measured standstill or fresh native disabled states\n";
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
