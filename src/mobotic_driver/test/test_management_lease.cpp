#include <gtest/gtest.h>
#include <mobotic_driver/management_lease.hpp>

TEST(ManagementLease, DefaultsDisabledAndExpiresIndependentlyOfMotion)
{
  ManagementLease lease;
  const auto start = ManagementLease::Clock::time_point{};
  EXPECT_FALSE(lease.permitsEnable(0.3, start));
  lease.refresh(true, start);
  EXPECT_TRUE(lease.permitsEnable(0.3, start + std::chrono::milliseconds(299)));
  EXPECT_FALSE(lease.permitsEnable(0.3, start + std::chrono::milliseconds(301)));
  // Only a new supervisor command renews the lease.
  lease.refresh(true, start + std::chrono::seconds(1));
  EXPECT_TRUE(lease.permitsEnable(0.3, start + std::chrono::seconds(1)));
  lease.refresh(false, start + std::chrono::seconds(1));
  EXPECT_TRUE(lease.fresh(0.3, start + std::chrono::seconds(1)));
  EXPECT_FALSE(lease.permitsEnable(0.3, start + std::chrono::seconds(1)));
}

TEST(ManagementLease, ClockAndBoundaryChecks)
{
  ManagementLease lease;
  const auto start = ManagementLease::Clock::time_point{} + std::chrono::seconds(1);
  lease.refresh(true, start);
  EXPECT_FALSE(lease.fresh(0.3, start - std::chrono::milliseconds(1)));
  EXPECT_TRUE(lease.fresh(0.3, start + std::chrono::milliseconds(300)));
  EXPECT_FALSE(lease.fresh(0.3, start + std::chrono::milliseconds(301)));
}
