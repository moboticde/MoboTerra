#include <gtest/gtest.h>
#include <mobotic_driver/communication.hpp>

namespace
{
using Clock = FeedbackFreshness::Clock;
const auto start = Clock::time_point{} + std::chrono::seconds(10);

MoboticDriveCanCommunication makeDrive(bool steering)
{
  return MoboticDriveCanCommunication(steering ? "steering" : "traction", steering ? 3u : 4u,
      steering ? POSITION : VELOCITY, 4096, steering ? 121.0 : 16.0, {});
}

can_msgs::msg::Frame reply(const MoboticDriveCanCommunication& drive, unsigned index,
    unsigned subindex, int value, FrameOperationsTypes type = READ_REPLY_4B)
{
  auto frame = createAndFillMoboticHeader(drive.replyNodeID(), type, index, subindex);
  intToFrame(frame, 4, value);
  return frame;
}

void refresh(MoboticDriveCanCommunication& drive, Clock::time_point at)
{
  drive.update(reply(drive, MICONTROL_STATUS_REGISTER, 0, 1), at);
  drive.update(reply(drive, MICONTROL_ERROR_REGISTER, 0, 0), at);
  drive.update(reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 0), at);
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 0), at);
}

bool ready(const MoboticDriveCanCommunication& drive, Clock::time_point at)
{
  return drive.feedbackFreshness().ready(0.3, 0.3, 2.5, at);
}
}

TEST(FeedbackFreshness, MissingFeedbackBlocksStartup)
{
  auto drive = makeDrive(true);
  EXPECT_FALSE(ready(drive, start));
  drive.update(reply(drive, MICONTROL_STATUS_REGISTER, 0, 1), start);
  EXPECT_TRUE(drive.enabled());
  EXPECT_FALSE(ready(drive, start));
  refresh(drive, start);
  EXPECT_TRUE(ready(drive, start));
}

TEST(FeedbackFreshness, CachedEnabledCannotHideLoss)
{
  auto drive = makeDrive(false);
  refresh(drive, start);
  EXPECT_TRUE(ready(drive, start + std::chrono::milliseconds(299)));
  EXPECT_TRUE(drive.enabled()); // Last physical state is cached, not freshness.
  EXPECT_FALSE(ready(drive, start + std::chrono::milliseconds(301)));
}

TEST(FeedbackFreshness, HeartbeatsCurrentAndUnrelatedSdosDoNotRenewMotion)
{
  auto drive = makeDrive(false);
  refresh(drive, start);
  const auto later = start + std::chrono::milliseconds(400);
  const auto motion_generation = drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY);
  drive.update(createHeartbeatFrame(drive.rawCanNodeID()), later);
  drive.update(reply(drive, 0x2086, 0, 1), later);
  auto current = reply(drive, CURRENT_FEEDBACK, 0, 0);
  drive.update(current, later);
  EXPECT_FALSE(ready(drive, later));
  EXPECT_EQ(drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY), motion_generation);
  EXPECT_TRUE(drive.feedbackFreshness().fresh(FeedbackFreshness::CURRENT, 0.3, later));
}

TEST(FeedbackFreshness, VelocityAloneDoesNotRenewSteeringPosition)
{
  auto drive = makeDrive(true);
  refresh(drive, start);
  const auto later = start + std::chrono::milliseconds(400);
  drive.update(reply(drive, MICONTROL_STATUS_REGISTER, 0, 1), later);
  drive.update(reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 0), later);
  EXPECT_FALSE(ready(drive, later));
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 0), later);
  EXPECT_TRUE(ready(drive, later));
}

TEST(FeedbackFreshness, PdoTrafficCannotRenewSdoFeedback)
{
  for (const bool steering : {false, true})
  {
    auto drive = makeDrive(steering);
    refresh(drive, start);
    const auto later = start + std::chrono::seconds(3);
    for (const unsigned offset : {0x180u, 0x280u, 0x380u, 0x480u})
    {
      auto frame = createHeartbeatFrame(drive.rawCanNodeID());
      frame.id = offset + drive.rawCanNodeID(); frame.dlc = 8;
      drive.update(frame, later);
    }
    EXPECT_FALSE(drive.feedbackFreshness().motionFresh(0.3, later));
    EXPECT_FALSE(ready(drive, later));
    refresh(drive, later);
    EXPECT_TRUE(ready(drive, later));
  }
  }

TEST(FeedbackFreshness, MalformedAndWrongResponsesCannotRefresh)
{
  auto drive = makeDrive(true);
  refresh(drive, start);
  const auto later = start + std::chrono::milliseconds(400);
  for (unsigned kind = 0; kind < 8; ++kind)
  {
    auto frame = reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 0);
    if (kind == 0) { frame.is_rtr = true; }
    if (kind == 1) { frame.is_error = true; }
    if (kind == 2) { frame.is_extended = true; }
    if (kind == 3) { frame.dlc = 7; }
    if (kind == 4) { frame.id += 1; }
    if (kind == 5) { frame.data[3] = 0; }
    if (kind == 6) { frame.data[0] = WRITE_REPLY; }
    if (kind == 7) { frame.data[0] = READ_REPLY_1B; }
    drive.update(frame, later);
    EXPECT_FALSE(drive.feedbackFreshness().fresh(FeedbackFreshness::VELOCITY, 0.3, later));
  }
  auto short_pdo = reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 0);
  short_pdo.id = 0x180 + drive.rawCanNodeID(); short_pdo.dlc = 7;
  drive.update(short_pdo, later);
  EXPECT_FALSE(drive.feedbackFreshness().motionFresh(0.3, later));
}

TEST(FeedbackFreshness, RecoveredDriveNeedsAllRequiredFields)
{
  auto front = makeDrive(true);
  auto rear = makeDrive(false);
  refresh(front, start); refresh(rear, start);
  const auto later = start + std::chrono::milliseconds(400);
  refresh(front, later);
  EXPECT_TRUE(ready(front, later));
  EXPECT_FALSE(ready(rear, later));
  rear.update(reply(rear, MICONTROL_STATUS_REGISTER, 0, 1), later);
  EXPECT_FALSE(ready(rear, later));
  refresh(rear, later);
  EXPECT_TRUE(ready(front, later) && ready(rear, later));
}

TEST(FeedbackFreshness, FutureAcquisitionAndResetDoNotFabricateFreshness)
{
  auto drive = makeDrive(true);
  refresh(drive, start + std::chrono::milliseconds(1));
  EXPECT_FALSE(ready(drive, start));
  drive.resetAllFeedbacks();
  EXPECT_FALSE(drive.hasFeedback(VELOCITY_FEEDBACK));
  EXPECT_TRUE(ready(drive, start + std::chrono::milliseconds(2)));
  EXPECT_FALSE(ready(drive, start + std::chrono::seconds(1)));
}
