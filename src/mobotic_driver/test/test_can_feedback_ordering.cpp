#include <mobotic_driver/communication.hpp>

#include <functional>
#include <iostream>
#include <stdexcept>

namespace
{
using Clock = FeedbackFreshness::Clock;
const auto start = Clock::time_point{} + std::chrono::seconds(10);
constexpr int64_t source_start = 10'000'000'000;

void check(bool condition, const char* description)
{
  if (!condition) { throw std::runtime_error(description); }
}

MoboticDriveCanCommunication makeDrive(bool steering, unsigned node = 0)
{
  return MoboticDriveCanCommunication(steering ? "steering" : "traction", node ? node : (steering ? 3u : 4u),
      steering ? POSITION : VELOCITY, 4096, steering ? 121.0 : 16.0, {});
}

can_msgs::msg::Frame stamped(can_msgs::msg::Frame frame, int64_t source)
{
  frame.header.stamp.sec = static_cast<int32_t>(source / 1'000'000'000);
  frame.header.stamp.nanosec = static_cast<uint32_t>(source % 1'000'000'000);
  return frame;
}

can_msgs::msg::Frame reply(const MoboticDriveCanCommunication& drive, unsigned index,
    unsigned subindex, int value, int64_t source, FrameOperationsTypes type = READ_REPLY_4B)
{
  auto frame = createAndFillMoboticHeader(drive.replyNodeID(), type, index, subindex);
  intToFrame(frame, 4, value);
  return stamped(frame, source);
}

can_msgs::msg::Frame tractionPdo(const MoboticDriveCanCommunication& drive,
    int status, DeviceMode mode, int velocity, int64_t source)
{
  auto frame = createHeartbeatFrame(drive.rawCanNodeID());
  frame.id = drive.tractionTpdoCobId(); frame.dlc = 7;
  shortToFrame(frame, 0, static_cast<short>(status)); frame.data[2] = mode;
  intToFrame(frame, 3, velocity);
  return stamped(frame, source);
}

void refresh(MoboticDriveCanCommunication& drive, int64_t source, Clock::time_point at)
{
  const bool steering = drive.isSteeringController();
  drive.update(reply(drive, steering ? MICONTROL_DEVICE_STATE : EC_STATUSWORD, 0,
                     steering ? 1 : 0x27, source), at);
  drive.update(reply(drive, steering ? MICONTROL_ERROR_REGISTER : EC_ERROR_REGISTER, 0, 0, source), at);
  drive.update(reply(drive, steering ? MICONTROL_VELOCITY_FEEDBACK : EC_TRACTION_VELOCITY_FEEDBACK,
                     steering ? 1 : 0, 0, source), at);
  if (steering) { drive.update(reply(drive, POSITION_FEEDBACK, 0, 0, source), at); }
  else { drive.update(reply(drive, EC_ERROR_CODE, 0, 0, source), at); }
}

void exactSourceOrdering()
{
  OrderedFeedbackSample sample;
  check(std::isinf(sample.age(start)), "unreceived sample age is unknown");
  check(sample.record(start, source_start), "first sample accepted");
  check(!sample.record(start + std::chrono::milliseconds(100), source_start), "same source stamp is replay despite later arrival");
  check(!sample.record(start + std::chrono::milliseconds(200), source_start - 1), "older source is rejected despite later arrival");
  check(sample.generation() == 1, "rejected messages cannot increment generation");
  check(sample.age(start + std::chrono::milliseconds(301)) > 0.3, "replay cannot renew age");
  check(sample.record(start + std::chrono::milliseconds(302), source_start + 1), "genuinely newer source recovers");
  check(!sample.record(start + std::chrono::milliseconds(303), -1), "invalid negative source is rejected");
}

void unstampedFallback()
{
  OrderedFeedbackSample sample;
  check(sample.record(start), "unstamped transport retains reception fallback");
  check(!sample.record(start), "same fallback time cannot be recorded twice");
  check(!sample.record(start - std::chrono::milliseconds(1)), "older fallback acquisition rejected");
  check(sample.record(start + std::chrono::milliseconds(1)), "new unstamped reception accepted");
  check(sample.record(start + std::chrono::milliseconds(2), source_start), "stamped source accepted");
  check(sample.record(start + std::chrono::milliseconds(3)), "fallback remains compatible");
  check(!sample.record(start + std::chrono::milliseconds(4), source_start), "fallback cannot erase last stamped watermark");
  check(sample.record(start + std::chrono::milliseconds(5), source_start + 1), "newer stamped source accepted");
}

void faultsCannotBeClearedByReplay()
{
  for (const bool steering : {false, true})
  {
    auto drive = makeDrive(steering);
    const auto index = steering ? MICONTROL_ERROR_REGISTER : EC_ERROR_REGISTER;
    drive.update(reply(drive, index, 0, 9, source_start + 2), start);
    const auto generation = drive.feedbackFreshness().generation(FeedbackFreshness::ERROR_REGISTER);
    for (const auto source : {source_start + 1, source_start + 2})
    {
      const auto frames = drive.update(reply(drive, index, 0, 0, source), start + std::chrono::milliseconds(10));
      check(drive.hasError(), "older/duplicate healthy register cannot clear newer fault");
      check(frames.empty(), "ignored healthy reply cannot trigger initialization side effects");
    }
    check(drive.feedbackFreshness().generation(FeedbackFreshness::ERROR_REGISTER) == generation, "fault age/generation unchanged");
    drive.update(reply(drive, index, 0, 0, source_start + 3), start + std::chrono::milliseconds(20));
    check(!drive.hasError(), "genuinely newer healthy register may clear fault indication");
  }
  auto drive = makeDrive(false);
  drive.update(reply(drive, EC_ERROR_CODE, 0, 0x1234, source_start + 2, READ_REPLY_2B), start);
  drive.update(reply(drive, EC_ERROR_CODE, 0, 0, source_start + 1, READ_REPLY_2B), start + std::chrono::milliseconds(1));
  drive.update(reply(drive, EC_ERROR_CODE, 0, 0, source_start + 2, READ_REPLY_2B), start + std::chrono::milliseconds(2));
  check(drive.getErrorCode() == 0x1234, "older/duplicate error code cannot erase fault");
  drive.update(reply(drive, EC_ERROR_CODE, 0, 0, source_start + 3, READ_REPLY_2B), start + std::chrono::milliseconds(3));
  check(drive.getErrorCode() == 0, "newer error code restores healthy indication");
}

void motionCannotBeOverwritten()
{
  auto drive = makeDrive(true);
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 123, source_start + 2), start);
  drive.update(reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 45, source_start + 2), start);
  for (const auto source : {source_start + 1, source_start + 2})
  {
    drive.update(reply(drive, POSITION_FEEDBACK, 0, 999, source), start + std::chrono::milliseconds(100));
    drive.update(reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 999, source), start + std::chrono::milliseconds(100));
  }
  check(drive.feedback(POSITION_FEEDBACK) == 123 && drive.feedback(VELOCITY_FEEDBACK) == 45, "motion values preserve latest acquisition");
  check(!drive.feedbackFreshness().motionFresh(true, 0.3, start + std::chrono::milliseconds(301)), "replayed motion cannot remain fresh");
}

void disabledStatusCannotBeReenabled()
{
  for (const bool steering : {false, true})
  {
    auto drive = makeDrive(steering);
    const auto index = steering ? MICONTROL_DEVICE_STATE : EC_STATUSWORD;
    const auto enabled_value = steering ? 1 : 0x27;
    drive.update(reply(drive, index, 0, enabled_value, source_start + 1), start);
    check(drive.enabled(), "setup enabled status");
    drive.update(reply(drive, index, 0, 0, source_start + 2), start + std::chrono::milliseconds(1));
    for (const auto source : {source_start + 1, source_start + 2})
    {
      drive.update(reply(drive, index, 0, enabled_value, source), start + std::chrono::milliseconds(2));
      check(!drive.enabled(), "older/duplicate enabled reply cannot undo newer disable");
    }
  }
}

void controlModeIsOrdered()
{
  auto drive = makeDrive(true);
  drive.update(reply(drive, MICONTROL_DEVICE_MODE, 0, S_VELOCITY, source_start + 2), start);
  drive.update(reply(drive, MICONTROL_DEVICE_MODE, 0, POSITION, source_start + 1), start + std::chrono::milliseconds(1));
  check(drive.currentMode() == S_VELOCITY && !drive.initialized(), "older mode cannot change cache or initialize controller");
  drive.update(reply(drive, MICONTROL_DEVICE_MODE, 0, POSITION, source_start + 3), start + std::chrono::milliseconds(2));
  drive.update(reply(drive, MICONTROL_DEVICE_MODE, 0, S_VELOCITY, source_start + 3), start + std::chrono::milliseconds(3));
  check(drive.currentMode() == POSITION && drive.initialized(), "duplicate mode cannot undo newer initialization");
}

void tractionPdoMergesPerField()
{
  auto drive = makeDrive(false);
  drive.update(reply(drive, EC_STATUSWORD, 0, 0, source_start + 4), start);
  drive.update(reply(drive, EC_TRACTION_MODE_OF_OPERATION, 0, CURRENT, source_start + 4, READ_REPLY_1B), start);
  drive.update(reply(drive, EC_TRACTION_VELOCITY_FEEDBACK, 0, 10, source_start + 1), start);
  auto pdo = tractionPdo(drive, 0x27, VELOCITY, 20, source_start + 2);
  drive.update(pdo, start + std::chrono::milliseconds(1));
  check(!drive.enabled() && drive.currentMode() == CURRENT && !drive.initialized(), "older PDO status/mode cannot defeat newer SDO fields");
  check(drive.feedback(VELOCITY_FEEDBACK) == 20, "same PDO's genuinely newer velocity must still be accepted");
  intToFrame(pdo, 3, 999);
  drive.update(pdo, start + std::chrono::milliseconds(2));
  check(drive.feedback(VELOCITY_FEEDBACK) == 20, "duplicate PDO velocity ignored");
  drive.update(tractionPdo(drive, 0x27, VELOCITY, 30, source_start + 5), start + std::chrono::milliseconds(3));
  check(drive.enabled() && drive.currentMode() == VELOCITY && drive.initialized(), "newer PDO updates status/mode");
  check(drive.feedback(VELOCITY_FEEDBACK) == 30, "newer PDO updates velocity");
}

void steeringPdoMergesPerField()
{
  auto drive = makeDrive(true);
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 444, source_start + 4), start);
  drive.update(reply(drive, MICONTROL_VELOCITY_FEEDBACK, 1, 10, source_start + 1), start);
  auto pdo = createHeartbeatFrame(drive.rawCanNodeID());
  pdo.id = drive.tractionTpdoCobId(); pdo.dlc = 8;
  intToFrame(pdo, 0, 222); intToFrame(pdo, 4, 20);
  pdo = stamped(pdo, source_start + 2);
  drive.update(pdo, start + std::chrono::milliseconds(1));
  check(drive.feedback(POSITION_FEEDBACK) == 444 && drive.feedback(VELOCITY_FEEDBACK) == 20, "steering PDO preserves newer position but accepts newer velocity");
  intToFrame(pdo, 0, 999); intToFrame(pdo, 4, 999);
  drive.update(pdo, start + std::chrono::milliseconds(2));
  check(drive.feedback(POSITION_FEEDBACK) == 444 && drive.feedback(VELOCITY_FEEDBACK) == 20, "replay cannot alter either steering field");
}

void currentPdoAndSdoShareOrdering()
{
  for (const bool steering : {false, true})
  {
    auto drive = makeDrive(steering);
    auto pdo = createHeartbeatFrame(drive.rawCanNodeID());
    pdo.id = drive.telemetryTpdo2CobId(); pdo.dlc = steering ? 8 : 2;
    if (steering) { intToFrame(pdo, 4, 44); }
    else { shortToFrame(pdo, 0, 44); }
    drive.update(reply(drive, steering ? MICONTROL_CURRENT_FEEDBACK : EC_TRACTION_CURRENT_FEEDBACK,
                       steering ? 1 : 0, 77, source_start + 4, steering ? READ_REPLY_4B : READ_REPLY_2B), start);
    drive.update(stamped(pdo, source_start + 3), start + std::chrono::milliseconds(1));
    drive.update(stamped(pdo, source_start + 4), start + std::chrono::milliseconds(2));
    check(drive.feedback(CURRENT_FEEDBACK) == 77, "older/duplicate PDO cannot overwrite SDO current");
    drive.update(stamped(pdo, source_start + 5), start + std::chrono::milliseconds(3));
    check(drive.feedback(CURRENT_FEEDBACK) == 44, "newer PDO current accepted");
  }
}

void rawReadAndAbortOrdering()
{
  auto drive = makeDrive(false);
  for (const auto index : {0x2086u, 0x60FDu})
  {
    RawRegisterTelemetry sample;
    check(sample.update(reply(drive, index, 0, 200, source_start + 2), start), "raw register read accepted");
    check(!sample.update(reply(drive, index, 0, 0, source_start + 1, ABORT_REPLY), start + std::chrono::milliseconds(1)), "older abort ignored");
    check(sample.available && sample.value == 200, "older abort cannot invalidate newer raw read");
    check(sample.update(reply(drive, index, 0, 0, source_start + 3, ABORT_REPLY), start + std::chrono::milliseconds(2)), "newer abort accepted");
    check(!sample.available, "newer abort invalidates availability");
    check(!sample.update(reply(drive, index, 0, 999, source_start + 2), start + std::chrono::milliseconds(3)), "older read cannot undo newer abort");
    check(!sample.available && sample.ordering.generation() == 2, "replay cannot restore raw availability or renew age");
    check(sample.update(reply(drive, index, 0, 400, source_start + 4, READ_REPLY_2B), start + std::chrono::milliseconds(4)), "newer read recovers");
    check(sample.available && sample.value == 400, "new raw value exported");
    check(!sample.update(reply(drive, index, 0, 0, source_start + 5, WRITE_REPLY), start + std::chrono::milliseconds(5)), "write acknowledgement cannot refresh raw telemetry");
  }
}

void delayedSamplesKeepTheirAge()
{
  auto drive = makeDrive(true);
  const auto acquired_at = start - std::chrono::milliseconds(290);
  refresh(drive, source_start, acquired_at);
  check(drive.feedbackFreshness().ready(true, 0.3, 0.3, 2.5, start + std::chrono::milliseconds(9)), "delayed motion initially usable");
  check(!drive.feedbackFreshness().ready(true, 0.3, 0.3, 2.5, start + std::chrono::milliseconds(11)), "delayed motion has only remaining source-age budget");
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 999, source_start), start + std::chrono::milliseconds(20));
  check(!drive.feedbackFreshness().fresh(FeedbackFreshness::POSITION, 0.3, start + std::chrono::milliseconds(20)), "duplicate with changed conversion time cannot renew acquisition age");
}

void resetDoesNotEraseWatermarks()
{
  auto drive = makeDrive(true);
  refresh(drive, source_start, start);
  drive.resetAllFeedbacks();
  refresh(drive, source_start, start + std::chrono::milliseconds(10));
  check(!drive.hasFeedback(VELOCITY_FEEDBACK) && !drive.hasFeedback(POSITION_FEEDBACK), "replay after consumption cannot fabricate a complete new joint-state batch");
  refresh(drive, source_start + 1, start + std::chrono::milliseconds(20));
  check(drive.hasFeedback(VELOCITY_FEEDBACK) && drive.hasFeedback(POSITION_FEEDBACK), "genuinely new motion restores batch after reset");
}

void differentFieldsAndDrivesAreIndependent()
{
  auto front = makeDrive(true, 3);
  auto rear = makeDrive(true, 7);
  front.update(reply(front, POSITION_FEEDBACK, 0, 30, source_start + 3), start);
  front.update(reply(front, MICONTROL_VELOCITY_FEEDBACK, 1, 10, source_start + 1), start);
  rear.update(reply(rear, POSITION_FEEDBACK, 0, 20, source_start + 2), start);
  check(front.feedback(VELOCITY_FEEDBACK) == 10, "position watermark cannot discard another field's first velocity");
  check(rear.feedback(POSITION_FEEDBACK) == 20, "another drive's watermark cannot discard rear feedback");
}

void malformedFramesCannotAdvanceWatermarks()
{
  auto drive = makeDrive(true);
  for (unsigned kind = 0; kind < 10; ++kind)
  {
    auto frame = reply(drive, POSITION_FEEDBACK, 0, 999, source_start + 2);
    if (kind == 0) { frame.header.stamp.sec = -1; }
    if (kind == 1) { frame.header.stamp.nanosec = 1'000'000'000u; }
    if (kind == 2) { frame.is_error = true; }
    if (kind == 3) { frame.is_rtr = true; }
    if (kind == 4) { frame.is_extended = true; }
    if (kind == 5) { frame.dlc = 7; }
    if (kind == 6) { frame.id += 1; }
    if (kind == 7) { frame.data[0] = WRITE_REPLY; }
    if (kind == 8) { frame.data[0] = READ_REPLY_1B; }
    if (kind == 9) { frame.data[3] = 1; }
    drive.update(frame, start);
  }
  check(drive.feedbackFreshness().generation(FeedbackFreshness::POSITION) == 0, "invalid input cannot create a source watermark");
  drive.update(reply(drive, POSITION_FEEDBACK, 0, 123, source_start + 1), start);
  check(drive.feedback(POSITION_FEEDBACK) == 123, "valid earlier source is not poisoned by rejected malformed newer frames");
}

void requiredFeedbackStillControlsReadiness()
{
  for (const bool steering : {false, true})
  {
    auto drive = makeDrive(steering);
    check(!drive.feedbackFreshness().ready(steering, 0.3, 0.3, 2.5, start), "missing data prevents startup");
    refresh(drive, source_start, start);
    check(drive.feedbackFreshness().ready(steering, 0.3, 0.3, 2.5, start), "all fresh fields permit readiness");
    check(!drive.feedbackFreshness().ready(steering, 0.3, 0.3, 2.5, start + std::chrono::milliseconds(301)), "cached enabled state cannot hide motion loss");
    drive.update(stamped(createHeartbeatFrame(drive.rawCanNodeID()), source_start + 1), start + std::chrono::milliseconds(400));
    check(!drive.feedbackFreshness().ready(steering, 0.3, 0.3, 2.5, start + std::chrono::milliseconds(400)), "heartbeat cannot restore motion freshness");
  }
}
} // namespace

int main()
{
  const std::vector<std::pair<const char*, std::function<void()>>> cases{
      {"exact source ordering and conversion jitter", exactSourceOrdering},
      {"unstamped compatibility and retained watermark", unstampedFallback},
      {"fault register/code replay protection", faultsCannotBeClearedByReplay},
      {"steering position/velocity ordering", motionCannotBeOverwritten},
      {"disabled status replay protection", disabledStatusCannotBeReenabled},
      {"control mode and initialization ordering", controlModeIsOrdered},
      {"traction PDO/SDO per-field merge", tractionPdoMergesPerField},
      {"steering PDO/SDO per-field merge", steeringPdoMergesPerField},
      {"current PDO/SDO ordering", currentPdoAndSdoShareOrdering},
      {"raw STO/digital input read/abort ordering", rawReadAndAbortOrdering},
      {"delayed feedback expiration", delayedSamplesKeepTheirAge},
      {"consumption/reset retains replay protection", resetDoesNotEraseWatermarks},
      {"independent field/drive watermarks", differentFieldsAndDrivesAreIndependent},
      {"malformed frames cannot poison ordering", malformedFramesCannotAdvanceWatermarks},
      {"required readiness and heartbeat isolation", requiredFeedbackStillControlsReadiness},
  };
  for (const auto& test : cases)
  {
    try { test.second(); }
    catch (const std::exception& error)
    {
      std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
      return 1;
    }
    std::cout << "PASS: " << test.first << '\n';
  }
  std::cout << cases.size() << " CAN feedback-ordering regression groups passed\n";
  return 0;
}
