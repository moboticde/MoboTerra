#include <mobotic_driver/communication.hpp>
#include <mobotic_driver/reset_recovery.hpp>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <iostream>
#include <stdexcept>

using Clock = FeedbackFreshness::Clock;
const auto start = Clock::time_point{} + std::chrono::seconds(10);
constexpr int64_t source_start = 10'000'000'000;

void require(bool condition, const char* message)
{ if (!condition) { throw std::runtime_error(message); } }

MoboticDriveCanCommunication backend()
{ return MoboticDriveCanCommunication("traction", 2, VELOCITY, 4096, 16, {}, "micontrol"); }

can_msgs::msg::Frame reply(unsigned index, int value, int64_t source,
                          FrameOperationsTypes type = READ_REPLY_4B)
{
  auto frame = createAndFillMoboticHeader(0x582, type, index, 0);
  intToFrame(frame, 4, value);
  frame.header.stamp.sec = static_cast<int32_t>(source / 1'000'000'000);
  frame.header.stamp.nanosec = static_cast<uint32_t>(source % 1'000'000'000);
  return frame;
}

bool mayReset(const MoboticDriveCanCommunication& drive, Clock::time_point at)
{
  return mayClearDriveFaults({{false, false, std::numeric_limits<double>::quiet_NaN(),
      drive.hasNativeStatus() && drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, .3, at),
      drive.enabled()}});
}

void distinctRequestsAndEvidence()
{
  auto drive = backend();
  require(shortFromFrame(drive.readDeviceStateRequest(), 1) == 0x3002,
          "actual-state request must address DEV_Status");
  require(shortFromFrame(drive.changeDeviceStateRequest(ENABLE), 1) == 0x3004 &&
          shortFromFrame(drive.readSoftwareEnableRequest(), 1) == 0x3004,
          "software enable command/readback remains DEV_Enable");
  drive.update(reply(0x3004, 1, source_start, READ_REPLY_1B), start);
  require(drive.softwareEnableKnown(.3, start) && drive.softwareEnableRequested(),
          "documented uint8 software setting is recorded independently");
  require(!drive.enabled() && !drive.hasNativeStatus() && !mayReset(drive, start),
          "software enable alone cannot prove enabled or disabled actual state for B2");
  require(drive.feedbackFreshness().generation(FeedbackFreshness::STATUS) == 0,
          "software readback cannot fabricate actual-state acquisition");

  drive.update(reply(0x3002, 1, source_start + 1), start);
  drive.update(reply(0x3004, 0, source_start + 2), start);
  require(drive.enabled() && !drive.softwareEnableRequested() && !mayReset(drive, start),
          "software disable readback cannot override actual enabled power-stage state");
  const int flags_without_enabled = 0x00040010;
  drive.update(reply(0x3002, flags_without_enabled, source_start + 3), start);
  require(!drive.enabled() && drive.nativeDeviceState() == flags_without_enabled && mayReset(drive, start),
          "actual bit 0 determines enable; other status bits and all 32 bits are preserved");
  drive.update(reply(0x3004, 1, source_start + 4), start);
  require(!drive.enabled() && mayReset(drive, start),
          "requested enable cannot override actual disabled hardware for B2 observation");

  const auto later = start + std::chrono::milliseconds(301);
  drive.update(reply(0x3004, 0, source_start + 5), later);
  require(!drive.feedbackFreshness().fresh(FeedbackFreshness::STATUS, .3, later) && !mayReset(drive, later),
          "fresh software replies cannot renew expired actual-state evidence");
}

void actualStatusOrderingAndValidation()
{
  auto drive = backend();
  drive.update(reply(0x3002, 1, source_start + 1), start);
  drive.update(reply(0x3002, 0, source_start + 2), start + std::chrono::milliseconds(1));
  for (const auto stamp : {source_start + 1, source_start + 2})
  { drive.update(reply(0x3002, 1, stamp), start + std::chrono::milliseconds(2)); }
  require(!drive.enabled(), "old or replayed actual status cannot resurrect enabled state");
  const auto generation = drive.feedbackFreshness().generation(FeedbackFreshness::STATUS);
  const auto later = start + std::chrono::milliseconds(400);
  for (unsigned kind = 0; kind < 9; ++kind)
  {
    auto frame = reply(0x3002, 1, source_start + 3);
    if (kind == 0) { frame.data[0] = READ_REPLY_2B; }
    if (kind == 1) { frame.data[0] = READ_REPLY_1B; }
    if (kind == 2) { frame.data[0] = WRITE_REPLY; }
    if (kind == 3) { frame.dlc = 7; }
    if (kind == 4) { frame.data[3] = 1; }
    if (kind == 5) { frame.is_extended = true; }
    if (kind == 6) { frame.is_rtr = true; }
    if (kind == 7) { frame.is_error = true; }
    if (kind == 8) { frame.id = 0x583; }
    drive.update(frame, later);
  }
  require(!drive.enabled() && drive.feedbackFreshness().generation(FeedbackFreshness::STATUS) == generation,
          "malformed, truncated, wrong-node, wrong-subindex and acknowledgement frames cannot update actual state");
  drive.update(reply(0x3002, 1, source_start + 3), later);
  require(drive.enabled(), "rejected malformed frames cannot poison the next genuine status acquisition");
}

void faultAndInvalidation()
{
  auto drive = backend();
  drive.update(reply(0x3001, 0, source_start), start);
  drive.update(reply(0x3002, 2, source_start), start);
  require(drive.hasError() && drive.activeErrorCode() != 0 && !drive.enabled(),
          "actual fault bit blocks readiness even while fault-register readback is still zero");
  drive.update(reply(0x3002, 0, source_start + 1), start);
  require(!drive.hasError(), "a new healthy status clears the status-only fault indication");
  drive.update(reply(0x3001, 9, source_start + 1), start);
  drive.update(reply(0x3002, 1, source_start + 2), start);
  require(drive.hasError(), "healthy actual state cannot erase a separately observed vendor fault");
  drive.update(reply(0x3004, 1, source_start), start);
  drive.prepareOperatingModeTransaction();
  require(!drive.hasNativeStatus() && !drive.enabled() && !drive.softwareEnableKnown(.3, start),
          "a mode transaction invalidates both observations independently");
  drive.update(reply(0x3004, 0, source_start), start + std::chrono::milliseconds(1));
  require(!drive.softwareEnableKnown(.3, start + std::chrono::milliseconds(1)),
          "a mode transaction keeps ordering watermarks against replayed software readbacks");
  drive.update(reply(0x3004, 1, source_start + 1), start + std::chrono::milliseconds(2));
  auto boot = createHeartbeatFrame(2); boot.data[0] = 0;
  drive.update(boot, start + std::chrono::milliseconds(3));
  require(!drive.hasNativeStatus() && !drive.softwareEnableKnown(.3, start + std::chrono::milliseconds(3)),
          "controller boot-up invalidates both actual and requested enable observations");
}

void hardwareEnableModel()
{
  auto drive = backend();
  VirtualDriveProtocol controller(2, true, 4096, 16, "micontrol");
  controller.handle(drive.setDeviceModeRequest(VELOCITY));
  controller.setHardwareEnable(false);
  controller.handle(drive.changeDeviceStateRequest(ENABLE));
  auto software = controller.handle(drive.readSoftwareEnableRequest()).front();
  auto actual = controller.handle(drive.readDeviceStateRequest()).front();
  require(intFromFrame(software, 4) == 1 && intFromFrame(actual, 4) == 0 && !controller.enabled(),
          "model can acknowledge software enable while hardware withholds actual power");
  controller.setHardwareEnable(true);
  actual = controller.handle(drive.readDeviceStateRequest()).front();
  require((intFromFrame(actual, 4) & 1) && controller.enabled(),
          "actual enable follows the hardware input and requested enable conjunction");
  auto forbidden = drive.readDeviceStateRequest(); forbidden.data[0] = WRITE_TO_DEVICE_4B;
  require(controller.handle(forbidden).front().data[0] == ABORT_REPLY && controller.enabled(),
          "DEV_Status remains a read-only register");
  controller.handle(drive.changeDeviceStateRequest(DISABLE));
  require(intFromFrame(controller.handle(drive.readSoftwareEnableRequest()).front(), 4) == 0 &&
          intFromFrame(controller.handle(drive.readDeviceStateRequest()).front(), 4) == 0,
          "software disable is independently readable while actual power is off");
}

int main()
{
  try
  {
    distinctRequestsAndEvidence(); actualStatusOrderingAndValidation();
    faultAndInvalidation(); hardwareEnableModel();
    std::cout << "PASS: actual 0x3002 status, independent 0x3004 setting, B2 evidence, faults, ordering and hardware-enable gate\n";
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
