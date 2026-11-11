#include <mobotic_driver/drive_startup.hpp>
#include <mobotic_driver/sdo_configuration.hpp>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char* message)
{ if (!condition) { throw std::runtime_error(message); } }

int main()
{
  try
  {
    DriveStartup startup;
    const auto now = DriveStartup::Clock::now();
    require(!startup.active() && !startup.complete(), "idle is not ready for motion");
    require(startup.begin(now), "supervisor grant starts initialization");
    require(!startup.begin(now + std::chrono::seconds(5)), "repeated grants cannot extend startup");
    require(!startup.expired(15, now + std::chrono::seconds(14)), "startup permits bounded acquisition");
    require(startup.expired(15, now + std::chrono::seconds(15)), "no-feedback startup times out");
    startup.fail(); startup.finish();
    require(startup.failed() && !startup.complete() && !startup.begin(), "failed startup cannot silently retry");
    require(startup.rearmAfterReset(1), "explicit reset transaction rearms failed startup");
    require(startup.begin(now) && startup.active(), "explicit authorized recovery rearms a bounded startup attempt");
    startup.fail();
    require(!startup.rearmAfterReset(1) && !startup.begin(now), "one reset cannot repeatedly restart a cancelled attempt");
    require(startup.rearmAfterReset(2) && startup.begin(now), "a new explicit reset admits one new attempt");
    startup.fail(); startup.newSupervisor();
    require(startup.rearmAfterReset(1), "reset IDs are scoped to the current supervisor instance");
    DriveStartup completed;
    completed.begin(now); completed.finish();
    require(completed.complete() && !completed.expired(15, now + std::chrono::seconds(20)), "acquired feedback finishes startup");
    require(completed.prepareModeChange() && completed.begin(now), "healthy completed startup permits an explicit mode transaction");
    completed.fail();
    require(!completed.prepareModeChange(), "mode changes cannot clear a startup failure");

    for (const std::string family : {"steering", "micontrol", "micontrol_current"})
    {
      const bool traction = family != "steering";
      const bool current = family == "micontrol_current";
      const std::string controller_type = "micontrol";
      MoboticDriveCanCommunication drive("fixture", traction ? 4 : 3, current ? CURRENT : traction ? VELOCITY : POSITION,
          4096, 16, {}, controller_type);
      drive.setAutomaticModeInitialization(false);
      {
        const auto mode = drive.setDeviceModeRequest(drive.targetMode());
        require(mode.data[0] == 0x23 && mode.data[1] == 0x03 && mode.data[2] == 0x30 &&
                mode.data[3] == 0 && mode.data[4] == (current ? 2 : traction ? 3 : 7),
                "miControl mode bytes match the supplied legacy header");
        for (const auto& action : {drive.changeDeviceStateRequest(ENABLE), drive.clearErrorRequest(CLEAR)})
        { require(action.data[0] == 0x23 && intFromFrame(action, 4) == 1, "miControl state/reset uses four-byte writes"); }
        require(drive.readFeedbackRequest(CURRENT_FEEDBACK).data[3] == 0 &&
                drive.readFeedbackRequest(VELOCITY_FEEDBACK).data[3] == 1 &&
                drive.readFeedbackRequest(POSITION_FEEDBACK).data[3] == 0, "miControl legacy feedback subindices");
        if (traction) { require(drive.inputId(VELOCITY) == 0x3300, "miControl ordinary velocity uses 0x3300"); }
        if (current) { require(drive.inputId(CURRENT) == 0x3200, "miControl current uses 0x3200"); }
      }
      VirtualDriveProtocol controller(traction ? 4 : 3, traction, 4096, 16, controller_type);
      for (const auto& reply : controller.handle(drive.readErrorRegisterRequest()))
      { require(drive.update(reply).empty(), "polling before supervisor grant cannot write a mode"); }
      controller.setFault(1);
      std::vector<can_msgs::msg::Frame> frames;
      const auto idle = traction ? drive.inputId(drive.targetMode()) : STEERING_PROFILE_VELOCITY_INPUT;
      frames.insert(frames.begin(), {drive.changeDeviceStateRequest(DISABLE), drive.setTargetRequest(idle, 0),
          drive.clearErrorRequest(CLEAR), drive.clearErrorRequest(NO_OPERATION)});
      frames.push_back(drive.setDeviceModeRequest(drive.targetMode()));
      SdoConfiguration configuration; configuration.start(frames);
      for (unsigned count = 0; count < 200 && !configuration.ready() && !configuration.failed(); ++count)
      {
        const auto frame = configuration.next();
        if (frame) { for (const auto& reply : controller.handle(*frame)) { configuration.receive(reply); } }
      }
      require(configuration.ready(), "SDO disable/reset/mode transaction completes");
      require(!controller.enabled(), "initialization never enables or moves a drive");
      // Deliberately acquire no position/velocity until AFTER the enable sequence.
      for (const auto& frame : drive.changeDeviceStateRequests(ENABLE)) { controller.handle(frame); }
      require(controller.enabled(), "enable can precede first motion-feedback sample");
      for (const auto& request : {drive.readDeviceModeRequest(), drive.readDeviceStateRequest(),
                                 drive.readFeedbackRequest(POSITION_FEEDBACK), drive.readFeedbackRequest(VELOCITY_FEEDBACK)})
      { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
      require(drive.enabled() && drive.initialized() && drive.hasFeedback(POSITION_FEEDBACK) &&
              drive.hasFeedback(VELOCITY_FEEDBACK), "real decoder acquires feedback after enable");
      auto velocity_reply = controller.handle(drive.readFeedbackRequest(VELOCITY_FEEDBACK)).front();
      velocity_reply.header.stamp.nanosec = 123;
      drive.update(velocity_reply);
      const auto generation = drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY);
      drive.prepareOperatingModeTransaction();
      require(!drive.enabled() && !drive.initialized() && !drive.hasNativeStatus() && drive.currentMode() == UNKNOWN &&
              !drive.hasFeedback(POSITION_FEEDBACK) && !drive.hasFeedback(VELOCITY_FEEDBACK),
              "operating-mode transactions invalidate cached enable/mode/motion acknowledgement");
      require(drive.feedbackFreshness().generation(FeedbackFreshness::VELOCITY) == generation,
              "transaction cache invalidation retains acquisition-order watermarks");
      drive.update(velocity_reply);
      require(!drive.hasFeedback(VELOCITY_FEEDBACK), "a replay cannot refill a cleared transaction cache");
      velocity_reply.header.stamp.nanosec = 124;
      drive.update(velocity_reply);
      require(drive.hasFeedback(VELOCITY_FEEDBACK), "fresh acquisition can refill the transaction cache");
    }
    std::cout << "PASS: bounded startup and SDO initialization/enable before motion feedback\n";
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
