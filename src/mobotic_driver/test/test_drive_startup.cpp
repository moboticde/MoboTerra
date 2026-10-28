#include <mobotic_driver/drive_startup.hpp>
#include <mobotic_driver/pdo_configuration.hpp>
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
    startup.rearmAfterReset();
    require(startup.begin(now) && startup.active(), "explicit authorized recovery rearms a bounded startup attempt");
    DriveStartup completed;
    completed.begin(now); completed.finish();
    require(completed.complete() && !completed.expired(15, now + std::chrono::seconds(20)), "acquired feedback finishes startup");

    for (const bool traction : {false, true})
    {
      MoboticDriveCanCommunication drive("fixture", traction ? 4 : 3, traction ? VELOCITY : POSITION, 4096, 16, {});
      drive.setAutomaticModeInitialization(false);
      VirtualDriveProtocol controller(traction ? 4 : 3, traction);
      for (const auto& reply : controller.handle(drive.readErrorRegisterRequest()))
      { require(drive.update(reply).empty(), "polling before supervisor grant cannot write a mode"); }
      controller.setFault(1, 0x2310);
      auto frames = traction ? drive.tractionPdoConfigurationRequests(1) : drive.steeringTelemetryPdoConfigurationRequests(1);
      const auto idle = traction ? EC_TRACTION_VELOCITY_INPUT : STEERING_PROFILE_VELOCITY_INPUT;
      frames.insert(frames.begin(), {drive.changeDeviceStateRequest(DISABLE), drive.setTargetRequest(idle, 0),
          drive.clearErrorRequest(CLEAR), drive.clearErrorRequest(NO_OPERATION)});
      frames.push_back(drive.setDeviceModeRequest(drive.targetMode()));
      PdoConfiguration configuration; configuration.start(frames);
      for (unsigned count = 0; count < 200 && !configuration.ready() && !configuration.failed(); ++count)
      {
        const auto frame = configuration.next();
        if (frame) { for (const auto& reply : controller.handle(*frame)) { configuration.receive(reply); } }
      }
      require(configuration.ready(), "EC-style disable/reset/configure/mode transaction completes");
      require(!controller.enabled(), "initialization never enables or moves a drive");
      // Deliberately acquire no position/velocity until AFTER the enable sequence.
      for (const auto& frame : drive.changeDeviceStateRequests(ENABLE)) { controller.handle(frame); }
      require(controller.enabled(), "enable can precede first motion-feedback sample");
      for (const auto& request : {drive.readDeviceModeRequest(), drive.readDeviceStateRequest(),
                                 drive.readFeedbackRequest(POSITION_FEEDBACK), drive.readFeedbackRequest(VELOCITY_FEEDBACK)})
      { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
      require(drive.enabled() && drive.initialized() && drive.hasFeedback(POSITION_FEEDBACK) &&
              drive.hasFeedback(VELOCITY_FEEDBACK), "real decoder acquires feedback after enable");
    }
    std::cout << "PASS: bounded startup and EC initialization/enable before motion feedback\n";
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
