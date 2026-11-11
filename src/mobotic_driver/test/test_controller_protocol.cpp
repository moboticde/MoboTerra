#include <mobotic_driver/can_scheduling.hpp>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

void require(bool value, const char* text) { if (!value) { throw std::runtime_error(text); } }
int main()
{
  try
  {
    MoboticDriveCanCommunication drive("traction", 4, VELOCITY, 4096, 16, {});
    drive.preserveControllerConfiguration(true);
    VirtualDriveProtocol controller(4, true);
    controller.handle(drive.setDeviceModeRequest(VELOCITY)); // Installed controller project fixture.
    for (const auto& reply : controller.handle(drive.readErrorRegisterRequest()))
    { require(drive.update(reply).empty(), "fault polling must not change installed mode"); }
    for (const auto& reply : controller.handle(drive.readDeviceModeRequest())) { drive.update(reply); }
    for (const auto& reply : controller.handle(drive.readDeviceStateRequest())) { drive.update(reply); }
    auto commands = drive.enableFromObservedState();
    require(commands.size() == 1 && shortFromFrame(commands.front(), 1) == MICONTROL_DEVICE_STATE,
            "initialized healthy miControl enables through DEV_Enable");
    controller.handle(commands.front());
    for (const auto& reply : controller.handle(drive.readDeviceStateRequest())) { drive.update(reply); }
    require(drive.enabled() && drive.enableFromObservedState().empty(), "already enabled drive is left enabled");
    controller.setFault(1);
    for (const auto& request : {drive.readDeviceStateRequest(), drive.readErrorRegisterRequest()})
    { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
    require(drive.enableFromObservedState().empty(), "enable never clears a fault");
    controller.handle(drive.clearErrorRequest(CLEAR));
    controller.handle(drive.clearErrorRequest(NO_OPERATION));
    for (const auto& request : {drive.readDeviceStateRequest(), drive.readErrorRegisterRequest()})
    { for (const auto& reply : controller.handle(request)) { drive.update(reply); } }
    require(!drive.hasError() && !drive.enableFromObservedState().empty(), "B2 reset allows a new enable sequence");
    controller.handle(drive.setDeviceModeRequest(CURRENT));
    for (const auto& reply : controller.handle(drive.readDeviceModeRequest())) { drive.update(reply); }
    require(!drive.initialized() && drive.enableFromObservedState().empty(), "incompatible mode blocks enable without rewriting it");
    auto unknown = createHeartbeatFrame(drive.rawCanNodeID());
    unknown.id = 0x380 + drive.rawCanNodeID(); unknown.dlc = 4;
    uint32ToFrame(unknown, 0, 999); drive.update(unknown);
    require(!drive.hasFeedback(POSITION_FEEDBACK), "unrelated CAN frames cannot fabricate controller feedback");

    SdoChannel channel;
    const auto now = SdoChannel::Clock::now();
    auto read = drive.readDeviceStateRequest();
    channel.push(read, now); channel.push(read, now);
    auto sent = channel.next(.1, now, 100);
    require(sent && !channel.next(.1, now, 101), "exactly one in-flight request and duplicate reads suppressed");
    auto reply = controller.handle(*sent).front(); reply.header.stamp.nanosec = 99;
    require(!channel.receive(reply), "pre-request response rejected");
    reply.header.stamp.nanosec = 0;
    require(!channel.receive(reply), "missing timestamp cannot satisfy a timestamped physical request");
    reply.header.stamp.nanosec = 100;
    require(channel.receive(reply) && !channel.next(.1, now), "matching read completes transaction");
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    require(!channel.next(.1, now + std::chrono::milliseconds(101)), "queued enable cannot outlive watchdog");
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.cancelWrites();
    require(!channel.next(.1, now), "disable cancels queued enable");
    channel.push(drive.clearErrorRequest(CLEAR), now);
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.cancelWrites(true);
    sent = channel.next(.1, now);
    require(sent && shortFromFrame(*sent, 4) == 1, "repeated disable must preserve an explicit pending B2 reset");
    require(channel.faultResetPending(), "reset pulse remains pending until its SDO exchange completes");
    auto reset_reply = controller.handle(*sent).front();
    require(channel.receive(reset_reply), "fault reset write is acknowledged independently of fault polling");
    require(!channel.faultResetPending(), "reset pulse may be released before error registers become zero");
    channel.push(drive.clearErrorRequest(NO_OPERATION), now);
    require(!channel.faultResetPending(), "reset release is not a new asserted reset pulse");
    channel.clear();
    channel.push(drive.setTargetRequest(VELOCITY_INPUT, 100), now);
    channel.push(drive.setTargetRequest(VELOCITY_INPUT, 200), now);
    sent = channel.next(.1, now);
    require(sent && intFromFrame(*sent, 4) == 200, "obsolete queued motion target replaced");
    channel.push(read, now);
    require(channel.next(.1, now + std::chrono::milliseconds(101)).has_value(), "missing response has bounded recovery");
    channel.clear();
    channel.push(read, now);
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.push(drive.changeDeviceStateRequest(DISABLE), now);
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && shortFromFrame(*sent, 1) == MICONTROL_DEVICE_STATE && shortFromFrame(*sent, 4) == 0,
            "disable survives queue delay and takes priority over polling and expired enable");
    channel.clear();
    channel.push(read, now);
    channel.push(drive.setTargetRequest(VELOCITY_INPUT, 0), now);
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && shortFromFrame(*sent, 1) == VELOCITY_INPUT && intFromFrame(*sent, 4) == 0,
            "zero target cannot expire behind telemetry reads");
    auto stop_ack = createAndFillMoboticHeader(drive.replyNodeID(), WRITE_REPLY, VELOCITY_INPUT, 0);
    require(channel.receive(stop_ack), "zero target acknowledged");
    channel.push(drive.setTargetRequest(VELOCITY_INPUT, 0), now + std::chrono::milliseconds(250));
    sent = channel.next(.1, now + std::chrono::milliseconds(250));
    require(sent && sent->data[0] == READ_FROM_DEVICE, "repeated idle zeros cannot starve status reads");
    channel.clear();
    const auto disable = drive.changeDeviceStateRequest(DISABLE);
    channel.push(disable, now);
    channel.push(drive.changeDeviceStateRequest(ENABLE), now);
    channel.cancelWrites(true, true);
    sent = channel.next(.1, now);
    require(sent && sent->data == disable.data, "revocation preserves an already queued disable and cancels enable");
    channel.push(disable, now);
    auto disabled_ack = createAndFillMoboticHeader(drive.replyNodeID(), WRITE_REPLY, MICONTROL_DEVICE_STATE, 0);
    require(channel.receive(disabled_ack), "pending disable acknowledges normally");
    require(!channel.next(.1, now), "duplicate pending disable cannot create another write after acknowledgement");
    std::cout << "PASS: preserved configuration, observed-state enable, explicit fault reset and serialized SDOs\n";
  }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
