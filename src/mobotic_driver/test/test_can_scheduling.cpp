#include <mobotic_driver/can_scheduling.hpp>
#include <iostream>
#include <stdexcept>
#include <map>
#include <array>
#include <string>

namespace queue_checks
{
void require(bool result, const char* reason) { if (!result) { throw std::runtime_error(reason); } }
using Frame = can_msgs::msg::Frame;
Frame make(unsigned id, unsigned index, unsigned command, int64_t stamp, unsigned value = 0)
{
  Frame f; f.id = id; f.dlc = 8; f.header.stamp.sec = stamp / 1'000'000'000;
  f.header.stamp.nanosec = stamp % 1'000'000'000; f.data[0] = command;
  f.data[1] = index & 0xff; f.data[2] = index >> 8;
  for (unsigned i = 0; i < 4; ++i) { f.data[4+i] = (value >> (i*8)) & 0xff; }
  return f;
}
int run()
{
  try
  {
    using namespace std::chrono_literals;
    const auto start = CanTxQueue::Clock::time_point{};
    const auto gap = can_scheduling::minimum_gap;
    const auto gap_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(gap).count();
    constexpr int64_t ns = 10'000'000'000;
    CanTxQueue shared;
    for (unsigned node = 1; node <= 8; ++node)
    { require(shared.push(make(0x600 + node, 0x3004, 0x40, ns), ns), "driver read admitted"); }
    require(shared.push(make(0x60a, 0x1018, 0x40, ns), ns), "PAIL read shares same queue");
    auto heartbeat = make(0x70b, 0, 5, ns); heartbeat.dlc = 1;
    require(shared.push(heartbeat, ns), "heartbeat shares same queue");
    for (unsigned slot = 0; slot < 10; ++slot)
    {
      auto frame = shared.next(ns + slot*gap_ns, start + slot*gap);
      require(frame.has_value(), "every queued producer gets a slot");
      if (slot == 0) { require(frame->id == 0x70b, "heartbeat cannot starve behind ordinary reads"); }
      shared.complete(true, start + slot*gap);
      require(!shared.next(ns + slot*gap_ns, start + slot*gap + gap - 1us), "no producer can bypass the common gap");
    }
    require(shared.size() == 0 && shared.sent() == 10, "exactly one send per slot");
    CanTxQueue deadlines;
    deadlines.push(make(0x601, 0x3002, 0x40, ns), ns);
    deadlines.push(make(0x602, 0x3300, 0x23, ns + 10'000'000, 123), ns + 10'000'000);
    const auto command_before_read = deadlines.next(ns + 10'000'000, start);
    require(command_before_read && command_before_read->id == 0x602,
            "a fresh command's shorter lifetime takes precedence over passive reads");

    CanTxQueue stalled;
    auto read = make(0x601, 0x3004, 0x40, ns);
    stalled.push(read, ns); stalled.push(make(0x602, 0x3004, 0x40, ns), ns);
    require(stalled.next(ns, start).has_value(), "first socket attempt available");
    stalled.complete(false, start + 500us);
    require(stalled.failures() == 1 && stalled.size() == 2, "ENOBUFS retains the unsent frame");
    require(!stalled.next(ns, start + 500us + can_scheduling::minimum_gap - 1us), "failed sends also consume the retry interval");
    require(stalled.next(ns, start + 10ms).has_value(), "late callback allows only one paced retry");
    stalled.complete(true, start + 10ms);
    require(!stalled.next(ns, start + 10ms), "late executor earns no catch-up credit");
    require(!stalled.next(ns + 101'000'000, start + 110ms), "reads expire within their bounded reply window");

    CanTxQueue stopping;
    stopping.push(make(0x602, 0x3004, 0x23, ns, 1), ns);
    stopping.push(make(0x602, 0x3300, 0x23, ns, 100), ns);
    stopping.push(make(0x602, 0x3000, 0x23, ns, 1), ns);
    stopping.push(read, ns);
    auto disable = make(0x602, 0x3004, 0x23, ns + 1);
    require(stopping.push(disable, ns + 1), "stop admitted over old writes");
    require(stopping.size() == 3, "disable removes old enable/target but retains reset and read");
    require(!stopping.push(make(0x602, 0x3004, 0x23, ns, 1), ns + 1), "late pre-stop enable cannot be resurrected");
    require(stopping.next(ns + 100'000'000, start)->data == disable.data, "stop survives delay and has first priority");
    stopping.complete(true, start);

    CanTxQueue startup;
    startup.push(make(0x602, 0x3004, 0x23, ns, 1), ns);
    startup.push(make(0x602, 0x3300, 0x23, ns, 100), ns);
    auto zero = make(0x602, 0x3300, 0x23, ns + 1);
    startup.push(zero, ns + 1);
    require(startup.size() == 2, "zero target revokes old moving target without cancelling startup enable");
    auto frame = startup.next(ns + 1, start); require(frame && frame->data == zero.data, "zero precedes enable");
    startup.complete(true, start);
    frame = startup.next(ns + 1, start + gap);
    require(frame && CanTxQueue::object(*frame) == 0x3004 && frame->data[4] == 1, "zero-target enable remains executable");

    CanTxQueue latest;
    latest.push(heartbeat, ns); heartbeat.header.stamp.nanosec += 1;
    latest.push(heartbeat, ns + 1);
    require(latest.size() == 1 && latest.coalesced() == 1, "queued heartbeats coalesce instead of catching up");
    require(CanTxQueue::stamp(*latest.next(ns + 1, start)) == ns + 1, "only the newest heartbeat is transmitted");

    CanTxQueue bounded;
    for (unsigned node = 1; node <= CanTxQueue::capacity; ++node)
    { require(bounded.push(make(0x600 + node, 0x3004, 0x40, ns), ns), "bounded read admitted"); }
    require(!bounded.push(make(0x650, 0x3004, 0x40, ns), ns), "queue cannot grow past capacity");
    require(bounded.push(disable, ns + 1), "urgent stop can evict ordinary traffic at capacity");
    require(bounded.size() == CanTxQueue::capacity && CanTxQueue::stop(*bounded.next(ns + 1, start)), "bounded stop takes next slot");

    CanTxQueue invalid;
    require(!invalid.push(read, ns - 1), "future source timestamp rejected");
    require(invalid.push(read, ns + 90'000'000), "passive reads retain their existing 100 ms reply window");
    require(!invalid.push(read, ns + 101'000'000), "old passive reads cannot outlive their reply window");
    require(!invalid.push(make(0x602, 0x3300, 0x23, ns, 123), ns + 51'000'000),
            "motion writes retain the strict 50 ms source lifetime");
    auto bad = read; bad.is_extended = true;
    require(!invalid.push(bad, ns + 51'000'000), "extended frames cannot enter SDO-only transport");
    require(!invalid.push(make(0x202, 0, 0, ns + 51'000'000), ns + 51'000'000), "accidental PDO traffic rejected");

    CanTxQueue clock;
    clock.push(disable, ns + 1);
    auto fresh = make(0x602, 0x3004, 0x23, ns - 1'000'000'000, 1);
    require(clock.push(fresh, CanTxQueue::stamp(fresh)), "new clock epoch cannot inherit an unreachable old stop cutoff");
    require(clock.size() == 1 && clock.dropped() == 1, "rollback discards pre-jump queued traffic");
    std::cout << "PASS: shared CAN admission, source expiry, stop priority, bounded retries and no catch-up\n";
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
}

namespace bus_checks
{
static void require(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }

int run()
{
  try
  {
    using namespace std::chrono_literals;
    using Clock = SdoBusScheduler::Clock;
    const auto gap = can_scheduling::minimum_gap;
    const auto start = Clock::time_point{} + 1s;
    const std::vector<unsigned> nodes{1, 2, 3, 4, 5, 6, 7, 8};
    // A stop Update must never latch an unacknowledged or newer positive
    // profile into the previous position move.
    {
    const auto update = createAndFillMoboticHeader(0x601, WRITE_TO_DEVICE_4B, MICONTROL_CLEAR_ERROR_STATUS, 0);
    auto stop_update = update; intToFrame(stop_update, 4, 5);
    auto ack = [](SdoChannel& channel, can_msgs::msg::Frame request) {
      request.id -= 0x80; request.data[0] = WRITE_REPLY;
      require(channel.receive(request), "profile stop acknowledgement must correlate");
    };
    SdoChannel profile_stop;
    profile_stop.push(stop_update, start);
    auto zero = profile_stop.next(.1, start);
    require(zero && shortFromFrame(*zero, 1) == 0x3300 && intFromFrame(*zero, 4) == 0,
            "Update without zero proof must first send profile zero");
    require(!profile_stop.next(.1, start + 2ms), "Update waits for zero acknowledgement");
    ack(profile_stop, *zero);
    auto applied = profile_stop.next(.1, start + 3ms);
    require(applied && shortFromFrame(*applied, 1) == 0x3000 && intFromFrame(*applied, 4) == 5,
            "acknowledged zero precedes profile Update");
    ack(profile_stop, *applied);
    profile_stop.push(*zero, start + 4ms); profile_stop.push(stop_update, start + 4ms);
    require(!profile_stop.next(.1, start + 4ms), "a proven stopped profile does not flood CAN");
    SdoChannel missing_ack;
    missing_ack.push(stop_update, start);
    const auto lost_zero = missing_ack.next(.1, start);
    const auto retry_zero = missing_ack.next(.1, start + 101ms);
    require(lost_zero && retry_zero && shortFromFrame(*retry_zero, 1) == 0x3300,
            "a lost zero acknowledgement must not release Update");
    SdoChannel replaced_stop;
    replaced_stop.push(stop_update, start);
    auto old_zero = replaced_stop.next(.1, start); ack(replaced_stop, *old_zero);
    auto positive = *old_zero; intToFrame(positive, 4, 347);
    replaced_stop.push(positive, start + 1ms);
    auto move = positive; shortToFrame(move, 1, 0x3790); intToFrame(move, 4, 123);
    replaced_stop.push(move, start + 1ms);
    auto next_move = replaced_stop.next(.1, start + 2ms);
    require(next_move && shortFromFrame(*next_move, 1) == 0x3300 && intFromFrame(*next_move, 4) == 347,
            "new move replaces an unsent profile-stop Update");
    ack(replaced_stop, *next_move);
    next_move = replaced_stop.next(.1, start + 3ms);
    require(next_move && shortFromFrame(*next_move, 1) == 0x3790,
            "new target follows the new profile without a delayed stop Update");
    }
    SdoBusScheduler scheduler;
    std::map<unsigned, SdoChannel> channels;
    unsigned selection_calls = 0;
    for (unsigned node : nodes)
    {
      channels[node].push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE,
          MICONTROL_DEVICE_STATE, 0), start);
    }
    auto now = start;
    const auto select = [&](unsigned node) {
      ++selection_calls;
      return channels.at(node).next(.1, now);
    };
    for (unsigned node : nodes)
    {
      now = start + (node - 1) * gap;
      const auto frame = scheduler.next(nodes, select, now);
      require(frame && frame->id == 0x600 + node, "eight queued nodes must get distinct round-robin slots");
      const auto calls = selection_calls;
      now += gap - 1us;
      require(!scheduler.next(nodes, select, now), "no second node or startup selector before the shared gap");
      require(selection_calls == calls, "a blocked slot must not mutate a node transaction");
    }
    now = start + 8 * gap;
    require(!scheduler.next(nodes, select, now), "no second request while each node awaits its reply");
    auto reply = createAndFillMoboticHeader(0x582, READ_REPLY_4B, MICONTROL_DEVICE_STATE, 0);
    require(channels[2].receive(reply), "node 2 read acknowledgement");
    channels[2].push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_DEVICE_MODE, 0), now);
    require(scheduler.next(nodes, select, now).has_value(), "one stalled node must not block healthy nodes");

    // A required fault batch must reach every node before native polling can
    // refill the queue. Names and optional telemetry cannot choose the winner.
    for (unsigned node : nodes)
    {
      SdoChannel required;
      required.push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, MICONTROL_DEVICE_MODE, 0), start);
      required.push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, 0x3262, 0), start);
      required.push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, 0x3001, 0), start);
      require(required.faultReadOutstanding(), "fault evidence is required service");
      const auto first = required.next(.1, start);
      require(first && shortFromFrame(*first, 1) == 0x3001, "required fault precedes native and optional polls on every node");
      require(required.faultReadOutstanding(), "pending replies retain the batch barrier");
    }

    SdoBusScheduler late;
    const auto always = [](unsigned node) -> std::optional<can_msgs::msg::Frame> {
      return createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, MICONTROL_DEVICE_STATE, 0);
    };
    require(late.next(nodes, always, start).has_value(), "initial grant");
    require(late.next(nodes, always, start + 100ms).has_value(), "one grant after a long executor stall");
    require(!late.next(nodes, always, start + 100ms), "a late callback must not catch up in a burst");
    late.published(start + 100ms + 200us);
    require(!late.next(nodes, always, start + 100ms + gap), "gap starts after actual publication work");
    require(late.next(nodes, always, start + 100ms + gap + 200us).has_value(), "next grant after full gap");

    SdoChannel stop;
    auto zero = createAndFillMoboticHeader(0x602, WRITE_TO_DEVICE, STEERING_PROFILE_VELOCITY_INPUT, 0);
    auto moving = zero; intToFrame(moving, 4, 123);
    auto ack = createAndFillMoboticHeader(0x582, WRITE_REPLY, STEERING_PROFILE_VELOCITY_INPUT, 0);
    stop.push(zero, start);
    require(stop.next(.1, start).has_value(), "initial stop is immediate");
    stop.push(moving, start + 1ms);
    stop.push(zero, start + 2ms);
    require(stop.receive(ack), "initial zero is acknowledged");
    require(!stop.next(.1, start + 2ms), "identical pending stop replaces an obsolete queued motion target");
    stop.push(zero, start + 50ms);
    require(!stop.next(.1, start + 50ms), "independent interlocks must not flood an acknowledged zero");
    stop.push(zero, start + 1000ms);
    require(!stop.next(.1, start + 1000ms), "a proven zero needs no repeated CAN write until its proof changes");
    stop.push(moving, start + 1001ms);
    require(stop.next(.1, start + 1001ms).has_value(), "changed target is never delayed by the zero refresh window");
    stop.push(zero, start + 1002ms);
    require(stop.receive(ack), "intervening nonzero write acknowledged");
    const auto immediate_stop = stop.next(.1, start + 1003ms);
    require(immediate_stop && intFromFrame(*immediate_stop, 4) == 0,
            "a stop after any intervening motion write is never suppressed");
    stop.clear();
    stop.push(zero, start);
    require(stop.next(.02, start).has_value() && stop.receive(ack), "zero with a shorter watchdog");
    stop.push(zero, start + 20ms, .02);
    require(!stop.next(.02, start + 20ms), "a short motion watchdog must not flood a proven unchanged zero");
    stop.clear();
    stop.push(moving, start);
    require(stop.next(.1, start).has_value() && stop.receive(ack), "nonzero target acknowledged");
    stop.push(moving, start + 20ms);
    require(!stop.next(.1, start + 20ms), "unchanged acknowledged targets must not fill every input callback slot");
    stop.push(moving, start + 50ms);
    require(stop.next(.1, start + 50ms).has_value() && stop.receive(ack), "nonzero target refreshes before the command watchdog");
    stop.cancelMotionTargets();
    stop.push(moving, start + 51ms);
    require(stop.next(.1, start + 51ms).has_value(), "authority stop invalidates old nonzero target acknowledgement");
    stop.clear();
    stop.push(zero, start);
    require(stop.next(.1, start).has_value() && stop.receive(ack), "zero before a new position target");
    auto position = createAndFillMoboticHeader(0x602, WRITE_TO_DEVICE, ABSOLUTE_POSITION_INPUT, 0);
    intToFrame(position, 4, 100);
    stop.push(position, start + 1ms);
    require(stop.next(.1, start + 1ms).has_value(), "position target can change motion independently of profile velocity");
    auto position_ack = createAndFillMoboticHeader(0x582, WRITE_REPLY, ABSOLUTE_POSITION_INPUT, 0);
    require(stop.receive(position_ack), "position target acknowledged");
    stop.push(zero, start + 2ms);
    require(stop.next(.1, start + 2ms).has_value(), "new position target invalidates previous profile-velocity stop proof");
    stop.clear();
    auto disable = createAndFillMoboticHeader(0x602, WRITE_TO_DEVICE, MICONTROL_DEVICE_STATE, 0);
    auto disable_ack = createAndFillMoboticHeader(0x582, WRITE_REPLY, MICONTROL_DEVICE_STATE, 0);
    stop.push(disable, start);
    require(stop.next(.1, start).has_value() && stop.receive(disable_ack), "disable acknowledged");
    stop.push(disable, start + 1ms);
    require(stop.next(.1, start + 1ms).has_value(), "disable actions must never use motion-target suppression");

    SdoBusScheduler batch;
    std::map<unsigned, SdoChannel> batch_channels;
    for (unsigned node : nodes)
    {
      auto target = moving; target.id = 0x600 + node;
      batch_channels[node].push(target, start);
      batch_channels[node].push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, POSITION_FEEDBACK, 0), start);
      batch_channels[node].push(createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, MICONTROL_VELOCITY_FEEDBACK, 1), start);
    }
    for (unsigned slot = 0; slot < 16; ++slot)
    {
      now = start + slot * gap;
      const auto frame = batch.next(nodes, [&](unsigned node) { return batch_channels[node].next(.1, now, 0, true); }, now);
      require(frame && frame->data[0] == READ_FROM_DEVICE && frame->id == 0x601 + slot % 8,
              "sixteen motion reads must form a fair shared phase ahead of normal target traffic");
      require(batch_channels[frame->id - 0x600].motionReadOutstanding(),
              "an in-flight motion read still blocks admission of the next feedback batch");
      auto response = *frame; response.id -= 0x80; response.data[0] = READ_REPLY_4B;
      require(batch_channels[frame->id - 0x600].receive(response), "batch read matched");
    }
    require(std::none_of(batch_channels.begin(), batch_channels.end(), [](const auto& item) {
      return item.second.motionReadQueued(); }), "motion phase ends after all nodes receive their read slots");
    require(std::none_of(batch_channels.begin(), batch_channels.end(), [](const auto& item) {
      return item.second.motionReadOutstanding(); }), "next batch is admitted only after all motion replies complete");
    now = start + 16 * gap;
    const auto target = batch.next(nodes, [&](unsigned node) { return batch_channels[node].next(.1, now); }, now);
    require(target && target->data[0] == WRITE_TO_DEVICE, "normal targets resume after coherent acquisition");
    SdoChannel urgent;
    urgent.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, POSITION_FEEDBACK, 0), start);
    urgent.push(disable, start);
    const auto priority = urgent.next(.1, start, 0, true);
    require(priority && shortFromFrame(*priority, 1) == MICONTROL_DEVICE_STATE,
            "disable must interrupt a motion-read phase");
    SdoChannel confirmed_stop;
    confirmed_stop.push(zero, start);
    require(confirmed_stop.next(.1, start).has_value() && confirmed_stop.receive(ack),
            "initial stop has a matching acknowledgement");
    confirmed_stop.push(zero, start + 1000ms);
    confirmed_stop.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, POSITION_FEEDBACK, 0), start + 1000ms);
    const auto scan_read = confirmed_stop.next(.1, start + 1000ms, 0, true);
    require(scan_read && scan_read->data[0] == READ_FROM_DEVICE,
            "refreshing an already acknowledged zero must not split a coherent scan");
    auto scan_reply = *scan_read; scan_reply.id -= 0x80; scan_reply.data[0] = READ_REPLY_4B;
    require(confirmed_stop.receive(scan_reply), "scan read acknowledged");
    const auto refreshed_zero = confirmed_stop.next(.1, start + 1001ms, 0, false);
    require(!refreshed_zero, "a confirmed unchanged zero does not consume scan service slots");
    confirmed_stop.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_DEVICE_MODE, 0), start + 1001ms);
    auto mode_observation = confirmed_stop.next(.1, start + 1001ms);
    require(mode_observation.has_value(), "native mode observation selected");
    auto confirmed_mode_reply = *mode_observation; confirmed_mode_reply.id -= 0x80; confirmed_mode_reply.data[0] = READ_REPLY_4B;
    intToFrame(confirmed_mode_reply, 4, 3);
    require(confirmed_stop.receive(confirmed_mode_reply), "native mode observation matched");
    confirmed_stop.push(zero, start + 1002ms);
    auto stop_after_mode = confirmed_stop.next(.1, start + 1002ms, 0, true);
    require(stop_after_mode && stop_after_mode->data == zero.data,
            "native state/mode observations invalidate obsolete zero-target proof");
    SdoChannel new_stop;
    new_stop.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, POSITION_FEEDBACK, 0), start);
    new_stop.push(zero, start);
    const auto urgent_zero = new_stop.next(.1, start, 0, true);
    require(urgent_zero && urgent_zero->data == zero.data,
            "a new unacknowledged zero still interrupts acquisition");
    SdoChannel service;
    service.push(moving, start);
    require(service.next(.1, start).has_value() && service.receive(ack), "ordinary write acknowledged");
    service.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_STATUS_REGISTER, 0), start);
    auto state_read = service.next(.1, start);
    require(state_read && state_read->data[0] == READ_FROM_DEVICE, "native phase read");
    auto service_state_reply = *state_read; service_state_reply.id -= 0x80; service_state_reply.data[0] = READ_REPLY_4B;
    require(service.receive(service_state_reply), "native phase acknowledged");
    auto changed_target = moving; intToFrame(changed_target, 4, 456);
    service.push(changed_target, start);
    service.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_ERROR_REGISTER, 0), start);
    const auto fault_read = service.next(.1, start);
    require(fault_read && fault_read->data[0] == READ_FROM_DEVICE,
            "native phases must not let target writes starve ordinary fault reads");
    SdoChannel management_priority;
    auto admitted_enable = createAndFillMoboticHeader(0x602, WRITE_TO_DEVICE, MICONTROL_DEVICE_STATE, 0);
    intToFrame(admitted_enable, 4, 1);
    management_priority.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_STATUS_REGISTER, 0), start);
    management_priority.push(admitted_enable, start);
    const auto enabled_first = management_priority.next(.1, start, 0, false, true);
    require(enabled_first && enabled_first->data == admitted_enable.data,
            "a fresh admitted enable cannot expire behind repeated native polls");
    SdoChannel stale_enable;
    stale_enable.push(admitted_enable, start);
    stale_enable.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_STATUS_REGISTER, 0), start);
    auto observation = stale_enable.next(.1, start, 0, false, false);
    require(observation && observation->data[0] == READ_FROM_DEVICE,
            "stale native evidence cancels the queued positive enable before dispatch");
    auto observation_reply = *observation; observation_reply.id -= 0x80; observation_reply.data[0] = READ_REPLY_4B;
    require(stale_enable.receive(observation_reply), "fresh observation matched");
    require(!stale_enable.next(.1, start + 1ms), "fresh observations cannot resurrect the cancelled enable");
    for (int state : {0})
    {
      SdoChannel safe_shutdown;
      auto command = createAndFillMoboticHeader(0x604, WRITE_TO_DEVICE, MICONTROL_DEVICE_STATE, 0);
      intToFrame(command, 4, state); safe_shutdown.push(command, start);
      require(safe_shutdown.next(.1, start, 0, false, false).has_value(),
              "stale mode evidence cannot cancel a miControl disable command");
    }
    SdoChannel native;
    native.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_CURRENT_FEEDBACK, 0), start);
    native.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_STATUS_REGISTER, 0), start);
    native.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_DEVICE_MODE, 0), start);
    native.push(moving, start);
    require(native.nativeReadOutstanding(), "queued native state/mode must block a new motion generation");
    const auto state = native.next(.1, start);
    require(state && shortFromFrame(*state, 1) == MICONTROL_STATUS_REGISTER,
            "native state precedes optional current and ordinary targets between motion phases");
    require(native.nativeReadOutstanding(), "in-flight state still holds the batch barrier");
    auto state_reply = *state; state_reply.id -= 0x80; state_reply.data[0] = READ_REPLY_4B;
    require(native.receive(state_reply), "state response matched");
    const auto mode = native.next(.1, start + 1ms);
    require(mode && shortFromFrame(*mode, 1) == MICONTROL_DEVICE_MODE,
            "native mode also precedes optional telemetry");
    auto mode_reply = *mode; mode_reply.id -= 0x80; mode_reply.data[0] = READ_REPLY_4B;
    require(native.receive(mode_reply) && !native.nativeReadOutstanding(),
            "only complete native replies release the motion admission barrier");
    native.push(createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, MICONTROL_STATUS_REGISTER, 0), start + 2ms);
    native.push(disable, start + 2ms);
    require(shortFromFrame(*native.next(.1, start + 2ms), 1) == MICONTROL_DEVICE_STATE,
            "disable still preempts native evidence");
    std::cout << "PASS: shared configured spacing, fair eight-node admission, blocked-node isolation and no catch-up bursts\n";
  }
  catch (const std::exception& error)
  { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
}

namespace freshness_checks
{
// Eight acknowledged channels, configured bus slots and 3 ms reply latency. Exercise
// sustained feedback/target/stop pressure without wall-clock/executor jitter.
static double load(bool native_barrier)
{
  using namespace std::chrono_literals;
  using Clock = SdoBusScheduler::Clock;
  const auto start = Clock::time_point{} + 1s;
  const std::vector<unsigned> nodes{1,2,3,4,5,6,7,8};
  std::map<unsigned, SdoChannel> channels;
  SdoBusScheduler scheduler;
  struct Reply { Clock::time_point at; can_msgs::msg::Frame frame; };
  std::deque<Reply> replies;
  std::map<unsigned, std::array<Clock::time_point, 2>> samples;
  for (unsigned node : nodes) { samples[node] = {start, start}; }
  double maximum_age = 0;
  const auto push_read = [&](unsigned node, unsigned index, Clock::time_point now) {
    channels[node].push(createAndFillMoboticHeader(0x600+node, READ_FROM_DEVICE, index,
        index == MICONTROL_VELOCITY_FEEDBACK ? 1 : 0), now);
  };
  for (unsigned tick = 0; tick < 3000; ++tick)
  {
    const auto now = start + tick * 1ms;
    while (!replies.empty() && replies.front().at <= now)
    {
      const auto reply = replies.front(); replies.pop_front();
      const unsigned node = reply.frame.id-0x580;
      if (!channels[node].receive(reply.frame)) { throw std::runtime_error("lost SDO correlation"); }
      const unsigned index = static_cast<uint16_t>(shortFromFrame(reply.frame, 1));
      if (index == MICONTROL_STATUS_REGISTER || index == MICONTROL_DEVICE_MODE)
      { samples[node][index == MICONTROL_DEVICE_MODE] = now; }
    }
    if (tick % 20 == 0)
    {
      const bool admit = std::none_of(channels.begin(), channels.end(), [native_barrier](const auto& item) {
        return item.second.motionReadOutstanding() || (native_barrier && item.second.nativeReadOutstanding());
      });
      for (unsigned node : nodes)
      {
        if (admit)
        {
          push_read(node, POSITION_FEEDBACK, now);
          push_read(node, MICONTROL_VELOCITY_FEEDBACK, now);
        }
        if (tick % 100 == 0)
        {
          push_read(node, MICONTROL_STATUS_REGISTER, now);
          push_read(node, MICONTROL_DEVICE_MODE, now);
        }
        if (tick % 1000 == 0)
        {
          push_read(node, MICONTROL_CURRENT_FEEDBACK, now);
          push_read(node, MICONTROL_ERROR_REGISTER, now);
        }
        auto target = createAndFillMoboticHeader(0x600+node, WRITE_TO_DEVICE, STEERING_PROFILE_VELOCITY_INPUT, 0);
        intToFrame(target, 4, (tick / 500) % 2 ? 123 : 0);
        channels[node].push(target, now);
      }
    }
    if (tick % can_scheduling::minimum_gap.count() == 0)
    {
      const bool motion = std::any_of(channels.begin(), channels.end(),
          [](const auto& item) { return item.second.motionReadQueued(); });
      const auto frame = scheduler.next(nodes, [&](unsigned node) {
        return channels[node].next(.1, now, 0, motion);
      }, now);
      if (frame)
      {
        auto reply = *frame; reply.id -= 0x80;
        reply.data[0] = frame->data[0] == READ_FROM_DEVICE ? READ_REPLY_4B : WRITE_REPLY;
        replies.push_back({now+3ms, reply});
      }
    }
    if (tick >= 250)
    {
      for (const auto& [node, fields] : samples)
      {
        (void)node;
        for (const auto stamp : fields)
        { maximum_age = std::max(maximum_age, std::chrono::duration<double, std::milli>(now-stamp).count()); }
      }
    }
  }
  return maximum_age;
}

int run()
{
  try
  {
    const auto without_barrier = load(false), with_barrier = load(true);
    std::cout << "NATIVE_SERVICE_MAX_AGE_MS without_barrier=" << without_barrier
              << " with_barrier=" << with_barrier << '\n';
    if (without_barrier <= 300) { throw std::runtime_error("stress no longer reproduces native starvation"); }
    if (with_barrier >= 250) { throw std::runtime_error("native service lacks headroom under the unchanged 300 ms deadline"); }
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
  return 0;
}
}

int main(int argc, char** argv)
{
  const std::string selection = argc > 1 ? argv[1] : "--all";
  if ((selection == "--all" || selection == "--queue") && queue_checks::run()) { return 1; }
  if ((selection == "--all" || selection == "--bus") && bus_checks::run()) { return 1; }
  if ((selection == "--all" || selection == "--freshness") && freshness_checks::run()) { return 1; }
  if (selection != "--all" && selection != "--queue" && selection != "--bus" && selection != "--freshness")
  { std::cerr << "Unknown scheduling test selection\n"; return 1; }
  return 0;
}
