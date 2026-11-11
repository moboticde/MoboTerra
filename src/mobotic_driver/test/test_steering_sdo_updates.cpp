#include <mobotic_driver/can_scheduling.hpp>
#include <array>
#include <iostream>
#include <string>

namespace
{
using namespace std::chrono_literals;
using Clock = SdoChannel::Clock;
using Frame = can_msgs::msg::Frame;
void require(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }
unsigned object(const Frame& frame) { return frame.data[1] | (unsigned(frame.data[2]) << 8); }
Frame make(unsigned node, unsigned index, int value = 0, bool read = false)
{
  auto frame = createAndFillMoboticHeader(0x600 + node,
      read ? READ_FROM_DEVICE : WRITE_TO_DEVICE_4B, static_cast<ParameterIds>(index),
      read && index == MICONTROL_VELOCITY_FEEDBACK ? 1 : 0);
  intToFrame(frame, 4, value);
  return frame;
}
void admit(SdoChannel& channel, const Frame& profile, const Frame& target, Clock::time_point now)
{ channel.pushSteering(profile, target, now); }
Frame reply(Frame frame, bool abort = false)
{
  frame.id -= 0x80;
  frame.data[0] = abort ? ABORT_REPLY : frame.data[0] == READ_FROM_DEVICE ? READ_REPLY_4B : WRITE_REPLY;
  return frame;
}
void acknowledge(SdoChannel& channel, const Frame& frame)
{ require(channel.receive(reply(frame)), "response must match the one outstanding SDO"); }

void ordering()
{
  const auto start = Clock::time_point{} + 1s;
  const auto profile = make(3, STEERING_PROFILE_VELOCITY_INPUT, 347);
  auto target = make(3, ABSOLUTE_POSITION_INPUT, 1);
  SdoChannel changing;
  admit(changing, profile, target, start);
  const auto first = changing.next(.1, start);
  require(first && first->data == profile.data, "the speed profile must be acknowledged before its target");
  require(!changing.next(.1, start + 1ms), "no target while the profile response is missing");
  intToFrame(target, 4, 2);
  admit(changing, profile, target, start + 2ms);
  acknowledge(changing, *first);
  changing.push(make(3, POSITION_FEEDBACK, 0, true), start + 2ms);
  changing.push(make(3, MICONTROL_STATUS_REGISTER, 0, true), start + 2ms);
  const auto latest = changing.next(.1, start + 3ms, 0, true);
  require(latest && object(*latest) == ABSOLUTE_POSITION_INPUT && intFromFrame(*latest, 4) == 2,
          "new samples retain the service slot and the started pair finishes with its newest target");
  acknowledge(changing, *latest);

  SdoChannel steady;
  unsigned profiles = 0, targets = 0;
  for (unsigned tick = 0; tick < 400; tick += 20)
  {
    intToFrame(target, 4, tick);
    admit(steady, profile, target, start + std::chrono::milliseconds(tick));
    for (unsigned slot = 0; slot < 2; ++slot)
    {
      const auto frame = steady.next(.1, start + std::chrono::milliseconds(tick + slot));
      if (!frame) { continue; }
      profiles += object(*frame) == STEERING_PROFILE_VELOCITY_INPUT;
      targets += object(*frame) == ABSOLUTE_POSITION_INPUT;
      acknowledge(steady, *frame);
    }
  }
  require(profiles == 1 && targets == 20, "an unchanged proven profile cannot consume every target service slot");

  // A same-position move must still latch a changed speed. An old target ACK
  // proves only the profile associated with that acknowledged transaction.
  SdoChannel changed_speed;
  admit(changed_speed, profile, target, start);
  auto speed_frame = changed_speed.next(.1, start); acknowledge(changed_speed, *speed_frame);
  auto position_frame = changed_speed.next(.1, start + 1ms); acknowledge(changed_speed, *position_frame);
  auto faster = profile; intToFrame(faster, 4, 500);
  admit(changed_speed, faster, target, start + 20ms);
  speed_frame = changed_speed.next(.1, start + 20ms);
  require(speed_frame && speed_frame->data == faster.data, "changed speed needs a matching profile write");
  acknowledge(changed_speed, *speed_frame);
  admit(changed_speed, faster, target, start + 21ms);
  position_frame = changed_speed.next(.1, start + 22ms);
  require(position_frame && object(*position_frame) == ABSOLUTE_POSITION_INPUT,
          "a previous target ACK cannot suppress latching the new profile at the same position");
  acknowledge(changed_speed, *position_frame);

  SdoChannel short_restart;
  admit(short_restart, profile, target, start);
  speed_frame = short_restart.next(.1, start); acknowledge(short_restart, *speed_frame);
  position_frame = short_restart.next(.1, start + 1ms); acknowledge(short_restart, *position_frame);
  short_restart.push(make(3, STEERING_PROFILE_VELOCITY_INPUT), start + 2ms);
  short_restart.push(make(3, MICONTROL_CLEAR_ERROR_STATUS, 5), start + 2ms);
  for (unsigned slot=0; slot<2; ++slot)
  { auto stop = short_restart.next(.1, start + std::chrono::milliseconds(3+slot));
    require(stop.has_value(), "short stop applies zero and Update"); acknowledge(short_restart, *stop); }
  admit(short_restart, profile, target, start + 6ms);
  speed_frame = short_restart.next(.1, start + 6ms); acknowledge(short_restart, *speed_frame);
  admit(short_restart, profile, target, start + 7ms);
  position_frame = short_restart.next(.1, start + 8ms);
  require(position_frame && object(*position_frame) == ABSOLUTE_POSITION_INPUT,
          "a short stop/resume must relatch the same profile and same target despite the old target ACK");
  acknowledge(short_restart, *position_frame);

  SdoChannel expired;
  admit(expired, profile, target, start);
  require(expired.next(.1, start).has_value(), "profile admitted before lost response");
  require(!expired.next(.1, start + 101ms), "an expired command cannot continue after a lost profile ACK");
  SdoChannel aborted;
  admit(aborted, profile, target, start);
  auto pending = aborted.next(.1, start);
  require(pending && aborted.receive(reply(*pending, true)), "profile abort is correlated");
  pending = aborted.next(.1, start + 3ms);
  require(pending && object(*pending) == STEERING_PROFILE_VELOCITY_INPUT,
          "a profile abort must not release a position target");
  aborted.cancelMotionTargets();
  acknowledge(aborted, *pending);
  require(!aborted.next(.1, start + 4ms), "an old profile reply cannot resurrect a revoked target");

  for (unsigned stop_object : {unsigned(STEERING_PROFILE_VELOCITY_INPUT), unsigned(MICONTROL_DEVICE_STATE)})
  {
    SdoChannel stopped;
    admit(stopped, profile, target, start);
    pending = stopped.next(.1, start);
    stopped.push(make(3, stop_object), start + 1ms);
    acknowledge(stopped, *pending);
    pending = stopped.next(.1, start + 2ms, 0, true);
    require(pending && object(*pending) == stop_object && intFromFrame(*pending, 4) == 0,
            "stop/disable preempts and revokes a partially admitted move");
    acknowledge(stopped, *pending);
    require(!stopped.next(.1, start + 3ms), "the pre-stop target must never leak after stopping");
  }

  SdoChannel unknown;
  admit(unknown, profile, target, start);
  unknown.push(make(3, MICONTROL_DEVICE_MODE, 0, true), start);
  auto observation = unknown.next(.1, start, 0, false, false);
  require(observation && observation->data[0] == READ_FROM_DEVICE,
          "stale native state cancels motion but still permits observation");
  acknowledge(unknown, *observation);
  require(!unknown.next(.1, start + 1ms), "later fresh native state cannot resurrect cancelled motion");

  SdoChannel lost_native;
  admit(lost_native, profile, target, start);
  pending = lost_native.next(.1, start);
  require(!lost_native.next(.1, start + 1ms, 0, false, false), "native loss is observed during an in-flight profile");
  acknowledge(lost_native, *pending);
  require(!lost_native.next(.1, start + 2ms), "a profile ACK cannot resurrect a target cancelled during native loss");

  SdoChannel changed_mode;
  admit(changed_mode, profile, target, start);
  pending = changed_mode.next(.1, start); acknowledge(changed_mode, *pending);
  pending = changed_mode.next(.1, start + 1ms); acknowledge(changed_mode, *pending);
  changed_mode.push(make(3, MICONTROL_DEVICE_MODE, 7, true), start + 2ms);
  observation = changed_mode.next(.1, start + 2ms); acknowledge(changed_mode, *observation);
  intToFrame(target, 4, 999); admit(changed_mode, profile, target, start + 3ms);
  pending = changed_mode.next(.1, start + 3ms);
  require(pending && object(*pending) == STEERING_PROFILE_VELOCITY_INPUT,
          "a changed or newly observed native state invalidates cached profile proof");
  std::cout << "PASS: steering ordering, coalescing, profile reuse, expiry and stop revocation\n";
}

void stress()
{
  const auto start = Clock::time_point{} + 1s;
  const std::vector<unsigned> nodes{1,2,3,4,5,6,7,8};
  const std::array<unsigned,4> steering{1,3,5,7};
  std::map<unsigned,SdoChannel> channels;
  SdoBusScheduler scheduler;
  struct Reply { Clock::time_point at; Frame frame; };
  std::deque<Reply> replies;
  std::map<unsigned,std::array<Clock::time_point,4>> samples;
  std::map<unsigned,Clock::time_point> last_target;
  std::map<unsigned,unsigned> target_count, latched_profiles;
  std::array<std::map<unsigned,double>,3> starts;
  double max_native_age = 0, max_motion_age = 0, max_target_gap = 0;
  for (unsigned node : nodes) { samples[node] = {start,start,start,start}; }
  auto last_batch = start - 100ms;
  for (unsigned tick = 0; tick < 3000; ++tick)
  {
    const auto now = start + std::chrono::milliseconds(tick);
    while (!replies.empty() && replies.front().at <= now)
    {
      const auto response = replies.front().frame; replies.pop_front();
      const unsigned node = response.id - 0x580;
      require(channels[node].receive(response), "stress response correlation");
      int field = -1;
      switch (object(response))
      {
        case MICONTROL_STATUS_REGISTER: field=0; break;
        case MICONTROL_DEVICE_MODE: field=1; break;
        case POSITION_FEEDBACK: field=2; break;
        case MICONTROL_VELOCITY_FEEDBACK: field=3; break;
      }
      if (field >= 0) { samples[node][field] = now; }
    }
    const bool moving = tick < 400 || (tick >= 600 && tick < 1500) || tick >= 1600;
    if (tick % 20 == 0)
    {
      for (unsigned node : nodes)
      {
        if (node % 2)
        {
          if (moving)
          { admit(channels[node], make(node, STEERING_PROFILE_VELOCITY_INPUT, 347),
                  make(node, ABSOLUTE_POSITION_INPUT, tick / 20), now); }
          else
          {
            channels[node].push(make(node, STEERING_PROFILE_VELOCITY_INPUT), now);
            channels[node].push(make(node, MICONTROL_CLEAR_ERROR_STATUS, 5), now);
          }
        }
        else { channels[node].push(make(node, MICONTROL_VELOCITY_INPUT, moving ? 123 : 0), now); }
      }
    }
    const bool read_batch_busy = std::any_of(channels.begin(), channels.end(), [](const auto& item) {
      return item.second.motionReadOutstanding() || item.second.nativeReadOutstanding() || item.second.faultReadOutstanding();
    });
    if (now-last_batch >= 100ms && !read_batch_busy)
    {
      last_batch = now;
      for (unsigned node : nodes)
      {
        for (unsigned index : {unsigned(POSITION_FEEDBACK), unsigned(MICONTROL_VELOCITY_FEEDBACK),
                              unsigned(MICONTROL_STATUS_REGISTER), unsigned(MICONTROL_DEVICE_MODE)})
        { channels[node].push(make(node,index,index == MICONTROL_DEVICE_MODE ? 7 : 0,true),now); }
        if (tick % 1000 < 100) { channels[node].push(make(node,MICONTROL_ERROR_REGISTER,0,true),now); }
      }
    }
    const bool motion_phase = std::any_of(channels.begin(), channels.end(), [](const auto& item) {
      return item.second.motionReadOutstanding();
    });
    const auto frame = scheduler.next(nodes, [&](unsigned node) {
      return channels[node].next(.1,now,0,motion_phase);
    },now);
    if (frame)
    {
      const unsigned node = frame->id - 0x600;
      if (frame->data[0] != READ_FROM_DEVICE && node % 2)
      {
        if (object(*frame) == STEERING_PROFILE_VELOCITY_INPUT) { latched_profiles[node] = intFromFrame(*frame,4); }
        if (object(*frame) == ABSOLUTE_POSITION_INPUT)
        {
          require(latched_profiles[node] == 347, "every dispatched target needs its applied positive profile");
          if (last_target.count(node) && moving && tick != 600 && tick != 1600)
          { max_target_gap = std::max(max_target_gap,std::chrono::duration<double,std::milli>(now-last_target[node]).count()); }
          last_target[node] = now; ++target_count[node];
          const unsigned phase = tick < 400 ? 0 : tick < 1500 ? 1 : 2;
          starts[phase].emplace(node,tick);
        }
        if (!moving) { last_target.erase(node); }
      }
      replies.push_back({now+3ms,reply(*frame)});
    }
    if (tick >= 250)
    {
      for (unsigned node : nodes)
      {
        for (unsigned field=0;field<4;++field)
        {
          const double age = std::chrono::duration<double,std::milli>(now-samples[node][field]).count();
          auto& maximum = field < 2 ? max_native_age : max_motion_age;
          maximum = std::max(maximum,age);
        }
      }
    }
  }
  double max_start_skew = 0;
  for (const auto& phase : starts)
  {
    require(phase.size() == steering.size(), "every steering node receives a target after each common start");
    auto bounds = std::minmax_element(phase.begin(),phase.end(),[](const auto& a,const auto& b) { return a.second < b.second; });
    max_start_skew = std::max(max_start_skew,bounds.second->second-bounds.first->second);
  }
  for (unsigned node : steering) { require(target_count[node] >= 10, "position updates cannot starve under 50-Hz command pressure"); }
  std::cout << "STEERING_SERVICE max_start_skew_ms=" << max_start_skew << " max_target_gap_ms=" << max_target_gap
            << " max_native_age_ms=" << max_native_age << " max_motion_age_ms=" << max_motion_age << '\n';
  require(max_start_skew <= 40, "common starts must fit a bounded four-node service round");
  require(max_target_gap < 180, "target updates must not wait through repeated feedback generations");
  require(max_native_age < 250 && max_motion_age < 250, "steering updates must retain headroom under 300-ms feedback deadlines");
}
}

int main(int argc,char** argv)
{
  try
  {
    if (argc == 1 || std::string(argv[1]) != "--stress") { ordering(); }
    stress();
    return 0;
  }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
