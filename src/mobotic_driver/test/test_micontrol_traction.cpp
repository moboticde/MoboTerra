// Actual physical driver on a local CAN-topic bus, without SocketCAN hardware.
#include <mobotic_driver/mobotic_driver.h>
#include <mobotic_driver/virtual_drive_protocol.hpp>
#include <mobotic_driver/can_scheduling.hpp>
#include <chrono>
#include <functional>
#include <fstream>
#include <mutex>
#include <iostream>
#include <thread>

void require(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }

// Production launches the final sender as a separate process. Keep its timer
// off the driver/controller-model executor and hand accepted frames back via
// a mutex-protected queue; controller fixtures remain single-threaded.
struct SenderExecutor
{
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread worker;
  explicit SenderExecutor(const std::shared_ptr<PacedSocketCanSender>& sender)
  { executor.add_node(sender); worker = std::thread([this] { executor.spin(); }); }
  void stop()
  { executor.cancel(); if (worker.joinable()) { worker.join(); } }
  ~SenderExecutor() { stop(); }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    const bool sdo_only = true;
    const bool paced_sender = true;
    const bool software_enable_diagnostics = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--software-enable-diagnostics"; });
    const bool align_on_enable = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--align-on-enable"; });
    const bool nearest_home = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--nearest-home"; });
    const bool idle_only = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--idle-only"; });
    const bool sustained_only = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--sustained-only"; });
    const bool steering_updates_only = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--steering-updates-only"; });
    const bool startup_hold_only = std::any_of(argv, argv + argc,
        [](const char* arg) { return std::string(arg) == "--startup-hold-only"; });
    require(!startup_hold_only || !align_on_enable, "position-preserving startup disables optional alignment");
    require(!paced_sender || sdo_only, "paced sender integration uses SDO-only transport");
    double feedback_period = .02;
    unsigned reply_delay_ms = 0;
    const std::string period_option = "--feedback-period=";
    for (int i = 1; i < argc; ++i)
    {
      const std::string argument(argv[i]);
      if (argument.rfind(period_option, 0) == 0)
      { feedback_period = std::stod(argument.substr(period_option.size())); }
      const std::string delay_option = "--reply-delay-ms=";
      if (argument.rfind(delay_option, 0) == 0)
      { reply_delay_ms = std::stoul(argument.substr(delay_option.size())); }
    }
    require(std::isfinite(feedback_period) && feedback_period > 0 && feedback_period < .3,
            "test feedback period must preserve the original 300 ms deadline");
    using Frame = can_msgs::msg::Frame;
    using State = mobotic_interfaces::msg::VehicleModeState;
    using Operation = mobotic_interfaces::msg::WheelModuleOperationCommand;
    using Modules = mobotic_interfaces::msg::WheelModuleStatusArray;
    using Command = mobotic_interfaces::msg::WheelModuleCommandArray;
    const std::vector<std::string> modules{"front_left", "front_right", "rear_left", "rear_right"};
    std::vector<std::string> names, modes, ids, steering, traction;
    std::vector<double> gearing;
    const unsigned nodes[] = {1, 2, 3, 4, 7, 8, 5, 6};
    for (const auto& name : modules)
    {
      steering.push_back(name + "_steering"); traction.push_back(name + "_traction");
      names.insert(names.end(), {steering.back(), traction.back()});
      modes.insert(modes.end(), {"position", "velocity"});
      gearing.insert(gearing.end(), {121, 16});
    }
    for (unsigned node : nodes) { ids.push_back(std::to_string(node)); }
    std::map<DriveLimits, double> limits{{MIN_TRACTION_VELOCITY, -1700}, {MAX_TRACTION_VELOCITY, 1700}};
    MoboticDriveCanCommunication backend("traction", 2, VELOCITY, 4096, 16, limits, "micontrol");
    require(backend.targetMode() == VELOCITY &&
            backend.inputId(VELOCITY) == MICONTROL_VELOCITY_INPUT,
            "legacy miControl velocity is native mode 3 and target 0x3300");
    require(backend.getTarget(backend.inputId(VELOCITY), 10000) == 1700 &&
            backend.getTarget(backend.inputId(VELOCITY), -10000) == -1700, "traction clamps in motor rpm");
    const auto mode_request = backend.setDeviceModeRequest(backend.targetMode());
    require(mode_request.data[0] == 0x23 && shortFromFrame(mode_request, 1) == 0x3003 &&
            intFromFrame(mode_request, 4) == 3, "legacy four-byte mode command");
    for (const auto& request : {backend.changeDeviceStateRequest(ENABLE), backend.clearErrorRequest(CLEAR)})
    { require(request.data[0] == 0x23 && intFromFrame(request, 4) == 1, "legacy four-byte enable/reset command"); }
    require(backend.readFeedbackRequest(CURRENT_FEEDBACK).data[3] == 0 &&
            backend.readFeedbackRequest(VELOCITY_FEEDBACK).data[3] == 1 &&
            backend.readFeedbackRequest(POSITION_FEEDBACK).data[3] == 0,
            "legacy current, velocity and position subindices");
    bool unsupported_rejected = false;
    try { MoboticDriveCanCommunication unsupported("traction", 4, VELOCITY, 4096, 16, limits, "elmo"); }
    catch (const std::invalid_argument&) { unsupported_rejected = true; }
    require(unsupported_rejected, "unsupported controller configuration must fail explicitly");
    VirtualDriveProtocol protocol(2, true, 4096, 16, "micontrol");
    for (unsigned index : {0x60C5, 0x60C6, 0x603F})
    {
      auto read = createAndFillMoboticHeader(0x602, READ_FROM_DEVICE, index, 0);
      require(protocol.handle(read).front().data[0] == ABORT_REPLY, "miControl fixture must reject Elmo-only objects");
    }
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"can_node_id", "0x0B"}, {"virtual_position_source", false},
      {"steering_align_on_enable", align_on_enable}, {"steering_alignment_velocity", .3},
      {"steering_alignment_timeout", 2.0},
      {"traction_pdo_enabled", !sdo_only}, {"telemetry_pdo_enabled", !sdo_only},
      {"feedback_request_period", feedback_period}, {"can_heartbeat_period", .1},
      {"drives.names", names}, {"drives.modes", modes}, {"drives.can_node_ids", ids},
      {"drives.controller_types", std::vector<std::string>(8, "micontrol")},
      {"drives.gear_ratios", gearing}, {"drives.resolutions", std::vector<int64_t>(8, 4096)},
      {"drives.min_steering_velocities", std::vector<int64_t>(8, -5000)},
      {"drives.max_steering_velocities", std::vector<int64_t>(8, 5000)},
      {"drives.min_traction_velocities", std::vector<int64_t>(8, -1700)},
      {"drives.max_traction_velocities", std::vector<int64_t>(8, 1700)},
      {"drives.min_currents", std::vector<int64_t>(8, -4000)},
      {"drives.max_currents", std::vector<int64_t>(8, 4000)},
      {"modules.names", modules}, {"modules.steering_drives", steering}, {"modules.traction_drives", traction}
    });
    if (software_enable_diagnostics)
    { options.append_parameter_override("software_enable_diagnostics", true); }
    if (sdo_only)
    {
      auto impossible = options;
      impossible.append_parameter_override("physical_sample_skew_budget", .9 * 15 * std::chrono::duration<double>(can_scheduling::minimum_gap).count());
      bool rejected = false;
      try { auto invalid_driver = std::make_shared<MoboticDriver>(impossible); }
      catch (const std::invalid_argument&) { rejected = true; }
      require(rejected, "an undersized coherence window must reject the configured eight-drive scan");
    }
    auto bus = std::make_shared<rclcpp::Node>("micontrol_traction_test_bus");
    auto rx = bus->create_publisher<Frame>("can_rx", rclcpp::QoS(rclcpp::KeepAll()));
    auto authority = bus->create_publisher<State>("vehicle/mode", 1);
    auto operation = bus->create_publisher<Operation>("wheel_modules/operation_command", 1);
    auto command = bus->create_publisher<Command>("wheel_modules/command", 1);
    std::map<unsigned, VirtualDriveProtocol> controllers;
    for (unsigned node : nodes)
    { controllers.emplace(node, VirtualDriveProtocol(node, node % 2 == 0, 4096, node % 2 ? 121 : 16, "micontrol")); }
    std::map<unsigned, int32_t> home_targets{{1, 0}, {3, 0}, {5, 0}, {7, 0}};
    if (align_on_enable) { controllers.at(5).setInitialPosition(119); }
    const std::map<unsigned, int32_t> startup_positions{{1, 700}, {3, -900}, {5, 4596}, {7, -7392}};
    if (startup_hold_only)
    { for (const auto& [node, position] : startup_positions) { controllers.at(node).setInitialPosition(position); } }
    if (nearest_home)
    {
      require(align_on_enable, "nearest-home fixture requires alignment");
      home_targets = {{1, 4096}, {3, -4096}, {7, 8192}, {5, -8192}};
      for (const auto& [node, target] : home_targets)
      { controllers.at(node).setInitialPosition(target + (node == 1 || node == 5 ? -119 : 119)); }
    }
    bool cross_home_boundary = false;
    std::vector<Frame> transmitted;
    std::vector<std::chrono::steady_clock::time_point> socket_attempts;
    std::deque<std::pair<std::chrono::steady_clock::time_point, Frame>> deferred_replies;
    std::map<unsigned, bool> software_enable_readback;
    bool inject_overspeed = false, reject_current_mode = false;
    bool drop_motion_sdos = false, ignore_enable_writes = false;
    // Fault injection affects one drive only: the complete enable batch must
    // wait for every member, even if all other cached observations look ready.
    unsigned enable_reply_fault = 0;
    int64_t enable_probe_started_ns = 0;
    std::mutex accepted_mutex;
    struct AcceptedFrame { std::chrono::steady_clock::time_point at; int64_t stamp; Frame frame; };
    std::deque<AcceptedFrame> accepted_frames;
    std::optional<std::chrono::steady_clock::time_point> accepted_at;
    std::optional<int64_t> accepted_stamp;
    auto deliver = [&](const Frame& frame) {
      socket_attempts.push_back(accepted_at.value_or(std::chrono::steady_clock::now()));
      transmitted.push_back(frame);
      if (sdo_only)
      {
        require(frame.id != 0x80 && frame.id != 0 && !(frame.id >= 0x200 && frame.id <= 0x27F),
                "SDO-only lifecycle must never transmit SYNC, NMT or drive RPDO commands");
        if (frame.id >= 0x601 && frame.id <= 0x608)
        {
          const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1));
          require(!(index >= 0x1400 && index <= 0x1BFF),
                  "SDO-only lifecycle must never remap or configure drive PDOs");
        }
      }
      for (auto& [node, controller] : controllers)
      {
        if (ignore_enable_writes && node == 2 && frame.id == 0x602 && frame.data[0] == 0x23 &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3004 && intFromFrame(frame, 4) == 1)
        {
          auto ack = createAndFillMoboticHeader(0x582, WRITE_REPLY, 0x3004, 0);
          ack.header.stamp = bus->now(); rx->publish(ack);
          continue;
        }
        if (frame.id == 0x600 + node && frame.data[0] != READ_FROM_DEVICE &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3003)
        { require(!controller.enabled(), "native operating-mode writes require disabled controllers"); }
        if (reject_current_mode && node == 2 && frame.id == 0x602 && frame.data[0] == 0x23 &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3003 && intFromFrame(frame, 4) == CURRENT)
        {
          auto abort = createAndFillMoboticHeader(0x582, ABORT_REPLY, 0x3003, 0);
          uint32ToFrame(abort, 4, 0x06090030); abort.header.stamp = bus->now(); rx->publish(abort);
          continue;
        }
        for (auto reply : controller.handle(frame))
        {
          const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1));
          if (cross_home_boundary && node == 5 && frame.id == 0x605 &&
              frame.data[0] == READ_FROM_DEVICE && index == 0x3762)
          { intToFrame(reply, 4, 2047); }
          if (frame.id == 0x600 + node && frame.data[0] == READ_FROM_DEVICE && index == 0x3004)
          { software_enable_readback[node] = reply.data[4] != 0; }
          const bool probe_mode = node == 2 && frame.id == 0x602 &&
              frame.data[0] == READ_FROM_DEVICE && index == 0x3003;
          const bool probe_state = node == 2 && frame.id == 0x602 &&
              frame.data[0] == READ_FROM_DEVICE && index == 0x3002;
          if ((probe_mode && enable_reply_fault == 1) || (probe_state && enable_reply_fault == 2)) { continue; }
          if (drop_motion_sdos && frame.id == 0x600 + node && frame.data[0] == READ_FROM_DEVICE &&
              (index == 0x3762 || index == 0x3A04)) { continue; }
          if (reply.id < 0x580 && !controller.enabled()) { continue; }
          if (inject_overspeed && node % 2 == 0 && reply.id == 0x180 + node && reply.dlc == 8)
          { intToFrame(reply, 4, 5000); }
          if (inject_overspeed && node % 2 == 0 && frame.id == 0x600 + node &&
              frame.data[0] == READ_FROM_DEVICE && index == 0x3A04 && frame.data[3] == 1)
          { intToFrame(reply, 4, 5000); }
          reply.header.stamp = rclcpp::Time(accepted_stamp.value_or(bus->now().nanoseconds()));
          if (probe_mode && enable_reply_fault == 3) { intToFrame(reply, 4, CURRENT); }
          if (probe_mode && enable_reply_fault == 4) { reply.header.stamp = decltype(reply.header.stamp){}; }
          if (probe_mode && enable_reply_fault == 5) { reply.header.stamp = rclcpp::Time(enable_probe_started_ns - 1); }
          if (probe_state && enable_reply_fault == 6) { reply.header.stamp = rclcpp::Time(enable_probe_started_ns - 1); }
          if (probe_mode && enable_reply_fault == 7) { reply.dlc = 7; }
          if (reply_delay_ms)
          { deferred_replies.emplace_back(accepted_at.value_or(std::chrono::steady_clock::now()) + std::chrono::milliseconds(reply_delay_ms), reply); }
          else { rx->publish(reply); }
        }
      }
    };
    rclcpp::Subscription<Frame>::SharedPtr tx;
    std::shared_ptr<PacedSocketCanSender> sender;
    std::unique_ptr<SenderExecutor> sender_executor;
    unsigned sender_attempt_count = 0;
    if (paced_sender)
    {
      rclcpp::NodeOptions sender_options;
      sender_options.parameter_overrides({{"interface", "controller_model_only"}});
      sender = std::make_shared<PacedSocketCanSender>(sender_options, [&](const Frame& frame) {
        // Deterministic executor/transport jitter in the loaded regression.
        if (sustained_only && ++sender_attempt_count % 50 == 0)
        { std::this_thread::sleep_for(std::chrono::milliseconds(3)); }
        const auto at = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(accepted_mutex);
        accepted_frames.push_back({at, sender->now().nanoseconds(), frame});
      });
      sender_executor = std::make_unique<SenderExecutor>(sender);
    }
    else { tx = bus->create_subscription<Frame>("can_tx", rclcpp::QoS(rclcpp::KeepAll()), deliver); }
    auto pail_tx = bus->create_publisher<Frame>("can_tx", 10);
    rclcpp::TimerBase::SharedPtr reply_timer;
    if (paced_sender || reply_delay_ms)
    { reply_timer = bus->create_wall_timer(std::chrono::milliseconds(1), [&] {
      std::deque<AcceptedFrame> accepted;
      { std::lock_guard<std::mutex> lock(accepted_mutex); accepted.swap(accepted_frames); }
      for (const auto& item : accepted)
      { accepted_at = item.at; accepted_stamp = item.stamp; deliver(item.frame); }
      accepted_at.reset(); accepted_stamp.reset();
      const auto now = std::chrono::steady_clock::now();
      while (!deferred_replies.empty() && deferred_replies.front().first <= now)
      { rx->publish(deferred_replies.front().second); deferred_replies.pop_front(); }
    }); }
    auto pail_timer = bus->create_wall_timer(std::chrono::seconds(1), [&] {
      if (!paced_sender) { return; }
      auto read = createAndFillMoboticHeader(0x60a, READ_FROM_DEVICE, 0x1001, 0);
      read.header.stamp = bus->now(); pail_tx->publish(read);
    });
    Modules latest;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    sensor_msgs::msg::JointState joints;
    auto status = bus->create_subscription<Modules>("wheel_modules/status", 10, [&](const Modules& msg) { latest = msg; });
    auto diagnostic_sub = bus->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10,
        [&](const diagnostic_msgs::msg::DiagnosticArray& msg) { diagnostics = msg; });
    auto feedback = bus->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
        [&](const sensor_msgs::msg::JointState& msg) { joints = msg; });
    auto driver = std::make_shared<MoboticDriver>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(bus); executor.add_node(driver);
    const auto spin_until = [&](std::function<bool()> predicate, double seconds) {
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      // Wait on the next executable like production spin(). Draining ready
      // work then sleeping 1 ms adds an unrelated delay to every paced slot.
      do { executor.spin_once(std::chrono::milliseconds(5)); if (predicate()) { return true; } }
      while (std::chrono::steady_clock::now() < end);
      return false;
    };
    const auto all = [&](auto predicate) {
      return latest.modules.size() == 4 && std::all_of(latest.modules.begin(), latest.modules.end(), predicate);
    };
    require(spin_until([&] { return latest.modules.size() == 4; }, 2), "passive status available");
    const auto motion_reads_since = [&](std::size_t begin) {
      return std::count_if(transmitted.begin() + begin, transmitted.end(), [](const Frame& frame) {
        if (frame.id < 0x601 || frame.id > 0x608 || frame.data[0] != READ_FROM_DEVICE) { return false; }
        const auto index = static_cast<uint16_t>(shortFromFrame(frame, 1));
        return index == 0x3762 || index == 0x3A04 || index == 0x3262;
      });
    };
    spin_until([] { return false; }, .4);
    require(motion_reads_since(0) == 0,
            "disabled miControl drives must not receive position, velocity or current polls before enable");
    const auto count_native_reads = [&](std::size_t begin, unsigned index) {
      return std::count_if(transmitted.begin() + begin, transmitted.end(), [index](const Frame& frame) {
        return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == READ_FROM_DEVICE &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == index;
      });
    };
    const auto check_idle_polling = [&] {
      // Let the previous enable/disable batch drain before measuring idle.
      spin_until([] { return false; }, .3);
      const auto idle_begin = transmitted.size();
      spin_until([] { return false; }, 2.2);
      const auto idle_states = count_native_reads(idle_begin, 0x3002);
      const auto idle_modes = count_native_reads(idle_begin, 0x3003);
      const auto software_enables = count_native_reads(idle_begin, 0x3004);
      require(idle_states >= 32 && idle_modes >= 8 && idle_modes <= 24 && idle_states >= 3 * idle_modes,
              "disabled mode polling is bounded to 1 Hz per drive while native state proof stays fast");
      require(all([](const auto& m) { return m.steering.status_known && m.traction.status_known && m.steering.fault_known && m.traction.fault_known; }),
              "idle polling preserves fresh disabled-state proof for B2 admission");
      require(motion_reads_since(idle_begin) == 0, "idle optimization must not add disabled motion queries");
      require(software_enable_diagnostics ? software_enables >= 8 && software_enables <= 24 : software_enables == 0,
              "software-enable diagnostic polling is opt-in and cannot add traffic by default");
      std::cout << "IDLE_NATIVE_POLL_SAMPLE seconds=2.2 state_reads=" << idle_states << " mode_reads=" << idle_modes
                << " software_enable_reads=" << software_enables
                << " reduction_vs_paired_percent=" << 100.0 * (idle_states - idle_modes) / (2.0 * idle_states) << std::endl;
    };
    check_idle_polling();
    for (const auto& frame : transmitted)
    { require(frame.id >= 0x700 || frame.data[0] == READ_FROM_DEVICE, "no writes before enable authorization"); }
    if (idle_only)
    {
      std::cout << "PASS: default disabled polling retains actual-state proof without software-enable reads\n";
      rclcpp::shutdown();
      return 0;
    }
    bool lease = true, enable = !align_on_enable, moving = false;
    bool jam_steering = false, safety_stop = false;
    bool transitioning = false, reconfigure = false, resetting = align_on_enable;
    uint64_t reset_id = align_on_enable ? 1 : 0;
    uint8_t selected = State::MANUAL, requested = State::MANUAL;
    double speed = .5;
    double amps = .75;
    double steering_goal = 0, steering_profile = 0;
    auto heartbeat = bus->create_wall_timer(std::chrono::milliseconds(20), [&] {
      for (auto& [node, controller] : controllers)
      { if (!(jam_steering && node == 5)) { controller.advance(.02); } }
      if (!lease) { return; }
      State state; state.header.stamp = bus->now(); state.selected_mode = selected;
      state.requested_mode = requested; state.transition_in_progress = transitioning;
      state.reset_in_progress = resetting; state.error_active = resetting; state.reset_id = reset_id;
      state.safety_stop_active = safety_stop;
      state.supervisor_instance = "micontrol-test"; state.authority_generation = 1;
      state.motion_permitted = moving && !transitioning && !resetting; authority->publish(state);
      Operation op; op.header.stamp = bus->now(); op.supervisor_instance = state.supervisor_instance;
      op.authority_generation = 1; op.enable = enable;
      op.traction_current_control = (transitioning ? requested : selected) == State::KINEMATICOFF;
      op.reconfigure_traction_mode = transitioning && reconfigure;
      op.reset_id = reset_id;
      operation->publish(op);
      if (!moving) { return; }
      Command msg; msg.header.stamp = bus->now(); msg.source_stamp = msg.header.stamp;
      msg.supervisor_instance = state.supervisor_instance; msg.authority_generation = 1;
      msg.selected_mode = selected; msg.purpose = transitioning ? Command::CONTROLLED_STOP : Command::MOTION;
      for (const auto& name : modules)
      {
        mobotic_interfaces::msg::WheelModuleCommand module;
        module.name = name;
        module.steering_position = steering_updates_only ? steering_goal : sustained_only ? .1 * std::sin(bus->now().seconds()) : 0;
        module.steering_profile_velocity = steering_updates_only ? steering_profile : sustained_only ? .2 : 0;
        module.traction_current_control = selected == State::KINEMATICOFF;
        module.traction_velocity = module.traction_current_control ? 0.0 : speed;
        module.traction_current = module.traction_current_control ? amps : 0.0;
        msg.modules.push_back(module);
      }
      command->publish(msg);
    });
    const auto dump_timing = [&] {
      if (sender_executor) { sender_executor->stop(); }
      if (sender) { std::cerr << "SENDER_COUNTERS dropped=" << sender->droppedFrames()
                             << " failures=" << sender->sendFailures() << '\n'; }
      std::ofstream trace("/tmp/moboterra-sdo-model-timing.csv");
          trace << "ms,id,command,index,value\n";
          for (std::size_t i = 0; i < transmitted.size(); ++i)
          {
            const auto& frame = transmitted[i];
            trace << std::chrono::duration<double, std::milli>(socket_attempts[i] - socket_attempts.front()).count()
                  << ',' << frame.id << ',' << unsigned(frame.data[0]) << ','
                  << unsigned(static_cast<uint16_t>(shortFromFrame(frame, 1))) << ',' << intFromFrame(frame, 4) << '\n';
          }
    };
    if (align_on_enable)
    {
      const auto reset = [&] {
        enable = false; resetting = true;
        require(spin_until([&] { return !latest.modules.empty() && std::all_of(latest.modules.begin(), latest.modules.end(),
            [&](const auto& m) { return m.reset_id == reset_id && m.reset_acknowledged; }); }, 4),
            "alignment needs a matching acknowledged B2 transaction");
        enable = true;
      };
      const auto traction_moved = [&](std::size_t first) {
        return std::any_of(transmitted.begin() + first, transmitted.end(), [](const Frame& frame) {
          return frame.id >= 0x602 && frame.id <= 0x608 && frame.id % 2 == 0 &&
              frame.data[0] != READ_FROM_DEVICE && shortFromFrame(frame, 1) == 0x3300 && intFromFrame(frame, 4) != 0;
        });
      };
      const auto before_alignment = transmitted.size();
      reset();
      require(spin_until([&] { return std::any_of(transmitted.begin() + before_alignment, transmitted.end(), [](const Frame& f) {
        return f.id == 0x605 && f.data[0] != READ_FROM_DEVICE && shortFromFrame(f, 1) == 0x3790;
      }); }, 6), "node 5 must receive an explicit move to nearest home during enable");
      require(all([](const auto& m) { return m.startup_pending && !m.startup_complete; }),
              "normal startup cannot finish before all steering units align");
      require(spin_until([&] { return all([](const auto& m) { return m.startup_complete && m.enabled && m.feedback_fresh; }); }, 5),
              "all four steering units must reach their nearest home before startup completes");
      require(!traction_moved(before_alignment), "traction must remain zero during alignment");
      for (unsigned node : {1u, 3u, 5u, 7u})
      {
        require(std::any_of(transmitted.begin() + before_alignment, transmitted.end(), [node, &home_targets](const Frame& f) {
          return f.id == 0x600 + node && f.data[0] != READ_FROM_DEVICE &&
              shortFromFrame(f, 1) == 0x3790 && intFromFrame(f, 4) == home_targets.at(node);
        }), "every steering drive must acknowledge its nearest-home target");
      }
      require(std::none_of(transmitted.begin() + before_alignment, transmitted.end(), [](const Frame& f) {
        return f.id >= 0x601 && f.id <= 0x608 && f.data[0] != READ_FROM_DEVICE &&
            (shortFromFrame(f, 1) == 0x3762 || shortFromFrame(f, 1) == 0x3772);
      }), "alignment must never reset a controller or encoder position origin");
      require(joints.position.size() == 8, "complete joint feedback after alignment");
      for (std::size_t i = 0; i < names.size(); ++i)
      {
        if (nodes[i] % 2 == 0) { continue; }
        const auto joint = std::find(joints.name.begin(), joints.name.end(), names[i]);
        require(joint != joints.name.end() && std::abs(joints.position[joint - joints.name.begin()] -
            ticksToAngle(home_targets.at(nodes[i]), 4096)) <= 2 * M_PI / 4096,
            "fresh steering feedback confirms the selected multi-turn home");
      }
      // Every later B2 cycle must align again; a jam must time out disabled.
      enable = false; resetting = false; reset_id = 0;
      require(spin_until([&] { return std::all_of(controllers.begin(), controllers.end(), [](const auto& c) { return !c.second.enabled(); }); }, 2),
              "disable before the second recovery");
      controllers.at(5).setInitialPosition(nearest_home ? 2049 : 119); jam_steering = true; reset_id = 2;
      const auto before_jam = transmitted.size();
      reset();
      if (nearest_home)
      {
        require(spin_until([&] { return std::any_of(transmitted.begin() + before_jam, transmitted.end(), [](const Frame& f) {
          return f.id == 0x605 && f.data[0] != READ_FROM_DEVICE && shortFromFrame(f, 1) == 0x3790 && intFromFrame(f, 4) == 4096;
        }); }, 6), "half-turn boundary starts with target 4096");
        cross_home_boundary = true;
      }
      require(spin_until([&] { return all([](const auto& m) { return m.startup_failed; }) &&
          std::all_of(controllers.begin(), controllers.end(), [](const auto& c) { return !c.second.enabled(); }); }, 6),
          "a jammed steering unit must fail alignment and disable every drive");
      require(!traction_moved(before_alignment), "a jam cannot release traction motion");
      if (nearest_home)
      {
        require(std::none_of(transmitted.begin() + before_jam, transmitted.end(), [](const Frame& f) {
          return f.id == 0x605 && f.data[0] != READ_FROM_DEVICE && shortFromFrame(f, 1) == 0x3790 && intFromFrame(f, 4) != 4096;
        }), "feedback crossing half-turn boundary cannot change the frozen target");
        cross_home_boundary = false;
      }
      spin_until([] { return false; }, .4);
      require(all([](const auto& m) { return m.startup_failed; }) &&
          std::all_of(controllers.begin(), controllers.end(), [](const auto& c) { return !c.second.enabled(); }),
          "alignment failure must stay disabled under the same repeated reset ID");
      // A new authorized recovery may retry, but an E-stop must interrupt it.
      enable = false; resetting = false; reset_id = 0; jam_steering = false;
      controllers.at(5).setInitialPosition(119); reset_id = 3;
      const auto before_safety = transmitted.size();
      reset();
      require(spin_until([&] { return std::any_of(transmitted.begin() + before_safety, transmitted.end(), [](const Frame& f) {
        return f.id == 0x605 && f.data[0] != READ_FROM_DEVICE && shortFromFrame(f, 1) == 0x3790 && intFromFrame(f, 4) == 0;
      }); }, 6), "retry alignment must begin before safety interruption");
      safety_stop = true;
      require(spin_until([&] { return all([](const auto& m) { return m.startup_failed; }) &&
          std::all_of(controllers.begin(), controllers.end(), [](const auto& c) { return !c.second.enabled(); }); }, 2),
          "safety stop must interrupt alignment and disable all drives");
      require(!traction_moved(before_alignment), "safety interruption cannot release traction");
      enable = false; resetting = false; reset_id = 0; safety_stop = false;
      controllers.at(5).setInitialPosition(119); reset_id = 4;
      const auto before_feedback = transmitted.size();
      reset();
      require(spin_until([&] { return std::any_of(transmitted.begin() + before_feedback, transmitted.end(), [](const Frame& f) {
        return f.id == 0x605 && f.data[0] != READ_FROM_DEVICE && shortFromFrame(f, 1) == 0x3790 && intFromFrame(f, 4) == 0;
      }); }, 6), "retry alignment must begin before feedback interruption");
      drop_motion_sdos = true;
      require(spin_until([&] { return all([](const auto& m) { return m.startup_failed; }) &&
          std::all_of(controllers.begin(), controllers.end(), [](const auto& c) { return !c.second.enabled(); }); }, 2),
          "stale motion feedback must interrupt alignment and disable all drives");
      require(!traction_moved(before_alignment), "feedback loss cannot release traction");
      std::cout << "PASS: B2 nearest-home alignment, traction inhibition, repeated recovery, jam timeout, safety stop and feedback loss\n";
      if (sender_executor) { sender_executor->stop(); }
      rclcpp::shutdown(); return 0;
    }
    controllers.at(2).setHardwareEnable(false);
    const bool withheld_verified = spin_until([&] { return software_enable_readback[2] && controllers.at(1).enabled() &&
        std::any_of(diagnostics.status.begin(), diagnostics.status.end(), [](const auto& status) {
          if (status.name.find("/front_left_traction") == std::string::npos) { return false; }
          const auto has = [&](const std::string& key, const std::string& value) {
            return std::any_of(status.values.begin(), status.values.end(), [&](const auto& pair) {
              return pair.key == key && pair.value == value;
            });
          };
          return has("software_enable_0x3004", "true") && has("actual_state_0x3002_raw", "0");
        }); }, 12);
    if (!withheld_verified)
    {
      dump_timing();
      for (const auto& status : diagnostics.status)
      {
        if (status.name.find("/front_left_traction") == std::string::npos) { continue; }
        std::cerr << "WITHHELD_DIAGNOSTIC " << status.message << '\n';
        for (const auto& value : status.values) { std::cerr << value.key << '=' << value.value << '\n'; }
      }
    }
    require(withheld_verified, "diagnostics distinguish requested software enable from hardware-withheld power");
    require(all([](const auto& m) { return m.startup_pending && !m.startup_complete; }) &&
        std::any_of(latest.modules.begin(), latest.modules.end(), [](const auto& m) {
          return m.name == "front_left" && !m.enabled && m.traction.status_known &&
              m.traction.device_state_known && !m.traction.enabled && (m.traction.device_state & 1) == 0;
        }), "software-enable readback alone cannot complete startup or report actual power enabled");
    const auto hardware_blocked_begin = transmitted.size(); moving = true;
    spin_until([] { return false; }, .35);
    require(std::none_of(transmitted.begin() + hardware_blocked_begin, transmitted.end(), [](const Frame& frame) {
      if (frame.id < 0x601 || frame.id > 0x608 || frame.data[0] == READ_FROM_DEVICE) { return false; }
      const auto index = static_cast<uint16_t>(shortFromFrame(frame, 1));
      return (index == 0x3300 || index == 0x3200 || index == 0x3790) && intFromFrame(frame, 4) != 0;
    }), "requested software enable with actual hardware disabled cannot authorize movement");
    moving = false; controllers.at(2).setHardwareEnable(true);
    const bool started = spin_until([&] { return all([](const auto& m) {
        return m.startup_complete && !m.startup_failed && m.enabled && m.position_valid && m.feedback_fresh;
      }) && joints.name.size() == 8; }, 12);
    if (!started)
    {
      for (const auto& m : latest.modules)
      { std::cerr << "STARTUP_STATE " << m.name << " enabled=" << m.enabled << " fresh=" << m.feedback_fresh
                  << " position=" << m.position_valid << " complete=" << m.startup_complete
                  << " problem=" << m.status_message << '\n'; }
      std::map<std::pair<unsigned, unsigned>, unsigned> counts;
      const auto end_ns = transmitted.empty() ? 0 : sourceStampNanoseconds(transmitted.back());
      for (const auto& frame : transmitted)
      {
        if (frame.id < 0x601 || frame.id > 0x608 || sourceStampNanoseconds(frame) < end_ns - 1'000'000'000) { continue; }
        ++counts[{frame.data[0], static_cast<uint16_t>(shortFromFrame(frame, 1))}];
      }
      for (const auto& [object, count] : counts)
      { std::cerr << "STARTUP_TRAFFIC command=" << object.first << " object=" << std::hex << object.second
                  << std::dec << " per_second=" << count << '\n'; }
    }
    require(started, "all four miControl modules complete physical startup");
    if (startup_hold_only)
    {
      spin_until([] { return false; }, .5);
      require(all([](const auto& m) { return m.startup_complete && m.enabled && m.feedback_fresh && m.position_valid; }),
              "startup completes with fresh feedback while steering remains away from zero");
      for (const auto& [node, position] : startup_positions)
      {
        const auto request = createAndFillMoboticHeader(0x600 + node, READ_FROM_DEVICE, POSITION_FEEDBACK, 0);
        const auto reply = controllers.at(node).handle(request);
        require(reply.size() == 1 && intFromFrame(reply.front(), 4) == position,
                "enable must retain each measured steering position, including negative and multiple turns");
      }
      require(std::none_of(transmitted.begin(), transmitted.end(), [](const Frame& frame) {
        if (frame.id < 0x601 || frame.id > 0x607 || frame.id % 2 == 0 || frame.data[0] == READ_FROM_DEVICE) { return false; }
        const auto index = static_cast<uint16_t>(shortFromFrame(frame, 1));
        return index == ABSOLUTE_POSITION_INPUT || index == POSITION_FEEDBACK || index == 0x3772 ||
            (index == STEERING_PROFILE_VELOCITY_INPUT && intFromFrame(frame, 4) != 0);
      }), "startup without alignment cannot write a position target, encoder value or positive steering profile");
      std::cout << "PASS: Terra Spiro-style enable preserves all four nonzero steering positions; no homing or encoder writes\n";
      if (sender_executor) { sender_executor->stop(); }
      rclcpp::shutdown();
      return 0;
    }
    if (steering_updates_only)
    {
      speed = 0; moving = true;
      double max_skew_ms = 0, max_delay_ms = 0;
      const auto steering_stopped = [&] {
        unsigned count = 0;
        for (std::size_t i = 0; i < joints.name.size(); ++i)
        { if (joints.name[i].find("steering") != std::string::npos && std::abs(joints.velocity[i]) < .005) { ++count; } }
        return count == 4;
      };
      for (unsigned phase = 0; phase < 6; ++phase)
      {
        steering_profile = 0;
        spin_until([] { return false; }, .25);
        require(spin_until(steering_stopped, 2), "each steering restart begins with measured standstill");
        const auto begin = transmitted.size();
        const auto at = std::chrono::steady_clock::now();
        steering_goal = phase % 2 ? -.3 : .3; steering_profile = .3;
        const int target = std::lround(steering_goal * 4096 / (2 * M_PI));
        std::map<unsigned, std::chrono::steady_clock::time_point> first;
        const auto received_targets = [&] {
          for (std::size_t i = begin; i < transmitted.size(); ++i)
          {
            const auto& frame = transmitted[i];
            if (frame.id >= 0x601 && frame.id <= 0x607 && frame.id % 2 && frame.data[0] == WRITE_TO_DEVICE_4B &&
                shortFromFrame(frame, 1) == ABSOLUTE_POSITION_INPUT && intFromFrame(frame, 4) == target)
            { first.emplace(frame.id, socket_attempts[i]); }
          }
          return first.size() == 4;
        };
        require(spin_until(received_targets, 1), "all four nodes must receive the common steering target");
        auto bounds = std::minmax_element(first.begin(), first.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
        max_skew_ms = std::max(max_skew_ms, std::chrono::duration<double, std::milli>(bounds.second->second-bounds.first->second).count());
        max_delay_ms = std::max(max_delay_ms, std::chrono::duration<double, std::milli>(bounds.second->second-at).count());
        spin_until([] { return false; }, .3);
        require(all([](const auto& m) { return m.enabled && m.feedback_fresh && m.position_valid; }),
                "common starts preserve required feedback freshness");
      }
      steering_profile = 0;
      require(spin_until(steering_stopped, 2), "zero profile still stops every steering drive");
      std::cout << "STEERING_RESTART_SAMPLE phases=6 max_start_skew_ms=" << max_skew_ms
                << " max_request_to_target_ms=" << max_delay_ms << '\n';
      require(max_skew_ms < 100 && max_delay_ms < 250,
              "steering targets cannot drift through repeated feedback rounds");
      rclcpp::shutdown();
      return 0;
    }
    if (sustained_only)
    {
      moving = true;
      auto varying_targets = bus->create_wall_timer(std::chrono::milliseconds(100), [&] {
        speed = .3 + .1 * std::sin(bus->now().seconds());
      });
      spin_until([] { return false; }, .4);
      const auto begin = transmitted.size();
      const auto healthy_drops = sender ? sender->droppedFrames() : 0;
      spin_until([&] {
        if (!all([](const auto& m) {
          return m.enabled && m.startup_complete && m.feedback_fresh && m.position_valid &&
              m.steering.status_known && m.traction.status_known && m.steering.fault_known && m.traction.fault_known;
        }))
        {
          for (const auto& m : latest.modules)
          { std::cerr << "SUSTAINED_STATE " << m.name << " enabled=" << m.enabled << " fresh=" << m.feedback_fresh
                      << " position=" << m.position_valid << " problem=" << m.status_message << '\n'; }
          dump_timing();
        }
        require(all([](const auto& m) {
          return m.enabled && m.startup_complete && m.feedback_fresh && m.position_valid &&
              m.steering.status_known && m.traction.status_known && m.steering.fault_known && m.traction.fault_known;
        }), "enabled feedback must remain continuously fresh under delayed SDO replies");
        const auto joint_stamp = rclcpp::Time(joints.header.stamp).nanoseconds();
        if (joint_stamp <= 0 || bus->now().nanoseconds() - joint_stamp >= 300'000'000)
        { dump_timing(); std::cerr << "JOINT_AGE_MS " << (bus->now().nanoseconds() - joint_stamp) / 1e6 << '\n'; }
        require(joint_stamp > 0 &&
            (bus->now().nanoseconds() - joint_stamp) < 300'000'000,
            "coherent JointState must stay within the unchanged 300 ms deadline");
        return false;
      }, 60);
      std::map<std::pair<unsigned, unsigned>, std::chrono::steady_clock::time_point> previous;
      double maximum_native_gap_ms = 0, maximum_fault_gap_ms = 0;
      for (auto i = begin; i < transmitted.size(); ++i)
      {
        const auto& frame = transmitted[i];
        const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1));
        if (frame.id < 0x601 || frame.id > 0x608 || frame.data[0] != READ_FROM_DEVICE ||
            (index != 0x3002 && index != 0x3003 && index != 0x3001)) { continue; }
        const auto key = std::make_pair(frame.id, index);
        if (previous.count(key))
        {
          const double age = std::chrono::duration<double, std::milli>(socket_attempts[i]-previous[key]).count();
          if (index == 0x3001) { maximum_fault_gap_ms = std::max(maximum_fault_gap_ms, age); }
          else { maximum_native_gap_ms = std::max(maximum_native_gap_ms, age); }
        }
        previous[key] = socket_attempts[i];
      }
      require(previous.size() == 24 && maximum_native_gap_ms < 300 && maximum_fault_gap_ms < 2500,
              "all eight actual-state and mode streams retain the unchanged deadline");
      require(!sender || sender->droppedFrames() == healthy_drops,
              "healthy sustained feedback must not lose requests at the final queue");
      require(count_native_reads(begin, 0x3262) >= 8 && count_native_reads(begin, 0x3262) <= 400,
              "optional current reads stay at the diagnostic rate during enabled operation");
      std::cout << "SUSTAINED_FRESHNESS_SAMPLE seconds=60 reply_delay_ms=" << reply_delay_ms
                << " max_native_gap_ms=" << maximum_native_gap_ms
                << " max_fault_gap_ms=" << maximum_fault_gap_ms << " current_reads=" << count_native_reads(begin, 0x3262) << '\n';
      rclcpp::shutdown();
      return 0;
    }
    if (sdo_only)
    {
      // Withhold/corrupt one member's replies during a new enable request.
      // Cached fresh observations from before the request cannot replace this
      // pre-enable check. No positive enable write may escape for ANY member.
      for (unsigned fault : {1u, 2u, 3u, 4u, 5u, 6u, 7u})
      {
        moving = false; enable = false;
        require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 2),
                "controllers disabled before pre-enable verification fault injection");
        enable_reply_fault = fault; enable_probe_started_ns = bus->now().nanoseconds();
        const auto blocked_begin = transmitted.size(); enable = true;
        unsigned diagnostic_replies = 0;
        rclcpp::TimerBase::SharedPtr diagnostic_noise;
        if (fault == 2)
        {
          diagnostic_noise = bus->create_wall_timer(std::chrono::milliseconds(100), [&] {
            for (unsigned node : nodes)
            {
              auto reply = createAndFillMoboticHeader(0x580 + node, READ_REPLY_4B, MICONTROL_DEVICE_STATE, 0);
              intToFrame(reply, 4, 1); reply.header.stamp = bus->now(); rx->publish(reply);
              ++diagnostic_replies;
            }
          });
        }
        spin_until([] { return false; }, fault == 2 ? 1.2 : .38);
        if (diagnostic_noise) { diagnostic_noise->cancel(); }
        require(all([](const auto& m) { return m.startup_pending && !m.startup_complete; }),
                "pending native verification cannot report completed startup");
        require(std::none_of(transmitted.begin() + blocked_begin, transmitted.end(), [](const Frame& frame) {
          return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == 0x23 &&
              static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3004 && intFromFrame(frame, 4) == 1;
        }), "missing state/mode, wrong mode, zero/old timestamps and malformed replies block the complete enable batch");
        if (fault == 2)
        {
          require(diagnostic_replies >= 8,
                  "fresh diagnostic software-enable readbacks cannot replace missing actual status");
        }
        require(count_native_reads(blocked_begin, 0x3003) >= 8,
                "enable automatically refreshes mode evidence for all configured drives");
        if (fault == 1)
        {
          enable = false; enable_reply_fault = 0;
          const auto revoked_begin = transmitted.size();
          spin_until([] { return false; }, .25);
          require(std::none_of(transmitted.begin() + revoked_begin, transmitted.end(), [](const Frame& frame) {
            return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == 0x23 &&
                static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3004 && intFromFrame(frame, 4) == 1;
          }), "revoking enable cancels a pending native feedback check even when late replies recover");
          enable = true;
        }
        if (fault == 2)
        {
          lease = false;
          // Expire the original lease while the required evidence is absent.
          spin_until([] { return false; }, .4);
          enable_reply_fault = 0;
          const auto expired_begin = transmitted.size();
          spin_until([] { return false; }, .4);
          require(all([](const auto& m) { return !m.enabled && !m.startup_pending; }),
                  "lease expiry cancels a pending enable check");
          require(std::none_of(transmitted.begin() + expired_begin, transmitted.end(), [](const Frame& frame) {
            return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == 0x23 &&
                static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3004 && intFromFrame(frame, 4) == 1;
          }), "late valid replies cannot enable drives after the supervisor lease expires");
          lease = true;
        }
        enable_reply_fault = 0;
        require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 3),
                "one admitted request proceeds after new matching mode/state evidence becomes available");
      }
      // An acknowledged enable may leave a controller disabled. Once its
      // previously verified evidence expires, retries must stop as well.
      for (unsigned fault : {1u, 2u})
      {
        enable = false;
        require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 2),
                "disabled before enable-retry freshness check");
        ignore_enable_writes = true; enable = true;
        require(spin_until([&] { return controllers.at(1).enabled() && !controllers.at(2).enabled() &&
            controllers.at(8).enabled(); }, 3), "admitted batch exercises a controller that remains disabled");
        enable_reply_fault = fault;
        spin_until([] { return false; }, .45);
        const auto stale_begin = transmitted.size();
        spin_until([] { return false; }, .4);
        require(std::none_of(transmitted.begin() + stale_begin, transmitted.end(), [](const Frame& frame) {
          return frame.id == 0x602 && frame.data[0] == 0x23 &&
              static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3004 && intFromFrame(frame, 4) == 1;
        }), "enable retries cannot use expired native state or mode evidence");
        enable_reply_fault = 0; ignore_enable_writes = false;
        require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 3),
                "fresh replies restore the authorized zero-target enable retry");
      }
      enable = false;
      require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 2),
              "disabled after pre-enable verification checks");
      check_idle_polling();
      enable = true;
      require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 3),
              "idle polling reduction permits a new verified enable cycle");
      std::cout << "ENABLE_FEEDBACK_CHECK_SAMPLE missing_mode/state,wrong_mode,zero/old_stamp,malformed,revocation,lease_expiry,retry=PASS\n";
    }
    moving = true;
    const auto velocity_matches = [&] {
      unsigned count = 0;
      for (std::size_t i = 0; i < joints.name.size(); ++i)
        if (joints.name[i].find("traction") != std::string::npos && std::abs(joints.velocity[i] - speed) < .005) { ++count; }
      return count == 4;
    };
    require(spin_until(velocity_matches, 3), "positive contact m/s becomes miControl motor rpm and decodes back");
    const int native_rpm = static_cast<int>(std::lround(angVelToRpm(speed / .325, 16)));
    require(std::any_of(transmitted.begin(), transmitted.end(), [native_rpm](const Frame& frame) {
      return frame.id == 0x602 && frame.data[0] != READ_FROM_DEVICE &&
          shortFromFrame(frame, 1) == 0x3300 && intFromFrame(frame, 4) == native_rpm;
    }), "contact m/s must convert to motor rpm using radius and gearing exactly once");
    if (sdo_only)
    {
      const auto sample_begin = transmitted.size();
      const auto sample_start = std::chrono::steady_clock::now();
      spin_until([] { return false; }, 1.0);
      const double seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - sample_start).count();
      const auto polls = motion_reads_since(sample_begin);
      require(all([](const auto& m) { return m.enabled && m.feedback_fresh && m.position_valid; }),
              "feedback stays fresh during sustained motion at the configured polling period");
      require(polls <= static_cast<long>((seconds / feedback_period + 3) * 24),
              "motion queries must follow the configured polling rate");
      require(std::none_of(transmitted.begin() + sample_begin, transmitted.end(), [](const Frame& frame) {
        return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == WRITE_TO_DEVICE &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == ABSOLUTE_POSITION_INPUT;
      }), "zero-speed steering holds cannot retrigger absolute position targets");
      std::cout << "CAN_RATE_SAMPLE feedback_period_s=" << feedback_period
                << " motion_sdo_reads=" << polls << " seconds=" << seconds << std::endl;
    }
    speed = -.5;
    require(spin_until(velocity_matches, 3), "negative traction velocity roundtrips");
    speed = 0;
    require(spin_until(velocity_matches, 3), "zero command stops every traction drive");
    speed = .5;
    require(spin_until(velocity_matches, 3), "velocity motion before controlled transition");
    requested = State::KINEMATICOFF; transitioning = true;
    speed = .4;
    require(spin_until(velocity_matches, 3), "desired current mode preserves the outgoing controlled velocity ramp");
    speed = 0;
    require(spin_until(velocity_matches, 3), "outgoing velocity ramp reaches standstill before native mode authorization");
    moving = false;
    spin_until([] { return false; }, .15);
    require(all([](const auto& m) { return m.control_mode == m.CONTROL_MODE_VELOCITY; }),
            "controller mode cannot change before supervisor standstill authorization");
    reject_current_mode = true;
    reconfigure = true;
    require(spin_until([&] { return all([](const auto& m) { return m.startup_failed && !m.enabled; }); }, 5),
            "rejected native mode change fails closed and disables every module");
    reject_current_mode = false;
    requested = selected = State::MANUAL; transitioning = reconfigure = false;
    resetting = true; reset_id = 1; enable = false;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 1 && m.reset_acknowledged; }); }, 3),
            "explicit reset admits the failed mode transaction with disabled-state proof");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && !m.startup_failed && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_VELOCITY;
    }); }, 12), "reset restores selected velocity mode after a failed entry to current mode");
    resetting = false; reset_id = 0;
    requested = State::KINEMATICOFF; transitioning = reconfigure = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_CURRENT;
    }); }, 12), "standstill authorizes acknowledged miControl mode 2 and zero-target enable");
    // Cancellation can race the final native acknowledgement. The old vehicle
    // mode must never run with the newly configured current controller.
    requested = State::MANUAL; transitioning = reconfigure = false;
    require(spin_until([&] { return all([](const auto& m) { return m.startup_failed && !m.enabled; }); }, 3),
            "cancelling selection after native acknowledgement fails closed");
    resetting = true; reset_id = 2; enable = false;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 2 && m.reset_acknowledged; }); }, 3),
            "cancelled native transaction requires a new explicit reset");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_VELOCITY;
    }); }, 12), "reset restores the retained selected mode after cancellation");
    resetting = false; reset_id = 0;
    requested = State::KINEMATICOFF; transitioning = reconfigure = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_CURRENT;
    }); }, 12), "fresh authorized current-mode transaction succeeds after cancellation recovery");
    selected = requested; transitioning = reconfigure = false;
    const auto currents_sent = [&](std::size_t begin, int target) {
      unsigned count = 0;
      for (unsigned node : {2u, 4u, 6u, 8u})
        if (std::any_of(transmitted.begin() + begin, transmitted.end(), [node, target](const Frame& frame) {
          return frame.id == 0x600 + node && frame.data[0] == 0x23 &&
              static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3200 && intFromFrame(frame, 4) == target;
        })) { ++count; }
      return count == 4;
    };
    auto begin = transmitted.size();
    moving = true;
    require(spin_until([&] { return currents_sent(begin, 750); }, 3), "positive current amperes reach all four 0x3200 inputs as mA");
    begin = transmitted.size(); amps = -.75;
    require(spin_until([&] { return currents_sent(begin, -750); }, 3), "negative current targets retain direction");
    begin = transmitted.size(); moving = false;
    require(spin_until([&] { return currents_sent(begin, 0); }, 2), "authority inhibition sends zero current");
    lease = false;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 2),
            "management lease expiry disables current-controlled miControl drives");
    // Let any SDO already queued at disable finish, then observe several idle
    // feedback ticks. Status/fault reads may continue; cyclic motion reads may not.
    spin_until([] { return false; }, .3);
    const auto disabled_begin = transmitted.size();
    spin_until([] { return false; }, .4);
    require(motion_reads_since(disabled_begin) == 0,
            "motion polling must stop again after all miControl drives are disabled");
    lease = true;
    require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 3),
            "renewed authority can re-enable only at zero targets");
    requested = State::MANUAL; transitioning = reconfigure = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_VELOCITY;
    }); }, 12), "leaving direct mode restores acknowledged velocity mode");
    selected = requested; transitioning = reconfigure = false;
    speed = .5; moving = true;
    require(spin_until(velocity_matches, 3), "velocity commands work after returning from current mode");
    speed = 0;
    require(spin_until(velocity_matches, 3), "standstill before second current-mode entry");
    moving = false; requested = State::KINEMATICOFF; transitioning = reconfigure = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.startup_complete && m.enabled && m.feedback_fresh && m.control_mode == m.CONTROL_MODE_CURRENT;
    }); }, 12), "current mode can be entered again without resetting encoder reference");
    selected = requested; transitioning = reconfigure = false;
    moving = true; amps = .5;
    inject_overspeed = true;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled && m.error_code == 0x7FFF0001; }); }, 2),
            "measured current-mode overspeed stops/disables and latches an explicit-reset fault");
    inject_overspeed = false;
    spin_until([] { return false; }, .4);
    require(all([](const auto& m) { return !m.enabled && m.error_code == 0x7FFF0001; }),
            "ordinary enable publications cannot clear current-mode overspeed");
    moving = false; resetting = true; reset_id = 3; enable = false;
    require(spin_until([&] { return all([](const auto& m) { return m.reset_id == 3 && m.reset_acknowledged; }); }, 3),
            "explicit reset admits fresh disabled-state proof and clears the overspeed latch");
    enable = true;
    require(spin_until([&] { return all([](const auto& m) {
      return m.enabled && m.feedback_fresh && m.error_code == 0 && m.control_mode == m.CONTROL_MODE_CURRENT;
    }); }, 3), "explicit reset re-enables at zero current");
    resetting = false; reset_id = 0; begin = transmitted.size(); moving = true; amps = .25;
    require(spin_until([&] { return currents_sent(begin, 250); }, 3), "fresh current command is required after reset");
    lease = false;
    require(spin_until([&] { return all([](const auto& m) { return !m.enabled; }); }, 2), "final lease expiry disables every controller");
    if (sdo_only)
    {
      // Realistic-looking PDOs must not substitute for lost SDO motion replies.
      moving = false; lease = true;
      require(spin_until([&] { return all([](const auto& m) { return m.enabled && m.feedback_fresh; }); }, 3),
              "SDO-only feedback recovers after renewed neutral authority");
      begin = transmitted.size(); moving = true; amps = .5;
      require(spin_until([&] { return currents_sent(begin, 500); }, 3),
              "fresh SDO feedback authorizes current before the feedback-loss check");
      auto stray_pdos = bus->create_wall_timer(std::chrono::milliseconds(10), [&] {
        for (unsigned node : nodes)
        {
          Frame frame; frame.id = 0x180 + node; frame.dlc = 8; frame.header.stamp = bus->now();
          rx->publish(frame);
        }
      });
      begin = transmitted.size(); drop_motion_sdos = true;
      require(spin_until([&] { return all([](const auto& m) { return !m.feedback_fresh; }) &&
          currents_sent(begin, 0); }, 2),
              "stray PDOs cannot refresh SDO-only feedback or defeat the zero-target stop");
      begin = transmitted.size();
      spin_until([] { return false; }, .25);
      require(std::none_of(transmitted.begin() + begin, transmitted.end(), [](const Frame& frame) {
        return frame.id >= 0x601 && frame.id <= 0x608 && frame.data[0] == 0x23 &&
            static_cast<uint16_t>(shortFromFrame(frame, 1)) == 0x3200 && intFromFrame(frame, 4) != 0;
      }), "continued commands and stray PDOs cannot resume current after SDO feedback expires");
      moving = false; lease = false;
    }
    // Several stop sources can fire during the same withdrawal. Repeated
    // requests must retain the pending stop and bounded retries, rather than
    // deleting/rebuilding an eight-node disable on every callback.
    moving = false; lease = false;
    auto disable_client = bus->create_client<std_srvs::srv::Trigger>("internal/driver/disable_all_drives");
    require(disable_client->wait_for_service(std::chrono::seconds(1)), "disable service discovered");
    const auto disable_begin = transmitted.size();
    unsigned disable_requests = 0;
    auto repeated_disable = bus->create_wall_timer(std::chrono::milliseconds(5), [&] {
      disable_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      ++disable_requests;
    });
    spin_until([] { return false; }, .25);
    repeated_disable->cancel();
    spin_until([] { return false; }, .05);
    std::map<unsigned, unsigned> disable_writes;
    for (auto it = transmitted.begin() + disable_begin; it != transmitted.end(); ++it)
    {
      if (it->id >= 0x601 && it->id <= 0x608 && it->data[0] == 0x23 &&
          static_cast<uint16_t>(shortFromFrame(*it, 1)) == 0x3004 && intFromFrame(*it, 4) == 0)
      { ++disable_writes[it->id]; }
    }
    require(disable_requests >= 10, "disable consolidation exercised repeated independent requests");
    if (sdo_only)
    {
      require(disable_writes.size() == 8, "disable remains effective for every controller");
      for (const auto& [node, count] : disable_writes)
      { (void)node; require(count <= 4, "disable writes are bounded by the 100 ms transaction retry interval"); }
    }
    std::cout << "DISABLE_ADMISSION_SAMPLE requests=" << disable_requests
              << " nodes=" << disable_writes.size() << std::endl;
    for (const auto& frame : transmitted)
    {
      if (frame.id < 0x601 || frame.id > 0x608) { continue; }
      const unsigned index = static_cast<uint16_t>(shortFromFrame(frame, 1));
      require(index != 0x6040 && index != 0x6041 && index != 0x6060 && index != 0x6061 &&
              index != 0x603F && index != 0x60C5 && index != 0x60C6 && index != 0x6075 && index != 0x60FF,
              "miControl path must never send Elmo operational or profile objects");
      if ((frame.id - 0x600) % 2 == 0 && frame.data[0] != READ_FROM_DEVICE)
      {
        require(index != 0x3500, "ordinary miControl traction must not use S_VELOCITY");
        if (index == 0x3003)
        { require(intFromFrame(frame, 4) == 3 || intFromFrame(frame, 4) == 2, "traction uses only verified velocity/current modes"); }
      }
    }
    int64_t previous_sdo_stamp = 0;
    int64_t minimum_sdo_gap = std::numeric_limits<int64_t>::max();
    unsigned paced_sdos = 0;
    for (const auto& frame : transmitted)
    {
      if (frame.id < 0x601 || frame.id > 0x608) { continue; }
      const auto stamp = sourceStampNanoseconds(frame);
      if (!paced_sender && previous_sdo_stamp > 0)
      {
        minimum_sdo_gap = std::min(minimum_sdo_gap, stamp - previous_sdo_stamp);
        require(stamp - previous_sdo_stamp >= std::chrono::duration_cast<std::chrono::nanoseconds>(can_scheduling::minimum_gap).count(),
                "all physical drive SDOs, including initialization/recovery/stop, share the configured minimum gap");
      }
      previous_sdo_stamp = stamp;
      ++paced_sdos;
    }
    require(paced_sdos > 100, "spacing check covers sustained physical-driver traffic");
    if (sender)
    {
      minimum_sdo_gap = std::numeric_limits<int64_t>::max();
      for (std::size_t i = 1; i < socket_attempts.size(); ++i)
      {
        const auto gap = std::chrono::duration_cast<std::chrono::nanoseconds>(socket_attempts[i] - socket_attempts[i-1]).count();
        require(gap >= std::chrono::duration_cast<std::chrono::nanoseconds>(can_scheduling::minimum_gap).count(), "driver, PAIL and heartbeat share the final nonblocking socket gap");
        minimum_sdo_gap = std::min(minimum_sdo_gap, gap);
      }
      require(std::any_of(transmitted.begin(), transmitted.end(), [](const Frame& frame) { return frame.id == 0x60a; }),
              "PAIL health queries reach the same sender during real driver operation");
      sender_executor->stop();
      require(sender->sendFailures() == 0, "controller-model sends must not fail");
      std::cout << "SOCKET_PACING_SAMPLE frames=" << socket_attempts.size()
                << " minimum_gap_ms=" << minimum_sdo_gap / 1e6
                << " fault_matrix_drops=" << sender->droppedFrames() << std::endl;
      sender_executor.reset(); sender.reset();
    }
    std::cout << "SDO_PACING_SAMPLE requests=" << paced_sdos
              << " minimum_gap_ms=" << minimum_sdo_gap / 1e6 << std::endl;
    const auto shutdown_begin = transmitted.size();
    executor.remove_node(driver); driver.reset();
    spin_until([] { return false; }, .1);
    if (!paced_sender)
    {
      int64_t previous = 0;
      for (std::size_t i = shutdown_begin; i < transmitted.size(); ++i)
      {
        const auto& frame = transmitted[i];
        if (frame.id < 0x601 || frame.id > 0x608) { continue; }
        const auto stamp = sourceStampNanoseconds(frame);
        require(previous == 0 || stamp - previous >= std::chrono::duration_cast<std::chrono::nanoseconds>(can_scheduling::minimum_gap).count(), "shutdown cannot bypass the shared 5 ms SDO gap");
        previous = stamp;
      }
    }
    executor.remove_node(bus);
    std::cout << "PASS: four miControl modules, velocity/current transitions, signed A/mA targets, stop, watchdog and overspeed latch\n";
  }
  catch (const std::exception& error)
  { std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
}
