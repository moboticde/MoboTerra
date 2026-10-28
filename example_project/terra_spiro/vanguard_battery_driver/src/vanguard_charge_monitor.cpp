#include "vanguard_battery_driver/battery_telemetry_logic.h"
#include "vanguard_battery_driver/charge_telemetry_logic.h"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vanguard_battery_driver
{

class VanguardChargeMonitor : public rclcpp::Node
{
public:
  explicit VanguardChargeMonitor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("vanguard_charge_monitor", options)
  {
    stale_timeout_ = declare_parameter("stale_timeout_sec", 0.75);
    active_current_threshold_ = declare_parameter("active_current_threshold_a", 1.0);
    recording_hz_ = declare_parameter("recording_hz", 10.0);
    recording_directory_ = declare_parameter("recording_dir", std::string("can_telemetry_records"));
    recording_prefix_ = declare_parameter("recording_prefix", std::string("vanguard_charge"));
    battery_source_address_ = parseUnsignedParameter(
        "battery_source_address", declare_parameter("battery_source_address", std::string("0xF3")), 0xFFU);
    battery_soc_pgn_ =
        parseUnsignedParameter("battery_soc_pgn", declare_parameter("battery_soc_pgn", std::string("0xF091")),
                               0x3FFFFU);
    battery_electrical_pgn_ = parseUnsignedParameter(
        "battery_electrical_pgn", declare_parameter("battery_electrical_pgn", std::string("0xF090")), 0x3FFFFU);
    const bool record_on_start = declare_parameter("record_on_start", false);
    if (stale_timeout_ <= 0.0 || active_current_threshold_ < 0.0 || recording_hz_ <= 0.0)
    {
      throw std::invalid_argument(
          "monitor timeouts and recording frequency must be positive; current threshold must be non-negative");
    }

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    bms_mode_pub_ = create_publisher<std_msgs::msg::UInt8>("bms_charge/mode", state_qos);
    bms_current_pub_ = create_publisher<std_msgs::msg::Float32>("bms_charge/requested_current", state_qos);
    bms_fresh_pub_ = create_publisher<std_msgs::msg::Bool>("bms_charge/fresh", state_qos);

    output_voltage_pub_ = create_publisher<std_msgs::msg::Float32>("charger/output_voltage", state_qos);
    output_current_pub_ = create_publisher<std_msgs::msg::Float32>("charger/output_current", state_qos);
    charger_status_pub_ = create_publisher<std_msgs::msg::UInt8>("charger/status", state_qos);
    ac_connected_pub_ = create_publisher<std_msgs::msg::Bool>("charger/ac_connected", state_qos);
    ac_voltage_pub_ = create_publisher<std_msgs::msg::Float32>("charger/ac_voltage", state_qos);
    charger_active_pub_ = create_publisher<std_msgs::msg::Bool>("charger/charging_active", state_qos);
    charger_fault_pub_ = create_publisher<std_msgs::msg::Bool>("charger/fault", state_qos);
    charger_status_fresh_pub_ = create_publisher<std_msgs::msg::Bool>("charger/status_fresh", state_qos);

    voltage_limit_pub_ = create_publisher<std_msgs::msg::Float32>("charger/voltage_limit", state_qos);
    current_limit_pub_ = create_publisher<std_msgs::msg::Float32>("charger/current_limit", state_qos);
    power_rating_pub_ = create_publisher<std_msgs::msg::Float32>("charger/power_rating_kw", state_qos);
    charger_limits_fresh_pub_ = create_publisher<std_msgs::msg::Bool>("charger/limits_fresh", state_qos);

    battery_soc_pub_ = create_publisher<std_msgs::msg::Float32>("battery/soc_percent", state_qos);
    battery_voltage_pub_ = create_publisher<std_msgs::msg::Float32>("battery/pack_voltage", state_qos);
    battery_current_pub_ = create_publisher<std_msgs::msg::Float32>("battery/pack_current", state_qos);
    battery_soc_fresh_pub_ = create_publisher<std_msgs::msg::Bool>("battery/soc_fresh", state_qos);
    battery_electrical_fresh_pub_ =
        create_publisher<std_msgs::msg::Bool>("battery/electrical_fresh", state_qos);

    can_rx_ = create_subscription<can_msgs::msg::Frame>(
        "can_rx", rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
        [this](const can_msgs::msg::Frame& frame) { canFrameCallback(frame); });

    start_recording_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_recording",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { startRecording(response); });
    stop_recording_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_recording",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) { stopRecording(response); });

    freshness_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishFreshness(); });
    recording_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / recording_hz_),
                                         [this]() { recordSnapshot(); });
    publishFreshness(true);

    if (record_on_start)
    {
      std::string error;
      if (!openRecording(error))
      {
        throw std::runtime_error(error);
      }
    }

    RCLCPP_INFO(get_logger(),
                "Vanguard charge monitor started in read-only mode; this node has no CAN transmit publisher.");
  }

  ~VanguardChargeMonitor() override { closeRecording(); }

private:
  static uint32_t parseUnsignedParameter(const std::string& name, const std::string& value, const uint32_t maximum)
  {
    try
    {
      std::size_t parsed_characters = 0;
      const auto parsed = std::stoul(value, &parsed_characters, 0);
      if (parsed_characters != value.size() || parsed > maximum)
      {
        throw std::out_of_range("outside valid range");
      }
      return static_cast<uint32_t>(parsed);
    }
    catch (const std::exception& exception)
    {
      throw std::invalid_argument("invalid " + name + " value '" + value + "': " + exception.what());
    }
  }

  template<typename MessageT, typename PublisherT, typename ValueT>
  static void publishValue(const PublisherT& publisher, const ValueT value)
  {
    MessageT message;
    message.data = value;
    publisher->publish(message);
  }

  void canFrameCallback(const can_msgs::msg::Frame& frame)
  {
    if (const auto soc = decodeBatterySoc(frame, static_cast<uint8_t>(battery_source_address_), battery_soc_pgn_))
    {
      latest_battery_soc_ = soc;
      last_battery_soc_ = now();
      publishValue<std_msgs::msg::Float32>(battery_soc_pub_, soc->percent);
      publishFreshness();
      return;
    }

    if (const auto electrical = decodeBatteryElectrical(
            frame, static_cast<uint8_t>(battery_source_address_), battery_electrical_pgn_))
    {
      latest_battery_electrical_ = electrical;
      last_battery_electrical_ = now();
      publishValue<std_msgs::msg::Float32>(battery_voltage_pub_, electrical->pack_voltage);
      publishValue<std_msgs::msg::Float32>(battery_current_pub_, electrical->pack_current);
      publishFreshness();
      return;
    }

    if (const auto request = decodeBmsChargeRequest(frame))
    {
      latest_bms_request_ = request;
      last_bms_request_ = now();
      publishValue<std_msgs::msg::UInt8>(bms_mode_pub_, request->mode);
      publishValue<std_msgs::msg::Float32>(bms_current_pub_, request->requested_current);
      publishFreshness();
      return;
    }

    if (const auto status = decodeChargerStatus(frame))
    {
      latest_charger_status_ = status;
      last_charger_status_ = now();
      publishValue<std_msgs::msg::Float32>(output_voltage_pub_, status->output_voltage);
      publishValue<std_msgs::msg::Float32>(output_current_pub_, status->output_current);
      publishValue<std_msgs::msg::UInt8>(charger_status_pub_, status->status);
      publishValue<std_msgs::msg::Bool>(ac_connected_pub_, status->ac_connected);
      publishValue<std_msgs::msg::Float32>(ac_voltage_pub_, status->ac_voltage);
      const bool charging = (status->status == 3 || status->status == 5) &&
                            status->output_current > active_current_threshold_;
      publishValue<std_msgs::msg::Bool>(charger_active_pub_, charging);
      publishValue<std_msgs::msg::Bool>(charger_fault_pub_, status->status == 254);
      publishFreshness();
      return;
    }

    if (const auto limits = decodeChargerLimits(frame))
    {
      latest_charger_limits_ = limits;
      last_charger_limits_ = now();
      publishValue<std_msgs::msg::Float32>(voltage_limit_pub_, limits->voltage_limit);
      publishValue<std_msgs::msg::Float32>(current_limit_pub_, limits->current_limit);
      publishValue<std_msgs::msg::Float32>(power_rating_pub_, limits->power_rating_kw);
      publishFreshness();
    }
  }

  bool isFresh(const std::optional<rclcpp::Time>& timestamp, const rclcpp::Time& current_time) const
  {
    return timestamp && (current_time - *timestamp).seconds() <= stale_timeout_;
  }

  void publishFreshness(const bool force = false)
  {
    const auto current_time = now();
    const bool bms_fresh = isFresh(last_bms_request_, current_time);
    const bool status_fresh = isFresh(last_charger_status_, current_time);
    const bool limits_fresh = isFresh(last_charger_limits_, current_time);
    const bool battery_soc_fresh = isFresh(last_battery_soc_, current_time);
    const bool battery_electrical_fresh = isFresh(last_battery_electrical_, current_time);

    if (force || bms_fresh_ != bms_fresh)
    {
      bms_fresh_ = bms_fresh;
      publishValue<std_msgs::msg::Bool>(bms_fresh_pub_, bms_fresh_);
    }
    if (force || charger_status_fresh_ != status_fresh)
    {
      charger_status_fresh_ = status_fresh;
      publishValue<std_msgs::msg::Bool>(charger_status_fresh_pub_, charger_status_fresh_);
    }
    if (force || charger_limits_fresh_ != limits_fresh)
    {
      charger_limits_fresh_ = limits_fresh;
      publishValue<std_msgs::msg::Bool>(charger_limits_fresh_pub_, charger_limits_fresh_);
    }
    if (force || battery_soc_fresh_ != battery_soc_fresh)
    {
      battery_soc_fresh_ = battery_soc_fresh;
      publishValue<std_msgs::msg::Bool>(battery_soc_fresh_pub_, battery_soc_fresh_);
    }
    if (force || battery_electrical_fresh_ != battery_electrical_fresh)
    {
      battery_electrical_fresh_ = battery_electrical_fresh;
      publishValue<std_msgs::msg::Bool>(battery_electrical_fresh_pub_, battery_electrical_fresh_);
    }
  }

  std::filesystem::path makeRecordingPath() const
  {
    const auto current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local_time{};
    localtime_r(&current_time, &local_time);
    std::ostringstream timestamp;
    timestamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");

    const std::filesystem::path directory(recording_directory_);
    const std::string prefix = recording_prefix_.empty() ? "vanguard_charge" : recording_prefix_;
    auto path = directory / (prefix + "_" + timestamp.str() + ".json");
    unsigned suffix = 1;
    while (std::filesystem::exists(path))
    {
      std::ostringstream name;
      name << prefix << "_" << timestamp.str() << "_" << std::setw(2) << std::setfill('0') << suffix++
           << ".json";
      path = directory / name.str();
    }
    return path;
  }

  bool openRecording(std::string& error)
  {
    if (recording_file_.is_open())
    {
      return true;
    }

    try
    {
      recording_path_ = makeRecordingPath();
      std::filesystem::create_directories(recording_path_.parent_path());
      recording_file_.open(recording_path_, std::ios::out | std::ios::trunc);
      if (!recording_file_.is_open())
      {
        error = "failed to open " + recording_path_.string();
        return false;
      }
      recording_file_ << "[\n";
      recording_file_.flush();
      recording_count_ = 0;
      recording_start_time_ = now();
      RCLCPP_INFO(get_logger(), "Started charge telemetry recording: %s", recording_path_.c_str());
      return true;
    }
    catch (const std::exception& exception)
    {
      error = std::string("failed to start recording: ") + exception.what();
      return false;
    }
  }

  std::optional<std::filesystem::path> closeRecording()
  {
    if (!recording_file_.is_open())
    {
      return std::nullopt;
    }
    if (recording_count_ > 0)
    {
      recording_file_ << "\n";
    }
    recording_file_ << "]\n";
    recording_file_.flush();
    recording_file_.close();
    auto path = recording_path_;
    recording_path_.clear();
    recording_count_ = 0;
    recording_start_time_.reset();
    return path;
  }

  void startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
  {
    std::string error;
    if (!openRecording(error))
    {
      response->success = false;
      response->message = error;
      return;
    }
    response->success = true;
    response->message = "Recording to " + recording_path_.string();
  }

  void stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
  {
    const auto path = closeRecording();
    response->success = true;
    response->message = path ? "Stopped recording " + path->string() : "Recorder was not active.";
    if (path)
    {
      RCLCPP_INFO(get_logger(), "Stopped charge telemetry recording: %s", path->c_str());
    }
  }

  static void writeOptionalFloat(std::ostream& output, const std::optional<float> value)
  {
    if (value)
    {
      output << *value;
    }
    else
    {
      output << "null";
    }
  }

  static void writeOptionalByte(std::ostream& output, const std::optional<uint8_t> value)
  {
    if (value)
    {
      output << static_cast<unsigned>(*value);
    }
    else
    {
      output << "null";
    }
  }

  static std::string wallTimestamp()
  {
    const auto current_time = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(current_time);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()).count() % 1000;
    std::tm local_time{};
    localtime_r(&seconds, &local_time);
    std::ostringstream timestamp;
    timestamp << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(3) << std::setfill('0')
              << milliseconds;
    return timestamp.str();
  }

  void recordSnapshot()
  {
    if (!recording_file_.is_open())
    {
      return;
    }

    const auto current_time = now();
    const bool bms_fresh = isFresh(last_bms_request_, current_time);
    const bool status_fresh = isFresh(last_charger_status_, current_time);
    const bool limits_fresh = isFresh(last_charger_limits_, current_time);
    const bool battery_soc_fresh = isFresh(last_battery_soc_, current_time);
    const bool battery_electrical_fresh = isFresh(last_battery_electrical_, current_time);
    const bool charging_active = latest_charger_status_ &&
                                 (latest_charger_status_->status == 3 || latest_charger_status_->status == 5) &&
                                 latest_charger_status_->output_current > active_current_threshold_;

    if (recording_count_++ > 0)
    {
      recording_file_ << ",\n";
    }
    recording_file_ << std::fixed << std::setprecision(6);
    recording_file_ << "  {\n";
    recording_file_ << "    \"tmps\": \"" << wallTimestamp() << "\",\n";
    recording_file_ << "    \"stamp_sec\": " << current_time.seconds() << ",\n";
    recording_file_ << "    \"elapsed_s\": "
                    << (recording_start_time_ ? (current_time - *recording_start_time_).seconds() : 0.0) << ",\n";
    recording_file_ << "    \"bms\": {\n";
    recording_file_ << "      \"fresh\": " << (bms_fresh ? "true" : "false") << ",\n";
    recording_file_ << "      \"mode\": ";
    writeOptionalByte(recording_file_, latest_bms_request_ ? std::optional<uint8_t>(latest_bms_request_->mode)
                                                           : std::nullopt);
    recording_file_ << ",\n      \"requested_current_a\": ";
    writeOptionalFloat(recording_file_, latest_bms_request_
                                            ? std::optional<float>(latest_bms_request_->requested_current)
                                            : std::nullopt);
    recording_file_ << "\n    },\n";
    recording_file_ << "    \"battery\": {\n";
    recording_file_ << "      \"soc_fresh\": " << (battery_soc_fresh ? "true" : "false") << ",\n";
    recording_file_ << "      \"electrical_fresh\": "
                    << (battery_electrical_fresh ? "true" : "false") << ",\n";
    recording_file_ << "      \"soc_percent\": ";
    writeOptionalFloat(recording_file_, latest_battery_soc_
                                            ? std::optional<float>(latest_battery_soc_->percent)
                                            : std::nullopt);
    recording_file_ << ",\n      \"pack_voltage_v\": ";
    writeOptionalFloat(recording_file_, latest_battery_electrical_
                                            ? std::optional<float>(latest_battery_electrical_->pack_voltage)
                                            : std::nullopt);
    recording_file_ << ",\n      \"pack_current_a\": ";
    writeOptionalFloat(recording_file_, latest_battery_electrical_
                                            ? std::optional<float>(latest_battery_electrical_->pack_current)
                                            : std::nullopt);
    recording_file_ << "\n    },\n";
    recording_file_ << "    \"charger\": {\n";
    recording_file_ << "      \"status_fresh\": " << (status_fresh ? "true" : "false") << ",\n";
    recording_file_ << "      \"output_voltage_v\": ";
    writeOptionalFloat(recording_file_, latest_charger_status_
                                            ? std::optional<float>(latest_charger_status_->output_voltage)
                                            : std::nullopt);
    recording_file_ << ",\n      \"output_current_a\": ";
    writeOptionalFloat(recording_file_, latest_charger_status_
                                            ? std::optional<float>(latest_charger_status_->output_current)
                                            : std::nullopt);
    recording_file_ << ",\n      \"status\": ";
    writeOptionalByte(recording_file_, latest_charger_status_
                                           ? std::optional<uint8_t>(latest_charger_status_->status)
                                           : std::nullopt);
    recording_file_ << ",\n      \"ac_connected\": ";
    if (latest_charger_status_)
      recording_file_ << (latest_charger_status_->ac_connected ? "true" : "false");
    else
      recording_file_ << "null";
    recording_file_ << ",\n      \"ac_voltage_v\": ";
    writeOptionalFloat(recording_file_, latest_charger_status_
                                            ? std::optional<float>(latest_charger_status_->ac_voltage)
                                            : std::nullopt);
    recording_file_ << ",\n      \"charging_active\": " << (charging_active ? "true" : "false") << ",\n";
    recording_file_ << "      \"fault\": "
                    << (latest_charger_status_ && latest_charger_status_->status == 254 ? "true" : "false")
                    << ",\n";
    recording_file_ << "      \"limits_fresh\": " << (limits_fresh ? "true" : "false") << ",\n";
    recording_file_ << "      \"voltage_limit_v\": ";
    writeOptionalFloat(recording_file_, latest_charger_limits_
                                            ? std::optional<float>(latest_charger_limits_->voltage_limit)
                                            : std::nullopt);
    recording_file_ << ",\n      \"current_limit_a\": ";
    writeOptionalFloat(recording_file_, latest_charger_limits_
                                            ? std::optional<float>(latest_charger_limits_->current_limit)
                                            : std::nullopt);
    recording_file_ << ",\n      \"power_rating_kw\": ";
    writeOptionalFloat(recording_file_, latest_charger_limits_
                                            ? std::optional<float>(latest_charger_limits_->power_rating_kw)
                                            : std::nullopt);
    recording_file_ << "\n    }\n  }";
    recording_file_.flush();
  }

  double stale_timeout_{0.75};
  double active_current_threshold_{1.0};
  double recording_hz_{10.0};
  std::string recording_directory_;
  std::string recording_prefix_;
  bool bms_fresh_{false};
  bool charger_status_fresh_{false};
  bool charger_limits_fresh_{false};
  bool battery_soc_fresh_{false};
  bool battery_electrical_fresh_{false};
  uint32_t battery_source_address_{DEFAULT_BATTERY_SOURCE_ADDRESS};
  uint32_t battery_soc_pgn_{DEFAULT_BATTERY_SOC_PGN};
  uint32_t battery_electrical_pgn_{DEFAULT_BATTERY_ELECTRICAL_PGN};
  std::optional<rclcpp::Time> last_bms_request_;
  std::optional<rclcpp::Time> last_charger_status_;
  std::optional<rclcpp::Time> last_charger_limits_;
  std::optional<rclcpp::Time> last_battery_soc_;
  std::optional<rclcpp::Time> last_battery_electrical_;
  std::optional<BmsChargeRequest> latest_bms_request_;
  std::optional<ChargerStatus> latest_charger_status_;
  std::optional<ChargerLimits> latest_charger_limits_;
  std::optional<BatterySoc> latest_battery_soc_;
  std::optional<BatteryElectrical> latest_battery_electrical_;
  std::ofstream recording_file_;
  std::filesystem::path recording_path_;
  std::size_t recording_count_{0};
  std::optional<rclcpp::Time> recording_start_time_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bms_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bms_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bms_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_current_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr charger_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ac_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ac_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_fault_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_status_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr power_rating_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charger_limits_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_soc_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_soc_fresh_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_electrical_fresh_pub_;
  rclcpp::TimerBase::SharedPtr freshness_timer_;
  rclcpp::TimerBase::SharedPtr recording_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_recording_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_recording_service_;
};

}  // namespace vanguard_battery_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vanguard_battery_driver::VanguardChargeMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
