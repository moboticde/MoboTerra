#include "vanguard_battery_driver/battery_telemetry_logic.h"
#include "vanguard_battery_driver/charge_rate_request_logic.h"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace vanguard_battery_driver
{

class VanguardChargeRateController : public rclcpp::Node
{
public:
  explicit VanguardChargeRateController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("vanguard_charge_rate_controller", options)
  {
    enabled_ = declare_parameter("enabled", false);
    system_capacity_ah_ = declare_parameter("system_capacity_ah", 0.0);
    maximum_c_rate_ = declare_parameter("maximum_c_rate", 0.5);
    absolute_current_limit_a_ = declare_parameter("absolute_current_limit_a", 60.0);
    soc_start_threshold_percent_ = declare_parameter("soc_start_threshold_percent", 50.0);
    soc_stop_threshold_percent_ = declare_parameter("soc_stop_threshold_percent", 80.0);
    battery_soc_timeout_sec_ = declare_parameter("battery_soc_timeout_sec", 0.75);
    source_address_ = static_cast<uint8_t>(parseUnsignedParameter(
        "source_address", declare_parameter("source_address", std::string("0xD0")), 0xFFU));
    battery_source_address_ = static_cast<uint8_t>(parseUnsignedParameter(
        "battery_source_address", declare_parameter("battery_source_address", std::string("0xF3")),
        0xFFU));
    battery_soc_pgn_ = parseUnsignedParameter(
        "battery_soc_pgn", declare_parameter("battery_soc_pgn", std::string("0xF091")), 0x3FFFFU);

    const std::vector<double> numeric_parameters{
        system_capacity_ah_, maximum_c_rate_, absolute_current_limit_a_,
        soc_start_threshold_percent_, soc_stop_threshold_percent_, battery_soc_timeout_sec_};
    if (!std::all_of(numeric_parameters.begin(), numeric_parameters.end(),
                     [](const double value) { return std::isfinite(value); }) ||
        system_capacity_ah_ < 0.0 || maximum_c_rate_ <= 0.0 || maximum_c_rate_ > 0.5 ||
        absolute_current_limit_a_ <= 0.0 || soc_start_threshold_percent_ < 0.0 ||
        soc_start_threshold_percent_ >= soc_stop_threshold_percent_ ||
        soc_stop_threshold_percent_ > 100.0 || battery_soc_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid SOC charge-rate controller parameter");
    }

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    requested_current_pub_ =
        create_publisher<std_msgs::msg::Float32>("charge_rate/requested_current", state_qos);
    charging_requested_pub_ =
        create_publisher<std_msgs::msg::Bool>("charge_rate/charging_requested", state_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("charge_rate/status", state_qos);
    can_tx_ = create_publisher<can_msgs::msg::Frame>(
        "can_tx", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    can_rx_ = create_subscription<can_msgs::msg::Frame>(
        "can_rx", rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
        [this](const can_msgs::msg::Frame& frame) {
          if (!frame.is_extended || j1939SourceAddress(frame) != battery_source_address_ ||
              j1939Pgn(frame) != battery_soc_pgn_)
            return;

          if (const auto soc = decodeBatterySoc(frame, battery_source_address_, battery_soc_pgn_))
          {
            battery_soc_percent_ = soc->percent;
            soc_charge_enabled_ = updateSocChargeHysteresis(
                soc_charge_enabled_, battery_soc_percent_, soc_start_threshold_percent_,
                soc_stop_threshold_percent_);
            last_battery_soc_time_ = now();
          }
          else
          {
            last_battery_soc_time_.reset();
            soc_charge_enabled_ = false;
          }
          transmitRequest();
        });

    transmit_timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() {
      transmitRequest();
    });

    publishState(0.0F, enabled_ ? "waiting_for_battery_soc" : "controller_disabled");
    if (!enabled_)
    {
      RCLCPP_WARN(get_logger(), "EFF3 SOC charge-rate controller is disabled; no CAN frames will be sent.");
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "EFF3 SOC charge-rate controller enabled: %.3f Ah, maximum %.3fC, SOC start/stop %.1f%%/%.1f%%.",
          system_capacity_ah_, maximum_c_rate_, soc_start_threshold_percent_,
          soc_stop_threshold_percent_);
    }
  }

private:
  static uint32_t parseUnsignedParameter(const std::string& name, const std::string& value,
                                         const uint32_t maximum)
  {
    try
    {
      std::size_t parsed_characters = 0;
      const auto parsed = std::stoul(value, &parsed_characters, 0);
      if (parsed_characters != value.size() || parsed > maximum)
        throw std::out_of_range("outside valid range");
      return static_cast<uint32_t>(parsed);
    }
    catch (const std::exception& exception)
    {
      throw std::invalid_argument("invalid " + name + " value '" + value + "': " + exception.what());
    }
  }

  bool isSocFresh(const rclcpp::Time& current_time) const
  {
    if (!last_battery_soc_time_)
      return false;
    const double age_sec = (current_time - *last_battery_soc_time_).seconds();
    return age_sec >= 0.0 && age_sec <= battery_soc_timeout_sec_;
  }

  ChargeRateDecision policyDecision(const rclcpp::Time& current_time) const
  {
    SocChargePolicyInputs inputs;
    inputs.enabled = enabled_;
    inputs.soc_fresh = isSocFresh(current_time);
    inputs.soc_hysteresis_allows_charging = soc_charge_enabled_;
    inputs.soc_percent = battery_soc_percent_;
    inputs.soc_start_threshold_percent = soc_start_threshold_percent_;
    inputs.soc_stop_threshold_percent = soc_stop_threshold_percent_;
    inputs.system_capacity_ah = system_capacity_ah_;
    inputs.maximum_c_rate = maximum_c_rate_;
    inputs.absolute_current_limit_a = absolute_current_limit_a_;
    return socChargeRateDecision(inputs);
  }

  void transmitRequest()
  {
    const auto current_time = now();
    const auto decision = policyDecision(current_time);
    publishState(decision.requested_current_a, decision.reason);

    if (!enabled_)
      return;

    auto frame = createCustomChargeRateRequest(decision.requested_current_a, source_address_);
    frame.header.stamp = current_time;
    frame.header.frame_id = "vanguard_custom_charge_rate_request";
    can_tx_->publish(frame);
  }

  void publishState(const float requested_current_a, const std::string& reason)
  {
    std_msgs::msg::Float32 current_message;
    current_message.data = requested_current_a;
    requested_current_pub_->publish(current_message);

    std_msgs::msg::Bool active_message;
    active_message.data = requested_current_a > 0.0F;
    charging_requested_pub_->publish(active_message);

    std_msgs::msg::String status_message;
    status_message.data = reason;
    status_pub_->publish(status_message);
  }

  bool enabled_{false};
  bool soc_charge_enabled_{false};
  double system_capacity_ah_{0.0};
  double maximum_c_rate_{0.5};
  double absolute_current_limit_a_{60.0};
  double soc_start_threshold_percent_{50.0};
  double soc_stop_threshold_percent_{80.0};
  double battery_soc_timeout_sec_{0.75};
  double battery_soc_percent_{0.0};
  uint8_t source_address_{0xD0};
  uint8_t battery_source_address_{0xF3};
  uint32_t battery_soc_pgn_{0xF091};
  std::optional<rclcpp::Time> last_battery_soc_time_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr requested_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr charging_requested_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
};

}  // namespace vanguard_battery_driver

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vanguard_battery_driver::VanguardChargeRateController>());
  rclcpp::shutdown();
  return 0;
}
