#include "vanguard_battery_driver/battery_telemetry_logic.h"
#include "vanguard_battery_driver/charge_rate_request_logic.h"
#include "vanguard_battery_driver/charge_telemetry_logic.h"

#include <gtest/gtest.h>

#include <array>
#include <limits>

using namespace vanguard_battery_driver;

namespace
{
SocChargePolicyInputs validPolicyInputs()
{
  SocChargePolicyInputs inputs;
  inputs.enabled = true;
  inputs.soc_fresh = true;
  inputs.soc_hysteresis_allows_charging = true;
  inputs.soc_percent = 50.0;
  inputs.soc_start_threshold_percent = 50.0;
  inputs.soc_stop_threshold_percent = 80.0;
  inputs.system_capacity_ah = 67.5;
  inputs.maximum_c_rate = 0.5;
  inputs.absolute_current_limit_a = 60.0;
  return inputs;
}
}  // namespace

TEST(ChargeRateRequestLogic, AppliesSocHysteresis)
{
  bool charging_allowed = false;
  charging_allowed = updateSocChargeHysteresis(charging_allowed, 75.0, 50.0, 80.0);
  EXPECT_FALSE(charging_allowed);
  charging_allowed = updateSocChargeHysteresis(charging_allowed, 50.0, 50.0, 80.0);
  EXPECT_TRUE(charging_allowed);
  charging_allowed = updateSocChargeHysteresis(charging_allowed, 70.0, 50.0, 80.0);
  EXPECT_TRUE(charging_allowed);
  charging_allowed = updateSocChargeHysteresis(charging_allowed, 80.0, 50.0, 80.0);
  EXPECT_FALSE(charging_allowed);
  EXPECT_FALSE(updateSocChargeHysteresis(true, 70.0, 80.0, 50.0));
  EXPECT_FALSE(updateSocChargeHysteresis(
      true, std::numeric_limits<double>::quiet_NaN(), 50.0, 80.0));
}

TEST(ChargeRateRequestLogic, DependsOnlyOnSocAndConfiguredLimits)
{
  const auto decision = socChargeRateDecision(validPolicyInputs());
  EXPECT_FLOAT_EQ(decision.requested_current_a, 33.75F);
  EXPECT_EQ(decision.reason, "soc_charge_requested");
}

TEST(ChargeRateRequestLogic, StopsAtUpperThresholdAndWaitsInsideBand)
{
  auto inputs = validPolicyInputs();
  inputs.soc_percent = 80.0;
  auto decision = socChargeRateDecision(inputs);
  EXPECT_FLOAT_EQ(decision.requested_current_a, 0.0F);
  EXPECT_EQ(decision.reason, "soc_at_or_above_stop_threshold");

  inputs.soc_percent = 70.0;
  inputs.soc_hysteresis_allows_charging = false;
  decision = socChargeRateDecision(inputs);
  EXPECT_FLOAT_EQ(decision.requested_current_a, 0.0F);
  EXPECT_EQ(decision.reason, "soc_waiting_for_start_threshold");
}

TEST(ChargeRateRequestLogic, FailsClosedWhenDisabledOrSocIsStale)
{
  auto inputs = validPolicyInputs();
  inputs.enabled = false;
  auto decision = socChargeRateDecision(inputs);
  EXPECT_EQ(decision.reason, "controller_disabled");
  EXPECT_FLOAT_EQ(decision.requested_current_a, 0.0F);

  inputs.enabled = true;
  inputs.soc_fresh = false;
  decision = socChargeRateDecision(inputs);
  EXPECT_EQ(decision.reason, "battery_soc_stale");
  EXPECT_FLOAT_EQ(decision.requested_current_a, 0.0F);
}

TEST(ChargeRateRequestLogic, AppliesCapacityAndAbsoluteCurrentLimits)
{
  auto inputs = validPolicyInputs();
  inputs.system_capacity_ah = 40.0;
  EXPECT_FLOAT_EQ(socChargeRateDecision(inputs).requested_current_a, 20.0F);

  inputs.absolute_current_limit_a = 9.03;
  EXPECT_FLOAT_EQ(socChargeRateDecision(inputs).requested_current_a, 9.0F);
}

TEST(ChargeRateRequestLogic, RejectsInvalidSocOrConfiguration)
{
  auto inputs = validPolicyInputs();
  inputs.soc_percent = 101.0;
  EXPECT_EQ(socChargeRateDecision(inputs).reason, "invalid_battery_soc");

  inputs = validPolicyInputs();
  inputs.system_capacity_ah = 0.0;
  EXPECT_EQ(socChargeRateDecision(inputs).reason, "system_capacity_not_configured");
}

TEST(ChargeRateRequestLogic, EncodesDocumentedEff3Payload)
{
  const auto zero_request = createCustomChargeRateRequest(0.0F, 0xD0);
  EXPECT_EQ(zero_request.id, 0x18EFF3D0U);
  EXPECT_TRUE(zero_request.is_extended);
  EXPECT_EQ(zero_request.dlc, 8);
  const std::array<uint8_t, 8> expected_zero{0xD6, 0x03, 0xF0, 0x00, 0x7D, 0x01, 0x00, 0x00};
  EXPECT_EQ(zero_request.data, expected_zero);

  const auto request = createCustomChargeRateRequest(33.75F, 0xD0);
  EXPECT_EQ(request.data[3], 0xA3);
  EXPECT_EQ(request.data[4], 0x7F);
}

TEST(ChargeTelemetryLogic, DecodesReadOnlyBmsAndChargerValues)
{
  can_msgs::msg::Frame frame;
  frame.is_extended = true;
  frame.dlc = 8;

  frame.id = BMS_CHARGE_REQUEST_ID;
  frame.data = {CONSTANT_CURRENT_REQUEST, 0xF4, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const auto request = decodeBmsChargeRequest(frame);
  ASSERT_TRUE(request);
  EXPECT_EQ(request->mode, CONSTANT_CURRENT_REQUEST);
  EXPECT_NEAR(request->requested_current, 25.0F, 0.001F);

  frame.id = CHARGER_STATUS_ID;
  frame.data = {0xC0, 0x03, 0xF4, 0x7E, 3, 1, 0xCC, 0x01};
  const auto status = decodeChargerStatus(frame);
  ASSERT_TRUE(status);
  EXPECT_NEAR(status->output_voltage, 48.0F, 0.001F);
  EXPECT_NEAR(status->output_current, 25.0F, 0.001F);
  EXPECT_EQ(status->status, 3);
  EXPECT_TRUE(status->ac_connected);
  EXPECT_NEAR(status->ac_voltage, 230.0F, 0.001F);

  frame.id = CHARGER_LIMITS_ID;
  frame.data = {0x90, 0x04, 0xE8, 0x80, 0x3C, 0x00, 0xFF, 0xFF};
  const auto limits = decodeChargerLimits(frame);
  ASSERT_TRUE(limits);
  EXPECT_NEAR(limits->voltage_limit, 58.4F, 0.001F);
  EXPECT_NEAR(limits->current_limit, 50.0F, 0.001F);
  EXPECT_NEAR(limits->power_rating_kw, 3.0F, 0.001F);
}

TEST(BatteryTelemetryLogic, DecodesParallelControllerSocVoltageAndCurrent)
{
  can_msgs::msg::Frame frame;
  frame.is_extended = true;
  frame.dlc = 8;
  frame.id = 0x18F091F3;
  frame.data = {0x00, 0x7D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const auto soc = decodeBatterySoc(frame);
  ASSERT_TRUE(soc);
  EXPECT_NEAR(soc->percent, 50.016F, 0.001F);

  frame.id = 0x18F090F3;
  frame.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xE8, 0x03, 0xC4, 0x7C};
  const auto electrical = decodeBatteryElectrical(frame);
  ASSERT_TRUE(electrical);
  EXPECT_NEAR(electrical->pack_voltage, 50.0F, 0.001F);
  EXPECT_NEAR(electrical->pack_current, -3.0F, 0.001F);
}
