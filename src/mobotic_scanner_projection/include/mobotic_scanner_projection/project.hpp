#pragma once
#include <sick_safetyscanners2_interfaces/msg/raw_micro_scan_data.hpp>

// Preserve the original raw stream. This distinct derived topic carries every
// field consumed by the Python evaluation adapter, with bulk arrays omitted.
inline sick_safetyscanners2_interfaces::msg::RawMicroScanData projectEvaluation(
    const sick_safetyscanners2_interfaces::msg::RawMicroScanData& input)
{
  sick_safetyscanners2_interfaces::msg::RawMicroScanData output;
  output.header = input.header;
  output.derived_values = input.derived_values;
  output.general_system_state = input.general_system_state;
  output.application_data = input.application_data;
  return output;
}
