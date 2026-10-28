#include <stdexcept>
#include "mobotic_scanner_projection/project.hpp"
int main()
{
  using Raw = sick_safetyscanners2_interfaces::msg::RawMicroScanData;
  Raw input;
  input.header.sequence_number = 42;
  input.general_system_state.run_mode_active = true;
  input.general_system_state.device_error = true;
  input.application_data.outputs.evaluation_path_outputs_eval_out = {true, false};
  input.application_data.outputs.evaluation_path_outputs_is_safe = {true, false};
  input.application_data.outputs.evaluation_path_outputs_is_valid = {true, false};
  const auto before = input;
  const auto output = projectEvaluation(input);
  if (input != before || output.header != input.header || output.derived_values != input.derived_values ||
      output.general_system_state != input.general_system_state || output.application_data != input.application_data ||
      output.measurement_data != Raw{}.measurement_data || output.intrusion_data != Raw{}.intrusion_data)
    throw std::runtime_error("Projection altered scanner state or retained bulk data");
}
