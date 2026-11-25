"""Non-safety-rated field telemetry from scanner evaluated output paths."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sick_safetyscanners2_interfaces.msg import RawMicroScanData
from mobotic_interfaces.msg import ScannerSafetyState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ScannerSafetyAdapter(Node):
    def __init__(self):
        super().__init__('scanner_safety_adapter')
        self.declare_parameter('status_timeout', 0.3)
        self.declare_parameter('scanner_input_suffix', 'raw_data')
        self.timeout = float(self.get_parameter('status_timeout').value)
        self.input_suffix = str(self.get_parameter('scanner_input_suffix').value)
        if self.input_suffix not in ('raw_data', 'evaluation_data'):
            raise ValueError('unsupported scanner input suffix')
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('status timeout must be positive and finite')
        self.samples = {}
        self.sequences = {}
        self.paths = {}
        self._last_output_signatures = {}
        self._last_diagnostic_signatures = {}
        self._last_diagnostic_at = {}
        self.input_subscriptions = [
            self.create_subscription(
                RawMicroScanData, f'scanner/{name}/{self.input_suffix}',
                lambda msg, location=name: self._receive_scanner(location, msg),
                qos_profile_sensor_data)
            for name in ('front_left', 'rear_right')
        ]
        self.scanner_publishers = {name: self.create_publisher(ScannerSafetyState, f'scanner/{name}/safety_zone_status', 1)
                           for name in ('front_left', 'rear_right')}
        self.diagnostics = {name: self.create_publisher(DiagnosticArray, f'scanner/{name}/diagnostics', 1)
                            for name in self.scanner_publishers}
        self.timer = self.create_timer(0.05, self._publish)

    def _receive_scanner(self, name, message):
        received = time.monotonic()
        sequence = message.header.sequence_number
        previous = self.sequences.get(name)
        restarted = False
        if previous is not None:
            # Reordered/duplicate UDP samples must not refresh a clear field.
            delta = (sequence - previous[0]) & 0xffffffff
            if delta == 0 or (delta >= 0x80000000 and received - previous[1] <= self.timeout):
                return
            restarted = delta >= 0x80000000
        if restarted:
            self.paths.pop(name, None)
        self.sequences[name] = (sequence, received)
        output = ScannerSafetyState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.scanner_id = name
        output.source = 'scanner Ethernet evaluated output paths'
        state = message.general_system_state
        outputs = message.application_data.outputs
        status = outputs.evaluation_path_outputs_eval_out
        safe = outputs.evaluation_path_outputs_is_safe
        valid = outputs.evaluation_path_outputs_is_valid
        usable = (not restarted and state.run_mode_active and not state.standby_mode_active
                  and not state.application_error and not state.device_error
                  and not state.contamination_error and len(status) == len(safe) == len(valid))
        if usable:
            # A monitoring case can select different outputs. Within a case,
            # a path losing validity must still make the field state unknown.
            monitoring_case = (
                state.current_monitoring_case_no_table_1,
                state.current_monitoring_case_no_table_2,
                state.current_monitoring_case_no_table_3,
                state.current_monitoring_case_no_table_4,
            )
            paths = self.paths.setdefault(name, {}).setdefault(monitoring_case, set())
            paths.update(i for i, flag in enumerate(valid) if flag)
            usable = bool(paths) and all(i < len(valid) and valid[i] for i in paths)
            protective = [i for i in sorted(paths) if i < len(safe) and safe[i]]
            warning = [i for i in sorted(paths) if i < len(safe) and not safe[i]]
            usable = usable and bool(protective) and bool(warning)
            if usable:
                output.communication_ok = True
                output.protective_state_known = output.warning_state_known = True
                output.protective_field_clear = all(status[i] for i in protective)
                output.warning_field_clear = all(status[i] for i in warning)
                output.source += f'; protective paths {protective}; warning paths {warning}'
        # Field evaluations do not establish physical OSSD state. Monitoring
        # case numbers are not field-set numbers; both remain explicitly unknown.
        self.samples[name] = (output, received)
        self._publish()

    def _publish(self):
        now = self.get_clock().now()
        for name, publisher in self.scanner_publishers.items():
            sample, received = self.samples.get(name, (None, None))
            fresh = (sample is not None and 0 <= time.monotonic() - received <= self.timeout
                     and 0 <= (now - Time.from_msg(sample.header.stamp)).nanoseconds <= int(self.timeout * 1e9))
            if fresh:
                output = sample
            else:
                output = ScannerSafetyState()
                output.scanner_id = name
                output.header.stamp = sample.header.stamp if sample is not None else now.to_msg()
                output.source = 'scanner Ethernet telemetry stale or missing'
            valid = output.communication_ok and output.protective_state_known and output.warning_state_known
            stamp_ns = Time.from_msg(output.header.stamp).nanoseconds if sample is not None else 0
            signature = (stamp_ns, output.communication_ok, output.protective_state_known,
                         output.warning_state_known, output.ossd_state_known,
                         output.protective_field_clear, output.warning_field_clear,
                         output.ossd_active, output.field_set_known, output.source)
            if signature != self._last_output_signatures.get(name):
                # Publish each new acquisition and every loss/state change once.
                # A timer or the other scanner cannot refresh this source.
                publisher.publish(output)
                self._last_output_signatures[name] = signature
            level = (DiagnosticStatus.ERROR if not valid else
                     DiagnosticStatus.WARN if not output.protective_field_clear or not output.warning_field_clear else
                     DiagnosticStatus.OK)
            message = ('Field monitoring unavailable' if not valid else
                       'Protective stop' if not output.protective_field_clear else
                       'Warning field active' if not output.warning_field_clear else 'Fields clear')
            diagnostic_signature = (level, message, output.source)
            steady_now = time.monotonic()
            previous = self._last_diagnostic_at.get(name)
            if (diagnostic_signature != self._last_diagnostic_signatures.get(name)
                    or previous is None or not 0 <= steady_now - previous < 1.0):
                diagnostic = DiagnosticStatus()
                diagnostic.name = 'scanner/' + name
                diagnostic.hardware_id = name
                diagnostic.level = level
                diagnostic.message = message
                diagnostic.values = [KeyValue(key='source', value=output.source),
                                     KeyValue(key='field_set_known', value='false')]
                array = DiagnosticArray()
                array.header = output.header
                array.status = [diagnostic]
                self.diagnostics[name].publish(array)
                self._last_diagnostic_signatures[name] = diagnostic_signature
                self._last_diagnostic_at[name] = steady_now


def main(args=None):
    rclpy.init(args=args)
    node = ScannerSafetyAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
