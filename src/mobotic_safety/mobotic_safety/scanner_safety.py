"""Non-safety-rated field telemetry from scanner evaluated output paths."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sick_safetyscanners2_interfaces.msg import RawMicroScanData
from mobotic_interfaces.msg import SafetyIOState, ScannerSafetyState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ScannerSafetyAdapter(Node):
    def __init__(self):
        super().__init__('scanner_safety_adapter')
        self.declare_parameter('status_timeout', 0.3)
        self.declare_parameter('scanner_source', 'scanner')
        self.timeout = float(self.get_parameter('status_timeout').value)
        self.source = str(self.get_parameter('scanner_source').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('status timeout must be positive and finite')
        if self.source not in ('scanner', 'safety_io'):
            raise ValueError('scanner_source must be scanner or safety_io')
        self.state = None
        self.samples = {}
        self.sequences = {}
        self.paths = {}
        self.input_subscriptions = []
        if self.source == 'safety_io':
            self.input_subscriptions.append(self.create_subscription(
                SafetyIOState, 'safety/io_state', self._receive, 1))
        else:
            for name in ('front_left', 'rear_right'):
                self.input_subscriptions.append(self.create_subscription(
                    RawMicroScanData, f'scanner/{name}/raw_data',
                    lambda msg, location=name: self._receive_scanner(location, msg),
                    qos_profile_sensor_data))
        self.scanner_publishers = {name: self.create_publisher(ScannerSafetyState, f'scanner/{name}/safety_zone_status', 1)
                           for name in ('front_left', 'rear_right')}
        self.diagnostics = {name: self.create_publisher(DiagnosticArray, f'scanner/{name}/diagnostics', 1)
                            for name in self.scanner_publishers}
        self.timer = self.create_timer(0.05, self._publish)

    def _receive(self, message):
        if message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1000000000:
            return
        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds <= 0 or not 0 <= (self.get_clock().now() - stamp).nanoseconds <= int(self.timeout * 1e9):
            return
        if self.state is None or Time.from_msg(message.header.stamp).nanoseconds > Time.from_msg(self.state.header.stamp).nanoseconds:
            self.state = message
            self._publish()

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
            # Learn valid paths; once observed, a path losing validity must
            # never disappear into an apparent clear state.
            paths = self.paths.setdefault(name, set())
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
            output = ScannerSafetyState()
            output.scanner_id = name
            if self.source == 'scanner':
                sample, received = self.samples.get(name, (None, None))
                fresh = (sample is not None and 0 <= time.monotonic() - received <= self.timeout
                         and 0 <= (now - Time.from_msg(sample.header.stamp)).nanoseconds <= int(self.timeout * 1e9))
                if fresh:
                    output = copy.deepcopy(sample)
                else:
                    output = ScannerSafetyState()
                    output.scanner_id = name
                    output.header.stamp = sample.header.stamp if sample is not None else now.to_msg()
                    output.source = 'scanner Ethernet telemetry stale or missing'
            else:
                valid = (self.state is not None and self.state.scanner_state_known and
                         0 <= (now - Time.from_msg(self.state.header.stamp)).nanoseconds <= int(self.timeout * 1e9))
                output.header = self.state.header if self.state is not None else output.header
                output.communication_ok = valid
                output.protective_state_known = output.warning_state_known = output.ossd_state_known = valid
                if valid:
                    output.protective_field_clear = getattr(self.state, name + '_protective_field_clear')
                    output.warning_field_clear = getattr(self.state, name + '_warning_field_clear')
                    output.ossd_active = getattr(self.state, name + '_ossd_active')
                output.source = 'virtual safety I/O; field-set number unavailable'
            valid = output.communication_ok and output.protective_state_known and output.warning_state_known
            publisher.publish(output)
            diagnostic = DiagnosticStatus()
            diagnostic.name = 'scanner/' + name
            diagnostic.hardware_id = name
            diagnostic.level = (DiagnosticStatus.ERROR if not valid else
                                DiagnosticStatus.WARN if not output.protective_field_clear or not output.warning_field_clear else
                                DiagnosticStatus.OK)
            diagnostic.message = ('Field monitoring unavailable' if not valid else
                                  'Protective stop' if not output.protective_field_clear else
                                  'Warning field active' if not output.warning_field_clear else 'Fields clear')
            diagnostic.values = [KeyValue(key='source', value=output.source),
                                 KeyValue(key='field_set_known', value='false')]
            array = DiagnosticArray()
            array.header = output.header
            array.status = [diagnostic]
            self.diagnostics[name].publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ScannerSafetyAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
