"""PAIL/Ecobox operator provider using the documented CANopen TPDO layout."""
import json

import rclpy
from can_msgs.msg import Frame
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mobotic_config.operator_inputs import RemdeviceMapping
from .manual_control import ManualControl
from .remdevice_protocol import RemdeviceReceiver, signed_axis


class RemdeviceControl(ManualControl):
    def __init__(self):
        super().__init__()
        self.declare_parameter('remdevice_mapping', '{}')
        self.mapping = RemdeviceMapping(json.loads(self.get_parameter('remdevice_mapping').value))
        self.decoder = RemdeviceReceiver(self.mapping.node_id, timeout_ns=int(self.joystick_timeout * 1e9))
        self.destroy_subscription(self.joy_subscription)
        # Internal normalized Joy layout, independent of the USB profile.
        self.axis_steer, self.axis_crab, self.axis_speed = 0, 2, 3
        self.axis_speed_sign = self.axis_steer_sign = self.axis_crab_sign = 1
        self.input_mapping_verified = True  # CAN uses its separate verified physical mapping.
        self.buttons_boost = [9]
        self.button_mode_switch = 4
        self.mode_switch_type = 'position'
        self.mode_switch_manual_value, self.mode_switch_auto_velocity_value = 0, 1
        self.button_deadman = 8 if self.mapping.verified and self.mapping.buttons['deadman'] else -1
        self.last_reset = None
        self.reset_future = None
        self.input_error = ''
        self.last_start_ns = None
        self.raw_publisher = self.create_publisher(String, 'operator/raw', 10)
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.can_publisher = self.create_publisher(Frame, 'can_tx', 10)
        self.reset_client = self.create_client(Trigger, 'wheel_modules/clear_errors')
        self.start_service = self.create_service(Trigger, 'operator/start_receiver', self._start_receiver)
        # Best-effort accepts both best-effort and reliable SocketCAN producers.
        self.can_subscription = self.create_subscription(Frame, 'can_rx', self._can_callback, qos_profile_sensor_data)
        if not self.mapping.verified:
            self.get_logger().warning('PAIL mapping unverified: capture only; motion, mode and reset requests inhibited.')

    def _send_start(self):
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id, frame.dlc = 0, 2
        frame.data = [1, self.mapping.node_id, 0, 0, 0, 0, 0, 0]
        self.can_publisher.publish(frame)
        self.last_start_ns = self.get_clock().now().nanoseconds

    def _start_receiver(self, request, response):
        # Explicit commissioning operation; never broadcast NMT or modify SDOs.
        self._send_start()
        response.success = True
        response.message = f'Targeted NMT start sent to receiver {self.mapping.node_id}; verify PDO reception.'
        return response

    def _drop_input(self, now):
        self._reset_command(now)
        self.last_reset = None

    def _healthy(self, now):
        if not hasattr(self, 'decoder') or not self.mapping.verified or self.input_error:
            return False
        sample = self.decoder.snapshot(now.nanoseconds)
        return (sample is not None and sample.digital['LINK'] and sample.digital['EMGY']
                and not sample.digital['STOP'])

    def _joystick_connected(self, now):
        return self._healthy(now) and super()._joystick_connected(now)

    def _manual_enabled(self):
        return self._healthy(self.get_clock().now()) and super()._manual_enabled()

    def _can_callback(self, frame):
        now = self.get_clock().now()
        if frame.is_error:
            self.decoder.invalidate('CAN error frame', now.nanoseconds)
            self._drop_input(now)
            return
        try:
            stamp = Time.from_msg(frame.header.stamp).nanoseconds
        except (TypeError, ValueError, OverflowError):
            if not frame.is_extended and frame.id in {
                    self.decoder.digital_id, self.decoder.analog_id,
                    self.decoder.heartbeat_id, self.decoder.emergency_id}:
                self.decoder.invalidate('invalid CAN timestamp', now.nanoseconds)
                self._drop_input(now)
            return
        before_epoch = self.decoder.epoch_ns
        accepted = self.decoder.update(frame.id, frame.dlc, frame.data, stamp, now.nanoseconds,
                                       is_extended=frame.is_extended, is_rtr=frame.is_rtr, is_error=frame.is_error)
        if self.decoder.epoch_ns != before_epoch:
            self._drop_input(now)
        if (accepted and frame.id == self.decoder.digital_id):
            self.input_error = ''
        if not self._healthy(now):
            self._drop_input(now)
        if not accepted or not self.mapping.verified:
            return
        sample = self.decoder.snapshot(now.nanoseconds, consume=True)
        if sample is None or not self._healthy(now):
            return
        axes = {}
        try:
            for role, spec in self.mapping.axes.items():
                channel = spec['channel']
                axes[role] = signed_axis(sample.analog[channel - 1], sample.digital[f'U{channel}'],
                                         sample.digital[f'D{channel}'], zero=spec['zero'],
                                         full_scale=spec['full_scale'], invert=spec['invert'])
                # Match the neutral-rearm threshold: accepted neutral input
                # must produce exactly zero, including small receiver noise.
                if abs(axes[role]) <= 0.08:
                    axes[role] = 0.0
        except ValueError as error:
            self.input_error = str(error)
            self._drop_input(now)
            return
        joy = Joy()
        joy.header.stamp = Time(nanoseconds=sample.stamp_ns).to_msg()
        joy.header.frame_id = self.frame_id
        joy.axes = [axes['steer'], 0.0, axes['crab'], axes['speed']]
        joy.buttons = [0] * 10
        for role, index in (('mode', 4), ('deadman', 8), ('boost', 9)):
            joy.buttons[index] = int(self.mapping.button(role, sample.digital))
        self.joy_publisher.publish(joy)
        self._joy_callback(joy)
        reset = self.mapping.button('reset', sample.digital)
        if reset and self.last_reset is False and self.controls_neutral:
            if (self.reset_future is None or self.reset_future.done()) and self.reset_client.service_is_ready():
                self.reset_future = self.reset_client.call_async(Trigger.Request())
                self.reset_future.add_done_callback(self._reset_done)
        self.last_reset = reset

    def _reset_done(self, future):
        try:
            response = future.result()
            log = self.get_logger().info if response.success else self.get_logger().warning
            log('B2 supervisor reset: ' + response.message)
        except Exception as error:
            self.get_logger().error('B2 supervisor reset failed: ' + str(error))

    def _operator_identity(self, state):
        state.source_id = 'remdevice'
        digital = self.decoder.digital(self.get_clock().now().nanoseconds)
        known = self.mapping.verified and digital is not None
        # STOP is an asserted stop event; EMGY loss by itself is not evidence
        # that the physical mushroom was pressed. Unknown is never 'released'.
        state.estop_state_known = bool(known and (digital['STOP'] or (digital['LINK'] and digital['EMGY'])))
        state.estop_pressed = bool(known and digital['STOP'])
        if not self.mapping.verified:
            state.status_message = 'PAIL mapping unverified; capture only'
        elif not self._healthy(self.get_clock().now()):
            if digital is not None and digital['STOP']:
                state.status_message = 'PAIL STOP asserted'
            elif digital is not None and not (digital['LINK'] and digital['EMGY']):
                state.status_message = 'receiver link/EMGY unavailable'
            else:
                state.status_message = self.input_error or self.decoder.reason

    def _publish(self):
        now = self.get_clock().now()
        if (self.mapping.verified and not self.decoder.samples
                and not (self.decoder.emergency_code or self.decoder.emergency_register)
                and (self.last_start_ns is None or now.nanoseconds - self.last_start_ns >= 1_000_000_000
                     or now.nanoseconds < self.last_start_ns)):
            self._send_start()
        if not self._healthy(now):
            self._drop_input(now)
        super()._publish()
        raw = String()
        sample = self.decoder.snapshot(now.nanoseconds)
        raw.data = json.dumps({
            'node_id': self.mapping.node_id, 'mapping_verified': self.mapping.verified,
            'stamp_ns': sample.stamp_ns if sample else None,
            'digital': self.decoder.digital(now.nanoseconds),
            'analog': list(sample.analog) if sample else None,
            'status': self.input_error or self.decoder.reason,
        }, sort_keys=True)
        self.raw_publisher.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = RemdeviceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
