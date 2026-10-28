"""PAIL/Ecobox operator provider using the documented CANopen TPDO layout."""
import json
import math

import rclpy
from can_msgs.msg import Frame
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy, JointState
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
        self.declare_parameter('stationary_steering_enabled', False)
        self.declare_parameter('stationary_steering_geometry', '{}')
        self.stationary_steering_enabled = bool(self.get_parameter('stationary_steering_enabled').value)
        self.steering_geometry = json.loads(self.get_parameter('stationary_steering_geometry').value)
        self.steering_publisher = None
        if self.stationary_steering_enabled:
            geometry = self.steering_geometry
            count = len(geometry.get('steering_joint_names', []))
            if (count < 2 or any(len(geometry.get(key, [])) != count for key in
                    ('traction_joint_names', 'wheels_x', 'wheels_y', 'wheel_mount_angles'))
                    or not all(math.isfinite(v) for key in ('wheels_x', 'wheels_y', 'wheel_mount_angles')
                               for v in geometry[key])
                    or not math.isfinite(geometry.get('max_steering_velocity', 0))
                    or geometry.get('max_steering_velocity', 0) <= 0):
                raise ValueError('Complete finite wheel geometry is required for stationary steering')
            self.steering_publisher = self.create_publisher(JointState, 'operator/steering_setpoints', 1)
        self.decoder = RemdeviceReceiver(self.mapping.node_id, timeout_ns=int(self.joystick_timeout * 1e9))
        self.last_complete_sample = None
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
        self.reset_observed_pair = None
        self.receiver_recovery = None
        self.receiver_reset_status = ''
        self.input_error = ''
        self.last_start_ns = None
        self.raw_publisher = self.create_publisher(String, 'operator/raw', 10)
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.selector_publisher = self.create_publisher(Joy, 'operator/scanner_selector', 1)
        self.can_publisher = self.create_publisher(Frame, 'can_tx', 10)
        self.reset_client = self.create_client(Trigger, 'wheel_modules/clear_errors')
        self.start_service = self.create_service(Trigger, 'operator/start_receiver', self._start_receiver)
        # Best-effort accepts both best-effort and reliable SocketCAN producers.
        self.can_subscription = self.create_subscription(Frame, 'can_rx', self._can_callback,
            QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT))
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
        self.last_complete_sample = None

    def _healthy(self, now):
        if (not hasattr(self, 'decoder') or not self.mapping.verified or self.input_error
                or self.decoder.emergency_code or self.decoder.emergency_register):
            return False
        sample = self.decoder.snapshot(now.nanoseconds)
        if sample is None:
            sample = getattr(self, 'last_complete_sample', None)
            if (sample is not None and (sample.stamp_ns < self.decoder.epoch_ns
                    or not 0 <= now.nanoseconds - sample.stamp_ns <= self.decoder.timeout_ns)):
                sample = None
        # A newly arrived digital PDO must stop us immediately, even while an
        # analog PDO is pending. A pending pair cannot renew the last Joy stamp.
        digital = self.decoder.digital(now.nanoseconds)
        return (sample is not None and digital is not None and digital['LINK']
                and digital['EMGY'] and not digital['STOP'])

    def _joystick_connected(self, now):
        return self._healthy(now) and super()._joystick_connected(now)

    def _manual_enabled(self):
        return (self.receiver_recovery is None
                and (self.reset_future is None or self.reset_future.done())
                and self._healthy(self.get_clock().now())
                and super()._manual_enabled())

    def _reset_sample(self, now):
        if not self.mapping.verified:
            return None
        sample = self.decoder.recovery_snapshot(now.nanoseconds)
        if sample is None:
            return None
        digital = sample.digital
        if not (digital['LINK'] and digital['EMGY'] and not digital['STOP']):
            return None
        return sample

    def _reset_sample_neutral(self, sample):
        try:
            return all(abs(signed_axis(sample.analog[spec['channel'] - 1],
                sample.digital[f"U{spec['channel']}"], sample.digital[f"D{spec['channel']}"],
                zero=spec['zero'], full_scale=spec['full_scale'], invert=spec['invert'])) <= 0.08
                for spec in self.mapping.axes.values())
        except ValueError:
            return False

    def _fail_receiver_recovery(self, reason, now):
        self.receiver_recovery = None
        self.receiver_reset_status = 'B2 reset blocked: ' + reason
        self._drop_input(now)
        self.get_logger().warning(self.receiver_reset_status)

    def _observe_reset_button(self, now):
        sample = self._reset_sample(now)
        if sample is None:
            self.last_reset = None
            self.reset_observed_pair = None
            return
        pair = tuple(self.decoder.samples[cob][0] for cob in
                     (self.decoder.digital_id, self.decoder.analog_id))
        if self.reset_observed_pair is not None and any(
                stamp <= previous for stamp, previous in zip(pair, self.reset_observed_pair)):
            return
        self.reset_observed_pair = pair
        pressed = self.mapping.button('reset', sample.digital)
        previous, self.last_reset = self.last_reset, pressed
        if (not pressed or previous is not False or not self._reset_sample_neutral(sample)
                or self.receiver_recovery is not None
                or (self.reset_future is not None and not self.reset_future.done())):
            return
        self._drop_input(now)
        self.receiver_recovery = {'stage': 'wait_operator', 'started_ns': now.nanoseconds,
                                  'deadline_ns': now.nanoseconds + 1_000_000_000}
        if self.decoder.emergency_code or self.decoder.emergency_register:
            self.receiver_recovery.update(stage='verify_receiver',
                emergency_stamp_ns=self.decoder.last_stamps.get(self.decoder.emergency_id))
            request = Frame()
            request.header.stamp = now.to_msg()
            request.id, request.dlc = 0x600 + self.mapping.node_id, 8
            request.data = [0x40, 0x01, 0x10, 0, 0, 0, 0, 0]
            self.can_publisher.publish(request)
            self.receiver_reset_status = 'B2: verifying receiver error register'

    def _receiver_recovery_reply(self, frame, data, stamp_ns, now):
        recovery = self.receiver_recovery
        if (recovery is None or recovery['stage'] != 'verify_receiver'
                or frame.id != 0x580 + self.mapping.node_id or frame.is_extended or frame.is_rtr
                or frame.dlc != 8 or len(data) != 8 or data[1:4] != [0x01, 0x10, 0]
                or any(type(value) is not int or not 0 <= value <= 255 for value in data)
                or not recovery['started_ns'] <= stamp_ns <= now.nanoseconds
                or now.nanoseconds - stamp_ns > int(self.joystick_timeout * 1e9)
                or not recovery['started_ns'] <= now.nanoseconds <= recovery['deadline_ns']):
            return
        sample = self._reset_sample(now)
        if sample is None or not self._reset_sample_neutral(sample):
            self._fail_receiver_recovery('fresh neutral controls and released STOP required', now)
        elif data[0] != 0x4f:
            self._fail_receiver_recovery('receiver error-register read failed or malformed', now)
        elif data[4] != 0:
            self._fail_receiver_recovery(f'receiver error register is still 0x{data[4]:02x}', now)
        elif not self.decoder.acknowledge_recovery(recovery['emergency_stamp_ns'], now.nanoseconds):
            self._fail_receiver_recovery('receiver emergency changed during verification', now)
        else:
            self._drop_input(now)
            self.reset_observed_pair = None
            recovery.update(stage='wait_operator', started_ns=now.nanoseconds,
                            deadline_ns=now.nanoseconds + 1_000_000_000)
            self.receiver_reset_status = 'B2: receiver clear; waiting for fresh neutral input'

    def _advance_receiver_recovery(self, now):
        recovery = self.receiver_recovery
        if recovery is None or recovery['stage'] == 'supervisor':
            return
        sample = self._reset_sample(now)
        if (not recovery['started_ns'] <= now.nanoseconds <= recovery['deadline_ns']
                or (sample is not None and not self._reset_sample_neutral(sample))):
            self._fail_receiver_recovery('verification timed out or controls moved', now)
            return
        digital = self.decoder.digital(now.nanoseconds)
        if digital is not None and not (digital['LINK'] and digital['EMGY'] and not digital['STOP']):
            self._fail_receiver_recovery('receiver link or STOP interlock unavailable', now)
            return
        if recovery['stage'] != 'wait_operator' or sample is None:
            return
        if not self._healthy(now) or not self.controls_neutral:
            return
        if not self.reset_client.service_is_ready():
            return
        # Publish the verified neutral operator state before requesting the
        # existing supervisor-owned standstill/disabled-drive reset transaction.
        super()._publish()
        recovery['stage'] = 'supervisor'
        self.receiver_reset_status = 'B2: supervisor reset requested'
        self.reset_future = self.reset_client.call_async(Trigger.Request())
        self.reset_future.add_done_callback(self._reset_done)

    def _can_callback(self, frame):
        if not frame.is_error and (frame.is_extended or frame.is_rtr or frame.id not in {
                self.decoder.digital_id, self.decoder.analog_id, self.decoder.heartbeat_id,
                self.decoder.emergency_id, 0x580 + self.mapping.node_id}):
            return
        now = self.get_clock().now()
        if frame.is_error:
            self.decoder.invalidate('CAN error frame', now.nanoseconds)
            self._drop_input(now)
            self.last_reset = self.reset_observed_pair = None
            if self.receiver_recovery is not None:
                self._fail_receiver_recovery('CAN error during verification', now)
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
        # ROS fixed-size uint8 arrays use NumPy scalars. Normalize at this
        # boundary; the protocol decoder deliberately validates Python bytes.
        data = frame.data.tolist() if hasattr(frame.data, 'tolist') else frame.data
        self._receiver_recovery_reply(frame, data, stamp, now)
        before_epoch = self.decoder.epoch_ns
        accepted = self.decoder.update(frame.id, frame.dlc, data, stamp, now.nanoseconds,
                                       is_extended=frame.is_extended, is_rtr=frame.is_rtr, is_error=frame.is_error)
        if self.decoder.epoch_ns != before_epoch:
            self._drop_input(now)
            self.last_reset = self.reset_observed_pair = None
            if self.receiver_recovery is not None:
                self._fail_receiver_recovery('receiver input invalidated during verification', now)
        if (accepted and frame.id == self.decoder.digital_id):
            self.input_error = ''
        if not self._healthy(now):
            self._drop_input(now)
        self._observe_reset_button(now)
        if not accepted or not self.mapping.verified:
            return
        sample = self.decoder.snapshot(now.nanoseconds, consume=True)
        if sample is None or not self._healthy(now):
            return
        self.last_complete_sample = sample
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
        override = self.mapping.buttons.get('scanner_override')
        joy.buttons = [0] * (11 if override else 10)
        if override:
            # Commissioned RP2 selector, separate from the receiver's SFTY bit.
            joy.buttons[10] = int(self.mapping.button('scanner_override', sample.digital))
        for role, index in (('mode', 4), ('deadman', 8), ('boost', 9)):
            joy.buttons[index] = int(self.mapping.button(role, sample.digital))
        self.joy_publisher.publish(joy)
        self._joy_callback(joy)
        self._publish_stationary_steering(joy, axes)
        self._advance_receiver_recovery(self.get_clock().now())

    def _publish_stationary_steering(self, joy, axes):
        # A source request, never a direct motor command. The supervisor checks
        # standstill, freshness and authority before applying any angle.
        if (not getattr(self, 'stationary_steering_enabled', False)
                or axes['speed'] != 0.0 or axes['crab'] != 0.0
                or any(abs(v) > 1e-9 for v in self.command)
                or not self._manual_enabled()):
            return
        geometry = self.steering_geometry
        curvature = axes['steer'] * self.max_velocity_angular / self.max_velocity_linear
        output = JointState()
        output.header = joy.header
        output.name = geometry['steering_joint_names'] + geometry['traction_joint_names']
        count = len(geometry['steering_joint_names'])
        output.position = [math.atan2(curvature * x, 1.0 - curvature * y) - mount
                           for x, y, mount in zip(geometry['wheels_x'], geometry['wheels_y'],
                                                  geometry['wheel_mount_angles'])] + [0.0] * count
        output.velocity = [geometry['max_steering_velocity']] * count + [0.0] * count
        output.effort = []
        self.steering_publisher.publish(output)

    def _reset_done(self, future):
        if future is not self.reset_future:
            return
        try:
            response = future.result()
            log = self.get_logger().info if response.success else self.get_logger().warning
            log('B2 supervisor reset: ' + response.message)
        except Exception as error:
            self.get_logger().error('B2 supervisor reset failed: ' + str(error))
        finally:
            self.receiver_recovery = None
            self.reset_future = None
            self.receiver_reset_status = ''
            self._drop_input(self.get_clock().now())

    def _operator_identity(self, state):
        state.source_id = 'remdevice'
        now = self.get_clock().now()
        physical = self.decoder.physical_snapshot(now.nanoseconds)
        digital = physical[1] if physical is not None else None
        known = self.mapping.verified and digital is not None
        # connected describes fresh physical radio/STOP status, not command
        # rearm. Initialization can remain authorized at zero while command_active
        # stays false on an EMCY, incomplete pair or neutral-rearm requirement.
        state.connected = bool(known and digital['LINK'] and digital['EMGY'] and not digital['STOP'])
        if known:
            # Keep physical acquisition time without renewing the Twist stamp or
            # claiming that old analog/motion input has become fresh.
            state.header = type(state.header)(stamp=Time(nanoseconds=physical[0]).to_msg(),
                                               frame_id=self.frame_id)
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
        if self.receiver_reset_status:
            state.status_message = self.receiver_reset_status

    def _publish_selector(self, now):
        if not self.mapping.verified or not self.mapping.buttons.get('scanner_override'):
            return
        physical = self.decoder.physical_snapshot(now.nanoseconds)
        if physical is None:
            return
        selector = Joy()
        selector.header.stamp = Time(nanoseconds=physical[0]).to_msg()
        selector.header.frame_id = self.frame_id
        selector.axes = []
        selector.buttons = [0] * 10 + [int(self.mapping.button('scanner_override', physical[1]))]
        # A physical selector sample only: never pass this to the teleop callback.
        self.selector_publisher.publish(selector)

    def _publish(self):
        now = self.get_clock().now()
        if (self.mapping.verified and not self.decoder.samples
                and not (self.decoder.emergency_code or self.decoder.emergency_register)
                and (self.last_start_ns is None or now.nanoseconds - self.last_start_ns >= 1_000_000_000
                     or now.nanoseconds < self.last_start_ns)):
            self._send_start()
        if not self._healthy(now):
            self._drop_input(now)
        self._observe_reset_button(self.get_clock().now())
        super()._publish()
        self._publish_selector(self.get_clock().now())
        self._advance_receiver_recovery(self.get_clock().now())
        raw = String()
        # The base publisher also checks decoder freshness with later clock
        # reads. Reusing the earlier timestamp would look like a clock rollback.
        now = self.get_clock().now()
        sample = self.decoder.snapshot(now.nanoseconds)
        raw.data = json.dumps({
            'node_id': self.mapping.node_id, 'mapping_verified': self.mapping.verified,
            'stamp_ns': sample.stamp_ns if sample else None,
            'digital': self.decoder.digital(now.nanoseconds),
            'analog': list(sample.analog) if sample else None,
            'status': self.input_error or self.decoder.reason,
            'receiver_error_code': self.decoder.emergency_code,
            'receiver_error_register': self.decoder.emergency_register,
            'reset_status': self.receiver_reset_status,
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
