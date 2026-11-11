"""PAIL/Ecobox operator provider using the documented CANopen TPDO layout."""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from can_msgs.msg import Frame
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mobotic_config.operator_inputs import RemdeviceMapping
from mobotic_interfaces.msg import OperatorState
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
        self.declare_parameter('receiver_can_overrun_advisory', False)
        self.decoder = RemdeviceReceiver(self.mapping.node_id, timeout_ns=int(self.joystick_timeout * 1e9),
            can_overrun_advisory=bool(self.get_parameter('receiver_can_overrun_advisory').value))
        # Homing observes live physical inputs independently of diagnostic EMCY
        # invalidation. This channel never authorizes joystick motion or B2.
        self.homing_decoder = RemdeviceReceiver(self.mapping.node_id,
            timeout_ns=int(self.joystick_timeout * 1e9))
        self.homing_state_publisher = self.create_publisher(OperatorState, 'operator/homing_state', 1)
        self.last_complete_sample = None
        self.destroy_subscription(self.joy_subscription)
        # Internal normalized Joy layout, independent of the USB profile.
        self.axis_steer, self.axis_crab, self.axis_speed = 0, 2, 3
        self.axis_speed_sign = self.axis_steer_sign = self.axis_crab_sign = 1
        self.input_mapping_verified = True  # CAN uses its separate verified physical mapping.
        self.buttons_boost = [9]
        self.button_mode_switch = 4
        self.mode_switch_type = 'position'
        self.mode_switch_manual_value, self.mode_switch_autonomous_value = 0, 1
        self.last_reset = None
        self.reset_future = None
        self.reset_observed_pair = None
        self.drive_toggle_status = ''
        self.input_error = ''
        self.last_start_ns = None
        self.declare_parameter('receiver_auto_recovery', True)
        self.receiver_auto_recovery = bool(self.get_parameter('receiver_auto_recovery').value)
        self.declare_parameter('receiver_auto_communication_reset', False)
        self.receiver_auto_communication_reset = bool(
            self.get_parameter('receiver_auto_communication_reset').value)
        self.receiver_poll_ns = None
        self.receiver_read = None
        self.receiver_register_ns = None
        self.receiver_register_value = None
        self.receiver_reset_ns = None
        self.receiver_last_reset_ns = None
        self.receiver_reset_attempts = 0
        self.last_receiver_connection = None
        self.raw_publisher = self.create_publisher(String, 'operator/raw', 10)
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.selector_publisher = self.create_publisher(Joy, 'operator/scanner_selector', 1)
        self.can_publisher = self.create_publisher(Frame, 'can_tx', 10)
        self.reset_client = self.create_client(Trigger, 'wheel_modules/enable_drives')
        self.start_service = self.create_service(Trigger, 'operator/start_receiver', self._start_receiver)
        # Hardware PDOs and receiver diagnostics have separate filtered CAN
        # readers/topics. An EMCY burst must not evict either control PDO.
        # Histories are bounded; acquisition-age validation still rejects old
        # samples, regardless of how long a DDS reader retained them.
        self.can_subscription = self.create_subscription(Frame, 'can_rx', self._can_callback,
            QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.can_event_subscription = self.create_subscription(Frame, 'can_events', self._can_callback,
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

    def _receiver_reset_allowed(self, now):
        # Communication recovery must never reset the receiver during motion
        # or a drive-enable operation. B2 remains a separate supervisor request.
        sample = self._reset_sample(now)
        return (self._vehicle_mode_fresh(now)
                and not self.vehicle_mode_state.drives_enabled
                and not self.vehicle_mode_state.reset_in_progress
                and (self.reset_future is None or self.reset_future.done())
                and sample is not None and self._reset_sample_neutral(sample))

    def _receiver_recovery_tick(self, now):
        if not self.receiver_auto_recovery or not self.mapping.verified:
            return
        if self.receiver_reset_ns is not None:
            # Allow reset-communication to finish before starting ONLY this
            # receiver. No broadcast, parameter writes, or automatic drive enable.
            age = now.nanoseconds - self.receiver_reset_ns
            if age < 0:
                self.receiver_reset_ns = None
                self.receiver_read = None
                self.decoder.invalidate('clock changed during receiver recovery', now.nanoseconds)
                return
            if (age >= 500_000_000 and self._vehicle_mode_fresh(now)
                    and not self.vehicle_mode_state.drives_enabled
                    and not self.vehicle_mode_state.reset_in_progress):
                self._send_start()
                self.receiver_reset_ns = None
                self.decoder.invalidate('receiver restarted; fresh neutral input required', now.nanoseconds)
                self._drop_input(now)
                self.last_reset = self.reset_observed_pair = None
            return
        if (self.receiver_poll_ns is not None
                and 0 <= now.nanoseconds - self.receiver_poll_ns < 1_000_000_000):
            return
        frame = Frame()
        frame.header.stamp = now.to_msg()
        frame.id, frame.dlc = 0x600 + self.mapping.node_id, 8
        # Read current health at startup and at most once per second. PDOs
        # alone cannot establish that an earlier EMCY has cleared, and an
        # EMCY sent before this subscriber started may have been missed.
        # Upload the one-byte CANopen error register, object 0x1001:0.
        frame.data = [0x40, 0x01, 0x10, 0, 0, 0, 0, 0]
        self.can_publisher.publish(frame)
        self.receiver_poll_ns = now.nanoseconds
        self.receiver_read = (now.nanoseconds,
            self.decoder.last_stamps.get(self.decoder.emergency_id))

    def _receiver_error_response(self, frame, now):
        pending = self.receiver_read
        if (not self.receiver_auto_recovery or not self.mapping.verified or pending is None
                or frame.is_error or frame.is_extended or frame.is_rtr or frame.dlc != 8):
            return
        try:
            stamp = Time.from_msg(frame.header.stamp).nanoseconds
            # ROS fixed-size uint8 arrays may contain NumPy scalar values.
            # Stored diagnostics must use ordinary integers for JSON publishing.
            data = [int(value) for value in frame.data]
            if (len(data) < 8 or any(not 0 <= int(value) <= 255 for value in data)
                    or not pending[0] <= stamp <= now.nanoseconds
                    or not 0 <= now.nanoseconds - pending[0] <= self.decoder.timeout_ns
                    or pending[1] != self.decoder.last_stamps.get(self.decoder.emergency_id)
                    or data[1:4] != [0x01, 0x10, 0]):
                return
        except (TypeError, ValueError, OverflowError):
            return
        if data[0] not in (0x4f, 0x80):
            return
        self.receiver_read = None
        if data[0] == 0x80:
            self.get_logger().warning('Receiver error-register read aborted; fault remains active')
            return
        first_read = self.receiver_register_ns is None
        had_diagnostic = bool(self.decoder.emergency_code or self.decoder.emergency_register)
        self.receiver_register_ns, self.receiver_register_value = stamp, data[4]
        was_blocking = self.decoder.has_blocking_error()
        if data[4] != 0:
            self.decoder.emergency_register = data[4]
            if self.decoder.has_blocking_error() and not was_blocking:
                self.decoder.invalidate('receiver current error register requires inhibition',
                                        now.nanoseconds, preserve_physical=True)
                self._drop_input(now)
                self.last_reset = self.reset_observed_pair = None
        if data[4] == 0:
            # A matching, fresh device read confirms a missed EMCY clear.
            # Discard all pre-clear inputs and button edges before rearming.
            self.decoder.emergency_code = self.decoder.emergency_register = 0
            if was_blocking:
                self.decoder.invalidate('receiver error register cleared; fresh neutral input required',
                                        now.nanoseconds, preserve_physical=True)
                self._drop_input(now)
                self.last_reset = self.reset_observed_pair = None
            if had_diagnostic:
                self.get_logger().info('Receiver reports zero error register; cleared local cached diagnostic (no receiver write)')
            elif first_read:
                self.get_logger().info('Receiver current error register verified zero (read-only startup baseline)')
        elif (self.receiver_auto_communication_reset
                and self.decoder.emergency_code == 0x8150 and data[4] == 0x11
                and self.receiver_reset_attempts < 3 and self._receiver_reset_allowed(now)
                and (self.receiver_reset_ns is None)
                and (self.receiver_last_reset_ns is None
                     or now.nanoseconds - self.receiver_last_reset_ns >= 5_000_000_000)):
            reset = Frame()
            reset.header.stamp = now.to_msg()
            reset.id, reset.dlc = 0, 2
            reset.data = [0x82, self.mapping.node_id, 0, 0, 0, 0, 0, 0]
            self.can_publisher.publish(reset)
            self.receiver_reset_ns = self.receiver_last_reset_ns = now.nanoseconds
            self.receiver_reset_attempts += 1
            self.decoder.invalidate('receiver communication recovery in progress', now.nanoseconds)
            self._drop_input(now)
            self.last_reset = self.reset_observed_pair = None
            self.get_logger().warning(
                f'Automatic receiver communication reset {self.receiver_reset_attempts}/3 '
                f'for EMCY 0x8150; node {self.mapping.node_id}, drives disabled')

    def _receiver_fault_message(self):
        if not self.decoder.emergency_code:
            return (f'joystick receiver current error register=0x{self.decoder.emergency_register:02X}; '
                    'EMCY detail not received; waiting for receiver recovery; '
                    'then keep controls neutral and release/press B2')
        description = {
            0x6000: 'EEPROM not programmed or EEPROM error',
            0x8110: 'CAN receive overrun',
            0x8120: 'CAN error-passive state',
            0x8130: 'heartbeat or life-guard error',
            0x8140: 'recovered from CAN bus-off',
            0x8150: 'CAN-ID collision',
        }.get(self.decoder.emergency_code, 'receiver fault')
        action = {
            0x8110: 'check CAN traffic load, bitrate, wiring and termination; ',
            0x8150: 'check duplicate CAN node/COB-IDs and CAN traffic load; ',
        }.get(self.decoder.emergency_code, '')
        return (f'joystick receiver fault EMCY=0x{self.decoder.emergency_code:04X} '
                f'({description}), error register=0x{self.decoder.emergency_register:02X}; '
                'waiting for receiver recovery; ' + action
                + 'restart joystick/receiver if the fault persists, '
                'then keep controls neutral and release/press B2')

    def _drop_input(self, now):
        self._reset_command(now)
        self.last_complete_sample = None

    def _healthy(self, now):
        if (not hasattr(self, 'decoder') or not self.mapping.verified or self.input_error
                or self.receiver_reset_ns is not None
                or self.decoder.has_blocking_error()):
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
        return ((self.reset_future is None or self.reset_future.done())
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

    def _observe_reset_button(self, now):
        # B2 requests initialization/enable, like X on the example Logitech.
        # Never toggle/disable drives or cancel an active enable request here.
        # Never read/clear receiver error registers or restart receiver
        # communication here. The independent monitor is read-only by default.
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
        if (not pressed or previous is not False
                or (self.reset_future is not None and not self.reset_future.done())):
            return
        if not self._vehicle_mode_fresh(now):
            self.get_logger().warning('B2 drive enable blocked: vehicle mode unavailable')
            return
        active = (self.vehicle_mode_state.drives_enabled
                  or self.vehicle_mode_state.reset_in_progress)
        if active:
            self.get_logger().info('B2: drives already enabled or initialization/enable in progress; request unchanged')
            return
        if not active:
            if not self._healthy(now):
                detail = (self._receiver_fault_message()
                          if self.decoder.emergency_code or self.decoder.emergency_register else
                          (self.input_error or self.decoder.reason)
                          + '; restore fresh joystick input, keep controls neutral and release/press B2 again')
                self.get_logger().warning(
                    'B2 enable blocked: receiver fault or input unavailable; ' + detail)
                return
            if not self._reset_sample_neutral(sample) or not self.controls_neutral:
                self.get_logger().warning('B2 enable blocked: neutral controls required')
                return
            # Report the fresh neutral state before requesting zero-target
            # initialization/enable. The supervisor validates every interlock.
            super()._publish()
        if not self.reset_client.service_is_ready():
            self.get_logger().warning('B2 drive enable blocked: supervisor service unavailable')
            return
        self._drop_input(self.get_clock().now())
        try:
            self.drive_toggle_status = 'B2 drive initialization/enable requested; completion not yet verified'
            self.reset_future = self.reset_client.call_async(Trigger.Request())
            self.reset_future.add_done_callback(self._reset_done)
        except Exception as error:
            self.reset_future = None
            self.drive_toggle_status = ''
            self.get_logger().error('B2 drive enable request failed: ' + str(error))

    def _can_callback(self, frame):
        if not frame.is_error and frame.id == 0x580 + self.mapping.node_id:
            self._receiver_error_response(frame, self.get_clock().now())
            return
        if not frame.is_error and (frame.is_extended or frame.is_rtr or frame.id not in {
                self.decoder.digital_id, self.decoder.analog_id, self.decoder.heartbeat_id,
                self.decoder.emergency_id}):
            return
        now = self.get_clock().now()
        if frame.is_error:
            if hasattr(self, 'homing_decoder'):
                self.homing_decoder.invalidate('CAN error frame', now.nanoseconds)
            self.decoder.invalidate('CAN error frame', now.nanoseconds)
            self._drop_input(now)
            self.last_reset = self.reset_observed_pair = None
            return
        try:
            stamp = Time.from_msg(frame.header.stamp).nanoseconds
        except (TypeError, ValueError, OverflowError):
            if not frame.is_extended and frame.id in {
                    self.decoder.digital_id, self.decoder.analog_id,
                    self.decoder.heartbeat_id, self.decoder.emergency_id}:
                if hasattr(self, 'homing_decoder') and frame.id != self.decoder.emergency_id:
                    self.homing_decoder.invalidate('invalid CAN timestamp', now.nanoseconds)
                self.decoder.invalidate('invalid CAN timestamp', now.nanoseconds)
                self._drop_input(now)
            return
        if hasattr(self, 'homing_decoder') and frame.id != self.decoder.emergency_id:
            self.homing_decoder.update(int(frame.id), int(frame.dlc), [int(v) for v in frame.data],
                stamp, now.nanoseconds, is_extended=frame.is_extended, is_rtr=frame.is_rtr)
        # ROS fixed-size uint8 arrays use NumPy scalars. Normalize at this
        # boundary; the protocol decoder deliberately validates Python bytes.
        data = frame.data.tolist() if hasattr(frame.data, 'tolist') else frame.data
        before_epoch = self.decoder.epoch_ns
        accepted = self.decoder.update(frame.id, frame.dlc, data, stamp, now.nanoseconds,
                                       is_extended=frame.is_extended, is_rtr=frame.is_rtr, is_error=frame.is_error)
        if self.decoder.epoch_ns != before_epoch:
            self._drop_input(now)
            self.last_reset = self.reset_observed_pair = None
        if (accepted and frame.id == self.decoder.digital_id):
            self.input_error = ''
        if not self._healthy(now):
            self._drop_input(now)
        if not accepted or not self.mapping.verified:
            self._observe_reset_button(self.get_clock().now())
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
        for role, index in (('mode', 4), ('boost', 9)):
            joy.buttons[index] = int(self.mapping.button(role, sample.digital))
        self.joy_publisher.publish(joy)
        self._joy_callback(joy)
        self._publish_stationary_steering(joy, axes)
        self._observe_reset_button(self.get_clock().now())

    def _publish_stationary_steering(self, joy, axes):
        # A source request, never a direct motor command. The supervisor checks
        # standstill, freshness and authority before applying any angle.
        if (getattr(self, 'steering_policy', None) is not None
                or not getattr(self, 'stationary_steering_enabled', False)
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
            # rclpy caches severity per call site. Separate sites are needed
            # when an accepted request is followed by a rejected request.
            if response.success:
                self.get_logger().info('B2 drive enable: ' + response.message)
            else:
                self.get_logger().warning('B2 drive enable: ' + response.message)
        except Exception as error:
            self.get_logger().error('B2 drive enable failed: ' + str(error))
        finally:
            self.reset_future = None
            self.drive_toggle_status = ''
            self._drop_input(self.get_clock().now())

    def _publish_homing_state(self, now):
        if not hasattr(self, 'homing_state_publisher'):
            return
        physical = self.homing_decoder.physical_snapshot(now.nanoseconds)
        pair = self.homing_decoder.recovery_snapshot(now.nanoseconds)
        digital = physical[1] if physical is not None else None
        state = OperatorState()
        state.source_id = 'remdevice_homing'
        # Never restamp cached data: a missing PDO/STOP observation expires.
        if physical is not None:
            state.header.stamp = Time(nanoseconds=physical[0]).to_msg()
        known = self.mapping.verified and digital is not None
        state.estop_pressed = bool(known and digital['STOP'])
        state.estop_state_known = bool(known and (digital['STOP'] or (digital['LINK'] and digital['EMGY'])))
        diagnostic_ok = (not self.decoder.has_blocking_error() or
                         (self.decoder.emergency_code in (0x8110, 0x8150)
                          and self.decoder.emergency_register == 0x11))
        state.connected = bool(known and digital['LINK'] and digital['EMGY'] and not digital['STOP']
                               and diagnostic_ok and self.receiver_reset_ns is None)
        state.controls_neutral = bool(state.connected and pair is not None and self._reset_sample_neutral(pair))
        # This topic is interlock evidence for an already accepted homing reset;
        # command_active stays false and normal operator state is unchanged.
        state.status_message = ('fresh physical homing interlocks' if state.connected and pair is not None
                                else 'homing interlock unavailable or STOP/link/fault active')
        self.homing_state_publisher.publish(state)

    def _operator_identity(self, state):
        state.source_id = 'remdevice'
        now = self.get_clock().now()
        self._publish_homing_state(now)
        physical = self.decoder.physical_snapshot(now.nanoseconds)
        digital = physical[1] if physical is not None else None
        known = self.mapping.verified and digital is not None
        # A receiver fault withdraws drive readiness so recovery requires B2.
        # Preserve independently known physical STOP evidence during that wait.
        # An incomplete motion pair or neutral rearm does not fabricate radio loss.
        state.connected = bool(known and digital['LINK'] and digital['EMGY']
                               and not digital['STOP'] and self.receiver_reset_ns is None
                               and not (self.decoder.has_blocking_error() or self.input_error))
        advisory = self.decoder.overrun_is_advisory()
        if advisory and not self._healthy(now):
            state.connected = False
        # Clearing cached motion for B2/authority changes does not change the
        # physical sticks. Keep their fresh neutral proof across that boundary
        # so an accepted enable is not cancelled before the next PDO pair.
        neutral_sample = self._reset_sample(now) if state.connected else None
        state.controls_neutral = bool(neutral_sample is not None
                                      and self._reset_sample_neutral(neutral_sample))
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
            elif self.receiver_reset_ns is not None:
                state.status_message = 'joystick receiver communication recovery in progress'
            elif self.decoder.has_blocking_error():
                state.status_message = self._receiver_fault_message()
            else:
                state.status_message = self.input_error or self.decoder.reason
        if advisory and state.connected:
            state.status_message = ('Joystick CAN receive-overrun warning: EMCY=0x8110, register=0x11; '
                                    'fresh validated controls available')
        if self.drive_toggle_status and state.connected and not advisory:
            state.status_message = self.drive_toggle_status
        # Report the first lost-input cause even without another B2 press.
        # A pending enable response must never hide a live receiver fault.
        key = (state.connected, state.status_message if not state.connected or advisory else '')
        if key != getattr(self, 'last_receiver_connection', None):
            if state.connected and advisory:
                self.get_logger().warning(state.status_message)
            elif state.connected:
                self.get_logger().info('Joystick receiver input available; drive enabling still requires B2 and ready interlocks')
            else:
                self.get_logger().warning('Joystick input unavailable: ' + state.status_message)
            self.last_receiver_connection = key

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
        self._receiver_recovery_tick(now)
        now = self.get_clock().now()
        if (self.mapping.verified and self.decoder.nmt_state in (None, 0, 4, 127)
                and self.receiver_reset_ns is None
                and not (self.decoder.emergency_code or self.decoder.emergency_register)
                and (self.last_start_ns is None or now.nanoseconds - self.last_start_ns >= 1_000_000_000
                     or now.nanoseconds < self.last_start_ns)):
            self._send_start()
        if not self._healthy(now):
            self._drop_input(now)
        super()._publish()
        self._publish_selector(self.get_clock().now())
        self._observe_reset_button(self.get_clock().now())
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
            'receiver_error_register_read_stamp_ns': self.receiver_register_ns,
            'receiver_error_register_read_value': self.receiver_register_value,
            'receiver_nmt_state': self.decoder.nmt_state,
            'receiver_nmt_stamp_ns': self.decoder.nmt_stamp_ns,
            'receiver_error_blocking': self.decoder.has_blocking_error(),
            'receiver_can_overrun_advisory': self.decoder.overrun_is_advisory(),
            'reset_status': self.drive_toggle_status,
            'receiver_recovery_pending': self.receiver_reset_ns is not None,
            'receiver_reset_attempts': self.receiver_reset_attempts,
        }, sort_keys=True)
        self.raw_publisher.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = RemdeviceControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
