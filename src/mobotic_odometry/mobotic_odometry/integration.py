"""ROS-independent SE(2) integration and covariance propagation.

Input is measured body velocity, never commanded velocity. Pose across a feedback
gap cannot be reconstructed: keep the estimate, increase uncertainty and rebase.
"""
from dataclasses import dataclass
import math


def diagonal(values):
    return [[values[i] if i == j else 0.0 for j in range(3)] for i in range(3)]


def propagate(jacobian, covariance):
    return [[sum(jacobian[i][k] * covariance[k][l] * jacobian[j][l]
                 for k in range(3) for l in range(3)) for j in range(3)] for i in range(3)]


def covariance_6d(planar, unobserved_variance):
    result = [0.0] * 36
    axes = (0, 1, 5)  # x, y, yaw in ROS covariance order.
    for i, row in enumerate(axes):
        for j, column in enumerate(axes):
            result[row * 6 + column] = planar[i][j]
    for axis in (2, 3, 4):
        result[axis * 6 + axis] = unobserved_variance
    return result


def body_step(vx, vy, wz, dt):
    """Exact constant-body-twist displacement and its velocity Jacobian."""
    angle = wz * dt
    if abs(angle) < 1e-4:
        sinc = 1.0 - angle**2 / 6.0 + angle**4 / 120.0
        cosc = angle / 2.0 - angle**3 / 24.0 + angle**5 / 720.0
        sinc_prime = -angle / 3.0 + angle**3 / 30.0 - angle**5 / 840.0
        cosc_prime = 0.5 - angle**2 / 8.0 + angle**4 / 144.0
    else:
        sinc = math.sin(angle) / angle
        cosc = (1.0 - math.cos(angle)) / angle
        sinc_prime = (angle * math.cos(angle) - math.sin(angle)) / angle**2
        cosc_prime = (angle * math.sin(angle) - (1.0 - math.cos(angle))) / angle**2
    a, b = dt * sinc, dt * cosc
    da, db = dt * dt * sinc_prime, dt * dt * cosc_prime
    dx, dy = a * vx - b * vy, b * vx + a * vy
    jacobian = [[a, -b, da * vx - db * vy],
                [b, a, db * vx + da * vy], [0.0, 0.0, dt]]
    return dx, dy, angle, jacobian


@dataclass(frozen=True)
class Estimate:
    stamp_ns: int
    pose: tuple
    twist: tuple
    pose_covariance: tuple
    twist_covariance: tuple


class PlanarOdometry:
    def __init__(self, feedback_timeout=0.3, max_integration_interval=0.3,
                 initial_pose=(0.0, 0.0, 0.0), initial_pose_variances=(0.01, 0.01, 0.01),
                 twist_variances=(0.01, 0.01, 0.01), process_variance_rates=(0.001, 0.001, 0.001),
                 gap_variance_rates=(1.0, 1.0, 1.0), unobserved_variance=1e6,
                 max_linear_speed=5.0, max_angular_speed=5.0):
        for value in (feedback_timeout, max_integration_interval, unobserved_variance,
                      max_linear_speed, max_angular_speed):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('timeouts, unobserved variance and plausibility limits must be positive/finite')
        for values in (initial_pose, initial_pose_variances, twist_variances,
                       process_variance_rates, gap_variance_rates):
            if len(values) != 3 or not all(math.isfinite(value) for value in values):
                raise ValueError('planar pose/variance arrays must have three finite elements')
        for values in (initial_pose_variances, twist_variances, process_variance_rates, gap_variance_rates):
            if not all(value > 0.0 for value in values):
                raise ValueError('variance arrays must be strictly positive')
        if max_integration_interval > feedback_timeout:
            raise ValueError('max integration interval must not exceed feedback timeout')
        self.timeout = feedback_timeout
        self.max_interval = max_integration_interval
        self.initial_pose = tuple(initial_pose)
        self.initial_variances = tuple(initial_pose_variances)
        self.twist_variances = tuple(twist_variances)
        self.process_rates = tuple(process_variance_rates)
        self.gap_rates = tuple(gap_variance_rates)
        self.unobserved_variance = unobserved_variance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.last_stamp_ns = 0
        self.last_received = None
        self.last_now_ns = None
        self.last_twist = None
        self.reset()

    def reset(self):
        # Retain acquisition/clock watermarks: reset must not admit replayed data.
        self.pose = list(self.initial_pose)
        self.pose[2] = math.remainder(self.pose[2], 2.0 * math.pi)
        self.covariance = diagonal(self.initial_variances)
        self.chain_ok = False
        self.valid = False
        self.reset_pending = True
        self.pose_degraded = False
        self.reason = 'Waiting for measured body velocity'

    def invalidate(self, reason):
        self.valid = False
        self.chain_ok = False
        self.reason = reason

    def fresh(self, now_ns, received_now):
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.invalidate('ROS clock moved backwards; wait for catch-up or restart')
            return False
        self.last_now_ns = now_ns
        if not self.valid or self.last_received is None:
            return False
        source_age = (now_ns - self.last_stamp_ns) / 1e9
        receipt_age = received_now - self.last_received
        if not (-0.1 <= source_age <= self.timeout and 0.0 <= receipt_age <= self.timeout):
            self.invalidate('Measured velocity stale: odometry/TF publication stopped')
            return False
        return True

    def observe(self, stamp_ns, now_ns, received_at, twist):
        # Check even when no timer callback occurred between samples.
        self.fresh(now_ns, received_at)
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            return None
        if stamp_ns > 0 and stamp_ns <= self.last_stamp_ns:
            return None  # Duplicate/reordered data cannot renew or move the estimate.
        age = (now_ns - stamp_ns) / 1e9
        if stamp_ns <= 0 or not -0.1 <= age <= self.timeout or not math.isfinite(received_at):
            self.invalidate('Invalid, stale or future acquisition timestamp')
            return None
        if (len(twist) != 3 or not all(math.isfinite(value) for value in twist)
                or math.hypot(twist[0], twist[1]) > self.max_linear_speed
                or abs(twist[2]) > self.max_angular_speed):
            self.invalidate('Non-finite or implausible measured velocity')
            return None
        if self.last_received is not None and received_at < self.last_received:
            self.invalidate('Steady receipt clock moved backwards')
            return None

        covariance = [row[:] for row in self.covariance]
        pose = self.pose[:]
        if self.last_stamp_ns and not self.reset_pending:
            dt = (stamp_ns - self.last_stamp_ns) / 1e9
            receipt_dt = received_at - self.last_received
            if self.chain_ok and dt <= self.max_interval and receipt_dt <= self.timeout:
                average = [(old + new) * 0.5 for old, new in zip(self.last_twist, twist)]
                dx, dy, dyaw, body_jacobian = body_step(*average, dt)
                c, s = math.cos(pose[2]), math.sin(pose[2])
                world_dx, world_dy = c * dx - s * dy, s * dx + c * dy
                f = [[1.0, 0.0, -world_dy], [0.0, 1.0, world_dx], [0.0, 0.0, 1.0]]
                rotation = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
                g = [[sum(rotation[i][k] * body_jacobian[k][j] for k in range(3))
                      for j in range(3)] for i in range(3)]
                predicted = propagate(f, covariance)
                measurement_noise = propagate(g, diagonal(self.twist_variances))
                covariance = [[predicted[i][j] + measurement_noise[i][j]
                               + (self.process_rates[i] * dt if i == j else 0.0)
                               for j in range(3)] for i in range(3)]
                pose = [pose[0] + world_dx, pose[1] + world_dy,
                        math.remainder(pose[2] + dyaw, 2.0 * math.pi)]
            else:
                # No assumed motion through unobserved intervals, in either time domain.
                gap = max(dt, receipt_dt)
                for i in range(3):
                    covariance[i][i] += self.gap_rates[i] * gap
                self.pose_degraded = True
        if not all(math.isfinite(value) for value in pose + [v for row in covariance for v in row]):
            self.invalidate('Non-finite integrated pose/covariance')
            return None
        self.pose, self.covariance = pose, covariance
        self.last_stamp_ns, self.last_received = stamp_ns, received_at
        self.last_twist = tuple(twist)
        self.chain_ok = self.valid = True
        self.reset_pending = False
        self.reason = ('Unobserved feedback gap: pose held/rebased, uncertainty increased'
                       if self.pose_degraded else 'Measured wheel odometry available')
        return Estimate(stamp_ns, tuple(pose), tuple(twist),
                        tuple(covariance_6d(covariance, self.unobserved_variance)),
                        tuple(covariance_6d(diagonal(self.twist_variances), self.unobserved_variance)))
