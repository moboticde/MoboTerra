"""Exclusive manual steering and final wheel alignment; radians and contact m/s."""
import copy
import math


class ManualSteering:
    def __init__(self, geometry):
        self.geometry = geometry
        self.max_yaw = math.inf
        self.x = geometry['wheels_x']; self.y = geometry['wheels_y']
        if (not self.x or len(self.x) != len(self.y)
                or not all(math.isfinite(v) for v in self.x + self.y)):
            raise ValueError('finite manual steering geometry required')
        # The innermost wheel reaches 45 degrees at full Ackermann stick.
        self.curvature_limit = 1.0 / max(abs(x) + abs(y) for x, y in zip(self.x, self.y))
        self.angle_limits_enabled = geometry.get('steering_angle_limits_enabled', True)
        if type(self.angle_limits_enabled) is not bool:
            raise ValueError('steering_angle_limits_enabled must be boolean')
        self.return_home_on_neutral = geometry.get('steering_return_home_on_neutral', False)
        if type(self.return_home_on_neutral) is not bool:
            raise ValueError('steering_return_home_on_neutral must be boolean')
        self.neutral_home_velocity = float(geometry.get('steering_neutral_home_velocity',
                                                       geometry.get('max_steering_velocity', .3)))
        if not math.isfinite(self.neutral_home_velocity) or self.neutral_home_velocity <= 0.:
            raise ValueError('steering_neutral_home_velocity must be finite and positive')
        self.crab_limit = float(geometry.get('max_crab_angle', math.pi / 2))
        if not math.isfinite(self.crab_limit) or not math.pi / 2 <= self.crab_limit <= math.pi:
            raise ValueError('max_crab_angle must be between pi/2 and pi radians')
        # Cap the correction deadband at one degree. Native position scale is
        # separate: ideal headings must not be rounded and accumulated.
        self.position_tolerances = geometry.get(
            'steering_position_tolerances', [1e-3] * len(self.x))
        if (len(self.position_tolerances) != len(self.x)
                or not all(math.isfinite(v) and 0.0 < v < math.pi/4
                           for v in self.position_tolerances)):
            raise ValueError('positive finite per-wheel steering position tolerances required')
        self.position_tolerances = [min(v, math.radians(1.)) for v in self.position_tolerances]
        self.position_resolutions = geometry.get('steering_position_resolutions')
        if self.position_resolutions is not None and (
                len(self.position_resolutions) != len(self.x)
                or not all(type(v) in (int, float) and math.isfinite(v) and v >= 1.
                           and v == math.floor(v) for v in self.position_resolutions)):
            raise ValueError('positive integer per-wheel steering position resolutions required')
        self.reset()
        self.reset_alignment()

    def reset(self):
        self.mode = None
        self.output = [0.0, 0.0, 0.0]

    def reset_alignment(self):
        # Reference is fixed for an authority/homing epoch, never recentered
        # from a moving or overshooting wheel's instantaneous feedback.
        self.home_references = None
        self.steering_targets = None
        self.crab_heading = None
        self.crab_offsets = None
        self.crab_blocked = False
        self.neutral_home_targets = None
        self.neutral_home_feedback_ns = None

    def release_crab_boundary(self):
        # Call only for a confirmed operator stop, not a zero command
        # caused by an interlock, missing source, or acceleration timing.
        self.crab_heading = None
        self.crab_offsets = None
        self.crab_blocked = False

    def _position_error(self, index, target, actual):
        # Match the driver's native command and round-half-away-from-zero rule.
        # This avoids chasing unreachable sub-count positions. Quantize only the
        # error comparison; ideal targets remain fixed for continuity and holds.
        if self.position_resolutions is not None:
            resolution = self.position_resolutions[index]
            ticks = target * resolution / (2 * math.pi)
            if not math.isfinite(ticks):
                raise ValueError('steering target exceeds native position range')
            fraction, whole = math.modf(abs(ticks))
            ticks = math.copysign(whole + (1. if fraction >= .5 else 0.), ticks)
            target = ticks * 2 * math.pi / resolution
        return abs(target - actual)

    @staticmethod
    def axis(value):
        return 0.0 if abs(value) <= .08 else max(-1.0, min(1.0, value))

    def command(self, speed, steer, crab, stationary, dt, max_speed, acceleration):
        speed, steer, crab = map(self.axis, (speed, steer, crab))
        dt = max(0.0, min(.1, dt))
        # Only the owning controls can release a mode. Ignore the other stick.
        released = (self.mode == 'ackermann' and not speed and not steer or
                    self.mode == 'crab' and not speed and not crab)
        if released and stationary and max(map(abs, self.output)) < .001:
            self.mode = None
        if self.mode is None:
            # Crab wins simultaneous input. Left stick alone never acquires.
            self.mode = 'crab' if crab else 'ackermann' if speed and steer else None
        # Neutral cancels the requested motion immediately. Ramping an uncapped
        # 1/2 m/s request down behind a 0.3 m/s supervisor cap can otherwise keep
        # the robot moving for seconds after the physical controls are released.
        # Keep the steering-family lock until measured standstill.
        motion_released = not speed and (self.mode != 'crab' or not crab)
        if motion_released:
            self.output = [0.0, 0.0, 0.0]
            if stationary and released:
                self.mode = None
            return list(self.output)
        if self.mode == 'crab':
            tx, ty = speed * max_speed, crab * max_speed
            norm = math.hypot(tx, ty)
            if norm > max_speed:
                tx *= max_speed / norm; ty *= max_speed / norm
            dx, dy = tx-self.output[0], ty-self.output[1]
            change = math.hypot(dx, dy)
            scale = min(1., acceleration*dt/change) if change else 1.
            self.output = [self.output[0]+dx*scale, self.output[1]+dy*scale, 0.]
        elif self.mode == 'ackermann':
            curvature = self.curvature_limit * steer
            speed_limit = min(max_speed, self.max_yaw / abs(curvature)) if curvature else max_speed
            target = speed * speed_limit
            step = acceleration * dt
            vx = min(max(target, self.output[0]-step), self.output[0]+step)
            # The yaw/forward ratio is curvature, including when reversing.
            # No yaw is permitted without a forward/backward stick request.
            self.output = [vx, 0., vx * self.curvature_limit * steer if speed else 0.]
        else:
            # Straight travel does not reserve a steering family. The first
            # qualifying steering input can still choose Ackermann or crab.
            step = acceleration*dt
            vx = min(max(speed*max_speed, self.output[0]-step), self.output[0]+step)
            self.output = [vx, 0., 0.]
        return list(self.output)

    @staticmethod
    def cap_velocity(values, linear_limit, angular_limit):
        """Apply both speed limits with one scale, preserving wheel headings."""
        vx, vy, yaw = values
        speed = math.hypot(vx, vy)
        scale = min(1., linear_limit/speed) if speed else 1.
        if yaw:
            scale = min(scale, angular_limit/abs(yaw))
        return vx*scale, vy*scale, yaw*scale

    def project(self, values, requested=None):
        """Keep downstream independent ramps inside the manual motion family."""
        vx, vy, yaw = values
        if requested is not None:
            rx, ry, rw = requested
            if abs(ry) > 1e-9 and abs(rw) < 1e-9:
                yaw = 0.
            elif abs(rx) > 1e-9 or abs(rw) > 1e-9:
                vy = 0.
        if abs(vy) > 1e-9 and abs(yaw) < 1e-9:
            return vx, vy, 0.
        bound = min(abs(vx) * self.curvature_limit, self.max_yaw)
        return vx, 0., max(-bound, min(bound, yaw))

    def align(self, candidate, feedback, body, *, neutral=False):
        """Bound headings while allowing traction and steering concurrently.

        Right modules mounted at pi may need opposite traction sign. Compare
        actual controller targets, not raw left/right encoder equality.
        """
        g = self.geometry
        steering, traction = g['steering_joint_names'], g['traction_joint_names']
        expected = set(steering + traction)
        for msg in (candidate, feedback):
            if (len(msg.name) != len(expected) or set(msg.name) != expected
                    or len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name)
                    or not all(math.isfinite(v) for v in msg.position + msg.velocity)):
                raise ValueError('complete fresh finite wheel samples required for alignment')
        out = copy.deepcopy(candidate)
        ids = {n:i for i,n in enumerate(out.name)}
        actual = dict(zip(feedback.name, feedback.position))
        if self.home_references is None:
            self.home_references = {name: round(actual[name]/(2*math.pi))*2*math.pi for name in steering}
            self.steering_targets = {name: actual[name] for name in steering}
        if max(map(abs, body)) <= 1e-9:
            if neutral and self.return_home_on_neutral:
                return self._neutral_home(out, feedback, ids, actual)
            self.neutral_home_targets = None
            self.neutral_home_feedback_ns = None
            self.steering_targets = {name: actual[name] for name in steering}
            for name in steering:
                out.position[ids[name]] = actual[name]
            for name in steering + traction:
                out.velocity[ids[name]] = 0.
            return out
        if self.neutral_home_targets is not None:
            # Fresh stick input cancels return-to-home, including its old
            # unachieved targets. Continue from the measured controller branch.
            self.steering_targets = {name: actual[name] for name in steering}
            self.release_crab_boundary()
            self.neutral_home_targets = None
            self.neutral_home_feedback_ns = None
        crab = abs(body[2]) < 1e-9 and (abs(body[1]) > 1e-9 or self.crab_heading is not None)
        limit = self.crab_limit if crab else math.pi/4
        errors = []
        blocked = False
        shared_heading = None
        if crab:
            # Choose one rolling axis for the vehicle, not one independent
            # half-turn branch per pi-mounted controller.
            previous_heading = self.crab_heading
            if previous_heading is None:
                first = steering[0]
                reference = actual[first] if self.angle_limits_enabled else self.steering_targets[first]
                previous_heading = reference - self.home_references[first]
                previous_heading += math.remainder(g['wheel_mount_angles'][0], math.pi)
                if self.angle_limits_enabled:
                    period = 2 * math.pi if limit > math.pi / 2 else math.pi
                    previous_heading = math.remainder(previous_heading, period)
                if abs(previous_heading) <= 2*self.position_tolerances[0]:
                    previous_heading = 0.0
                if not self.angle_limits_enabled:
                    # Retain each controller's existing turn/traction branch.
                    # Independently rounded homes near +/-180 degrees can lie
                    # on opposite revolutions. Adding one shared heading to
                    # those homes must not introduce an extra full turn.
                    self.crab_offsets = {
                        name: round((self.steering_targets[name] -
                                     (self.home_references[name] + previous_heading -
                                      math.remainder(mount, math.pi))) / math.pi) * math.pi
                        for name, mount in zip(steering, g['wheel_mount_angles'])}
            if self.angle_limits_enabled and abs(body[1]) <= 1e-9 and limit <= math.pi / 2:
                # Straight travel has a zero-degree rolling axis in either
                # direction. At +/-90, modulo-pi continuity has a tie that
                # must resolve inward, even before the old move completes.
                shared_heading = 0.0
            else:
                requested_heading = math.atan2(body[1], body[0])
                delta = math.remainder(requested_heading-previous_heading, math.pi)
                if abs(abs(delta)-math.pi/2) <= 1e-4:
                    # A straight request at a +/-90-degree tie returns inward.
                    # Continuous sweeps beyond 90 keep their selected axis.
                    delta = (math.copysign(math.pi/2, body[1]) if abs(body[1]) > 1e-9
                             else -math.remainder(previous_heading, math.pi))
                shared_heading = previous_heading + delta
                # Evaluate each new heading, rather than latching all reverse
                # input. Outward requests stay at the limit; inward requests
                # can resume without a neutral step.
                if self.angle_limits_enabled:
                    blocked = abs(shared_heading) > limit+1e-4
                    shared_heading = max(-limit, min(limit, shared_heading))
        for index, (name, drive, mount, tolerance) in enumerate(zip(
                steering, traction, g['wheel_mount_angles'], self.position_tolerances)):
            i, j = ids[name], ids[drive]
            physical = math.remainder(out.position[i] + mount, math.pi)
            if abs(abs(physical)-math.pi/2) <= 1e-4:
                # Full sideways endpoint is a shared body-angle tie.
                physical = math.copysign(math.pi/2, body[1] if body[1] else physical)
            if self.angle_limits_enabled and abs(physical) > limit + 1e-4:
                raise ValueError('manual kinematics angle exceeds selected steering envelope')
            physical = max(-limit, min(limit, physical))
            home = self.home_references[name]
            if crab:
                relative = shared_heading - math.remainder(mount, math.pi)
                target = home + (max(-limit, min(limit, relative)) if self.angle_limits_enabled else relative)
                if not self.angle_limits_enabled:
                    target += self.crab_offsets[name]
            else:
                # Example-project continuity: unwrap around the previous requested
                # controller position, then prefer traction reversal to a >90-degree
                # steering move. Never chase changing measured feedback.
                previous = self.steering_targets[name]
                delta = math.remainder(out.position[i] - previous, 2*math.pi)
                if delta > math.pi/2 + 1e-4:
                    delta -= math.pi
                elif delta < -math.pi/2 - 1e-4:
                    delta += math.pi
                target = previous + delta
                # Retain the requested mechanical envelope when continuity chooses
                # an equivalent heading outside this homing epoch's travel range.
                if self.angle_limits_enabled:
                    if target > home+limit+1e-4:
                        target -= math.pi
                    elif target < home-limit-1e-4:
                        target += math.pi
                    if not home-limit-1e-4 <= target <= home+limit+1e-4:
                        raise ValueError('manual controller target exceeds selected steering envelope')
                    target = max(home-limit, min(home+limit, target))
            if math.cos(target-out.position[i]) < 0:
                out.velocity[j] *= -1
            out.position[i] = target
            if blocked:
                out.velocity[j] = 0.0
            error = self._position_error(index, target, actual[name])
            # Correct overshoot and later drift toward the retained target,
            # including at an envelope endpoint. Deviation is not a disable.
            errors.append(0.0 if error <= tolerance + 1e-12 else error)
        # Like the example, every moving steering motor uses the same profile
        # limit in the single all-wheel command. Stop each motor at its target.
        for error, name in zip(errors, steering):
            out.velocity[ids[name]] = g['max_steering_velocity'] if error else 0.0
        self.steering_targets = {name: out.position[ids[name]] for name in steering}
        self.crab_heading = shared_heading
        self.crab_blocked = blocked
        return out

    def _neutral_home(self, out, feedback, ids, actual):
        g = self.geometry
        steering, traction = g['steering_joint_names'], g['traction_joint_names']
        stamp = getattr(getattr(feedback, 'header', None), 'stamp', None)
        feedback_ns = (stamp.sec * 1_000_000_000 + stamp.nanosec
                       if stamp is not None and hasattr(stamp, 'sec') else stamp)
        if self.neutral_home_targets is None:
            # Match native B2 homing: freeze nearest full-turn targets once;
            # exact half-turn ties favor the original zero, never chase feedback.
            def home(angle):
                turns = angle / (2 * math.pi)
                whole = math.trunc(turns)
                if abs(turns - whole) > .5:
                    whole += 1 if turns > 0 else -1
                return whole * 2 * math.pi
            self.neutral_home_targets = {name: home(actual[name]) for name in steering}
            self.neutral_home_feedback_ns = feedback_ns
        velocities = dict(zip(feedback.name, feedback.velocity))
        # Publish a zero batch first, then wait for a newer complete feedback
        # sample showing traction standstill. Stale/missing feedback never homes.
        stopped = (feedback_ns is not None and self.neutral_home_feedback_ns is not None
                   and feedback_ns > self.neutral_home_feedback_ns
                   and all(abs(velocities[name]) <= .02 for name in traction))
        for name in steering + traction:
            out.velocity[ids[name]] = 0.
        for index, (name, tolerance) in enumerate(zip(steering, self.position_tolerances)):
            target = self.neutral_home_targets[name]
            out.position[ids[name]] = target if stopped else actual[name]
            if stopped and self._position_error(index, target, actual[name]) > tolerance + 1e-12:
                out.velocity[ids[name]] = self.neutral_home_velocity
        self.steering_targets = {name: out.position[ids[name]] for name in steering}
        self.release_crab_boundary()
        return out
