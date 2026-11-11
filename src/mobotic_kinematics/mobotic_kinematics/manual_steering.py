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
        # A controller cannot resolve sub-count endpoint corrections. Use each
        # configured steering encoder's one-count angle when it is available.
        self.position_tolerances = geometry.get(
            'steering_position_tolerances', [1e-3] * len(self.x))
        if (len(self.position_tolerances) != len(self.x)
                or not all(math.isfinite(v) and 0.0 < v < math.pi/4
                           for v in self.position_tolerances)):
            raise ValueError('positive finite per-wheel steering position tolerances required')
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
        self.crab_blocked = False

    def release_crab_boundary(self):
        # Call only for confirmed neutral operator controls, not a zero command
        # caused by an interlock, missing source, or acceleration timing.
        self.crab_heading = None
        self.crab_blocked = False

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

    def align(self, candidate, feedback, body):
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
            for name in steering:
                out.position[ids[name]] = actual[name]
            for name in steering + traction:
                out.velocity[ids[name]] = 0.
            return out
        crab = abs(body[2]) < 1e-9 and (abs(body[1]) > 1e-9 or self.crab_heading is not None)
        limit = math.pi/2 if crab else math.pi/4
        errors = []
        blocked = False
        shared_heading = None
        if crab:
            # Choose one rolling axis for the vehicle, not one independent
            # half-turn branch per pi-mounted controller.
            previous_heading = self.crab_heading
            if previous_heading is None:
                first = steering[0]
                previous_heading = actual[first] - self.home_references[first]
                previous_heading += math.remainder(g['wheel_mount_angles'][0], math.pi)
                previous_heading = math.remainder(previous_heading, math.pi)
                if abs(previous_heading) <= 2*self.position_tolerances[0]:
                    previous_heading = 0.0
            requested_heading = math.atan2(body[1], body[0])
            delta = math.remainder(requested_heading-previous_heading, math.pi)
            if abs(abs(delta)-math.pi/2) <= 1e-4:
                delta = math.copysign(math.pi/2, body[1] if body[1] else delta)
            shared_heading = previous_heading + delta
            blocked = abs(shared_heading) > limit+1e-4
            shared_heading = max(-limit, min(limit, shared_heading))
            if self.crab_blocked and body[0] < -1e-9:
                shared_heading = self.crab_heading
                blocked = True
        for name, drive, mount, tolerance in zip(
                steering, traction, g['wheel_mount_angles'], self.position_tolerances):
            i, j = ids[name], ids[drive]
            physical = math.remainder(out.position[i] + mount, math.pi)
            if abs(abs(physical)-math.pi/2) <= 1e-4:
                # Full sideways endpoint is a shared body-angle tie.
                physical = math.copysign(math.pi/2, body[1] if body[1] else physical)
            if abs(physical) > limit + 1e-4:
                raise ValueError('manual kinematics angle exceeds selected steering envelope')
            physical = max(-limit, min(limit, physical))
            home = self.home_references[name]
            if crab:
                relative = shared_heading - math.remainder(mount, math.pi)
                target = home + max(-limit, min(limit, relative))
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
            error = abs(target-actual[name])
            # A held endpoint must stop at/past its limit, including a rounded
            # pi mount or small overshoot. A new inward target resumes normally.
            at_limit = (target-home >= limit-tolerance and actual[name] >= target-tolerance
                        or target-home <= -limit+tolerance and actual[name] <= target+tolerance)
            errors.append(0.0 if error <= tolerance or at_limit else error)
        # Like the example, every moving steering motor uses the same profile
        # limit in the single all-wheel command. Stop each motor at its target.
        for error, name in zip(errors, steering):
            out.velocity[ids[name]] = g['max_steering_velocity'] if error else 0.0
        self.steering_targets = {name: out.position[ids[name]] for name in steering}
        self.crab_heading = shared_heading
        self.crab_blocked = blocked
        return out
