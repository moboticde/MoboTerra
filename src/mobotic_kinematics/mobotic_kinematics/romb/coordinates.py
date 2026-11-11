"""Frame/unit adaptation only; vehicle inverse/direct kinematics stays in romb."""
import math


class RombContract:
    def __init__(self, modules, steering, traction, mounts, max_speed, profile_speed,
                 epsilon=.001):
        self.modules = list(modules)
        self.steering = list(steering)
        self.traction = list(traction)
        self.mounts = list(mounts)
        n = len(self.modules)
        if (n < 2 or any(len(values) != n for values in (steering, traction, mounts))
                or len(set(steering + traction)) != 2 * n
                or steering != [m + '_steering' for m in modules]
                or traction != [m + '_traction' for m in modules]):
            raise ValueError('Romb backend requires unique <module>_steering/_traction joints')
        if (not all(math.isfinite(v) for v in [*mounts, max_speed, profile_speed, epsilon])
                or min(max_speed, profile_speed) <= 0 or epsilon < 0):
            raise ValueError('Invalid romb mounting orientation or speed limits')
        self.max_speed, self.profile_speed, self.epsilon = max_speed, profile_speed, epsilon
        self.targets = None

    def reset(self):
        self.targets = None

    def indices(self, names, positions, velocities):
        if (len(set(names)) != len(names) or len(positions) != len(names)
                or len(velocities) != len(names)
                or not set(self.steering + self.traction).issubset(names)
                or not all(math.isfinite(v) for v in positions + velocities)):
            raise ValueError('Incomplete or non-finite romb joint sample')
        return {name: i for i, name in enumerate(names)}

    def feedback_positions(self, names, positions, velocities):
        indices = self.indices(names, positions, velocities)
        output = list(positions)
        for name, mount in zip(self.steering, self.mounts):
            output[indices[name]] += mount
        return output

    def commands(self, names, positions, velocities, measured_names, measured_positions):
        indices = self.indices(names, positions, velocities)
        measured = dict(zip(measured_names, measured_positions))
        if self.targets is None:
            self.targets = [measured[name] for name in self.steering]
        targets, speeds, profiles = [], [], []
        for i, (steer, traction, mount) in enumerate(zip(self.steering, self.traction, self.mounts)):
            speed = velocities[indices[traction]]  # Already wheel-contact m/s.
            target = self.targets[i]
            if abs(speed) > self.epsilon:
                # Preserve the romb's selected wheel direction, translating
                # its physical angle into the installed controller coordinate.
                delta = math.remainder(positions[indices[steer]] - mount - target, 2 * math.pi)
                if delta > math.pi / 2 + 1e-4:
                    delta -= math.pi
                    speed = -speed
                elif delta < -math.pi / 2 - 1e-4:
                    delta += math.pi
                    speed = -speed
                target += delta
                profile = self.profile_speed
            else:
                speed, profile = 0.0, 0.0
            targets.append(target)
            speeds.append(speed)
            profiles.append(profile)
        peak = max(map(abs, speeds))
        ratio = min(1.0, self.max_speed / peak) if peak else 1.0
        self.targets = targets
        return targets, [v * ratio for v in speeds], profiles
