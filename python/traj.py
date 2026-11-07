
class Traj:

    def __init__(self, a_max, v_max):
        if a_max <= 0:
            raise ValueError("a_max must be positive")
        if v_max <= 0:
            raise ValueError("v_max must be positive")

        self.a_max = a_max
        self.v_max = v_max

        self.p = 0.0
        self.v = 0.0
        self.a = 0.0


    def step(self, pos, Ts):
        if Ts <= 0:
            raise ValueError("Ts must be positive")

        error = pos - self.p
        if abs(error) < 1e-12 and abs(self.v) < self.a_max * Ts:
            self.p = pos
            self.v = 0.0
            self.a = 0.0
            return self.p

        # Limit the requested velocity by the speed that can be stopped in the
        # remaining distance, then apply the acceleration limit.
        braking_velocity = (2.0 * self.a_max * abs(error)) ** 0.5
        target_velocity = min(self.v_max, braking_velocity)
        target_velocity *= 1.0 if error >= 0 else -1.0

        requested_acceleration = (target_velocity - self.v) / Ts
        self.a = max(-self.a_max, min(self.a_max, requested_acceleration))
        next_velocity = self.v + self.a * Ts
        next_position = self.p + next_velocity * Ts

        if (pos - self.p) * (pos - next_position) <= 0:
            self.p = pos
            self.v = 0.0
            self.a = 0.0
        else:
            self.v = next_velocity
            self.p = next_position

        return self.p

        


    def follow_up(self, act_pos):
        self.p = act_pos
        self.v = 0.0
        self.a = 0.0