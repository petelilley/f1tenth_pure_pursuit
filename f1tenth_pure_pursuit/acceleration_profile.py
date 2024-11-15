from dataclasses import dataclass
import numpy as np

class AccelerationProfile:
    def __init__(self, acceleration: float):
        if acceleration <= 0.0:
            raise ValueError("acceleration must be greater than 0")

        self.is_configured = False

        self.acceleration = acceleration

        self.velocity_init = 0.0
        self.velocity_final = 0.0

        self.time_s = 0.0

    def reset(self):
        self.is_configured = False
        self.velocity_init = 0.0
        self.velocity_final = 0.0

    def get_final_velocity(self) -> float:
        return self.velocity_final

    # Configure the profile to accelerate/decelerate to a target velocity
    def configure(self, start_ts, velocity: float):
        if self.is_configured:
            last_sample = self.sample(start_ts)
            self.velocity_init = last_sample.velocity

        self.is_configured = True
        self.start_ts = start_ts

        self.velocity_final = velocity

        a = self.acceleration
        if self.velocity_init > velocity:
            a = -a

        # v = v_0 + at
        self.time_s = (self.velocity_final - self.velocity_init) / a

    @dataclass
    class Sample:
        velocity: float
        distance: float

    def sample(self, ts) -> Sample:
        if not self.is_configured:
            return AccelerationProfile.Sample(0.0, 0.0)

        t = (ts.nanoseconds - self.start_ts.nanoseconds) / 1e9
        t = max(t, 0.0)

        a = self.acceleration
        if self.velocity_init > self.velocity_final:
            a = -a

        if t <= self.time_s: # Accelerate
            # v = v_0 + at
            v = self.velocity_init + a * t

            # x = vt + 1/2 at^2
            d = self.velocity_init * t + 0.5 * a * t**2

        else: # Cruise at final velocity
            v = self.velocity_final

            d_rise = self.velocity_init * self.time_s + 0.5 * a * self.time_s**2

            # d = vt
            d = d_rise + v * (t - self.time_s)

        return AccelerationProfile.Sample(v, d)
