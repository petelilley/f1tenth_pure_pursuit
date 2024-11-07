from dataclasses import dataclass
import numpy as np

class TrapezoidProfile:
    # max_velocity and acceleration should both be positive
    def __init__(self, max_velocity: float, acceleration: float):
        self.is_configured = False

        # self.start_ts = Time()
        self.max_velocity = max_velocity
        self.acceleration = acceleration

        self.velocity_init = 0.0
        self.velocity_cruise = 0.0
        self.velocity_final = 0.0

        self.time_rise_s = 0.0
        self.time_cruise_s = 0.0
        self.time_fall_s = 0.0
        self.time_total_s = 0.0

        self.dist_rise = 0.0
        self.dist_cruise = 0.0
        self.dist_fall = 0.0
        self.dist_total = 0.0

    @dataclass
    class Sample:
        velocity: float
        distance: float

    # Configure the profile to accelerate/decelerate to a target velocity
    def configure_velocity(self, start_ts, velocity: float):
        velocity = np.clip(velocity, -self.max_velocity, +self.max_velocity)

        if self.is_configured:
            t = (start_ts.nanoseconds - self.start_ts.nanoseconds) / 1e9

            if velocity == self.velocity_final and t > self.time_total_s:
                return

            if velocity == self.velocity_cruise and (t < (self.time_total_s - self.time_fall_s) or self.time_fall_s == 0.0):
                self.velocity_final = self.velocity_cruise
                self.time_fall_s = 0.0
                self.time_total_s = self.time_rise_s + self.time_cruise_s
                self.dist_fall = 0.0
                self.dist_total = self.dist_rise + self.dist_cruise
                return
            else:
                last_sample = self.sample(start_ts)
                self.velocity_init = last_sample.velocity

        self.is_configured = True
        self.start_ts = start_ts

        self.velocity_cruise = velocity
        self.velocity_final = velocity

        a = self.acceleration
        if self.velocity_init > velocity:
            a = -a

        # v = v_0 + at
        self.time_rise_s = (velocity - self.velocity_init) / a
        self.time_cruise_s = 0.0
        self.time_fall_s = 0.0
        self.time_total_s = self.time_rise_s

        # v^2 = v_0^2 + 2ax
        self.dist_rise = (velocity**2 - self.velocity_init**2) / (2 * a)
        self.dist_cruise = 0.0
        self.dist_fall = 0.0
        self.dist_total = self.dist_rise

    def sample(self, ts) -> Sample:
        if not self.is_configured:
            return TrapezoidProfile.Sample(0.0, 0.0)

        time_s = (ts.nanoseconds - self.start_ts.nanoseconds) / 1e9

        if time_s < 0:
            v = 0.0
            d = 0.0

        elif time_s <= self.time_rise_s: # Acceleration
            a = self.acceleration
            if self.velocity_init > self.velocity_cruise:
                a = -a

            # v = v_0 + at
            v = self.velocity_init + a * time_s

            # x = vt + 1/2 at^2
            d = self.velocity_init * time_s + 0.5 * a * time_s**2

        elif time_s <= (self.time_rise_s + self.time_cruise_s): # Cruise
            t = time_s - self.time_rise_s

            v = self.velocity_cruise

            # x = vt
            d = self.dist_rise + self.velocity_cruise * t

        elif time_s < self.time_total_s: # Deceleration
            a = -self.acceleration
            if self.velocity_final > self.velocity_cruise:
                a = -a

            t = time_s - self.time_rise_s - self.time_cruise_s

            # v = v_0 + at
            v = self.velocity_cruise + a * t

            # x = vt + 1/2 at^2
            x = self.velocity_cruise * t + 0.5 * a * t**2

        else:
            t = time_s - self.time_total_s

            v = self.velocity_final
            d = self.dist_total + v * t

        return TrapezoidProfile.Sample(v, d)
