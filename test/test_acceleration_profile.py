import pytest
from f1tenth_pure_pursuit.acceleration_profile import AccelerationProfile

class Time:
    def __init__(self, seconds):
        self.nanoseconds = seconds * 1e9

def test_init():
    profile = AccelerationProfile(1.0)
    assert not profile.is_configured
    assert profile.acceleration == 1.0

def test_configure_simple_positive():
    profile = AccelerationProfile(1.0)

    t = Time(0)
    profile.configure(t, 1.0)

    assert profile.velocity_init == 0.0
    assert profile.velocity_final == 1.0
    assert profile.time_s == 1.0

    t = Time(0)
    sample = profile.sample(t)
    assert sample.velocity == 0.0
    assert sample.distance == 0.0

    t = Time(0.5)
    sample = profile.sample(t)
    assert sample.velocity == 0.5
    assert sample.distance == 0.125

    t = Time(1.0)
    sample = profile.sample(t)
    assert sample.velocity == 1.0
    assert sample.distance == 0.5

    t = Time(2.0)
    sample = profile.sample(t)
    assert sample.velocity == 1.0
    assert sample.distance == 1.5

def test_configure_simple_negative():
    profile = AccelerationProfile(1.0)

    t = Time(0)
    profile.configure(t, -1.0)

    assert profile.velocity_init == 0.0
    assert profile.velocity_final == -1.0
    assert profile.time_s == 1.0

    t = Time(0)
    sample = profile.sample(t)
    assert sample.velocity == 0.0
    assert sample.distance == 0.0

    t = Time(0.5)
    sample = profile.sample(t)
    assert sample.velocity == -0.5
    assert sample.distance == -0.125

    t = Time(1.0)
    sample = profile.sample(t)
    assert sample.velocity == -1.0
    assert sample.distance == -0.5

def test_configure_linked_same_direction():
    profile = AccelerationProfile(1.0)

    t = Time(0)
    profile.configure(t, 1.0)

    t = Time(1)
    profile.configure(t, 2.0)

    assert profile.velocity_init == 1.0
    assert profile.velocity_final == 2.0
    assert profile.time_s == 1.0

    t = Time(1)
    sample = profile.sample(t)
    assert sample.velocity == 1.0
    assert sample.distance == 0.0

    t = Time(1.5)
    sample = profile.sample(t)
    assert sample.velocity == 1.5
    assert sample.distance == 0.625

    t = Time(2.0)
    sample = profile.sample(t)
    assert sample.velocity == 2.0
    assert sample.distance == 1.5

def test_configure_linked_change_direction():
    profile = AccelerationProfile(1.0)

    t = Time(0)
    profile.configure(t, 1.0)

    t = Time(1)
    profile.configure(t, -1.0)

    assert profile.velocity_init == 1.0
    assert profile.velocity_final == -1.0
    assert profile.time_s == 2.0

    t = Time(1)
    sample = profile.sample(t)
    assert sample.velocity == 1.0
    assert sample.distance == 0.0

    t = Time(2)
    sample = profile.sample(t)
    assert sample.velocity == 0.0
    assert sample.distance == 0.5

    t = Time(3)
    sample = profile.sample(t)
    assert sample.velocity == -1.0
    assert sample.distance == 0.0
