import pytest
from f1tenth_pure_pursuit.trapezoid_profile import TrapezoidProfile

def test_configure_velocity_1():
    profile = TrapezoidProfile(1.0, 1.0)
    assert not profile.is_configured

    # TODO: More testing
