"""Measured steering alignment must bound the complete manual traction batch."""
import math
from types import SimpleNamespace

import pytest

from mobotic_kinematics.manual_steering import ManualSteering
from test_exclusive_manual_steering import geometry, model, sample, full_traction


def stamped(candidate, seconds):
    ns = round(seconds * 1e9)
    candidate.header = SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9, nanosec=ns%10**9))
    return candidate


@pytest.mark.parametrize('error, fraction', [(0., 1.), (4., 1.), (6., 1.), (18., .5), (30., 0.), (90., 0.)])
@pytest.mark.parametrize('direction', [-1., 1.])
def test_worst_wheel_sets_one_common_fraction_with_mounting_signs(error, fraction, direction):
    g = geometry()
    g['wheel_mount_angles'] = [0., math.pi, 0., math.pi]
    p = ManualSteering(g)
    body = (direction*.4, 0., 0.)
    candidate = model(g, body)
    feedback = sample(g, [-math.radians(error), 0., 0., 0.])
    out = p.align(candidate, feedback, body)
    assert out.velocity[4:] == pytest.approx([direction*.4*fraction, -direction*.4*fraction]*2)
    assert out.velocity[0] == (.3 if error else 0.)
    assert out.velocity[1:4] == [0.]*3


def test_unequal_turning_speeds_keep_their_ratios_under_one_lagging_wheel():
    g = geometry()
    p = ManualSteering(g)
    body = (.4, 0., .12)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    first = p.align(candidate, feedback, body)
    feedback.position[:4] = first.position[:4]
    feedback.position[2] += math.radians(18.)
    out = p.align(candidate, feedback, body)
    expected = full_traction(g, body, out.position[:4])
    assert len({round(abs(v), 4) for v in expected}) > 1
    assert out.velocity[4:] == pytest.approx([v*.5 for v in expected], abs=3e-5)
    assert out.velocity[:4] == [0., 0., .3, 0.]


def test_121_unit_feedback_compares_the_reachable_native_target():
    g = geometry()
    g['steering_position_resolutions'] = [121.]*4
    p = ManualSteering(g)
    angle = math.radians(16.)  # Target rounds to 5 counts, about 14.88 degrees.
    body = (.4*math.cos(angle), .4*math.sin(angle), 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    first = p.align(candidate, feedback, body)
    unit = 2*math.pi/121
    native = [round(a/unit)*unit for a in first.position[:4]]
    feedback.position[:4] = [a+2*unit for a in native]
    out = p.align(candidate, feedback, body)
    assert out.velocity[4:] == pytest.approx(full_traction(g, body, out.position[:4]), abs=3e-5)
    assert out.velocity[:4] == [.3]*4  # Position correction is finer than the traction threshold.
    feedback.position[0] = native[0]+11*unit  # More than 30 degrees on just one wheel.
    assert p.align(candidate, feedback, body).velocity[4:] == [0.]*4


def test_one_stalled_steering_wheel_holds_all_traction():
    g = geometry()
    p = ManualSteering(g)
    body = (0., .4, 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    for _ in range(150):
        out = p.align(candidate, feedback, body)
        assert out.velocity[4:] == [0.]*4
        for k in range(1, 4):
            error = out.position[k] - feedback.position[k]
            feedback.position[k] += max(-.3*.05, min(.3*.05, error))
    assert out.velocity[:4] == [.3, 0., 0., 0.]


def test_held_full_speed_request_recovers_from_zero_without_windup():
    g = geometry()
    g['max_acceleration_linear'] = .5
    p = ManualSteering(g)
    body = (0., 2., 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    for k in range(40):
        out = p.align(stamped(candidate, 100+k*.05), feedback, body)
        assert out.velocity[4:] == [0.]*4
    feedback.position[:4] = out.position[:4]
    previous = 0.
    for k in range(40, 140):
        out = p.align(stamped(candidate, 100+k*.05), feedback, body)
        peak = max(map(abs, out.velocity[4:]))
        assert 0 <= peak-previous <= .025+1e-10
        assert out.velocity[4:] == pytest.approx([peak, -peak, peak, -peak])
        previous = peak
    assert previous == pytest.approx(2.)
    # A new large error cuts traction even after the ramp has reached full speed.
    feedback.position[0] -= math.radians(45.)
    assert p.align(stamped(candidate, 107.), feedback, body).velocity[4:] == [0.]*4


def test_partial_alignment_does_not_repeatedly_shrink_the_ramp():
    g = geometry()
    g['max_acceleration_linear'] = .5
    p = ManualSteering(g)
    body = (2., 0., 0.)
    candidate = model(g, body)
    feedback = sample(g, [-math.radians(18.), 0., 0., 0.])
    for k in range(100):
        out = p.align(stamped(candidate, 100+k*.05), feedback, body)
    assert max(map(abs, out.velocity[4:])) == pytest.approx(1.)
    feedback.position[0] = 0.
    out = p.align(stamped(candidate, 105.), feedback, body)
    assert max(map(abs, out.velocity[4:])) == pytest.approx(1.025)
    # Duplicate samples cannot create extra acceleration allowance.
    for _ in range(10):
        assert p.align(candidate, feedback, body).velocity == out.velocity


def test_long_gap_and_authority_reset_cannot_release_accumulated_speed():
    g = geometry()
    g['max_acceleration_linear'] = .5
    p = ManualSteering(g)
    body = (1., 0., 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    assert p.align(stamped(candidate, 100.), feedback, body).velocity[4:] == [0.]*4
    out = p.align(stamped(candidate, 110.), feedback, body)
    assert max(map(abs, out.velocity[4:])) == pytest.approx(.05)  # At most 100 ms of ramp time.
    p.reset_alignment()
    assert p.align(stamped(candidate, 110.05), feedback, body).velocity[4:] == [0.]*4


@pytest.mark.parametrize('acceleration', [0., -1., math.nan, math.inf])
def test_invalid_recovery_acceleration_is_rejected(acceleration):
    g = geometry()
    g['max_acceleration_linear'] = acceleration
    with pytest.raises(ValueError, match='acceleration'):
        ManualSteering(g)


def test_boost_changes_only_the_moving_steering_profile_and_releases_immediately():
    g = geometry()
    g['steering_boost_velocity'] = .6
    p = ManualSteering(g)
    body = (0., .4, 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    targets = None
    for boost in (False, True, True, False):
        out = p.align(candidate, feedback, body, boost=boost)
        assert out.velocity[:4] == ([.6]*4 if boost else [.3]*4)
        assert out.velocity[4:] == [0.]*4  # B1 does not bypass the alignment gate.
        if targets is not None:
            assert out.position == targets
        targets = out.position
    feedback.position[:4] = targets[:4]
    out = p.align(candidate, feedback, body, boost=True)
    assert out.velocity[:4] == [0.]*4
    assert out.velocity[4:] == pytest.approx(full_traction(g, body, out.position[:4]), abs=3e-5)


def test_profiles_without_a_steering_boost_setting_keep_the_normal_speed():
    g = geometry()
    p = ManualSteering(g)
    body = (0., .4, 0.)
    assert p.align(model(g, body), sample(g, [0.]*4), body, boost=True).velocity[:4] == [.3]*4


def test_boost_does_not_change_neutral_homing_speed():
    g = geometry()
    g.update(steering_boost_velocity=.6, steering_return_home_on_neutral=True, steering_neutral_home_velocity=1.2)
    p = ManualSteering(g)
    candidate = model(g, (0., 0., 0.))
    feedback = sample(g, [math.radians(20.)]*4)
    feedback.header = SimpleNamespace(stamp=1)
    p.align(candidate, feedback, (0., 0., 0.), neutral=True, boost=True)
    feedback.header.stamp = 2
    assert p.align(candidate, feedback, (0., 0., 0.), neutral=True, boost=True).velocity[:4] == [1.2]*4


@pytest.mark.parametrize('boost', [0., .2, math.nan, math.inf])
def test_invalid_steering_boost_speed_is_rejected(boost):
    g = geometry()
    g['steering_boost_velocity'] = boost
    with pytest.raises(ValueError, match='steering boost'):
        ManualSteering(g)
