"""Neutral homing is an explicit, cancellable manual motion with zero traction."""
import math
from types import SimpleNamespace
import pytest
from mobotic_kinematics.manual_steering import ManualSteering
from test_exclusive_manual_steering import geometry, model, sample


def configured():
    g = geometry()
    g.update(steering_angle_limits_enabled=False, steering_return_home_on_neutral=True,
             steering_neutral_home_velocity=1.2, steering_position_tolerances=[2*math.pi/121]*4)
    return g, ManualSteering(g)


def test_home_corrects_one_count_drift_without_rebasing_or_traction():
    g, _ = configured()
    g['steering_position_resolutions'] = [121.]*4
    g['steering_position_tolerances'] = [math.radians(1.)]*4
    p = ManualSteering(g)
    home = [2*math.pi, -2*math.pi, 4*math.pi, 0.]
    f = feedback(g, home)
    c = model(g, (0., 0., 0.))
    p.align(c, f, (0., 0., 0.), neutral=True)
    for _ in range(100):
        f.header.stamp += 1
        f.position[:4] = home
        settled = p.align(c, f, (0., 0., 0.), neutral=True)
        assert settled.velocity == [0.]*8
        f.header.stamp += 1
        f.position[:4] = [v + 2*math.pi/121 for v in home]
        correcting = p.align(c, f, (0., 0., 0.), neutral=True)
        assert correcting.position[:4] == home
        assert correcting.velocity[:4] == [1.2]*4
        assert correcting.velocity[4:] == [0.]*4


def feedback(g, angles, stamp=100, traction=None):
    msg = sample(g, angles, traction)
    msg.header = SimpleNamespace(stamp=stamp)
    return msg


def test_zero_traction_precedes_home_and_requires_new_standstill_feedback():
    g, p = configured(); f = feedback(g, [.8]*4, traction=[.3]*4)
    c = model(g, (.3, 0., 0.))
    first = p.align(c, f, (0.,0.,0.), neutral=True)
    assert first.velocity == [0.]*8
    assert first.position[:4] == f.position[:4]
    f.velocity[4:] = [0.]*4
    assert p.align(c, f, (0.,0.,0.), neutral=True).velocity == [0.]*8
    f.header.stamp += 1
    f.velocity[-1] = .03
    assert p.align(c, f, (0.,0.,0.), neutral=True).velocity == [0.]*8
    f.header.stamp += 1; f.velocity[-1] = .02
    homing = p.align(c, f, (0.,0.,0.), neutral=True)
    assert homing.position[:4] == [0.]*4
    assert homing.velocity[:4] == [1.2]*4
    assert homing.velocity[4:] == [0.]*4


@pytest.mark.parametrize('degrees,expected', [(179,0),(181,360),(-179,0),(-181,-360),
                                               (180,0),(-180,0),(540,360),(-540,-360),
                                               (1079,1080),(-1079,-1080)])
def test_nearest_home_is_frozen_and_holds_when_complete(degrees, expected):
    g, p = configured(); f = feedback(g, [math.radians(degrees)]*4)
    c = model(g, (.3,.1,0.)); p.align(c, f, (0.,0.,0.), neutral=True)
    # Crossing a half-turn boundary after selection cannot flip the target.
    f.position[:4] = [a+.1 for a in f.position[:4]]
    f.header.stamp += 1
    homing = p.align(c, f, (0.,0.,0.), neutral=True)
    assert homing.position[:4] == pytest.approx([math.radians(expected)]*4)
    for _ in range(20):
        f.position[:4] = homing.position[:4]
        f.header.stamp += 1
        f.velocity[:4] = [0.]*4
        homing = p.align(c, f, (0.,0.,0.), neutral=True)
        assert homing.velocity == [0.]*8
        assert homing.position[:4] == pytest.approx([math.radians(expected)]*4)


@pytest.mark.parametrize('cause', ['interlock','input'])
def test_home_cancels_on_inhibition_or_fresh_motion(cause):
    g, p = configured(); f = feedback(g, [math.radians(170)]*4)
    c = model(g, (0.,.3,0.)); p.align(c, f, (0.,0.,0.), neutral=True)
    f.header.stamp += 1
    assert p.align(c, f, (0.,0.,0.), neutral=True).velocity[:4] == [1.2]*4
    if cause == 'interlock':
        hold = p.align(c, f, (0.,0.,0.))
        assert hold.position[:4] == f.position[:4]
        assert hold.velocity == [0.]*8
    else:
        body=(-.3,-.03,0.)
        resumed = p.align(model(g, body), f, body)
        assert max(abs(a-b) for a,b in zip(resumed.position[:4], f.position[:4])) < math.pi/2
        assert max(resumed.velocity[:4]) <= .3
        assert any(abs(v)>.01 for v in resumed.velocity[4:])
    assert p.neutral_home_targets is None


def test_missing_feedback_stamp_never_starts_neutral_homing():
    g,p=configured();f=sample(g,[.5]*4);c=model(g,(0.,0.,0.))
    for _ in range(10):
        assert p.align(c,f,(0.,0.,0.),neutral=True).velocity==[0.]*8


@pytest.mark.parametrize('bad', [None, 1, 'true'])
def test_neutral_home_enable_requires_boolean(bad):
    g=geometry();g['steering_return_home_on_neutral']=bad
    with pytest.raises(ValueError,match='steering_return_home_on_neutral'):ManualSteering(g)
