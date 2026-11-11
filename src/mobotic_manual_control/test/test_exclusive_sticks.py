import pytest
import test_manual_control as base
from mobotic_kinematics.manual_steering import ManualSteering


@pytest.fixture
def fixture():
    f=base.ManualControlTest();f.setUp()
    f.node.steering_policy=ManualSteering({'wheels_x':[.845,-.845],'wheels_y':[.495,-.495]})
    f.node.require_neutral_rearm=False
    f.node._stationary=lambda now:True
    return f


def send(f, speed, left, side):
    f.now=base.Time(f.now.nanoseconds/1e9+.02)
    joy=f.joy();joy.axes[3]=speed;joy.axes[0]=left;joy.axes[2]=side
    f.node._joy_callback(joy)
    return f.node.command


def test_real_callback_first_ackermann_ignores_sideways(fixture):
    f=fixture;send(f,1.,1.,0.)
    for _ in range(10):out=send(f,1.,1.,-1.)
    assert f.node.steering_policy.mode=='ackermann'
    assert out[0]>0 and out[1]==0 and out[2]>0


def test_real_callback_first_crab_ignores_left_and_tie_is_crab(fixture):
    f=fixture
    for _ in range(10):out=send(f,1.,1.,1.)
    assert f.node.steering_policy.mode=='crab'
    assert out[0]>0 and out[1]>0 and out[2]==0.


def test_real_callback_left_alone_has_no_motion(fixture):
    f=fixture
    for _ in range(10):out=send(f,0.,1.,0.)
    assert out==[0.,0.,0.] and f.node.steering_policy.mode is None


def test_real_callback_input_loss_discards_old_mode(fixture):
    f=fixture;send(f,1.,1.,0.)
    f.node._reset_command(f.now)
    assert f.node.steering_policy.mode is None
    assert send(f,0.,0.,1.)[2]==0.
    assert f.node.steering_policy.mode=='crab'


def test_authority_change_preserves_fresh_neutral_but_discards_mode(fixture):
    from types import SimpleNamespace
    f=fixture;send(f,0.,0.,0.)
    def state(g):
        return SimpleNamespace(header=SimpleNamespace(stamp=f.now.to_msg()), current_mode=0,
            supervisor_instance='test',authority_generation=g)
    f.node._vehicle_mode_callback(state(1))
    f.now=base.Time(f.now.nanoseconds/1e9+.02)
    f.node._vehicle_mode_callback(state(2))
    assert f.node.controls_neutral
    assert f.node.command==[0.,0.,0.] and f.node.steering_policy.mode is None


def test_mode_release_requires_wheel_standstill_not_only_body_average(fixture):
    from types import SimpleNamespace
    import types
    f=fixture;n=f.node
    n._stationary=types.MethodType(base.Controller._stationary,n)
    n.steering_policy.geometry['traction_joint_names']=['left_traction','right_traction']
    n.measured_velocity=base.TwistStamped();n.measured_velocity.header.stamp=f.now.to_msg()
    n.measured_velocity_received=f.now
    n.measured_joints=SimpleNamespace(header=SimpleNamespace(stamp=f.now.to_msg()),
        name=['left_traction','right_traction'],velocity=[0.,0.])
    n.measured_joints_received=f.now
    assert n._stationary(f.now)
    n.measured_joints.velocity=[.2,-.2]
    assert not n._stationary(f.now)
