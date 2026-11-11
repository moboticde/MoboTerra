"""Exercise manual authority with real messages, without CAN or physical nodes."""
import copy
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import rclpy
from rclpy.time import Time
from mobotic_bringup.manual_drive_control import ManualDriveControl
from mobotic_interfaces.msg import OperatorState, WheelModuleStatus, WheelModuleStatusArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


@pytest.fixture
def node():
    rclpy.init()
    n = ManualDriveControl()
    n.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=n.test_now)))
    n.test_now = 100.0
    n.count_publishers = Mock(return_value=1)
    n.mode_pub = Mock()
    n.operation_pub = Mock()
    n.velocity_pub = Mock()
    n.command_pub = Mock()
    n.boundary = 99_000_000_000
    n.last_ros_tick = 99_990_000_000
    with patch('mobotic_bringup.manual_drive_control.time.monotonic', side_effect=lambda: n.test_now):
        n.last_tick = 99.99
        yield n
    n.destroy_node()
    rclpy.shutdown()


def seed(n, enabled=False):
    stamp = Time(seconds=n.test_now).to_msg()
    operator = OperatorState()
    operator.header.stamp = stamp
    operator.connected = operator.command_active = operator.controls_neutral = True
    n._receive('operator', operator)
    status = WheelModuleStatusArray()
    status.header.stamp = stamp
    m = WheelModuleStatus()
    m.name = 'front_left'
    m.initialized = m.startup_complete = m.position_valid = m.feedback_fresh = enabled
    m.enabled = enabled
    m.reset_id = n.reset_id
    m.reset_acknowledged = enabled
    for d, name in ((m.steering, 'front_left_steering'), (m.traction, 'front_left_traction')):
        d.name = name
        d.status_known = d.device_state_known = d.fault_known = True
        d.status_stamp = stamp
        d.enabled = d.initialized = enabled
    status.modules = [m]
    n._receive('status', status)
    joints = JointState()
    joints.header.stamp = stamp
    joints.name = ['front_left_steering', 'front_left_traction']
    joints.position = [0., 0.]
    joints.velocity = [0., 0.]
    n._receive('joints', joints)
    cmd = TwistStamped()
    cmd.header.stamp, cmd.header.frame_id = stamp, 'base_link'
    cmd.twist.linear.x = .1
    n._receive('command', cmd)
    return joints


def tick(n, dt=.02, enabled=True):
    n.test_now += dt
    joints = seed(n, enabled)
    n._tick()
    return joints


def test_startup_and_explicit_neutral_enable(node):
    seed(node)
    node._tick()
    assert not node.operation_pub.publish.call_args.args[0].enable
    response = node._enable(None, Trigger.Response())
    assert response.success and node.phase == 'recovering'
    node.test_now += .02
    seed(node)
    node._tick()
    assert node.operation_pub.publish.call_args.args[0].enable  # matching driver reset ID
    assert not node.mode_pub.publish.call_args.args[0].motion_permitted
    for _ in range(15):
        tick(node)
        if node.phase == 'enabled':
            break
    assert node.phase == 'enabled'
    assert node._motion_source() is None  # commands before completion discarded


@pytest.mark.parametrize('fault', ['operator', 'feedback', 'clock', 'authority'])
def test_loss_latches_disable_and_requires_b2(node, fault):
    seed(node, True)
    node.phase = 'enabled'
    if fault == 'operator':
        node.samples['operator'][0].estop_pressed = True
    elif fault == 'feedback':
        node.samples['status'][0].modules[0].feedback_fresh = False
    elif fault == 'clock':
        node.test_now += .4
    else:
        node.count_publishers.return_value = 2
    node._tick()
    assert node.phase == 'disabled'
    assert not node.operation_pub.publish.call_args.args[0].enable
    assert not node.mode_pub.publish.call_args.args[0].motion_permitted
    tick(node)
    assert node.phase == 'disabled'


def test_fresh_correlated_candidate_only_and_replay_rejected(node):
    node.phase = 'enabled'
    joints = seed(node, True)
    node._tick()
    joints.header = copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
    joints.velocity = [.2, .1]
    node._candidate(joints)
    assert node.command_pub.publish.call_count == 1
    result = node.command_pub.publish.call_args.args[0]
    assert result.modules[0].traction_velocity == .1
    node._candidate(joints)
    assert node.command_pub.publish.call_count == 1


@pytest.mark.parametrize('velocity', [-2., -1., .1, 1., 2.])
@pytest.mark.parametrize('curvature', [-.6, -.22827042, .22827042, .6])
def test_ackermann_cap_preserves_partial_steering_through_final_wheel_command(node, velocity, curvature):
    from mobotic_kinematics.manual_steering import ManualSteering
    node.max_velocity_linear = node.max_traction_velocity = 2.
    node.max_steering_velocity = .3
    node.steering_policy = ManualSteering({
        'wheels_x': [.845], 'wheels_y': [.495],
        'steering_joint_names': node.steering_joint_names,
        'traction_joint_names': node.traction_joint_names,
        'wheel_mount_angles': [0.], 'max_steering_velocity': .3})
    node.phase = 'enabled'
    joints = seed(node, True)
    node.samples['operator'][0].controls_neutral = False
    command = node.samples['command'][0]
    command.twist.linear.x = velocity
    command.twist.angular.z = velocity*curvature
    node._tick()
    output = node.velocity_pub.publish.call_args.args[0]
    body = output.twist
    assert body.linear.x == pytest.approx(math.copysign(min(abs(velocity), .3), velocity))
    assert body.angular.z/body.linear.x == pytest.approx(curvature)
    # Feed a complete solver result back through the real final authority.
    candidate = copy.deepcopy(joints)
    candidate.header = copy.deepcopy(output.header)
    wheel_x = body.linear.x - body.angular.z*.495
    wheel_y = body.angular.z*.845
    candidate.position[0] = math.atan2(wheel_y, wheel_x)
    candidate.velocity = [.3, math.hypot(wheel_x, wheel_y)]
    node._candidate(candidate)
    wheel = node.command_pub.publish.call_args.args[0].modules[0]
    expected = math.atan2(curvature*.845, 1.-curvature*.495)
    assert wheel.steering_position == pytest.approx(expected)
    assert abs(wheel.traction_velocity) <= .3 + 1e-12
    assert node.phase == 'enabled'


@pytest.mark.parametrize('limit', [.15, 2.])
def test_manual_only_caps_body_speed_without_scanner_zone_evidence(node, limit):
    node.max_velocity_linear = limit
    node.max_traction_velocity = 2.
    node.phase = 'enabled'
    seed(node, True)
    command = node.samples['command'][0]
    command.twist.linear.x = command.twist.linear.y = 2.
    command.twist.angular.z = node.max_velocity_angular
    node._tick()
    velocity = node.velocity_pub.publish.call_args.args[0].twist
    cap = min(limit, .3)
    assert math.hypot(velocity.linear.x, velocity.linear.y) == pytest.approx(cap)
    assert velocity.angular.z == pytest.approx(node.max_velocity_angular * cap / math.hypot(2., 2.))


@pytest.mark.parametrize('speed', [-2., 2.])
def test_manual_only_caps_final_wheels_from_a_fast_robot_profile(node, speed):
    node.max_velocity_linear = node.max_traction_velocity = 2.
    node.phase = 'enabled'
    joints = seed(node, True)
    node._tick()
    joints.header = copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
    joints.velocity = [.2, speed]
    node._candidate(joints)
    result = node.command_pub.publish.call_args.args[0].modules[0]
    assert result.traction_velocity == pytest.approx(math.copysign(.3, speed))
    assert result.steering_profile_velocity == .2


@pytest.mark.parametrize('fault', ['uncorrelated', 'stale', 'nan', 'limit', 'duplicate', 'generation'])
def test_invalid_candidates_cannot_move(node, fault):
    node.phase = 'enabled'
    joints = seed(node, True)
    node._tick()
    joints.header = copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
    if fault == 'uncorrelated':
        joints.header.stamp.nanosec += 1
    elif fault == 'stale':
        node.test_now += .26
    elif fault == 'nan':
        joints.velocity[0] = float('nan')
    elif fault == 'limit':
        joints.velocity[1] = 100.
    elif fault == 'duplicate':
        joints.name[1] = joints.name[0]
    else:
        node._new_generation()
    node._candidate(joints)
    node.command_pub.publish.assert_not_called()


def test_enable_rejects_non_neutral_and_missing_native_status(node):
    seed(node)
    node.samples['operator'][0].controls_neutral = False
    assert not node._enable(None, Trigger.Response()).success
    node.samples['operator'][0].controls_neutral = True
    node.samples['status'][0].modules[0].traction.device_state_known = False
    assert not node._enable(None, Trigger.Response()).success


def test_recovery_neutral_and_timeout(node):
    seed(node)
    assert node._enable(None, Trigger.Response()).success
    node.samples['operator'][0].controls_neutral = False
    node._tick()
    assert node.phase == 'recovering'
    assert node.velocity_pub.publish.call_args.args[0].twist.linear.x == 0.0
    node.recovery_started -= node.reset_timeout
    node.last_ros_tick -= 1
    node._tick()
    assert node.phase == 'disabled'


def test_replayed_input_does_not_extend_lifetime(node):
    seed(node, True)
    original = node.samples['operator'][1]
    node.test_now += .1
    node._receive('operator', node.samples['operator'][0])
    assert node.samples['operator'][1] == original
    node.test_now += .2
    assert node._fresh('operator') is None


def test_homing_diagnostic_channel_only_applies_to_active_recovery(node):
    seed(node)
    normal=node.samples['operator'][0]
    normal.source_id='remdevice'
    normal.connected=False
    normal.controls_neutral=False
    h=OperatorState()
    h.header=copy.deepcopy(normal.header)
    h.source_id='remdevice_homing'
    h.connected=h.controls_neutral=h.estop_state_known=True
    node._receive('homing',h)
    assert not node._operator(neutral=True)  # cannot accept initial B2 this way
    node.phase='recovering'
    assert node._operator(neutral=True)
    normal.estop_pressed=True
    assert not node._operator(neutral=True)
    normal.estop_pressed=False
    node.phase='enabled'
    assert not node._operator()  # never grants joystick motion


def test_final_manual_command_keeps_traction_running_while_steering(node):
    import math
    from mobotic_kinematics.manual_steering import ManualSteering
    node.steering_policy=ManualSteering({'wheels_x':[1.], 'wheels_y':[.5],
        'steering_joint_names':['front_left_steering'], 'traction_joint_names':['front_left_traction'],
        'wheel_mount_angles':[0.], 'max_steering_velocity':.3})
    node.phase='enabled'
    joints=copy.deepcopy(seed(node,True))
    node.samples['command'][0].twist.angular.z=.1/1.5
    node._tick()
    joints.header=copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
    joints.position=[math.pi/4,0.]
    joints.velocity=[.3,.1]
    node._candidate(joints)
    final=node.command_pub.publish.call_args.args[0].modules[0]
    assert final.traction_velocity==.1 and final.steering_profile_velocity==.3
    assert final.steering_position==math.pi/4


def test_crab_boundary_releases_only_on_confirmed_operator_neutral(node):
    from mobotic_kinematics.manual_steering import ManualSteering
    node.steering_policy=ManualSteering({'wheels_x':[1.], 'wheels_y':[.5],
        'steering_joint_names':['front_left_steering'], 'traction_joint_names':['front_left_traction'],
        'wheel_mount_angles':[0.], 'max_steering_velocity':.3})
    node.phase='enabled'
    seed(node,True)
    node.steering_policy.crab_heading=-1.5707963267948966
    node.steering_policy.crab_blocked=True
    node.samples['operator'][0].controls_neutral=False
    node._tick()
    assert node.steering_policy.crab_blocked
    node.samples['operator'][0].controls_neutral=True
    node.test_now+=.02
    node._tick()
    assert node.steering_policy.crab_blocked  # nonzero command cannot prove neutral
    node.samples['command'][0].twist.linear.x=0.0
    node.test_now+=.02
    node._tick()
    assert not node.steering_policy.crab_blocked
    assert node.steering_policy.crab_heading is None


def test_recovery_finishes_with_stick_held_without_authorizing_motion(node):
    seed(node)
    assert node._enable(None, Trigger.Response()).success
    for _ in range(20):
        node.test_now+=.02
        seed(node,True)
        node.samples['operator'][0].controls_neutral=False
        node._tick()
        assert not node.mode_pub.publish.call_args.args[0].motion_permitted
        assert node.velocity_pub.publish.call_args.args[0].twist.linear.x==0.0
        if node.phase=='enabled':
            break
    assert node.phase=='enabled'
    assert node._motion_source() is None
    node.command_pub.publish.assert_not_called()


@pytest.mark.parametrize('gap',[.12,.2])
def test_short_scheduling_gap_flushes_motion_without_latching_disable(node,gap):
    seed(node,True); node.phase='enabled'; node._tick()
    assert node.ledger
    node.test_now+=gap
    node._tick()
    assert node.phase=='enabled'
    assert node.operation_pub.publish.call_args.args[0].enable
    assert not node.mode_pub.publish.call_args.args[0].motion_permitted
    assert not node.ledger
    assert node.velocity_pub.publish.call_args.args[0].twist.linear.x==0.0
    tick(node)
    assert node._motion_source() is not None


@pytest.mark.parametrize('ros_offset',[0.,-.01])
def test_repeated_or_slightly_reversed_ros_clock_does_not_disable(node,ros_offset):
    seed(node,True); node.phase='enabled'; node._tick()
    node.test_now+=ros_offset
    with patch('mobotic_bringup.manual_drive_control.time.monotonic',return_value=100.02):
        node._tick()
        assert node.phase=='enabled'
        assert node.operation_pub.publish.call_args.args[0].enable
        assert not node.mode_pub.publish.call_args.args[0].motion_permitted
        assert not node.ledger


@pytest.mark.parametrize('fault',['stop','disconnected','stale'])
def test_recovery_still_aborts_for_operator_stop_or_loss(node,fault):
    seed(node)
    assert node._enable(None,Trigger.Response()).success
    if fault=='stop':
        node.samples['operator'][0].estop_pressed=True
    elif fault=='disconnected':
        node.samples['operator'][0].connected=False
    else:
        node.samples.pop('operator')
    node._tick()
    assert node.phase=='disabled'
    assert not node.operation_pub.publish.call_args.args[0].enable


def test_neutral_drift_is_corrected_without_disabling_steering(node):
    from mobotic_kinematics.manual_steering import ManualSteering
    node.max_steering_velocity = .3
    node.steering_policy = ManualSteering({
        'wheels_x': [.845], 'wheels_y': [.495],
        'steering_joint_names': node.steering_joint_names,
        'traction_joint_names': node.traction_joint_names,
        'wheel_mount_angles': [0.], 'max_steering_velocity': .3,
        'steering_return_home_on_neutral': True, 'steering_neutral_home_velocity': 1.2,
        'steering_position_tolerances': [math.radians(1.)],
        'steering_position_resolutions': [121.]})
    node.phase = 'enabled'
    generation = node.generation
    unit = 2*math.pi/121
    for i, position in enumerate([0., 0., unit, 0., -unit, 0.]*10):
        node.test_now = 100. + i*.02
        joints = seed(node, True)
        joints.position[0] = position
        node.samples['command'][0].twist.linear.x = 0.
        node._tick()
        candidate = copy.deepcopy(joints)
        candidate.header = copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
        node._candidate(candidate)
        output = node.command_pub.publish.call_args.args[0].modules[0]
        assert output.steering_position == 0.
        assert output.steering_profile_velocity == (1.2 if position else 0.)
        assert output.traction_velocity == 0.
        assert node.phase == 'enabled' and node.generation == generation
        assert node.operation_pub.publish.call_args.args[0].enable


def test_manual_only_neutral_homing_uses_faster_profile_with_zero_traction(node):
    from mobotic_kinematics.manual_steering import ManualSteering
    node.max_steering_velocity = .3
    node.steering_policy = ManualSteering({
        'wheels_x': [.845], 'wheels_y': [.495],
        'steering_joint_names': node.steering_joint_names,
        'traction_joint_names': node.traction_joint_names,
        'wheel_mount_angles': [0.], 'max_steering_velocity': .3,
        'steering_return_home_on_neutral': True, 'steering_neutral_home_velocity': 1.2})
    node.phase = 'enabled'
    for i in range(2):
        node.test_now = 100. + i*.1
        joints = seed(node, True)
        joints.position[0] = .8
        node.samples['command'][0].twist.linear.x = 0.
        node._tick()
        candidate = copy.deepcopy(joints)
        candidate.header = copy.deepcopy(node.velocity_pub.publish.call_args.args[0].header)
        candidate.velocity = [.3, 2.]  # Old candidate cannot restart traction.
        node._candidate(candidate)
        result = node.command_pub.publish.call_args.args[0].modules[0]
        assert result.traction_velocity == 0.
        assert result.steering_profile_velocity == (0. if i == 0 else 1.2)
        if i:
            assert result.steering_position == 0.
