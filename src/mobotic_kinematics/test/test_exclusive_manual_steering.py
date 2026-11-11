import math
from types import SimpleNamespace
import pytest
from mobotic_kinematics.manual_steering import ManualSteering


def geometry():
    names=['front_left','front_right','rear_left','rear_right']
    return {'wheels_x':[.845,.845,-.845,-.845], 'wheels_y':[.495,-.495,.495,-.495],
        'steering_joint_names':[n+'_steering' for n in names],
        'traction_joint_names':[n+'_traction' for n in names],
        'wheel_mount_angles':[0.,3.14152,0.,3.14152], 'max_steering_velocity':.3}


def sample(g, angles, speeds=None):
    return SimpleNamespace(name=g['steering_joint_names']+g['traction_joint_names'],
        position=list(angles)+[0.]*4, velocity=[.3]*4+(speeds or [0.]*4))


def model(g, body):
    vx,vy,yaw=body
    vectors=[(vx-yaw*y,vy+yaw*x) for x,y in zip(g['wheels_x'],g['wheels_y'])]
    angles=[math.atan2(y,x)-m for (x,y),m in zip(vectors,g['wheel_mount_angles'])]
    return sample(g,angles,[math.hypot(x,y) for x,y in vectors])


@pytest.mark.parametrize('body', [(1.,0.,.22827042), (-1.,0.,-.22827042), (2.,0.,-.7),
                                  (.1,0.,.02), (0.,1.,0.), (.6,-.8,0.), (0.,0.,.4), (0.,0.,0.)])
@pytest.mark.parametrize('limits', [(.3,.2265), (.15,.1), (2.,1.51), (.3,.01)])
def test_uniform_speed_caps_preserve_all_four_wheel_headings(body, limits):
    g = geometry()
    output = ManualSteering.cap_velocity(body, *limits)
    assert math.hypot(*output[:2]) <= limits[0] + 1e-12
    assert abs(output[2]) <= limits[1] + 1e-12
    assert model(g, output).position[:4] == pytest.approx(model(g, body).position[:4])
    if math.hypot(*body[:2]) <= limits[0] and abs(body[2]) <= limits[1]:
        assert output == body
    else:
        assert math.hypot(*output[:2]) == pytest.approx(limits[0]) or abs(output[2]) == pytest.approx(limits[1])


def test_left_alone_cannot_select_or_rotate():
    p=ManualSteering(geometry())
    assert p.command(0.,1.,0.,True,.1,.3,.5)==[0.,0.,0.]
    assert p.mode is None


def test_crab_wins_same_sample_and_ignores_left_stick():
    p=ManualSteering(geometry()); q=ManualSteering(geometry())
    a=p.command(1.,1.,1.,True,.1,.3,.5)
    b=q.command(1.,-1.,1.,True,.1,.3,.5)
    assert a==b and p.mode=='crab' and a[2]==0.


@pytest.mark.parametrize('limits_enabled',[True,False])
def test_ackermann_first_ignores_subsequent_crab(limits_enabled):
    g=geometry();g['steering_angle_limits_enabled']=limits_enabled
    p=ManualSteering(g);q=ManualSteering(g)
    p.command(1.,1.,0.,True,.1,.3,.5);q.command(1.,1.,0.,True,.1,.3,.5)
    assert p.command(1.,1.,1.,False,.1,.3,.5)==q.command(1.,1.,0.,False,.1,.3,.5)
    assert p.mode=='ackermann' and p.output[1]==0.


def test_mode_release_needs_neutral_owner_and_measured_standstill():
    p=ManualSteering(geometry());p.command(1.,1.,0.,True,.1,.3,.5)
    for _ in range(5):p.command(0.,0.,1.,False,.1,.3,.5)
    assert p.mode=='ackermann'
    p.command(0.,0.,1.,True,.1,.3,.5)
    assert p.mode=='crab'


@pytest.mark.parametrize('direction',[-1.,1.])
def test_sideways_alone_is_full_crab(direction):
    p=ManualSteering(geometry())
    for _ in range(10):body=p.command(0.,-direction,direction,True,.1,.3,.5)
    assert body==[0.,direction*.3,0.]
    assert abs(math.atan2(body[1],body[0]))==math.pi/2


@pytest.mark.parametrize('speed',[-1.,1.])
@pytest.mark.parametrize('steer',[-1.,1.])
def test_ackermann_full_throw_limits_every_wheel_to_45(speed,steer):
    g=geometry();p=ManualSteering(g)
    for _ in range(10):body=p.command(speed,steer,0.,True,.1,.3,.5)
    vx,_,yaw=body;k=yaw/vx
    angles=[math.atan2(k*x,1-k*y) for x,y in zip(g['wheels_x'],g['wheels_y'])]
    assert max(map(abs,angles))==pytest.approx(math.pi/4)
    assert angles[0]*angles[2]<0  # front/rear steer about the same centre
    assert abs(angles[0])!=pytest.approx(abs(angles[1]))  # inner/outer differ


def test_ackermann_left_only_after_releasing_speed_has_no_yaw():
    p=ManualSteering(geometry());p.command(1.,1.,0.,True,.1,.3,.5)
    assert p.command(0.,1.,0.,False,.1,.3,.5)[2]==0.


@pytest.mark.parametrize('body',[(.3,0.,.2),(-.3,0.,-.2),(0.,.3,0.),(0.,-.3,0.)])
def test_traction_runs_while_steering_and_mounting_signs_stay_correct(body):
    g=geometry();p=ManualSteering(g);body=p.project(body)
    candidate=model(g,body);feedback=sample(g,[0.]*4)
    output=p.align(candidate,feedback,body)
    assert any(abs(v)>.01 for v in output.velocity[4:])
    assert any(v>0 for v in output.velocity[:4])
    feedback.position[:4]=output.position[:4]
    aligned=p.align(candidate,feedback,body)
    assert any(abs(v)>.01 for v in aligned.velocity[4:])
    for i,m in enumerate(g['wheel_mount_angles']):
        v=aligned.velocity[4+i];a=aligned.position[i]+m
        assert v*math.cos(a)==pytest.approx(body[0]-body[2]*g['wheels_y'][i],abs=3e-5)
        assert v*math.sin(a)==pytest.approx(body[1]+body[2]*g['wheels_x'][i],abs=3e-5)
    feedback.position[1]+=.2
    assert any(abs(v)>.01 for v in p.align(candidate,feedback,body).velocity[4:])


def test_steady_stop_holds_headings_without_turning():
    g=geometry();p=ManualSteering(g);c=sample(g,[1.]*4,[.1]*4)
    out=p.align(c,sample(g,[1.]*4),(0.,0.,0.))
    assert out.position==c.position and out.velocity==[0.]*8


def test_out_of_envelope_candidate_is_rejected_not_clipped():
    g=geometry();p=ManualSteering(g)
    with pytest.raises(ValueError,match='envelope'):
        p.align(model(g,(.1,0.,1.)),sample(g,[0.]*4),(.1,0.,.01))


def test_direction_does_not_accumulate_extra_turns():
    g=geometry();p=ManualSteering(g)
    feedback=sample(g,[2*math.pi]*4)
    for direction in (1.,-1.,1.,-1.):
        body=(0.,direction*.3,0.)
        out=p.align(model(g,body),feedback,body)
        for target in out.position[:4]: assert abs(target-2*math.pi)<math.pi/2+.0001
        feedback.position=out.position[:]


@pytest.mark.parametrize('left,side,expected',[(1.,0.,'ackermann'),(0.,1.,'crab'),(1.,1.,'crab')])
def test_straight_travel_does_not_reserve_steering(left,side,expected):
    p=ManualSteering(geometry())
    assert p.command(1.,0.,0.,True,.1,.3,.5)[0]>0
    assert p.mode is None
    p.command(1.,left,side,False,.1,.3,.5)
    assert p.mode==expected


@pytest.mark.parametrize('requested,zero_index', [((0.,.1,0.),2),((.1,0.,.01),1)])
def test_downstream_ramp_cannot_mix_old_and_new_families(requested,zero_index):
    p=ManualSteering(geometry())
    output=p.project((.01,.01,.01),requested)
    assert output[zero_index]==0.


def test_home_reference_cannot_follow_an_overshooting_wheel():
    g=geometry();p=ManualSteering(g);body=(0.,.3,0.)
    feedback=sample(g,[0.]*4)
    p.align(model(g,body),feedback,body)
    feedback.position[:4]=[math.radians(220)]*4
    output=p.align(model(g,body),feedback,body)
    assert all(-math.pi/2<=x<=math.pi/2 for x in output.position[:4])
    assert p.home_references==dict.fromkeys(g['steering_joint_names'],0.)


@pytest.mark.parametrize('direction',[-1.,1.])
def test_rounded_mounts_cannot_exceed_numeric_90_degree_command(direction):
    g=geometry();p=ManualSteering(g);body=(0.,direction*.3,0.)
    output=p.align(model(g,body),sample(g,[0.]*4),body)
    assert all(-math.pi/2<=x<=math.pi/2 for x in output.position[:4])
    assert all(abs(v)>.01 for v in output.velocity[4:])


def test_reference_is_renewed_only_by_explicit_epoch_reset():
    g=geometry();p=ManualSteering(g);body=(0.,.3,0.)
    p.align(model(g,body),sample(g,[0.]*4),body)
    feedback=sample(g,[2*math.pi]*4)
    p.align(model(g,body),feedback,body)
    assert set(p.home_references.values())=={0.}
    p.reset_alignment()
    p.align(model(g,body),feedback,body)
    assert set(p.home_references.values())=={2*math.pi}


def test_profile_stops_inside_encoder_tolerance_without_stopping_traction():
    g=geometry();p=ManualSteering(g);body=(0.,.3,0.)
    candidate=model(g,body);feedback=sample(g,[0.]*4)
    output=p.align(candidate,feedback,body)
    feedback.position[:4]=[x-.001 for x in output.position[:4]]
    near=p.align(candidate,feedback,body)
    assert near.velocity[:4]==[0.]*4
    assert all(abs(v)>.01 for v in near.velocity[4:])


@pytest.mark.parametrize('direction', [-1., 1.])
@pytest.mark.parametrize('family', ['crab', 'ackermann'])
def test_held_full_stick_stops_each_steering_at_its_bounded_target(direction, family):
    g=geometry(); p=ManualSteering(g)
    body=(0., direction*.3, 0.) if family=='crab' else (.3, 0., direction*.3*p.curvature_limit)
    candidate=model(g,body); feedback=sample(g,[0.]*4)
    first=p.align(candidate,feedback,body)
    limit=math.pi/2 if family=='crab' else math.pi/4
    for _ in range(100):
        # Simulate feedback rounded to the configured 4096-count encoder.
        feedback.position[:4]=[round(a*4096/(2*math.pi))*2*math.pi/4096
                                for a in first.position[:4]]
        output=p.align(candidate,feedback,body)
        assert output.position==first.position
        assert all(abs(a)<=limit for a in output.position[:4])
        assert output.velocity[:4]==[0.]*4
        assert output.velocity[4:]==first.velocity[4:]


@pytest.mark.parametrize('direction', [-1., 1.])
@pytest.mark.parametrize('home_turns', [-2, 0, 3])
@pytest.mark.parametrize('position_units', [121, 4096])
def test_held_crab_reaches_native_target_and_stays_stopped(direction, home_turns, position_units):
    g = geometry()
    unit_angle = 2*math.pi/position_units
    g['steering_position_tolerances'] = [unit_angle]*4
    policy = ManualSteering(g)
    body = (0., direction*.3, 0.)
    candidate = model(g, body)
    native_positions = [float(home_turns*position_units)]*4
    feedback = sample(g, [home_turns*2*math.pi]*4)
    first = policy.align(candidate, feedback, body)
    native_targets = [round(angle/unit_angle) for angle in first.position[:4]]
    if position_units == 121:
        assert native_targets == [home_turns*121 + direction*30]*4

    stopped_at = None
    for tick in range(1000):
        # Model the controller in its native coordinates; feedback is integer
        # user units, independently of the floating-point requested angle.
        output = policy.align(candidate, feedback, body)
        assert output.position == first.position
        for i, target in enumerate(native_targets):
            step = output.velocity[i]*.02/unit_angle
            native_positions[i] += max(-step, min(step, target-native_positions[i]))
        feedback.position[:4] = [round(value)*unit_angle for value in native_positions]
        if output.velocity[:4] == [0.]*4:
            stopped_at = tick if stopped_at is None else stopped_at
        elif stopped_at is not None:
            pytest.fail('unchanged held input restarted a completed steering move')

    assert stopped_at is not None and stopped_at < 500
    assert all(abs(target-position) <= 1 for target, position in zip(native_targets, native_positions))
    assert output.velocity[4:] == first.velocity[4:]
    neutral = policy.align(candidate, feedback, (0., 0., 0.))
    assert neutral.position[:4] == feedback.position[:4]
    assert neutral.velocity == [0.]*8


@pytest.mark.parametrize('direction', [-1., 1.])
@pytest.mark.parametrize('family', ['crab', 'ackermann'])
def test_held_limit_corrects_overshoot_and_allows_inward_input(direction, family):
    g=geometry(); p=ManualSteering(g)
    body=(0., direction*.3, 0.) if family=='crab' else (.3, 0., direction*.3*p.curvature_limit)
    candidate=model(g,body); feedback=sample(g,[0.]*4)
    first=p.align(candidate,feedback,body)
    limit=math.pi/2 if family=='crab' else math.pi/4
    feedback.position[:4]=first.position[:4]
    endpoints=[]
    for i,target in enumerate(first.position[:4]):
        if abs(abs(target)-limit)<1e-4:
            endpoints.append(i)
            feedback.position[i]+=math.copysign(.01,target)
    assert endpoints
    corrected=p.align(candidate,feedback,body)
    assert corrected.position==first.position
    assert corrected.velocity[4:]==first.velocity[4:]
    for i in endpoints:
        assert corrected.velocity[i]>0.
    inward=(.3, direction*.1, 0.) if family=='crab' else (.3, 0., 0.)
    moving=p.align(model(g,inward),feedback,inward)
    for i in endpoints:
        assert moving.velocity[i]>0.
        assert abs(moving.position[i])<abs(first.position[i])


@pytest.mark.parametrize('limits_enabled', [True, False])
@pytest.mark.parametrize('family', ['ackermann', 'crab'])
@pytest.mark.parametrize('sign', [-1., 1.])
def test_one_degree_corrects_native_drift_without_accumulating_target(limits_enabled, family, sign):
    g = geometry()
    g.update(steering_angle_limits_enabled=limits_enabled,
             steering_position_tolerances=[math.radians(1.)]*4,
             steering_position_resolutions=[121.]*4)
    p = ManualSteering(g)
    body = (.3, 0., sign*.3*p.curvature_limit) if family == 'ackermann' else (
        .3*math.cos(math.radians(16)), sign*.3*math.sin(math.radians(16)), 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    target = p.align(candidate, feedback, body)
    unit = 2*math.pi/121
    native = [math.copysign(math.floor(abs(v/unit)+.5), v)*unit for v in target.position[:4]]
    for _ in range(100):
        feedback.position[:4] = native
        settled = p.align(candidate, feedback, body)
        assert settled.position == target.position
        assert settled.velocity[:4] == [0.]*4
        # Even one reported count is larger than the new one-degree threshold.
        # Correct every wheel, even beyond its old endpoint, and retain traction.
        feedback.position[:4] = [v + sign*unit for v in native]
        correcting = p.align(candidate, feedback, body)
        assert correcting.position == target.position
        assert correcting.velocity[:4] == [.3]*4
        assert correcting.velocity[4:] == target.velocity[4:]


def test_one_degree_boundary_and_reachable_half_count_target():
    g = geometry()
    g['steering_position_tolerances'] = [math.radians(1.)]*4
    p = ManualSteering(g)
    body = (.3, .1, 0.)
    candidate = model(g, body)
    feedback = sample(g, [0.]*4)
    target = p.align(candidate, feedback, body)
    feedback.position[:4] = [v + math.radians(1.) for v in target.position[:4]]
    assert p.align(candidate, feedback, body).velocity[:4] == [0.]*4
    feedback.position[:4] = [v + math.radians(1.01) for v in target.position[:4]]
    assert p.align(candidate, feedback, body).velocity[:4] == [.3]*4
    g['steering_position_resolutions'] = [121.]*4
    p = ManualSteering(g)
    unit = 2*math.pi/121
    # Native +/-.5 count rounds away from zero, as on the CAN wire.
    for sign in (-1., 1.):
        assert p._position_error(0, sign*.5*unit, sign*unit) == 0.
        assert p._position_error(0, sign*.5*unit, 0.) > math.radians(1.)
        # Adding .5 before floor would incorrectly round the closest float
        # below a half count upward, disagreeing with std::round on the wire.
        below = sign*math.nextafter(unit/2, 0.)
        above = sign*math.nextafter(unit/2, math.inf)
        assert p._position_error(0, below, 0.) == 0.
        assert p._position_error(0, above, sign*unit) == 0.


@pytest.mark.parametrize('bad', [[121.], [0.]*4, [121.5]*4, [math.nan]*4, [True]*4])
def test_invalid_native_position_resolutions_rejected(bad):
    g = geometry(); g['steering_position_resolutions'] = bad
    with pytest.raises(ValueError, match='resolution'):
        ManualSteering(g)


def test_encoder_tolerance_is_individual_to_each_wheel():
    g=geometry(); g['steering_position_tolerances']=[.002,.0001,.002,.0001]
    p=ManualSteering(g); body=(.3,.1,0.)
    candidate=model(g,body); feedback=sample(g,[0.]*4)
    first=p.align(candidate,feedback,body)
    feedback.position[:4]=[a-.001 for a in first.position[:4]]
    output=p.align(candidate,feedback,body)
    assert output.velocity[0]==output.velocity[2]==0.
    assert output.velocity[1]>0. and output.velocity[3]>0.


@pytest.mark.parametrize('tolerances', [[.001], [0.]*4, [-.001]*4, [math.nan]*4, [math.inf]*4])
def test_invalid_encoder_tolerances_are_rejected(tolerances):
    g=geometry(); g['steering_position_tolerances']=tolerances
    with pytest.raises(ValueError,match='tolerance'):
        ManualSteering(g)


@pytest.mark.parametrize('direction', [-1., 1.])
def test_crab_reversal_keeps_all_four_targets_and_reverses_traction(direction):
    g=geometry(); p=ManualSteering(g); feedback=sample(g,[0.]*4)
    body=(0.,direction*.3,0.)
    first=p.align(model(g,body),feedback,body)
    for _ in range(50):
        body=(0.,-body[1],0.)
        # Deliberately leave measured positions behind the target: continuity
        # follows the previous setpoint, not individual motor lag.
        output=p.align(model(g,body),feedback,body)
        assert output.position==pytest.approx(first.position)
        for i,mount in enumerate(g['wheel_mount_angles']):
            assert output.velocity[4+i]*math.sin(output.position[i]+mount)==pytest.approx(body[1],abs=3e-5)
            assert abs(output.position[i])<=math.pi/2
        assert output.velocity[:4]==[g['max_steering_velocity']]*4


def test_previous_setpoints_survive_wrapping_feedback_lag_and_joint_reordering():
    g=geometry(); p=ManualSteering(g); feedback=sample(g,[0.]*4)
    first=p.align(model(g,(0.,.3,0.)),feedback,(0.,.3,0.))
    candidate=model(g,(0.,-.3,0.))
    candidate.position[:4]=[a+8*math.pi for a in candidate.position[:4]]
    candidate.name.reverse(); candidate.position.reverse(); candidate.velocity.reverse()
    feedback.name.reverse(); feedback.position.reverse(); feedback.velocity.reverse()
    output=p.align(candidate,feedback,(0.,-.3,0.))
    ids={name:i for i,name in enumerate(output.name)}
    assert [output.position[ids[n]] for n in g['steering_joint_names']]==pytest.approx(first.position[:4])


def test_all_moving_motors_share_the_example_profile_speed():
    g=geometry(); p=ManualSteering(g); body=p.project((.3,0.,.2))
    feedback=sample(g,[0.]*4); candidate=model(g,body)
    first=p.align(candidate,feedback,body)
    feedback.position[0]=first.position[0]-.02
    output=p.align(candidate,feedback,body)
    assert output.velocity[:4]==[g['max_steering_velocity']]*4
    feedback.position[:4]=first.position[:4]
    stopped=p.align(candidate,feedback,body)
    assert stopped.velocity[:4]==[0.]*4
    assert stopped.velocity[4:]==first.velocity[4:]


@pytest.mark.parametrize('direction', [-1., 1.])
def test_direct_sideways_and_gradual_approach_choose_one_shared_axis(direction):
    g=geometry(); direct=ManualSteering(g); gradual=ManualSteering(g)
    feedback=sample(g,[0.]*4)
    body=(0.,direction*.3,0.)
    initial=direct.align(model(g,body),feedback,body)
    for degree in range(0,91,3):
        angle=math.radians(degree)*direction
        vector=(.3*math.cos(angle),.3*math.sin(angle),0.)
        approached=gradual.align(model(g,vector),feedback,vector)
    assert initial.position==pytest.approx(approached.position,abs=1e-4)
    assert max(initial.position[:4])-min(initial.position[:4])<1e-4
    assert initial.position[0]*direction>0


@pytest.mark.parametrize('direction', [-1., 1.])
def test_sweep_past_sideways_stays_bounded_and_straight_reverse_returns_inward(direction):
    g=geometry(); p=ManualSteering(g); feedback=sample(g,[0.]*4)
    previous=None
    for degree in range(1,180):
        angle=direction*math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        output=p.align(model(g,body),feedback,body)
        targets=output.position[:4]
        if previous is not None:
            assert max(abs(a-b) for a,b in zip(targets,previous))<=math.radians(1.01)
        assert max(targets)-min(targets)<1e-4
        if degree>90:
            assert targets[0]==pytest.approx(direction*math.pi/2)
            assert output.velocity[4:]==[0.]*4
        previous=targets
    # Straight reverse is a valid 0-degree rolling axis, even after this
    # outward sweep. It must not require a neutral command to be accepted.
    reverse=(-.3,0.,0.)
    resumed=p.align(model(g,reverse),feedback,reverse)
    assert max(map(abs,resumed.position[:4]))<1e-4
    assert not p.crab_blocked
    assert all(abs(v)>.01 for v in resumed.velocity[4:])
    angle=direction*math.radians(80)
    body=(.3*math.cos(angle),.3*math.sin(angle),0.)
    resumed=p.align(model(g,body),feedback,body)
    assert all(abs(v)>.01 for v in resumed.velocity[4:])
    assert abs(resumed.position[0])<math.pi/2
    stop=p.align(model(g,(0.,0.,0.)),feedback,(0.,0.,0.))
    assert stop.position[:4]==feedback.position[:4]
    p.release_crab_boundary()  # confirmed physical neutral
    reverse=(-.3,0.,0.)
    resumed=p.align(model(g,reverse),feedback,reverse)
    assert all(abs(v)>.01 for v in resumed.velocity[4:])
    assert max(map(abs,resumed.position[:4]))<1e-4


def test_zero_command_during_interlock_cannot_release_crab_boundary():
    g=geometry(); p=ManualSteering(g); feedback=sample(g,[0.]*4)
    for degree in (30,60,90,110):
        angle=-math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        p.align(model(g,body),feedback,body)
    assert p.crab_blocked
    p.align(model(g,(0.,0.,0.)),feedback,(0.,0.,0.))
    body=(-.2,-.2,0.)
    output=p.align(model(g,body),feedback,body)
    assert output.velocity[4:]==[0.]*4
    assert output.position[0]==-math.pi/2
    assert p.crab_blocked


@pytest.mark.parametrize('direction', [-1.,1.])
@pytest.mark.parametrize('speed', [-.3,.3])
@pytest.mark.parametrize('measured_angle', [.2,1.55])
def test_blocked_crab_accepts_straight_target_before_wheels_reach_old_limit(direction,speed,measured_angle):
    g=geometry();p=ManualSteering(g)
    homes=[0.,0.,2*math.pi,0.]
    feedback=sample(g,homes)
    for body in ((0.,direction*.3,0.),(-.03,direction*.3,0.)):
        blocked=p.align(model(g,body),feedback,body)
    assert p.crab_blocked and blocked.velocity[4:]==[0.]*4
    # The actual bag changes direction while the wheels are still travelling
    # toward the old endpoint. Keep feedback away from both old and new goals.
    feedback.position[:4]=[home+direction*measured_angle for home in homes]
    body=(speed,0.,0.)
    output=p.align(model(g,body),feedback,body)
    assert not p.crab_blocked
    for i,(home,mount) in enumerate(zip(homes,g['wheel_mount_angles'])):
        assert output.position[i]-home==pytest.approx(0.,abs=1e-4)
        assert output.velocity[i]==g['max_steering_velocity']
        assert output.velocity[4+i]*math.cos(output.position[i]+mount)==pytest.approx(speed,abs=3e-5)
    targets=output.position[:4]
    feedback.position[:4]=targets
    completed=p.align(model(g,body),feedback,body)
    assert completed.position[:4]==pytest.approx(targets)
    assert completed.velocity[:4]==[0.]*4
    assert all(abs(v)>.01 for v in completed.velocity[4:])


@pytest.mark.parametrize('direction', [-1.,1.])
def test_blocked_crab_accepts_inward_reverse_diagonal_without_neutral(direction):
    g=geometry();p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for body in ((0.,direction*.3,0.),(-.03,direction*.3,0.)):
        p.align(model(g,body),feedback,body)
    assert p.crab_blocked
    # Reverse with opposite lateral input asks for an interior heading on
    # the same side of zero; only its forward velocity is negative.
    body=(-.3,-direction*.1,0.)
    output=p.align(model(g,body),feedback,body)
    assert not p.crab_blocked
    assert output.position[0]==pytest.approx(direction*math.atan(1/3))
    for i,mount in enumerate(g['wheel_mount_angles']):
        assert abs(output.position[i])<math.pi/2
        assert output.velocity[4+i]*math.cos(output.position[i]+mount)==pytest.approx(body[0],abs=3e-5)
        assert output.velocity[4+i]*math.sin(output.position[i]+mount)==pytest.approx(body[1],abs=3e-5)


def test_interlock_hold_preserves_boundary_but_does_not_block_a_new_inward_request():
    g=geometry();p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for body in ((0.,-.3,0.),(-.03,-.3,0.),(0.,0.,0.)):
        p.align(model(g,body),feedback,body)
    assert p.crab_blocked and p.crab_heading==-math.pi/2
    body=(-.2,.2,0.)
    output=p.align(model(g,body),feedback,body)
    assert not p.crab_blocked
    assert output.position[0]==pytest.approx(-math.pi/4)
    assert all(abs(v)>.01 for v in output.velocity[4:])


@pytest.mark.parametrize('direction', [-1.,1.])
@pytest.mark.parametrize('home_turns', [-2,0,3])
def test_180_degree_crab_sweep_crosses_sideways_and_stays_in_fixed_home_range(direction,home_turns):
    g=geometry();g['max_crab_angle']=math.pi
    p=ManualSteering(g);home=home_turns*2*math.pi
    feedback=sample(g,[home]*4)
    previous=None
    for degree in range(1,181):
        angle=direction*math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        output=p.align(model(g,body),feedback,body)
        relative=[target-home for target in output.position[:4]]
        assert relative[0]==pytest.approx(angle)
        assert all(abs(target)<=math.pi for target in relative)
        assert not p.crab_blocked
        if previous is not None:
            assert max(abs(a-b) for a,b in zip(relative,previous))<math.radians(1.01)
        for i,mount in enumerate(g['wheel_mount_angles']):
            assert output.velocity[4+i]*math.cos(output.position[i]+mount)==pytest.approx(body[0],abs=3e-5)
            assert output.velocity[4+i]*math.sin(output.position[i]+mount)==pytest.approx(body[1],abs=3e-5)
        previous=relative
    feedback.position[:4]=output.position[:4]
    held=p.align(model(g,body),feedback,body)
    assert held.velocity[:4]==[0.]*4
    for degree in range(181,221):
        angle=direction*math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        blocked=p.align(model(g,body),feedback,body)
        assert blocked.position[:4]==pytest.approx(held.position[:4])
        assert blocked.velocity[4:]==[0.]*4
        assert p.crab_blocked
    angle=direction*math.radians(170)
    body=(.3*math.cos(angle),.3*math.sin(angle),0.)
    inward=p.align(model(g,body),feedback,body)
    assert inward.position[0]-home==pytest.approx(angle)
    assert not p.crab_blocked
    assert all(abs(v)>.01 for v in inward.velocity[4:])


@pytest.mark.parametrize('direction', [-1.,1.])
@pytest.mark.parametrize('limits_enabled',[True,False])
def test_180_range_straight_reverse_tie_at_sideways_still_returns_to_zero(direction,limits_enabled):
    g=geometry();g['max_crab_angle']=math.pi;g['steering_angle_limits_enabled']=limits_enabled
    p=ManualSteering(g);feedback=sample(g,[0.]*4)
    body=(0.,direction*.3,0.)
    p.align(model(g,body),feedback,body)
    body=(-.3,0.,0.)
    output=p.align(model(g,body),feedback,body)
    assert max(map(abs,output.position[:4]))<1e-4
    assert not p.crab_blocked


def test_180_range_initial_heading_keeps_existing_large_crab_angle():
    g=geometry();g['max_crab_angle']=math.pi
    p=ManualSteering(g);angle=math.radians(150)
    body=(.3*math.cos(angle),.3*math.sin(angle),0.)
    output=p.align(model(g,body),sample(g,[angle]*4),body)
    assert output.position[0]==pytest.approx(angle)


@pytest.mark.parametrize('angle', [0.,math.pi/4,math.pi+.001,math.nan,math.inf])
def test_invalid_crab_envelope_is_rejected(angle):
    g=geometry();g['max_crab_angle']=angle
    with pytest.raises(ValueError,match='max_crab_angle'):
        ManualSteering(g)


@pytest.mark.parametrize('direction',[-1.,1.])
@pytest.mark.parametrize('home_turns',[-2,0,3])
def test_unbounded_crab_keeps_continuity_through_multiple_turns(direction,home_turns):
    g=geometry();g['steering_angle_limits_enabled']=False
    p=ManualSteering(g);home=home_turns*2*math.pi
    feedback=sample(g,[home]*4);previous=None
    for degree in range(1,1081):
        angle=direction*math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        output=p.align(model(g,body),feedback,body)
        relative=[target-home for target in output.position[:4]]
        assert relative[0]==pytest.approx(angle)
        assert max(relative)-min(relative)<1e-4
        assert not p.crab_blocked
        if previous is not None:
            assert max(abs(a-b) for a,b in zip(relative,previous))<math.radians(1.01)
        for i,mount in enumerate(g['wheel_mount_angles']):
            assert output.velocity[4+i]*math.cos(output.position[i]+mount)==pytest.approx(body[0],abs=3e-5)
            assert output.velocity[4+i]*math.sin(output.position[i]+mount)==pytest.approx(body[1],abs=3e-5)
        previous=relative
    assert abs(output.position[0]-home)>5*math.pi


def test_unbounded_crab_neutral_does_not_unwind_completed_turns():
    g=geometry();g['steering_angle_limits_enabled']=False
    p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for degree in range(0,766,15):
        angle=math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        output=p.align(model(g,body),feedback,body)
    targets=output.position[:4]
    feedback.position[:4]=targets
    held=p.align(model(g,body),feedback,(0.,0.,0.))
    assert held.position[:4]==targets and held.velocity==[0.]*8
    p.release_crab_boundary()
    resumed=p.align(model(g,body),feedback,body)
    assert resumed.position[:4]==pytest.approx(targets)
    assert resumed.velocity[:4]==[0.]*4
    reverse=tuple(-value for value in body)
    reversed_output=p.align(model(g,reverse),feedback,reverse)
    assert reversed_output.position[:4]==pytest.approx(targets)
    assert reversed_output.velocity[4:]==pytest.approx([-v for v in resumed.velocity[4:]])


def test_unbounded_ackermann_uses_previous_targets_instead_of_home_angle_caps():
    g=geometry();g['steering_angle_limits_enabled']=False
    p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for degree in range(0,271,10):
        angle=math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        previous=p.align(model(g,body),feedback,body)
    body=(.3,0.,.1)
    output=p.align(model(g,body),feedback,body)
    assert all(abs(target)>math.pi for target in output.position[:4])
    for i,mount in enumerate(g['wheel_mount_angles']):
        assert abs(output.position[i]-previous.position[i])<=math.pi/2+1e-4
        speed=output.velocity[4+i];angle=output.position[i]+mount
        assert speed*math.cos(angle)==pytest.approx(body[0]-body[2]*g['wheels_y'][i],abs=3e-5)
        assert speed*math.sin(angle)==pytest.approx(body[1]+body[2]*g['wheels_x'][i],abs=3e-5)


def test_unbounded_native_endpoint_stays_stopped_without_clipping_target():
    g=geometry();g['steering_angle_limits_enabled']=False
    unit=2*math.pi/121
    g['steering_position_tolerances']=[unit]*4
    p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for degree in range(0,766,15):
        angle=math.radians(degree)
        body=(.3*math.cos(angle),.3*math.sin(angle),0.)
        first=p.align(model(g,body),feedback,body)
    targets=first.position[:4]
    feedback.position[:4]=[round(angle/unit)*unit for angle in targets]
    for _ in range(100):
        output=p.align(model(g,body),feedback,body)
        assert output.position[:4]==pytest.approx(targets)
        assert output.velocity[:4]==[0.]*4
        assert all(abs(v)>.01 for v in output.velocity[4:])


@pytest.mark.parametrize('value',[None,0,1,'false'])
def test_steering_angle_limit_switch_requires_boolean(value):
    g=geometry();g['steering_angle_limits_enabled']=value
    with pytest.raises(ValueError,match='steering_angle_limits_enabled'):
        ManualSteering(g)


@pytest.mark.parametrize('speed_limit',[1.,2.])
@pytest.mark.parametrize('controls',[(1.,1.,0.),(1.,0.,1.),(0.,0.,-1.)])
def test_neutral_cancels_full_speed_request_without_waiting_for_ramp(speed_limit,controls):
    p=ManualSteering(geometry())
    for _ in range(100):
        p.command(*controls,False,.05,speed_limit,.5)
    mode=p.mode
    assert math.hypot(*p.output[:2])>.9
    assert p.command(0.,0.,0.,False,.05,speed_limit,.5)==[0.,0.,0.]
    assert p.mode==mode  # Physical motion still locks the family.
    assert p.command(0.,0.,0.,True,.05,speed_limit,.5)==[0.,0.,0.]
    assert p.mode is None
    resumed=p.command(*controls,True,.05,speed_limit,.5)
    assert math.hypot(*resumed[:2])<=.025+1e-10


@pytest.mark.parametrize('sign',[-1.,1.])
@pytest.mark.parametrize('turns',[-3,0,4])
def test_unbounded_restart_near_half_turn_cannot_add_a_revolution(sign,turns):
    g=geometry();g['steering_angle_limits_enabled']=False
    unit=2*math.pi/121;g['steering_position_tolerances']=[unit]*4
    # Opposite sides of the half-turn rounding boundary; different encoders
    # have already accumulated different numbers of complete turns.
    angles=[2*math.pi*(turns+t)+sign*math.pi+error for t,error in
            zip((0,1,0,2),(-unit/2,unit/2,-unit/2,unit/2))]
    feedback=sample(g,angles);p=ManualSteering(g)
    p.align(model(g,(0.,0.,0.)),feedback,(0.,0.,0.))
    for degree in range(1,541):
        heading=sign*(math.pi+math.radians(degree))
        body=(.3*math.cos(heading),.3*math.sin(heading),0.)
        output=p.align(model(g,body),feedback,body)
        if degree==1:
            assert max(abs(a-b) for a,b in zip(output.position[:4],angles))<math.radians(5)
        else:
            assert max(abs(a-b) for a,b in zip(output.position[:4],previous))<math.radians(1.01)
        for i,mount in enumerate(g['wheel_mount_angles']):
            assert output.velocity[4+i]*math.cos(output.position[i]+mount)==pytest.approx(body[0],abs=3e-5)
            assert output.velocity[4+i]*math.sin(output.position[i]+mount)==pytest.approx(body[1],abs=3e-5)
        previous=output.position[:4]
    assert abs(previous[0]-angles[0])>2*math.pi  # Continuous travel remains unrestricted.


def test_neutral_rebases_cancelled_unbounded_target_to_measured_position():
    g=geometry();g['steering_angle_limits_enabled']=False
    p=ManualSteering(g);feedback=sample(g,[0.]*4)
    for degree in range(1,271):
        a=math.radians(degree);body=(.3*math.cos(a),.3*math.sin(a),0.)
        p.align(model(g,body),feedback,body)
    feedback.position[:4]=[math.radians(15)]*4  # Actual wheels lag the old target.
    held=p.align(model(g,(0.,0.,0.)),feedback,(0.,0.,0.))
    assert held.velocity==[0.]*8
    p.release_crab_boundary()
    body=(.3,0.,0.)
    resumed=p.align(model(g,body),feedback,body)
    assert max(abs(a-b) for a,b in zip(resumed.position[:4],feedback.position[:4]))<math.pi/2


@pytest.mark.parametrize('family', ['ackermann','crab'])
def test_releasing_motion_stick_stops_even_with_other_stick_held(family):
    p=ManualSteering(geometry())
    initial=(1.,1.,0.) if family=='ackermann' else (1.,0.,1.)
    for _ in range(100):
        p.command(*initial,False,.05,2.,.5)
    assert math.hypot(*p.output[:2])>1.9
    # Left steering alone must never keep propulsion active. The ignored
    # sideways stick cannot keep an Ackermann move active either.
    released=(0.,1.,1.) if family=='ackermann' else (0.,1.,0.)
    assert p.command(*released,False,.05,2.,.5)==[0.,0.,0.]
    assert p.mode==family
