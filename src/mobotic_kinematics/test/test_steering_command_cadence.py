"""Exercise production Romb adapter callbacks without ROS nodes or CAN."""
import ast
import copy
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from mobotic_kinematics.romb.coordinates import RombContract

SOURCE=Path(__file__).resolve().parents[1]/'mobotic_kinematics'/'romb'/'adapter.py'
tree=ast.parse(SOURCE.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
cls.bases=[]
cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name!='__init__']
stamp=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='stamp_ns')
namespace={'copy':copy,'math':math,'time':time,'JointState':lambda:NS()}
exec(compile(ast.Module(body=[stamp,cls],type_ignores=[]),str(SOURCE),'exec'),namespace)
Adapter=namespace['RombKinematicsAdapter']


def message(stamp=100.):
    return NS(header=NS(stamp=NS(sec=int(stamp),nanosec=round(stamp%1*1e9)),frame_id='base_link'),
              name=['fl_steering','fr_steering','fl_traction','fr_traction'],
              position=[0.,0.,0.,0.],velocity=[0.,0.,.1,.1])


@pytest.fixture
def adapter():
    a=Adapter()
    a.contract=RombContract(['fl','fr'],['fl_steering','fr_steering'],
                           ['fl_traction','fr_traction'],[0.,math.pi],.3,.3)
    a.frame='base_link'; a.feedback_timeout=.3; a.command_timeout=.25
    a.last_feedback_stamp=a.last_command_stamp=a.last_output_stamp=0
    a.feedback=a.feedback_received=a.authority=None
    a.pending_feedback={}; a.pending_commands={}
    a.romb_feedback=Mock(); a.command_publisher=Mock()
    a.now=100.
    a._now=lambda:round(a.now*1e9)
    return a


def command(a,stamp=100.):
    msg=message(stamp)
    a.pending_commands[namespace['stamp_ns'](msg.header.stamp)]=time.monotonic()
    a._setpoints(msg)
    return msg


def feedback(a,stamp):
    a.now=stamp
    msg=message(stamp); msg.velocity=[0.]*4
    a._feedback(msg)


def test_fresh_batches_are_not_throttled_by_140ms_sdo_feedback(adapter):
    a=adapter
    feedback(a,100.)
    for offset in (0.,.02,.04,.06,.08,.10,.12,.14):
        a.now=100.+offset
        latest=command(a,a.now)
        output=a.command_publisher.publish.call_args.args[0]
        assert output.header.stamp==latest.header.stamp
        assert output.name==latest.name and len(output.position)==4
        assert output.velocity[:2]==[.3,.3]
        assert output.velocity[2:]==pytest.approx([.1,-.1])
    assert a.command_publisher.publish.call_count==8


def test_stale_feedback_does_not_allow_new_motion_batches(adapter):
    a=adapter
    feedback(a,100.)
    a.now=100.4
    command(a,100.4)
    a.command_publisher.publish.assert_not_called()


def test_authority_change_discards_pending_command(adapter):
    a=adapter
    feedback(a,100.)
    a.pending_commands[100_000_000_000]=time.monotonic()
    a._authority(NS(header=message().header,supervisor_instance='new',
                    authority_generation=2,reset_id=0))
    a._setpoints(message())
    a.command_publisher.publish.assert_not_called()
