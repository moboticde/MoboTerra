"""Real flock regressions; no SocketCAN access and no receiver reset."""
import fcntl
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = PROJECT_ROOT / 'scripts/start_ros2_four_micontrol.sh'
SOURCE=Path(__file__).with_name('clear_joystick_error.py')
spec=importlib.util.spec_from_file_location('pail_clear',SOURCE)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_standalone_lock_excludes_second_clear(tmp_path):
    path=tmp_path/'stack.lock'
    with module.acquire_stack_lock(lock_path=str(path)):
        with pytest.raises(SystemExit,match='Stop the native ROS stack'):
            module.acquire_stack_lock(lock_path=str(path))


def test_launcher_lock_is_accepted_by_child_and_stays_owned_by_parent(tmp_path):
    path=tmp_path/'stack.lock'
    with module.acquire_stack_lock(lock_path=str(path)) as parent:
        code="""
import importlib.util,sys
spec=importlib.util.spec_from_file_location('pail_clear',sys.argv[1])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with m.acquire_stack_lock(int(sys.argv[2]),sys.argv[3]):
    pass
"""
        result=subprocess.run([sys.executable,'-c',code,str(SOURCE),str(parent.fileno()),str(path)],
                              pass_fds=(parent.fileno(),),capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        with pytest.raises(SystemExit,match='Stop the native ROS stack'):
            module.acquire_stack_lock(lock_path=str(path))


def test_other_file_cannot_be_used_as_inherited_lock(tmp_path):
    expected=tmp_path/'stack.lock'; expected.touch()
    with (tmp_path/'other.lock').open('a') as wrong:
        with pytest.raises(SystemExit,match='not the native stack lock'):
            module.acquire_stack_lock(wrong.fileno(),str(expected))


def test_closed_inherited_descriptor_is_rejected(tmp_path):
    expected=tmp_path/'stack.lock'; expected.touch()
    with pytest.raises(SystemExit,match='Cannot acquire'):
        module.acquire_stack_lock(-1,str(expected))


@pytest.mark.parametrize('clear_result',[0,1])
def test_startup_hook_runs_clear_first_and_aborts_on_failed_verification(tmp_path,clear_result):
    launcher=LAUNCHER.read_text()
    # Run the actual launcher lock/header and clear hook with a fake receiver
    # utility. The preflight and ROS/CAN launch stages are not executed.
    header=launcher.split('# Acquire shared stack lock',1)[1].split('cd "$PROJECT_ROOT"',1)[0]
    header=header.replace('$PROJECT_ROOT/runtime/native-stack.lock',str(tmp_path/'stack.lock'))
    header='set -e\n# Acquire shared stack lock'+header
    hook=launcher.split('# One-time PAIL recovery,',1)[1].split('fastdds shm clean',1)[0]
    hook='# One-time PAIL recovery,'+hook
    fake=tmp_path/'clear.py'
    fake.write_text("import os,sys\nos.fstat(9)\nprint('CLEAR_ATTEMPT')\nraise SystemExit("+str(clear_result)+")\n")
    hook=hook.replace('$PROJECT_ROOT/tools/pi/clear_joystick_error.py',str(fake))
    result=subprocess.run(['bash','-c',header+'receiver_transmit_enabled=true\n'+hook+"\necho ROS_START_MARKER\n"],capture_output=True,text=True)
    assert 'CLEAR_ATTEMPT' in result.stdout
    if clear_result:
        assert result.returncode!=0
        assert 'ROS_START_MARKER' not in result.stdout
    else:
        assert result.returncode==0,result.stderr
        assert result.stdout.index('CLEAR_ATTEMPT')<result.stdout.index('ROS_START_MARKER')


def test_receive_only_startup_skips_clear_without_aborting_launch(tmp_path):
    launcher=LAUNCHER.read_text()
    hook='# One-time PAIL recovery,'+launcher.split('# One-time PAIL recovery,',1)[1].split('fastdds shm clean',1)[0]
    fake=tmp_path/'must_not_run.py'
    fake.write_text("raise SystemExit('RECEIVER_TX_ATTEMPTED')\n")
    hook=hook.replace('$PROJECT_ROOT/tools/pi/clear_joystick_error.py',str(fake))
    result=subprocess.run(['bash','-c','set -e\nreceiver_transmit_enabled=false\n'+hook+
                           '\necho ROS_START_MARKER\n'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert 'skipping receiver startup reads, reset and NMT start' in result.stdout
    assert 'ROS_START_MARKER' in result.stdout
    assert 'RECEIVER_TX_ATTEMPTED' not in result.stderr


def test_receive_only_standalone_clear_refuses_before_socket_or_lock(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(module,'Configuration',lambda *args,**kwargs: SimpleNamespace(
        parameters=lambda component:{'receiver_transmit_enabled':False}))
    monkeypatch.setattr(module,'acquire_stack_lock',lambda *args:pytest.fail('lock attempted'))
    monkeypatch.setattr(module.socket,'socket',lambda *args:pytest.fail('CAN socket attempted'))
    with pytest.raises(SystemExit,match='no CAN messages sent'):
        module.main([])


def test_explicit_one_time_clear_still_requires_exclusive_stopped_stack(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(module,'Configuration',lambda *args,**kwargs: SimpleNamespace(
        parameters=lambda component:{'receiver_transmit_enabled':False}))
    def refuse_running_stack(*args):
        raise SystemExit('Stop the native ROS stack')
    monkeypatch.setattr(module,'acquire_stack_lock',refuse_running_stack)
    monkeypatch.setattr(module.socket,'socket',lambda *args:pytest.fail('CAN socket attempted'))
    with pytest.raises(SystemExit,match='Stop the native ROS stack'):
        module.main(['--allow-receiver-transmit-once'])
