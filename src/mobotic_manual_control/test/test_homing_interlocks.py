"""Live neutral/STOP evidence for homing survives diagnostic-only PAIL faults."""
from types import SimpleNamespace
import pytest
import test_remdevice_control as base
from mobotic_manual_control.remdevice_protocol import RemdeviceReceiver


@pytest.fixture
def fixture():
    f = base.RemdeviceControlTest(); f.setUp()
    f.node.homing_decoder = RemdeviceReceiver(9)
    f.node.homing_state_publisher = base.base.Publisher()
    previous = base.namespace.get('OperatorState')
    base.namespace['OperatorState'] = base.Message
    yield f
    if previous is None: base.namespace.pop('OperatorState')
    else: base.namespace['OperatorState'] = previous


def homing(f):
    f.node._publish_homing_state(f.now)
    return f.node.homing_state_publisher.messages[-1]


@pytest.mark.parametrize('code', [0x8110, 0x8150])
def test_pail_communication_diagnostic_preserves_homing_but_blocks_motion(fixture, code):
    f=fixture; f.pair()
    before=homing(f).header.stamp
    f.send(0x89, [code & 255, code >> 8, 0x11, 0, 0, 0, 0, 0])
    h=homing(f)
    assert h.connected and h.controls_neutral and h.estop_state_known and not h.estop_pressed
    assert h.header.stamp == before  # diagnostics cannot refresh physical freshness
    assert not f.node._healthy(f.now)
    f.send(0x89, [0]*8)
    assert homing(f).controls_neutral


@pytest.mark.parametrize('status', [0xa8, 0x08, 0x20, 0])
def test_stop_and_radio_loss_cancel_homing(fixture, status):
    f=fixture;f.pair();f.send(0x89,[0x50,0x81,0x11,0,0,0,0,0])
    f.pair(status=status)
    h=homing(f)
    assert not h.connected
    if status & 0x80: assert h.estop_pressed and h.estop_state_known


def test_actual_non_neutral_controls_still_abort(fixture):
    f=fixture;f.pair(speed=100)
    f.send(0x89,[0x50,0x81,0x11,0,0,0,0,0])
    h=homing(f)
    assert h.connected and not h.controls_neutral


def test_stale_pdos_are_never_kept_alive_by_diagnostics(fixture):
    f=fixture;f.pair();f.send(0x89,[0x50,0x81,0x11,0,0,0,0,0],dt=.3)
    assert not homing(f).connected and not homing(f).controls_neutral


def test_eeprom_or_unknown_fault_is_not_covered(fixture):
    f=fixture;f.pair();f.send(0x89,[0,0x60,1,0,0,0,0,0])
    assert not homing(f).connected


def test_can_error_frame_invalidates_homing_proof(fixture):
    f=fixture;f.pair()
    f.node._can_callback(SimpleNamespace(is_error=True))
    assert not homing(f).connected
