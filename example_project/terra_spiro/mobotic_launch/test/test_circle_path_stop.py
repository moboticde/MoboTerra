import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'circle_path_publisher.py'
SPEC = importlib.util.spec_from_file_location('circle_path_publisher', SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_stop_duration_is_non_negative_and_finite():
    assert MODULE.sanitize_stop_duration(3.5) == 3.5
    assert MODULE.sanitize_stop_duration(-1.0) == 0.0
    assert MODULE.sanitize_stop_duration(float('nan')) == 0.0


def test_timer_completion_restarts_path():
    events = []

    class FakeTimer:
        def cancel(self):
            events.append('cancel')

    node = SimpleNamespace(
        _stop_timer=FakeTimer(),
        _stop_restart_pending=('stop', 'next'),
        destroy_timer=lambda timer: events.append('destroy'),
        restart_loop_goal=lambda begin, end: events.append(('restart', begin, end)),
    )

    MODULE.AgvPathFetcherPublisher.stop_timer_callback(node)

    assert events[-1] == ('restart', 'stop', 'next')
    assert node._stop_restart_pending is None


def restart_node(stop_enabled, events):
    return SimpleNamespace(
        _last_active_end='cw_loop_entry_0',
        _current_goal_end=None,
        _stop_before_turn_enabled=stop_enabled,
        next_loop_target=lambda begin: 'ccw_loop_exit_-18000',
        is_stop_node=lambda begin: True,
        schedule_loop_restart_after_stop=lambda begin, end: events.append(
            ('stop', begin, end)
        ),
        restart_loop_goal=lambda begin, end: events.append(('restart', begin, end)),
        get_logger=lambda: SimpleNamespace(warn=lambda message: events.append(('warn', message))),
        send_initial_goal=lambda: events.append(('initial',)),
    )


def test_stop_before_turn_can_be_disabled():
    events = []
    MODULE.AgvPathFetcherPublisher.restart_loop_after_result(
        restart_node(False, events)
    )
    assert events == [('restart', 'cw_loop_entry_0', 'ccw_loop_exit_-18000')]


def test_enabled_stop_before_turn_schedules_stop():
    events = []
    MODULE.AgvPathFetcherPublisher.restart_loop_after_result(
        restart_node(True, events)
    )
    assert events == [('stop', 'cw_loop_entry_0', 'ccw_loop_exit_-18000')]


def active_segments_message():
    return SimpleNamespace(
        path=SimpleNamespace(
            segments=[
                SimpleNamespace(
                    begin=SimpleNamespace(id='center_0'),
                    end=SimpleNamespace(id='cw_loop_entry_0'),
                )
            ]
        )
    )


def append_node(stop_enabled, events):
    return SimpleNamespace(
        _last_appended='cw_loop_entry_0',
        _last_active_end=None,
        _stop_before_turn_enabled=stop_enabled,
        is_stop_node=lambda node_name: True,
        loop_nodes=lambda: ('cw_loop_entry_0', 'ccw_loop_exit_-18000'),
        append_path=lambda begin, end: events.append(('append', begin, end)),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: events.append(('info', message)),
            debug=lambda message: events.append(('debug', message)),
        ),
    )


def test_disabled_stop_before_turn_allows_path_preappend():
    events = []
    MODULE.AgvPathFetcherPublisher.cb_active_segments(
        append_node(False, events), active_segments_message()
    )
    assert ('append', 'cw_loop_entry_0', 'ccw_loop_exit_-18000') in events


def test_enabled_stop_before_turn_blocks_path_preappend():
    events = []
    MODULE.AgvPathFetcherPublisher.cb_active_segments(
        append_node(True, events), active_segments_message()
    )
    assert not any(event[0] == 'append' for event in events)
