"""Exercise the production TCP worker/lifecycle with ROS adapters and loopback TCP."""
import ast
import math
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).parents[1] / 'mobotic_safety' / 'flexisoft_tcp_bridge.py'


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class NodeAdapter:
    parameters = {}

    def __init__(self, name):
        self.values = {}
        self.destroyed = False
        self.logs = []

    def declare_parameter(self, name, value):
        self.values[name] = self.parameters.get(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_timer(self, period, callback, **kwargs):
        timer = SimpleNamespace(period=period, callback=callback, clock=kwargs['clock'], cancelled=False)
        timer.cancel = lambda: setattr(timer, 'cancelled', True)
        return timer

    def get_logger(self):
        return SimpleNamespace(**{level: self.logs.append for level in ('info', 'warning', 'error')})

    def destroy_node(self):
        self.destroyed = True
        return True


class ServerAdapter:
    def __init__(self, fail_at):
        self.fail_at = fail_at
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def setsockopt(self, *args):
        pass

    def bind(self, *args):
        if self.fail_at == 'bind':
            raise OSError('bind failed')

    def listen(self):
        if self.fail_at == 'listen':
            raise OSError('listen failed')

    def settimeout(self, *args):
        pass

    def accept(self):
        raise OSError('accept failed')


class ConnectionAdapter:
    def __init__(self, chunks):
        self.chunks = iter(chunks)

    def recv(self, size):
        value = next(self.chunks, b'')
        if isinstance(value, Exception):
            raise value
        return value


class TcpBridgeTest(unittest.TestCase):
    def setUp(self):
        NodeAdapter.parameters = {'bind_address': '127.0.0.1', 'accept_timeout': 0.05}
        self.ros_calls = []
        self.ros = SimpleNamespace(
            init=lambda **kwargs: self.ros_calls.append('init'),
            shutdown=lambda: self.ros_calls.append('shutdown'),
        )
        tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
        self.namespace = dict(
            math=math, socket=socket, threading=threading, rclpy=self.ros,
            Node=NodeAdapter, UInt8=SimpleNamespace, UInt8MultiArray=SimpleNamespace,
            Clock=lambda **kwargs: SimpleNamespace(**kwargs),
            ClockType=SimpleNamespace(STEADY_TIME='steady'),
        )
        exec(compile(ast.Module(body=[item for item in tree.body
                                     if not isinstance(item, (ast.Import, ast.ImportFrom))],
                                type_ignores=[]), str(SOURCE), 'exec'), self.namespace)
        self.bridge_class = self.namespace['FlexiSoftTcpBridge']
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            if not node.destroyed:
                node.destroy_node()

    def node(self):
        node = self.bridge_class()
        self.nodes.append(node)
        return node

    def bare_node(self):
        node = self.bridge_class.__new__(self.bridge_class)
        NodeAdapter.__init__(node, 'test')
        node.stop_event = threading.Event()
        node.telegram_size, node.status_byte_offset = 15, 14
        node.publisher, node.telegram_publisher = Publisher(), Publisher()
        return node

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        self.fail('TCP test condition did not complete within two seconds')

    def loopback_node(self):
        # Port zero is test-only: skip __init__ so the real listener can choose an
        # ephemeral port. Public production parameters still reject port zero.
        node = self.bare_node()
        node.bind_address, node.port, node.accept_timeout = '127.0.0.1', 0, 0.05
        node.server_socket = node.connection_socket = None
        node.worker_error, node.worker_failed = None, threading.Event()
        node.health_timer = SimpleNamespace(cancel=lambda: None)
        node.worker = threading.Thread(target=node._run_server, daemon=True)
        self.nodes.append(node)
        node.worker.start()
        self.wait_for(lambda: any('listening' in log for log in node.logs))
        return node, node.server_socket.getsockname()

    def test_bind_listen_and_accept_errors_are_fatal(self):
        for operation in ('bind', 'listen', 'accept'):
            with self.subTest(operation=operation):
                server = ServerAdapter(operation)
                with patch.object(socket, 'socket', return_value=server):
                    node = self.node()
                    node.worker.join(timeout=2.0)
                self.assertFalse(node.worker.is_alive())
                self.assertTrue(node.worker_failed.is_set())
                self.assertIn(operation, node.worker_error)
                self.assertTrue(server.closed)
                with self.assertRaisesRegex(RuntimeError, operation):
                    node._check_worker()

    def test_real_occupied_port_is_detected(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.bind(('127.0.0.1', 0))
            occupied.listen()
            NodeAdapter.parameters['port'] = occupied.getsockname()[1]
            node = self.node()
            node.worker.join(timeout=2.0)
        self.assertTrue(node.worker_failed.is_set())
        # Windows reports PermissionError; Linux usually reports OSError/EADDRINUSE.
        with self.assertRaisesRegex(RuntimeError, 'bridge failed'):
            node._check_worker()

    def test_new_process_instance_recovers_when_port_is_released(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as occupied:
            occupied.bind(('127.0.0.1', 0))
            occupied.listen()
            NodeAdapter.parameters['port'] = occupied.getsockname()[1]
            failed = self.node()
            failed.worker.join(timeout=2.0)
            self.assertTrue(failed.worker_failed.is_set())
        failed.destroy_node()
        replacement = self.node()
        self.wait_for(lambda: any('listening' in log for log in replacement.logs))
        self.assertEqual(replacement.publisher.messages, [])
        with socket.create_connection(('127.0.0.1', replacement.port), timeout=1.0) as client:
            client.sendall(bytes(range(15)))
            self.wait_for(lambda: len(replacement.publisher.messages) == 1)
        self.assertEqual(replacement.publisher.messages[0].data, 14)
        self.assertFalse(replacement.worker_failed.is_set())
        replacement._check_worker()

    def test_unexpected_worker_return_and_non_socket_exceptions_are_fatal(self):
        for error in (None, RuntimeError('publisher failed')):
            with self.subTest(error=error):
                with patch.object(self.bridge_class, '_serve', side_effect=error):
                    node = self.node()
                    node.worker.join(timeout=2.0)
                with self.assertRaises(RuntimeError):
                    node._check_worker()

    def test_missing_failure_signal_still_detects_dead_worker(self):
        with patch.object(self.bridge_class, '_serve'):
            node = self.node()
            node.worker.join(timeout=2.0)
        node.worker_failed.clear()
        node.worker_error = None
        with self.assertRaisesRegex(RuntimeError, 'not running'):
            node._check_worker()

    def test_health_timer_uses_steady_time(self):
        with patch.object(self.bridge_class, '_serve'):
            node = self.node()
            node.worker.join(timeout=2.0)
        self.assertEqual(node.health_timer.clock.clock_type, 'steady')
        self.assertEqual(node.health_timer.period, 0.1)

    def test_invalid_timeout_is_rejected_before_starting_worker(self):
        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value):
                NodeAdapter.parameters['accept_timeout'] = value
                with patch.object(threading, 'Thread') as worker:
                    with self.assertRaisesRegex(ValueError, 'finite and positive'):
                        self.bridge_class()
                    worker.assert_not_called()

    def test_telegram_fragmentation_batching_and_timeout(self):
        node = self.bare_node()
        first, second = bytes(range(15)), bytes([42] * 15)
        connection = ConnectionAdapter([first[:4], socket.timeout(), first[4:] + second, b''])
        node._read_connection(connection)
        self.assertEqual([msg.data for msg in node.publisher.messages], [14, 42])
        self.assertEqual([msg.data for msg in node.telegram_publisher.messages],
                         [list(first), list(second)])

    def test_disconnect_discards_incomplete_telegram(self):
        node = self.bare_node()
        node._read_connection(ConnectionAdapter([bytes([255] * 10), ConnectionResetError()]))
        self.assertEqual(node.publisher.messages, [])
        node._read_connection(ConnectionAdapter([bytes(range(15)), b'']))
        self.assertEqual([msg.data for msg in node.publisher.messages], [14])

    def test_loopback_disconnect_then_reconnect_keeps_listener_alive(self):
        node, address = self.loopback_node()
        with socket.create_connection(address, timeout=1.0) as client:
            client.sendall(bytes([255] * 10))
        self.wait_for(lambda: any('disconnected' in log for log in node.logs))
        with socket.create_connection(address, timeout=1.0) as client:
            client.sendall(bytes(range(15)))
            self.wait_for(lambda: len(node.publisher.messages) == 1)
        self.assertEqual(node.publisher.messages[0].data, 14)
        self.assertFalse(node.worker_failed.is_set())
        node._check_worker()

    def test_idle_accept_does_not_count_as_worker_failure(self):
        node, _ = self.loopback_node()
        time.sleep(0.12)  # More than two accept timeouts, on loopback only.
        node._check_worker()
        self.assertTrue(node.worker.is_alive())
        self.assertFalse(node.worker_failed.is_set())

    def test_publisher_failure_terminates_the_real_worker(self):
        node, address = self.loopback_node()
        with patch.object(node.publisher, 'publish', side_effect=RuntimeError('publish failed')):
            with socket.create_connection(address, timeout=1.0) as client:
                client.sendall(bytes(range(15)))
                node.worker.join(timeout=2.0)
        self.assertFalse(node.worker.is_alive())
        with self.assertRaisesRegex(RuntimeError, 'publish failed'):
            node._check_worker()

    def test_shutdown_unblocks_active_receive_without_spurious_failure(self):
        node, address = self.loopback_node()
        with socket.create_connection(address, timeout=1.0):
            self.wait_for(lambda: node.connection_socket is not None)
            self.assertTrue(node.destroy_node())
        self.assertFalse(node.worker.is_alive())
        self.assertIsNone(node.connection_socket)
        self.assertIsNone(node.server_socket)
        self.assertFalse(node.worker_failed.is_set())
        node._check_worker()  # Intentional shutdown is not a fatal error.

    def test_main_propagates_failure_and_cleans_up(self):
        with patch.object(self.bridge_class, '_serve', side_effect=OSError('listener lost')):
            def spin(node):
                self.nodes.append(node)
                node.worker.join(timeout=2.0)
                node.health_timer.callback()
            self.ros.spin = spin
            with self.assertRaisesRegex(RuntimeError, 'listener lost'):
                self.namespace['main']()
        self.assertEqual(self.ros_calls, ['init', 'shutdown'])
        self.assertTrue(self.nodes[-1].destroyed)
        self.assertTrue(self.nodes[-1].health_timer.cancelled)

    def test_main_constructor_failure_still_shuts_down_ros(self):
        NodeAdapter.parameters['accept_timeout'] = math.nan
        with self.assertRaises(ValueError):
            self.namespace['main']()
        self.assertEqual(self.ros_calls, ['init', 'shutdown'])

    def test_keyboard_interrupt_is_a_normal_clean_exit(self):
        with patch.object(self.bridge_class, '_serve'):
            def spin(node):
                self.nodes.append(node)
                raise KeyboardInterrupt()
            self.ros.spin = spin
            self.namespace['main']()
        self.assertEqual(self.ros_calls, ['init', 'shutdown'])
        self.assertTrue(self.nodes[-1].destroyed)


if __name__ == '__main__':
    unittest.main()
