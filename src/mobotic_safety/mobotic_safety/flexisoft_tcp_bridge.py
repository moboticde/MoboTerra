import math
import socket
import threading

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt8MultiArray


class FlexiSoftTcpBridge(Node):
    def __init__(self):
        super().__init__('flexisoft_tcp_bridge')

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 9100)
        self.declare_parameter('telegram_size', 15)
        self.declare_parameter('status_byte_offset', 14)
        self.declare_parameter('accept_timeout', 0.5)

        self.bind_address = str(self.get_parameter('bind_address').value)
        self.port = int(self.get_parameter('port').value)
        self.telegram_size = int(self.get_parameter('telegram_size').value)
        self.status_byte_offset = int(self.get_parameter('status_byte_offset').value)
        self.accept_timeout = float(self.get_parameter('accept_timeout').value)

        if not 0 < self.port < 65536:
            raise ValueError('port must be in [1, 65535]')
        if self.telegram_size <= 0 or not 0 <= self.status_byte_offset < self.telegram_size:
            raise ValueError('status_byte_offset must be within the configured telegram')
        if not math.isfinite(self.accept_timeout) or self.accept_timeout <= 0.0:
            raise ValueError('accept_timeout must be finite and positive')

        self.publisher = self.create_publisher(UInt8, 'flexisoft/status_byte', 10)
        self.telegram_publisher = self.create_publisher(UInt8MultiArray, 'flexisoft/telegram', 10)
        self.stop_event = threading.Event()
        self.worker_failed = threading.Event()
        self.worker_error = None
        self.server_socket = None
        self.connection_socket = None
        # A wall/steady timer still detects worker loss if ROS time is paused.
        self.health_timer = self.create_timer(
            0.1, self._check_worker, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.worker = threading.Thread(target=self._run_server, daemon=True)
        self.worker.start()

    def _run_server(self):
        try:
            self._serve()
        except Exception as error:
            if not self.stop_event.is_set():
                self.worker_error = f'{type(error).__name__}: {error}'
        finally:
            if not self.stop_event.is_set():
                if self.worker_error is None:
                    self.worker_error = 'TCP worker terminated unexpectedly'
                self.worker_failed.set()

    def _check_worker(self):
        if self.stop_event.is_set():
            return
        if self.worker_failed.is_set() or not self.worker.is_alive():
            message = self.worker_error or 'TCP worker is not running'
            self.get_logger().error(f'FlexiSoft TCP bridge failed: {message}; exiting for restart')
            # Propagate through spin/main, giving launch a nonzero process exit.
            raise RuntimeError(f'FlexiSoft TCP bridge failed: {message}')

    def _serve(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                self.server_socket = server
                if self.stop_event.is_set():
                    return
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.bind_address, self.port))
                server.listen()
                server.settimeout(self.accept_timeout)
                self.get_logger().info(
                    f'FlexiSoft TCP bridge listening on {self.bind_address}:{self.port}'
                )
                while not self.stop_event.is_set():
                    try:
                        connection, address = server.accept()
                    except socket.timeout:
                        continue
                    # Listener errors are fatal, unlike an individual client reset.
                    self.get_logger().info(f'FlexiSoft client connected: {address[0]}')
                    with connection:
                        self.connection_socket = connection
                        try:
                            connection.settimeout(self.accept_timeout)
                            self._read_connection(connection)
                        finally:
                            self.connection_socket = None
                    self.get_logger().warning('FlexiSoft client disconnected')
        finally:
            self.server_socket = None

    def _read_connection(self, connection):
        buffered = bytearray()
        while not self.stop_event.is_set():
            try:
                data = connection.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            buffered.extend(data)
            while not self.stop_event.is_set() and len(buffered) >= self.telegram_size:
                telegram = buffered[:self.telegram_size]
                del buffered[:self.telegram_size]
                output = UInt8()
                output.data = telegram[self.status_byte_offset]
                raw = UInt8MultiArray()
                raw.data = list(telegram)
                self.telegram_publisher.publish(raw)
                self.publisher.publish(output)

    def destroy_node(self):
        self.stop_event.set()
        self.health_timer.cancel()
        # Unblock both accept() and an active recv() before destroying publishers.
        for endpoint in (self.connection_socket, self.server_socket):
            if endpoint is None:
                continue
            try:
                endpoint.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                endpoint.close()
            except OSError:
                pass
        if self.worker.is_alive():
            self.worker.join(timeout=self.accept_timeout + 0.5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlexiSoftTcpBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
