"""Catch collisions with rclpy's read-only Node API without a ROS installation."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class ReadOnlyPublisherNode:
    def __init__(self, name):
        self.created_publishers = []

    @property
    def publishers(self):
        return iter(self.created_publishers)

    def declare_parameter(self, name, value):
        pass

    def get_parameter(self, name):
        return SimpleNamespace(value=0.3)

    def create_publisher(self, kind, topic, depth):
        publisher = SimpleNamespace(topic=topic)
        self.created_publishers.append(publisher)
        return publisher

    def create_subscription(self, *args):
        return object()

    def create_timer(self, *args):
        return object()


class ScannerSafetyTest(unittest.TestCase):
    def test_adapter_constructs_with_read_only_node_publishers(self):
        source = Path(__file__).parents[1] / 'mobotic_safety/scanner_safety.py'
        cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
        namespace = dict(Node=ReadOnlyPublisherNode, SafetyIOState=object,
                         ScannerSafetyState=object, DiagnosticArray=object)
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
        node = namespace['ScannerSafetyAdapter']()
        self.assertEqual({p.topic for p in node.publishers}, {
            'scanner/front_left/safety_zone_status', 'scanner/rear_right/safety_zone_status',
            'scanner/front_left/diagnostics', 'scanner/rear_right/diagnostics',
        })


if __name__ == '__main__':
    unittest.main()
