"""Actual node callbacks with lightweight ROS-message/transport adapters."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from test_integration import integration


def header():
    return NS(stamp=NS(sec=0, nanosec=0), frame_id='')


def vector():
    return NS(x=0.0, y=0.0, z=0.0)


def quaternion():
    return NS(x=0.0, y=0.0, z=0.0, w=0.0)


class OdomMessage:
    def __init__(self):
        self.header = header()
        self.child_frame_id = ''
        self.pose = NS(pose=NS(position=vector(), orientation=quaternion()), covariance=[])
        self.twist = NS(twist=NS(linear=vector(), angular=vector()), covariance=[])


class TransformMessage:
    def __init__(self):
        self.header = header()
        self.child_frame_id = ''
        self.transform = NS(translation=vector(), rotation=quaternion())


class Diagnostics:
    OK, WARN, STALE = 0, 1, 3
    def __init__(self):
        self.header = header()
        self.status = []


SOURCE = Path(__file__).parents[1] / 'mobotic_odometry' / 'odometry_node.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
cls.bases = []
cls.body = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name != '__init__']
namespace = dict(math=math, time=NS(monotonic=lambda: 10.0), Odometry=OdomMessage,
                 TransformStamped=TransformMessage, DiagnosticArray=Diagnostics, DiagnosticStatus=Diagnostics,
                 KeyValue=lambda **kwargs: NS(**kwargs))
exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), namespace)


class OdometryNodeTest(unittest.TestCase):
    def setUp(self):
        self.node = namespace['MoboticOdometry']()
        self.node.engine = integration.PlanarOdometry()
        self.node.odom_frame, self.node.base_frame = 'odom', 'base_link'
        self.odom, self.tf, self.diagnostics = [], [], []
        self.node.publisher = NS(publish=self.odom.append)
        self.node.broadcaster = NS(sendTransform=self.tf.append)
        self.node.diagnostic_publisher = NS(publish=self.diagnostics.append)
        self.node.get_fully_qualified_name = lambda: '/moboterra/mobotic_odometry'
        self.now = 10.0
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(self.now * 1e9), to_msg=lambda: self.now))
        namespace['time'].monotonic = lambda: self.now

    def sample(self, seconds, vx=1.0, vy=0.0, wz=0.0):
        self.now = seconds
        seconds_ns = round(seconds * 1e9)
        return NS(header=NS(stamp=NS(sec=seconds_ns // 1_000_000_000, nanosec=seconds_ns % 1_000_000_000),
                            frame_id='base_link'), twist=NS(linear=NS(x=vx, y=vy, z=0.0), angular=NS(x=0.0, y=0.0, z=wz)))

    def test_odometry_and_tf_match_pose_frames_and_acquisition_stamp(self):
        self.node._receive(self.sample(10, 1, 0.3, 0.5))
        source = self.sample(10.1, 1, 0.3, 0.5)
        self.node._receive(source)
        odom, transform = self.odom[-1], self.tf[-1]
        self.assertIs(odom.header.stamp, source.header.stamp)
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_link')
        self.assertEqual(transform.header, odom.header)
        self.assertEqual(transform.child_frame_id, odom.child_frame_id)
        self.assertEqual(transform.transform.translation.x, odom.pose.pose.position.x)
        self.assertEqual(transform.transform.translation.y, odom.pose.pose.position.y)
        self.assertEqual(transform.transform.rotation, odom.pose.pose.orientation)
        q = odom.pose.pose.orientation
        self.assertAlmostEqual(q.z * q.z + q.w * q.w, 1.0)
        self.assertEqual(odom.twist.twist.linear.y, 0.3)
        self.assertEqual(len(odom.pose.covariance), 36)
        self.assertEqual(len(odom.twist.covariance), 36)

    def test_tf_can_be_disabled_without_disabling_odometry(self):
        self.node.broadcaster = None
        self.node._receive(self.sample(10))
        self.assertEqual(len(self.odom), 1)
        self.assertFalse(self.tf)

    def test_wrong_frame_nonplanar_and_malformed_stamp_have_no_outputs(self):
        for kind in ('frame', 'z', 'roll', 'stamp'):
            msg = self.sample(10)
            if kind == 'frame':
                msg.header.frame_id = 'map'
            elif kind == 'z':
                msg.twist.linear.z = 0.1
            elif kind == 'roll':
                msg.twist.angular.x = math.nan
            else:
                msg.header.stamp.nanosec = 1_000_000_000
            self.node._receive(msg)
        self.assertFalse(self.odom)
        self.assertFalse(self.tf)

    def test_duplicate_and_stale_feedback_do_not_publish_new_outputs(self):
        msg = self.sample(10)
        self.node._receive(msg)
        self.now = 10.1
        self.node._receive(msg)
        self.now = 11
        self.node._receive(msg)
        self.assertEqual(len(self.odom), 1)
        self.assertEqual(len(self.tf), 1)

    def test_loss_diagnostics_are_stale_without_fresh_zero_twist(self):
        self.node._receive(self.sample(10))
        self.now = 10.4
        self.node._diagnostics()
        self.assertEqual(self.diagnostics[-1].status[0].level, Diagnostics.STALE)
        self.assertEqual(len(self.odom), 1)
        self.assertEqual(len(self.tf), 1)

    def test_recovery_reports_persistent_pose_uncertainty(self):
        self.node._receive(self.sample(10))
        self.node._receive(self.sample(11))
        self.node._diagnostics()
        self.assertEqual(self.diagnostics[-1].status[0].level, Diagnostics.WARN)
        self.assertEqual(self.odom[-1].pose.pose.position.x, 0.0)

    def test_reset_requires_fresh_measured_standstill_and_no_immediate_publish(self):
        self.node._receive(self.sample(10))
        response = self.node._reset(None, NS())
        self.assertFalse(response.success)
        self.node._receive(self.sample(10.1, 0, 0, 0))
        count = len(self.odom)
        response = self.node._reset(None, NS())
        self.assertTrue(response.success)
        self.assertEqual(len(self.odom), count)
        self.node._receive(self.sample(10.2, 0, 0, 0))
        self.assertEqual(self.odom[-1].pose.pose.position.x, 0.0)
        self.now = 11
        self.assertFalse(self.node._reset(None, NS()).success)


if __name__ == '__main__':
    unittest.main()
