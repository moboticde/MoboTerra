"""Calibrates base_link -> laser transform from SLAM + odometry.

Expects slam_toolbox to be running with base_frame == odom_frame == the
laser frame so that maps -> <laser> is pure scan matching unaffected by
any (wrong) static laser TF.

Solves AX = XB in SE(2):
  A = body-frame deltas of base_link from /mobotic/odometry
  B = body-frame deltas of the laser from maps -> <laser_frame>
  X = base_link -> laser (what we want)
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def pose_from_odom(msg: Odometry):
    p = msg.pose.pose
    return np.array(
        [
            p.position.x,
            p.position.y,
            quat_to_yaw(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            ),
        ]
    )


def pose_from_tf(t: TransformStamped):
    r = t.transform.rotation
    return np.array(
        [
            t.transform.translation.x,
            t.transform.translation.y,
            quat_to_yaw(r.x, r.y, r.z, r.w),
        ]
    )


def body_delta(p1, p2):
    """Return body-frame motion from pose p1 to pose p2 (both in the same world frame)."""
    dx_w = p2[0] - p1[0]
    dy_w = p2[1] - p1[1]
    c, s = math.cos(p1[2]), math.sin(p1[2])
    dx = c * dx_w + s * dy_w
    dy = -s * dx_w + c * dy_w
    dtheta = (p2[2] - p1[2] + math.pi) % (2.0 * math.pi) - math.pi
    return np.array([dx, dy, dtheta])


class LaserTfCalibrator(Node):
    def __init__(self):
        super().__init__("laser_tf_calibrator")
        self.declare_parameter("odom_topic", "/mobotic/odometry")
        self.declare_parameter("map_frame", "maps")
        self.declare_parameter("laser_frame", "front_scan")
        self.declare_parameter("min_translation", 0.05)
        self.declare_parameter("min_rotation", 0.02)
        self.declare_parameter("max_pair_age", 5.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.min_trans = self.get_parameter("min_translation").value
        self.min_rot = self.get_parameter("min_rotation").value
        self.max_pair_age = Duration(
            seconds=float(self.get_parameter("max_pair_age").value)
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, 50
        )
        self.process_timer = self.create_timer(0.1, self.process_queue)

        self.odom_queue: deque = deque()
        self.last_base = None
        self.last_laser = None
        self.base_deltas = []
        self.laser_deltas = []
        self.dropped_stale = 0

        self.get_logger().info(
            f"Calibrator ready. Drive the robot around (translation + rotation), then Ctrl+C."
        )
        self.get_logger().info(
            f"  odom topic: {self.odom_topic}"
        )
        self.get_logger().info(
            f"  maps frame:  {self.map_frame}   laser frame: {self.laser_frame}"
        )

    def odom_cb(self, msg: Odometry):
        self.odom_queue.append(msg)

    def process_queue(self):
        """Drain the odom queue, pairing each msg with maps->laser at its own stamp."""
        now = self.get_clock().now()
        while self.odom_queue:
            msg = self.odom_queue[0]
            stamp = Time.from_msg(msg.header.stamp)
            if not self.tf_buffer.can_transform(
                self.map_frame, self.laser_frame, stamp
            ):
                if (now - stamp) > self.max_pair_age:
                    self.odom_queue.popleft()
                    self.dropped_stale += 1
                    continue
                # TF not ready yet; try again next tick.
                return
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.laser_frame, stamp
                )
            except Exception:
                self.odom_queue.popleft()
                self.dropped_stale += 1
                continue
            self.odom_queue.popleft()
            self._accumulate(pose_from_odom(msg), pose_from_tf(tf))

    def _accumulate(self, base_pose, laser_pose):
        if self.last_base is None:
            self.last_base = base_pose
            self.last_laser = laser_pose
            return

        db = body_delta(self.last_base, base_pose)
        dl = body_delta(self.last_laser, laser_pose)

        if math.hypot(db[0], db[1]) < self.min_trans and abs(db[2]) < self.min_rot:
            return

        self.base_deltas.append(db)
        self.laser_deltas.append(dl)
        self.last_base = base_pose
        self.last_laser = laser_pose

        if len(self.base_deltas) % 25 == 0:
            self.get_logger().info(
                f"collected {len(self.base_deltas)} motion pairs "
                f"(dropped {self.dropped_stale} stale, queue {len(self.odom_queue)})"
            )

    def solve(self):
        n = len(self.base_deltas)
        if n < 3:
            self.get_logger().warn(
                f"Not enough motion pairs ({n}); need at least 3 with varied rotation."
            )
            return

        # For SE(2), AX = XB with X = (t, R(yaw)) and A=db=(tbx,tby,θb), B=dl=(tlx,tly,θl):
        #   (I - R(θl)) t + R(yaw) [tlx, tly]^T = [tbx, tby]^T
        # Treating c = cos(yaw), s = sin(yaw) as free unknowns plus constraint c^2+s^2=1.
        rows_A = []
        rows_b = []
        rot_residuals = []
        for db, dl in zip(self.base_deltas, self.laser_deltas):
            tbx, tby, thb = db
            tlx, tly, thl = dl
            cl, sl = math.cos(thl), math.sin(thl)
            rows_A.append([1.0 - cl,      sl, tlx, -tly])
            rows_b.append(tbx)
            rows_A.append([     -sl, 1.0 - cl, tly,  tlx])
            rows_b.append(tby)
            rot_residuals.append(
                ((thb - thl + math.pi) % (2.0 * math.pi)) - math.pi
            )

        A = np.asarray(rows_A)
        b = np.asarray(rows_b)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        tx, ty, c, s = sol

        norm = math.hypot(c, s)
        if norm < 1e-6:
            self.get_logger().error(
                "Degenerate system — the robot needs rotational motion for yaw to be observable."
            )
            return
        yaw = math.atan2(s / norm, c / norm)

        rot_rmse = float(np.sqrt(np.mean(np.square(rot_residuals))))

        # Report residual of the LSQ fit
        res = A @ sol - b
        trans_rmse = float(np.sqrt(np.mean(res * res)))

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Calibration result from {n} motion pairs:")
        self.get_logger().info(f"  base_link -> {self.laser_frame}:")
        self.get_logger().info(f"    x:   {tx:+.4f} m")
        self.get_logger().info(f"    y:   {ty:+.4f} m")
        self.get_logger().info(
            f"    yaw: {yaw:+.4f} rad  ({math.degrees(yaw):+.2f} deg)"
        )
        self.get_logger().info(f"  translation fit RMSE: {trans_rmse:.4f} m")
        self.get_logger().info(
            f"  rotation  consistency RMSE: {rot_rmse:.4f} rad "
            f"(should be small; large means scan matching and odom disagree on rotation)"
        )
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = LaserTfCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.solve()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
