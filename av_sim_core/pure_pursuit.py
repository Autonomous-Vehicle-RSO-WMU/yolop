#!/usr/bin/env python3
import csv
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive

MAX_STEER = math.radians(70)
WHEELBASE = 1.63
LOOKAHEAD = 10.0
CMD_SPEED = 5.0

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.waypoints = self._load_waypoints(Path(__file__).parent / 'data' / 'waypoints.csv')
        if not self.waypoints:
            raise RuntimeError('No waypoints loaded')
        self.x = self.y = self.z = None
        self.yaw = 0.0
        self.speed = 0.0
        self.create_subscription(Odometry, '/carla/hero/odometry', self._odom_cb, 10)
        self.create_subscription(Float32, '/carla/hero/speedometer', self._speed_cb, 10)
        self.pub = self.create_publisher(AckermannDrive, '/carla/hero/ackermann_cmd', 10)
        self.create_timer(0.05, self._loop)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y, self.z = p.x, p.y, p.z
        o = msg.pose.pose.orientation
        self.yaw = self._quat_to_yaw(o.x, o.y, o.z, o.w)

    def _speed_cb(self, msg: Float32):
        self.speed = float(msg.data)

    def _loop(self):
        if self.x is None:
            return
        steer = self._compute_steer()
        drive = AckermannDrive()
        drive.steering_angle = steer
        drive.speed = CMD_SPEED
        self.pub.publish(drive)

    def _compute_steer(self) -> float:
        idx = self._nearest_wp_idx(self.x, self.y)
        tx, ty = self.waypoints[idx]
        err = math.atan2(ty - self.y, tx - self.x) - self.yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi
        steer = math.atan2(2 * WHEELBASE * math.sin(err) / LOOKAHEAD, 1.0)
        return max(-MAX_STEER, min(MAX_STEER, steer))

    def _nearest_wp_idx(self, x: float, y: float) -> int:
        best_i, best_d = 0, float('inf')
        for i, (wx, wy) in enumerate(self.waypoints):
            d = math.hypot(wx - x, wy - y)
            if d < best_d:
                best_i, best_d = i, d
        return best_i

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _load_waypoints(path: Path):
        if not path.exists():
            return []
        with path.open() as f:
            rdr = csv.reader(f)
            next(rdr, None)
            return [(float(x), float(y)) for _t, x, y, *_rest in rdr]


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
