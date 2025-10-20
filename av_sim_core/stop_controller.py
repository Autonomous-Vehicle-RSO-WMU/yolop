#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool

class StopController(Node):
    def __init__(self):
        super().__init__('stop_controller')
        self.pub = self.create_publisher(AckermannDrive, '/carla/hero/ackermann_cmd', 10)
        self.sub = self.create_subscription(Bool, '/carla/hero/stop_flag', self.flag_cb, 10)
        self.stop = False
        self.timer = self.create_timer(0.1, self.loop)

    def flag_cb(self, msg: Bool):
        if msg.data:
            self.stop = True

    def loop(self):
        drive = AckermannDrive()
        if self.stop:
            drive.speed = 0.0
            drive.steering_angle = 0.0
        else:
            drive.speed = 8.0
            drive.steering_angle = 0.0
        self.pub.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    node = StopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
