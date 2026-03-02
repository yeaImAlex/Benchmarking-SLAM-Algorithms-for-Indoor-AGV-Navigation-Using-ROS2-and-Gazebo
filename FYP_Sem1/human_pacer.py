#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class HumanPacer(Node):
    def __init__(self):
        super().__init__('human_pacer')
        self.pub = self.create_publisher(Twist, '/human/cmd_vel', 10)

        # Tunable parameters
        self.speed = 0.6      # m/s
        self.leg_time = 5.0   # seconds before reversing direction

        self.elapsed = 0.0
        self.direction = 1.0  # +1 forward, -1 backward

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        self.elapsed += 0.1
        if self.elapsed >= self.leg_time:
            self.elapsed = 0.0
            self.direction *= -1.0

        msg = Twist()
        msg.linear.x = self.direction * self.speed
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPacer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
