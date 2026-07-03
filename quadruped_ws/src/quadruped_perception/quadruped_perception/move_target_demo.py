"""Script demo: di chuyen target_ball qua lai (khong phai node san xuat, chi
dung khi test track_object voi muc tieu DI DONG thay vi dung yen).

Publish Twist len /model/target_ball/cmd_vel (bridge ROS->GZ trong
quadruped_description/launch/gazebo.launch.py, nhan boi VelocityControl
plugin trong worlds/flat_ground.sdf). Chay doc lap:
    ros2 run quadruped_perception move_target_demo
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PERIOD_S = 6.0     # thoi gian 1 chu ky qua lai
SPEED = 0.15       # m/s


class MoveTargetDemo(Node):
    def __init__(self):
        super().__init__('move_target_demo')
        self.pub = self.create_publisher(Twist, '/model/target_ball/cmd_vel', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info('move_target_demo: dang di chuyen target_ball qua lai theo truc Y')

    def _on_timer(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        phase = math.sin(2.0 * math.pi * elapsed / PERIOD_S)
        cmd = Twist()
        cmd.linear.y = SPEED * phase
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTargetDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
