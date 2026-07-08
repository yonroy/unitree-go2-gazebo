"""Node ROS ne vat can: subscribe /scan (LiDAR) -> compute_avoidance -> /cmd_vel.

Logic thuan o obstacle_avoider.py (co self-test). Node nay chi la vo ROS. Robot
tu di quanh ne vat can + giup slam_toolbox dung ban do. Locomotion (gait) giu nguyen.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .obstacle_avoider import AvoidParams, compute_avoidance

PUBLISH_RATE_HZ = 20


class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.params = AvoidParams()
        self.vx = 0.0
        self.wz = 0.0
        self._have_scan = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        # Publish deu (khong phu thuoc nhip scan) de gait nhan lenh muot
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info('obstacle_avoider san sang - ne vat can bang /scan -> /cmd_vel')

    def _on_scan(self, msg: LaserScan):
        self.vx, self.wz = compute_avoidance(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max, self.params)
        self._have_scan = True

    def _publish(self):
        if not self._have_scan:
            return
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.wz
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # dung robot khi tat
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
