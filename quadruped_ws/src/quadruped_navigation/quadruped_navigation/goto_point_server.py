"""Action server 'goto_point': nhan goal (x,y), dieu khien robot toi do roi
dung, publish /cmd_vel dua tren /odom (xem goto_point.py cho logic thuan,
da tu kiem chung boi self-test rieng).

Odometry (/odom) la ground-truth lay tu Gazebo (xem
quadruped_description/xacro/gazebo.xacro va launch/gazebo.launch.py) - phu
hop pham vi baseline (chua co SLAM/localization that).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from quadruped_interfaces.action import GotoPoint

from .goto_point import GotoPointParams, compute_cmd

CONTROL_RATE_HZ = 20.0


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GotoPointServer(Node):
    def __init__(self):
        super().__init__('goto_point_server')
        self.params = GotoPointParams()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._has_odom = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._action_server = ActionServer(
            self, GotoPoint, 'goto_point',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('goto_point_server san sang - cho goal tren action "goto_point"')

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._has_odom = True

    def _on_goal(self, _goal_request):
        if not self._has_odom:
            self.get_logger().warn('Chua nhan duoc /odom, tu choi goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        feedback = GotoPoint.Feedback()
        result = GotoPoint.Result()
        period = 1.0 / CONTROL_RATE_HZ

        self.get_logger().info(f'Nhan goal: ({target_x:.2f}, {target_y:.2f})')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal bi huy'
                return result

            linear_x, angular_z, distance, heading_error, reached = compute_cmd(
                self.x, self.y, self.yaw, target_x, target_y, self.params
            )

            feedback.distance_remaining = distance
            feedback.heading_error = heading_error
            goal_handle.publish_feedback(feedback)

            if reached:
                self._stop_robot()
                goal_handle.succeed()
                result.success = True
                result.message = f'Da toi ({target_x:.2f}, {target_y:.2f})'
                self.get_logger().info(result.message)
                return result

            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.cmd_pub.publish(cmd)

            time.sleep(period)

        self._stop_robot()
        result.success = False
        result.message = 'rclpy shutdown truoc khi hoan tat'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GotoPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
