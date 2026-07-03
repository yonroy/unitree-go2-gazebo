"""Action server 'track_object': nhan goal (target_id, stop_distance), dieu
khien robot bam theo muc tieu va giu khoang cach stop_distance, publish
/cmd_vel dua tren /target_pose (xem follow_object.py cho logic thuan).

/target_pose (geometry_msgs/PointStamped, khung "trunk") den tu
quadruped_perception/target_pose_node.py - CHI publish khi dang thay muc
tieu trong frame hien tai. Neu mat muc tieu qua LOST_TIMEOUT_S, robot dung
lai va cho; neu mat qua MAX_LOST_S lien tuc thi huy goal.
"""
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from quadruped_interfaces.action import TrackObject

from .follow_object import FollowParams, compute_cmd

CONTROL_RATE_HZ = 20.0
LOST_TIMEOUT_S = 1.0   # qua nguong nay -> coi la dang mat muc tieu, dung lai cho
MAX_LOST_S = 5.0       # mat lien tuc qua nguong nay -> huy goal


class TrackObjectServer(Node):
    def __init__(self):
        super().__init__('track_object_server')
        self.params = FollowParams()

        self.point_x = 0.0
        self.point_y = 0.0
        self._has_target = False
        self._last_seen_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PointStamped, '/target_pose', self._on_target_pose, 10
        )

        self._action_server = ActionServer(
            self, TrackObject, 'track_object',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('track_object_server san sang - cho goal tren action "track_object"')

    def _on_target_pose(self, msg: PointStamped):
        self.point_x = msg.point.x
        self.point_y = msg.point.y
        self._has_target = True
        self._last_seen_time = time.monotonic()

    def _on_goal(self, _goal_request):
        if not self._has_target:
            self.get_logger().warn('Chua thay muc tieu lan nao (/target_pose), tu choi goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        stop_distance = goal_handle.request.stop_distance
        feedback = TrackObject.Feedback()
        result = TrackObject.Result()
        period = 1.0 / CONTROL_RATE_HZ

        self.get_logger().info(f'Nhan goal: bam muc tieu, stop_distance={stop_distance:.2f}m')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._stop_robot()
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal bi huy'
                return result

            lost_duration = time.monotonic() - self._last_seen_time
            target_visible = lost_duration < LOST_TIMEOUT_S

            if not target_visible:
                self._stop_robot()
                feedback.current_distance = 0.0
                feedback.current_angle = 0.0
                feedback.target_visible = False
                goal_handle.publish_feedback(feedback)

                if lost_duration > MAX_LOST_S:
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Mat muc tieu qua {MAX_LOST_S:.0f}s'
                    self.get_logger().warn(result.message)
                    return result

                time.sleep(period)
                continue

            linear_x, angular_z, distance, angle, _in_range = compute_cmd(
                self.point_x, self.point_y, stop_distance, self.params
            )

            feedback.current_distance = distance
            feedback.current_angle = angle
            feedback.target_visible = True
            goal_handle.publish_feedback(feedback)

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
    node = TrackObjectServer()
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
