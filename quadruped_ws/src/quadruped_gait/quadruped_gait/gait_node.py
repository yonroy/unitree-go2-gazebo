"""Node ROS 2: subscribe /cmd_vel, chay TrotGait ~50Hz, publish goc 12 khop
toi forward_position_controller/commands.

Chuoi dieu khien dung §3 tai lieu: cmd_vel -> gait planner -> IK -> joint controller.

Co them vong heading-hold don gian dung IMU: gait mo (open-loop) khong tu sua
duoc lech huong do bat doi xung vat ly khi di thang lau (§6 tai lieu - han che
da biet cua tang reactive). Vong nay CHI sua goc yaw (huong quay mong muon,
tich luy tu wz nguoi dung gui) chu khong sua vi tri/quy dao day du - van nam
trong pham vi "baseline reactive", khong phai MPC/whole-body.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

from .trot_gait import TrotGait, LEG_NAMES

# Thu tu joint phai khop CHINH XAC voi forward_position_controller.joints trong
# quadruped_description/config/go2_controllers.yaml.
JOINT_ORDER = [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
]

CONTROL_RATE_HZ = 50.0
CMD_VEL_TIMEOUT_S = 0.5  # qua thoi gian nay khong nhan cmd_vel moi -> coi nhu dung

# Heading-hold: bu wz de giu huong mong muon khi di thang lau (xem docstring).
YAW_HOLD_KP = 1.5           # rad/s bu moi rad lech huong
YAW_HOLD_MAX_CORRECTION = 1.0  # rad/s, gioi han bu de khong lam gait mat on dinh


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')

        self.gait = TrotGait()
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._last_cmd_time = self.get_clock().now()

        self.measured_yaw = 0.0
        self.target_yaw = None  # None = chua co IMU / chua khoa huong
        self._was_moving = False

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._on_imu, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self._on_timer)
        self.get_logger().info('gait_node started - cho lenh tren /cmd_vel')

    def _on_cmd_vel(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self.measured_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_timer(self):
        dt = 1.0 / CONTROL_RATE_HZ
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > CMD_VEL_TIMEOUT_S:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.vx, self.vy, self.wz

        moving = abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(wz) > 1e-3
        if moving:
            if not self._was_moving or self.target_yaw is None:
                self.target_yaw = self.measured_yaw  # khoa huong hien tai khi bat dau di
            self.target_yaw += wz * dt
            yaw_error = _wrap_to_pi(self.target_yaw - self.measured_yaw)
            correction = max(-YAW_HOLD_MAX_CORRECTION,
                              min(YAW_HOLD_MAX_CORRECTION, YAW_HOLD_KP * yaw_error))
            wz_effective = wz + correction
        else:
            self.target_yaw = None
            wz_effective = 0.0
        self._was_moving = moving

        angles = self.gait.step(dt, vx, vy, wz_effective)

        commands = []
        for name in LEG_NAMES:
            hip, thigh, calf = angles[name]
            commands.extend([hip, thigh, calf])
        # commands hien theo thu tu LEG_NAMES = (FR, FL, RR, RL), da khop JOINT_ORDER
        assert JOINT_ORDER[0].startswith('FR') and JOINT_ORDER[3].startswith('FL')

        msg = Float64MultiArray()
        msg.data = commands
        self.joint_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
