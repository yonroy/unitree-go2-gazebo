"""Node RL locomotion: thay gait_node rule-based bang policy neural (phi tuyen).

Kien truc 2 vong (giong deploy that, tranh blow-up):
  - Vong POLICY @ 50Hz (step_dt): dung obs 45 chieu -> ONNX -> cap nhat goc khop
    MUC TIEU (target).
  - Vong PD nhanh @ nhip /joint_states (~200Hz): moi khi co state moi, tinh lai
    tau = Kp*(target-q) - Kd*qd voi q,qd MOI, publish /joint_effort_controller/commands.
    (Tinh PD 1 lan/20ms roi giu torque suot 20ms se bom nang luong -> mat on dinh,
     da kiem chung thuc te: khop quay 20-30 rad/s, torque bao hoa, robot nga.)

target khoi tao = default pose -> PD giu robot dung ngay khi controller active,
truoc khi policy "am". Model: diasAiMaster/unitree-go2-velocity-flat (BSD-3).
"""
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from .policy_runner import (
    ACT_DIM, DEFAULT_JOINT_POS, JOINT_ORDER, STEP_DT,
    PolicyRunner, pd_torque, projected_gravity)

CMD_VEL_TIMEOUT_S = 0.5
TORQUE_LIMIT = 23.7  # N*m - gioi han torque motor Go2 (an toan)
HOLD_DURATION_S = 3.0  # giu tu the default (position ctrl) truoc khi chuyen sang effort
# Throttle goi switch_controller: callback /joint_states chay ~200Hz, neu retry
# switch moi callback se HAMMER controller_manager (executor don luong) -> spawner
# joint_effort_controller khong goi noi list_controllers -> controller khong bao gio
# nap -> policy khong chay (DA KIEM CHUNG bang log: spawner timeout 3 lan). Gioi han
# 1 lan/giay du de switch thanh cong ma khong lam nghen controller_manager.
SWITCH_RETRY_PERIOD_S = 1.0
# Khuech dai lenh yaw truoc khi dua vao policy. Ly do (DA DO THUC TE trong Gazebo):
# policy under-track yaw nang - lenh wz=0.5 va wz=1.0 deu chi cho ~0.24 rad/s thuc
# (bao hoa) do sim-to-sim gap. Nhung lenh yaw LON lam robot xoay "tai cho" sach hon
# (giam troi tinh tien: wz=1.0 -> vi tri gan nhu dung yen, wz=0.5 -> van troi ~0.19m/s).
# Teleop chi gui toi ~0.5 rad/s -> khuech dai x2 (clamp ±1.0 = trong khoang train
# ang_vel_z [-1,1] cua deploy.yaml) de dat vung xoay tai cho tot nhat policy lam duoc.
YAW_CMD_GAIN = 2.0
YAW_CMD_MAX = 1.0


class RLPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy_node')

        default_onnx = os.path.join(
            get_package_share_directory('quadruped_rl'), 'models', 'policy.onnx')
        self.declare_parameter('onnx_path', default_onnx)
        onnx_path = self.get_parameter('onnx_path').value
        self.get_logger().info(f'Nap policy ONNX: {onnx_path}')
        self.runner = PolicyRunner(onnx_path)

        self.vx = self.vy = self.wz = 0.0
        self._last_cmd_time = self.get_clock().now()
        self.ang_vel = np.zeros(3, np.float32)
        self.proj_gravity = np.array([0, 0, -1], np.float32)
        self.joint_pos = None   # theo thu tu JOINT_ORDER
        self.joint_vel = np.zeros(ACT_DIM, np.float32)
        self._name_index = None  # map ten khop -> index trong /joint_states
        self.target = DEFAULT_JOINT_POS.copy()

        # State machine: HOLD (position ctrl giu default pose) -> SWITCHING ->
        # RUN (effort + policy). Tranh robot nga luc khoi dong (xem launch).
        self.phase = 'HOLD'
        self._hold_start = None
        self._switch_pending = False
        self._last_switch_attempt = None  # throttle switch_controller (xem SWITCH_RETRY_PERIOD_S)

        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/joint_effort_controller/commands', 10)
        self.pos_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self._switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        # Vong PD nhanh chay trong callback nay (nhip broadcaster ~200Hz)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

        # Vong policy cham: chi cap nhat target
        self.create_timer(STEP_DT, self._policy_step)
        self.get_logger().info('rl_policy_node: HOLD (giu tu the default), cho on dinh...')

    def _request_switch(self):
        """Goi switch_controller: tat position, bat effort (retry neu chua san sang)."""
        if not self._switch_cli.service_is_ready():
            return  # controller_manager/effort ctrl chua san sang -> thu lai sau
        req = SwitchController.Request()
        req.activate_controllers = ['joint_effort_controller']
        req.deactivate_controllers = ['forward_position_controller']
        req.strictness = SwitchController.Request.STRICT
        self._switch_pending = True
        future = self._switch_cli.call_async(req)
        future.add_done_callback(self._on_switch_done)

    def _on_switch_done(self, future):
        self._switch_pending = False
        try:
            ok = future.result().ok
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'switch_controller loi: {exc}')
            return
        if ok:
            self.runner.reset()
            self.target = self.joint_pos.copy()  # bat dau muot: PD ~0 tai tu the hien tai
            self.phase = 'RUN'
            self.get_logger().info('Da chuyen sang RUN (effort + policy).')
        else:
            self.get_logger().warn('switch_controller tra ve that bai, se thu lai.')

    def _on_imu(self, msg: Imu):
        self.ang_vel = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            np.float32)
        q = msg.orientation
        self.proj_gravity = projected_gravity((q.x, q.y, q.z, q.w))

    def _on_joints(self, msg: JointState):
        if self._name_index is None:
            idx = {n: i for i, n in enumerate(msg.name)}
            missing = [n for n in JOINT_ORDER if n not in idx]
            if missing:
                self.get_logger().warn(f'Thieu khop trong /joint_states: {missing}',
                                       throttle_duration_sec=5.0)
                return
            self._name_index = [idx[n] for n in JOINT_ORDER]
        self.joint_pos = np.array([msg.position[i] for i in self._name_index], np.float32)
        if msg.velocity:
            self.joint_vel = np.array([msg.velocity[i] for i in self._name_index], np.float32)
        else:
            self.joint_vel = np.zeros(ACT_DIM, np.float32)

        if self.phase == 'HOLD':
            # Giu tu the default qua position controller (gz P noi bo giu robot dung).
            if self._hold_start is None:
                self._hold_start = self.get_clock().now()
            out = Float64MultiArray()
            out.data = DEFAULT_JOINT_POS.astype(float).tolist()
            self.pos_pub.publish(out)
            elapsed = (self.get_clock().now() - self._hold_start).nanoseconds / 1e9
            if elapsed > HOLD_DURATION_S and not self._switch_pending:
                now = self.get_clock().now()
                # Throttle: chi thu switch 1 lan/SWITCH_RETRY_PERIOD_S de khong nghen
                # controller_manager (neu khong, spawner effort khong nap duoc controller).
                due = (self._last_switch_attempt is None or
                       (now - self._last_switch_attempt).nanoseconds / 1e9 > SWITCH_RETRY_PERIOD_S)
                if due:
                    self._last_switch_attempt = now
                    self._request_switch()  # da on dinh -> chuyen sang effort
        elif self.phase == 'RUN':
            # --- Vong PD nhanh: torque theo state MOI + target hien tai ---
            tau = pd_torque(self.target, self.joint_pos, self.joint_vel)
            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            out = Float64MultiArray()
            out.data = tau.astype(float).tolist()
            self.effort_pub.publish(out)

    def _on_cmd(self, msg: Twist):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _policy_step(self):
        if self.phase != 'RUN' or self.joint_pos is None:
            return  # chi chay policy khi da chuyen sang effort
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if dt > CMD_VEL_TIMEOUT_S:
            self.vx = self.vy = self.wz = 0.0  # het lenh -> dung tai cho

        # Khuech dai lenh yaw toi vung bao hoa cua policy (xem YAW_CMD_GAIN).
        wz_cmd = float(np.clip(self.wz * YAW_CMD_GAIN, -YAW_CMD_MAX, YAW_CMD_MAX))
        cmd = np.array([self.vx, self.vy, wz_cmd], np.float32)
        _action, target = self.runner.infer(
            self.ang_vel, self.proj_gravity, cmd, self.joint_pos, self.joint_vel)
        self.target = target


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
