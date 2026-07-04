"""MuJoCo Go2 + LiDAR 3D -> publish PointCloud2 + TF + robot marker ra ROS de xem
trong RViz (bieu do point cloud cua can phong). Robot tu di quanh ne vat can (policy
RL + compute_avoidance) -> quet dan ca phong -> dam may diem 3D lon dan.

Chay:
  source /opt/ros/jazzy/setup.bash && source ~/Ros2/quadruped_ws/install/setup.bash
  python3 mujoco_lidar_rviz.py
  rviz2 -d config_rviz.rviz    # (fixed frame: odom, PointCloud2 /mujoco/points)

Khong can GL/cua so - mj_ray + mj_step chay tren CPU, RViz lo render.
"""
import os
import sys
import math
import numpy as np

import mujoco
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'quadruped_navigation')))
from quadruped_navigation.obstacle_avoider import AvoidParams, compute_avoidance

_scene = sys.argv[1] if len(sys.argv) > 1 else 'lidar'  # 'lidar' (phong) hoac 'maze'
XML = os.path.join(HERE, 'go2_model', f'scene_{_scene}.xml')
ONNX = os.path.join(HERE, '..', 'models', 'policy.onnx')
DEFAULT = np.array([0.1, 0.9, -1.8, -0.1, 0.9, -1.8,
                    0.1, 0.9, -1.8, -0.1, 0.9, -1.8], np.float32)
KP = np.array([20, 20, 40] * 4, np.float32)
KD = np.array([1, 1, 2] * 4, np.float32)
SIM_DT = 0.002
DECIMATION = 10
STEPS_PER_TICK = 18
N_H = 120                 # tia ngang
N_V = 16                  # kenh doc (giong Velodyne)
V_MIN, V_MAX = math.radians(-12), math.radians(10)
LIDAR_H = 0.5
RANGE_MAX = 8.0
VOXEL = 0.08              # gop diem theo o 8cm (dam may khong phinh vo han)
MAX_POINTS = 120000
_GG = np.array([0, 0, 0, 1, 0, 0], np.uint8)


def grav(q):
    w, x, y, z = q
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)], np.float32)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class MujocoLidarRviz(Node):
    def __init__(self):
        super().__init__('mujoco_lidar_rviz')
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.m.opt.timestep = SIM_DT
        self.d = mujoco.MjData(self.m)
        self.d.qpos[2] = 0.30
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[7:] = DEFAULT
        mujoco.mj_forward(self.m, self.d)
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name

        # huong tia 3D (trong khung robot: se cong yaw luc quet)
        hs = -np.pi + np.arange(N_H) * (2*np.pi/N_H)
        vs = np.linspace(V_MIN, V_MAX, N_V)
        self.dirs = []  # (base_h, v) de xoay theo yaw
        for v in vs:
            for h in hs:
                self.dirs.append((h, v))
        self.gid = np.zeros(1, np.int32)

        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.cmd = np.zeros(3, np.float32)
        self.counter = 0
        self.tick = 0
        self.params = AvoidParams(cruise_vx=0.55, max_wz=1.2, clear_dist=1.0, stop_dist=0.5)
        self.voxels = {}   # key -> (x,y,z) diem tich luy

        self.pc_pub = self.create_publisher(PointCloud2, '/mujoco/points', 5)
        self.mk_pub = self.create_publisher(Marker, '/mujoco/robot', 5)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.05, self._tick)   # 20Hz
        self.get_logger().info('mujoco_lidar_rviz: publish /mujoco/points + TF odom->base_link. Mo RViz (fixed frame: odom).')

    def _scan3d(self, pos, yaw):
        pnt = np.array([pos[0], pos[1], LIDAR_H], np.float64)
        pts = []
        r2d_min = RANGE_MAX
        for h, v in self.dirs:
            wa = h + yaw
            cv = math.cos(v)
            vec = np.array([math.cos(wa)*cv, math.sin(wa)*cv, math.sin(v)], np.float64)
            dist = mujoco.mj_ray(self.m, self.d, pnt, vec, _GG, 1, -1, self.gid)
            if 0.2 <= dist < RANGE_MAX:
                pts.append((pnt[0]+dist*vec[0], pnt[1]+dist*vec[1], pnt[2]+dist*vec[2]))
                if abs(v) < 0.06:
                    r2d_min = min(r2d_min, dist)
        return pts, r2d_min

    def _scan2d_ranges(self, pos, yaw):
        # ranges ngang (v=0) cho compute_avoidance
        r = np.full(N_H, RANGE_MAX)
        pnt = np.array([pos[0], pos[1], LIDAR_H], np.float64)
        for i in range(N_H):
            h = -np.pi + i*(2*np.pi/N_H); wa = h + yaw
            vec = np.array([math.cos(wa), math.sin(wa), 0.0], np.float64)
            dist = mujoco.mj_ray(self.m, self.d, pnt, vec, _GG, 1, -1, self.gid)
            if 0.2 <= dist < RANGE_MAX:
                r[i] = dist
        return r

    def _tick(self):
        pos = self.d.qpos[:3].copy()
        yaw = yaw_of(self.d.qpos[3:7])
        # ne vat can (dung scan ngang)
        r = self._scan2d_ranges(pos, yaw)
        vx, wz = compute_avoidance(r, -np.pi, 2*np.pi/N_H, RANGE_MAX, self.params)
        self.cmd[:] = [vx, 0.0, wz]

        for _ in range(STEPS_PER_TICK):
            tau = KP*(self.target - self.d.qpos[7:]) - KD*self.d.qvel[6:]
            self.d.ctrl[:] = np.clip(tau, -23.7, 23.7)
            mujoco.mj_step(self.m, self.d)
            self.counter += 1
            if self.counter % DECIMATION == 0:
                obs = np.concatenate([self.d.qvel[3:6], grav(self.d.qpos[3:7]), self.cmd,
                                      self.d.qpos[7:]-DEFAULT, self.d.qvel[6:], self.action]).astype(np.float32)
                self.action = self.sess.run(None, {self.inp: obs[None]})[0].flatten()
                self.target = self.action*0.5 + DEFAULT

        self.tick += 1
        # TF odom -> base_link + marker moi tick
        self._pub_tf(pos, self.d.qpos[3:7])
        self._pub_marker()
        # quet 3D + publish dam may moi 2 tick (~10Hz)
        if self.tick % 2 == 0:
            pts, _ = self._scan3d(pos, yaw)
            for p in pts:
                key = (round(p[0]/VOXEL), round(p[1]/VOXEL), round(p[2]/VOXEL))
                self.voxels[key] = p
            if len(self.voxels) > MAX_POINTS:
                # bo bot diem cu (gioi han bo nho)
                for k in list(self.voxels.keys())[:len(self.voxels)-MAX_POINTS]:
                    del self.voxels[k]
            self._pub_cloud()

    def _pub_cloud(self):
        hdr = Header(); hdr.stamp = self.get_clock().now().to_msg(); hdr.frame_id = 'odom'
        msg = point_cloud2.create_cloud_xyz32(hdr, list(self.voxels.values()))
        self.pc_pub.publish(msg)

    def _pub_tf(self, pos, quat_wxyz):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x = float(pos[0]); t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        w, x, y, z = quat_wxyz
        t.transform.rotation.x = float(x); t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z); t.transform.rotation.w = float(w)
        self.tf.sendTransform(t)

    def _pub_marker(self):
        mk = Marker()
        mk.header.frame_id = 'base_link'; mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'robot'; mk.id = 0; mk.type = Marker.CUBE; mk.action = Marker.ADD
        mk.scale.x, mk.scale.y, mk.scale.z = 0.38, 0.10, 0.12
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.9, 0.2, 1.0
        mk.pose.orientation.w = 1.0
        self.mk_pub.publish(mk)


def main():
    rclpy.init()
    node = MujocoLidarRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
