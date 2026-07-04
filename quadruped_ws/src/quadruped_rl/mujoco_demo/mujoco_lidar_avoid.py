"""Go2 tu di NE VAT CAN bang LiDAR trong MuJoCo, dung policy RL de di + dung
BAN DO occupancy (giong SLAM). Khong ROS, khong Gazebo, khong train.

Pipeline: LiDAR (mj_ray) -> compute_avoidance (logic da test, tai dung tu
quadruped_navigation) -> cmd_vel -> policy ONNX -> torque -> robot buoc.
Pose lay tu MuJoCo (ground-truth) nen mapping don gian: danh dau o trong/o vat.
"""
import os
import sys
import math
import tkinter as tk
import numpy as np

os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import onnxruntime as ort
from PIL import Image, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
# tai dung logic ne vat can (pure numpy) tu quadruped_navigation (package con)
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
N_RAYS = 90
LIDAR_H = 0.5           # tren noc than (0.35 nam trong than -> tia bi chan)
RANGE_MAX = 8.0
VIEW_W, VIEW_H = 560, 420
MAP_PX = 300
MAP_M = 8.0          # ban do +/- 8m -> 16m
MAP_RES = MAP_M * 2 / MAP_PX

_GG = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)  # raycast chi group 3 (vat can)


def grav(q):
    w, x, y, z = q
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)], np.float32)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class Lidar:
    def __init__(self, m):
        self.m = m
        self.ang = -np.pi + np.arange(N_RAYS) * (2*np.pi/N_RAYS)
        self.gid = np.zeros(1, np.int32)

    def scan(self, d, pos, yaw):
        """Tra ve (ranges robot-frame, hit_points world). front(angle0)=huong robot."""
        pnt = np.array([pos[0], pos[1], LIDAR_H], np.float64)
        ranges = np.full(N_RAYS, RANGE_MAX)
        hits = []
        for i, a in enumerate(self.ang):
            wa = a + yaw
            vec = np.array([math.cos(wa), math.sin(wa), 0.0], np.float64)
            dist = mujoco.mj_ray(self.m, d, pnt, vec, _GG, 1, -1, self.gid)
            # loc tia gan gia (self-hit/glitch < 0.2m) -> coi nhu khong trung
            if 0.2 <= dist < RANGE_MAX:
                ranges[i] = dist
                hits.append((pnt[0]+dist*vec[0], pnt[1]+dist*vec[1]))
            else:
                hits.append(None)
        return ranges, hits


class OccMap:
    """Ban do occupancy log-odds don gian (pose biet truoc tu MuJoCo)."""
    def __init__(self):
        self.g = np.zeros((MAP_PX, MAP_PX), np.float32)  # log-odds

    def _cell(self, x, y):
        cx = int((x + MAP_M) / MAP_RES)
        cy = int((y + MAP_M) / MAP_RES)
        return cx, cy

    def update(self, pos, hits):
        rx, ry = self._cell(pos[0], pos[1])
        for h in hits:
            if h is None:
                continue
            hx, hy = self._cell(h[0], h[1])
            # danh dau o TRONG doc tia (Bresenham tho)
            n = max(abs(hx-rx), abs(hy-ry))
            if n > 0:
                for k in range(n):
                    x = rx + (hx-rx)*k//n
                    y = ry + (hy-ry)*k//n
                    if 0 <= x < MAP_PX and 0 <= y < MAP_PX:
                        self.g[y, x] = max(-5, self.g[y, x] - 0.4)
            if 0 <= hx < MAP_PX and 0 <= hy < MAP_PX:
                self.g[hy, hx] = min(8, self.g[hy, hx] + 1.2)  # o VAT CAN

    def image(self, pos, yaw):
        # unknown=xam, free=sang, occ=dam
        img = np.full((MAP_PX, MAP_PX, 3), 130, np.uint8)
        img[self.g < -0.5] = (215, 220, 225)   # trong
        img[self.g > 0.5] = (40, 60, 90)        # vat can
        pil = Image.fromarray(img[::-1])        # lat truc y len tren
        return pil


class App:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.m.opt.timestep = SIM_DT
        self.d = mujoco.MjData(self.m)
        self.d.qpos[2] = 0.30
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[7:] = DEFAULT
        mujoco.mj_forward(self.m, self.d)
        self.ren = mujoco.Renderer(self.m, VIEW_H, VIEW_W)
        self.cam = mujoco.MjvCamera()
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 120, -35, 3.2
        # bat hien group 3 (vat can) - mac dinh renderer an group 3,4,5
        self.opt = mujoco.MjvOption(); self.opt.geomgroup[3] = 1
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.lidar = Lidar(self.m)
        self.occ = OccMap()
        self.params = AvoidParams(cruise_vx=0.55, max_wz=1.2, clear_dist=1.0, stop_dist=0.5)
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.cmd = np.zeros(3, np.float32)
        self.counter = 0

        self.root = tk.Tk()
        self.root.title('Go2 LiDAR né vật cản — MuJoCo')
        self.root.configure(bg='#1e1e1e')
        top = tk.Frame(self.root, bg='#1e1e1e'); top.pack(padx=10, pady=10)
        lf = tk.Frame(top, bg='#1e1e1e'); lf.grid(row=0, column=0, padx=6)
        tk.Label(lf, text='MuJoCo — robot tự né', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.view = tk.Label(lf, bg='#000'); self.view.pack()
        rf = tk.Frame(top, bg='#1e1e1e'); rf.grid(row=0, column=1, padx=6)
        tk.Label(rf, text='Bản đồ LiDAR (occupancy)', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.mapv = tk.Label(rf, bg='#000'); self.mapv.pack()
        self.status = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.status, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 4))
        tk.Button(self.root, text='RESET', bg='#37474f', fg='white', command=self._reset).pack(pady=(0, 8))
        self._p1 = self._p2 = None
        self.root.after(30, self._tick)

    def _reset(self):
        self.d.qpos[:] = 0
        self.d.qpos[2] = 0.30; self.d.qpos[3:7] = [1, 0, 0, 0]; self.d.qpos[7:] = DEFAULT
        self.d.qvel[:] = 0; self.action[:] = 0; self.target = DEFAULT.copy()
        self.occ = OccMap()
        mujoco.mj_forward(self.m, self.d)

    def _tick(self):
        pos = self.d.qpos[:3].copy()
        yaw = yaw_of(self.d.qpos[3:7])
        # LiDAR + ne (moi tick)
        ranges, hits = self.lidar.scan(self.d, pos, yaw)
        vx, wz = compute_avoidance(ranges, -np.pi, 2*np.pi/N_RAYS, RANGE_MAX, self.params)
        self.cmd[:] = [vx, 0.0, wz]
        self.occ.update(pos, hits)

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

        self.cam.lookat[:] = self.d.qpos[:3]
        self.ren.update_scene(self.d, self.cam, self.opt)
        self._p1 = ImageTk.PhotoImage(Image.fromarray(self.ren.render()))
        self.view.configure(image=self._p1)
        self._p2 = ImageTk.PhotoImage(self.occ.image(pos, yaw).resize((MAP_PX, MAP_PX)))
        self.mapv.configure(image=self._p2)
        self.status.set(f'vx={vx:+.2f} wz={wz:+.2f}   vật gần nhất={ranges.min():.2f}m   pos=({pos[0]:+.1f},{pos[1]:+.1f})')
        self.root.after(30, self._tick)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    App().run()
