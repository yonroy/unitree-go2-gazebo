"""Go2 DIEU HUONG TOI DICH (A* + pure pursuit) tren ban do LiDAR, trong MuJoCo.

Click 1 diem tren ban do -> A* lap duong ne vat can -> pure pursuit -> policy RL di.
Reactive avoidance lam lop AN TOAN (vat can bat ngo/chua tren ban do). Khong ROS.

Tai dung: planner.py (A*, pure_pursuit) + obstacle_avoider.py (compute_avoidance) tu
quadruped_navigation (deu co self-test). Policy: unitree-go2-velocity-flat (ONNX).
"""
import os
import sys
import math
import tkinter as tk
import numpy as np

os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import onnxruntime as ort
from PIL import Image, ImageDraw, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'quadruped_navigation')))
from quadruped_navigation.planner import astar, inflate, pure_pursuit
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
LIDAR_H = 0.5
RANGE_MAX = 8.0
# Gioi han lenh yaw an toan (DA DO trong MuJoCo): wz <= -0.80 (xoay CW gap) lam robot
# NGA (z tut 0.33->0.22, nghieng ~28 deg); -0.75 con vung. CCW gioi han +1.0 (range train).
WZ_MIN, WZ_MAX = -0.75, 1.0
VIEW_W, VIEW_H = 520, 400
MAP_PX = 300
MAP_M = 4.0
MAP_RES = MAP_M * 2 / MAP_PX
ROBOT_R_CELLS = int(0.35 / MAP_RES)   # phinh vat can theo ban kinh robot
SAFETY_DIST = 0.5                     # vat can gan hon -> reactive override
REPLAN_EVERY = 40                     # tick (~1.2s) giua 2 lan A*
_GG = np.array([0, 0, 0, 1, 0, 0], np.uint8)


def grav(q):
    w, x, y, z = q
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)], np.float32)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def w2c(wx, wy):
    return int((wx + MAP_M) / MAP_RES), int((wy + MAP_M) / MAP_RES)


def c2w(cx, cy):
    return cx * MAP_RES - MAP_M, cy * MAP_RES - MAP_M


def c2disp(cx, cy):
    return int(cx), int(MAP_PX - 1 - cy)   # anh lat truc y


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
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 120, -40, 3.5
        self.opt = mujoco.MjvOption(); self.opt.geomgroup[3] = 1   # hien vat can (group 3)
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.ang = -np.pi + np.arange(N_RAYS) * (2*np.pi/N_RAYS)
        self.gid = np.zeros(1, np.int32)
        self.occ = np.zeros((MAP_PX, MAP_PX), np.float32)
        self.params = AvoidParams(cruise_vx=0.55, max_wz=1.2, clear_dist=1.0, stop_dist=0.5)
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.cmd = np.zeros(3, np.float32)
        self.counter = 0
        self.tick_n = 0
        self.goal = None          # (wx, wy)
        self.path_world = []
        self.mode = 'ĐỨNG'

        self.root = tk.Tk()
        self.root.title('Go2 Nav — A* + LiDAR (MuJoCo)')
        self.root.configure(bg='#1e1e1e')
        top = tk.Frame(self.root, bg='#1e1e1e'); top.pack(padx=10, pady=10)
        lf = tk.Frame(top, bg='#1e1e1e'); lf.grid(row=0, column=0, padx=6)
        tk.Label(lf, text='MuJoCo', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.view = tk.Label(lf, bg='#000'); self.view.pack()
        rf = tk.Frame(top, bg='#1e1e1e'); rf.grid(row=0, column=1, padx=6)
        tk.Label(rf, text='Bản đồ — CLICK để đặt đích', fg='#8fd', bg='#1e1e1e', font=('sans', 9)).pack()
        self.mapv = tk.Label(rf, bg='#000'); self.mapv.pack()
        self.mapv.bind('<Button-1>', self._on_click)
        self.status = tk.StringVar(value='Click lên bản đồ để chọn đích')
        tk.Label(self.root, textvariable=self.status, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 4))
        tk.Button(self.root, text='DỪNG / xoá đích', bg='#37474f', fg='white',
                  command=self._clear).pack(pady=(0, 8))
        self._p1 = self._p2 = None
        self.root.after(30, self._tick)

    def _on_click(self, e):
        cx = int(e.x); cy = MAP_PX - 1 - int(e.y)   # dao lai truc y
        self.goal = c2w(cx, cy)
        self.path_world = []

    def _clear(self):
        self.goal = None
        self.path_world = []

    def _scan(self, pos, yaw):
        pnt = np.array([pos[0], pos[1], LIDAR_H], np.float64)
        r = np.full(N_RAYS, RANGE_MAX)
        hits = []
        for i, a in enumerate(self.ang):
            wa = a + yaw
            v = np.array([math.cos(wa), math.sin(wa), 0.0], np.float64)
            dist = mujoco.mj_ray(self.m, self.d, pnt, v, _GG, 1, -1, self.gid)
            if 0.2 <= dist < RANGE_MAX:
                r[i] = dist
                hits.append((pnt[0]+dist*v[0], pnt[1]+dist*v[1]))
            else:
                hits.append(None)
        return r, hits

    def _map_update(self, pos, hits):
        rx, ry = w2c(pos[0], pos[1])
        for h in hits:
            if h is None:
                continue
            hx, hy = w2c(h[0], h[1])
            n = max(abs(hx-rx), abs(hy-ry))
            for k in range(n):
                x = rx + (hx-rx)*k//n; y = ry + (hy-ry)*k//n
                if 0 <= x < MAP_PX and 0 <= y < MAP_PX:
                    self.occ[y, x] = max(-5, self.occ[y, x] - 0.4)
            if 0 <= hx < MAP_PX and 0 <= hy < MAP_PX:
                self.occ[hy, hx] = min(8, self.occ[hy, hx] + 1.2)

    def _replan(self, pos):
        if self.goal is None:
            self.path_world = []
            return
        blocked = inflate(self.occ > 0.5, ROBOT_R_CELLS)
        start = w2c(pos[0], pos[1])
        goal = w2c(self.goal[0], self.goal[1])
        # neu o start bi ke't vao vung phinh -> tam bo chan tai start
        if 0 <= start[1] < MAP_PX and 0 <= start[0] < MAP_PX:
            blocked[start[1], start[0]] = False
        cells = astar(blocked, start, goal)
        self.path_world = [c2w(x, y) for x, y in cells] if cells else []

    def _tick(self):
        pos = self.d.qpos[:3].copy()
        yaw = yaw_of(self.d.qpos[3:7])
        r, hits = self._scan(pos, yaw)
        self._map_update(pos, hits)
        self.tick_n += 1
        if self.tick_n % REPLAN_EVERY == 0:
            self._replan(pos)

        near = float(r.min())
        vy = 0.0
        if self.goal is None:
            vx, wz = 0.0, 0.0; self.mode = 'ĐỨNG (chưa có đích)'
        elif near < SAFETY_DIST:
            vx, wz = compute_avoidance(r, -np.pi, 2*np.pi/N_RAYS, RANGE_MAX, self.params)
            self.mode = 'NÉ khẩn cấp'
        else:
            # theo duong A*; neu chua co path -> di THANG toi dich (safety van ne)
            path = self.path_world if self.path_world else [self.goal]
            vx, vy, wz, reached = pure_pursuit(path, pos[0], pos[1], yaw,
                                               cruise=0.6, lookahead=0.9, max_vy=0.7)
            if reached:
                self.goal = None; self.path_world = []; vx = vy = wz = 0.0; self.mode = 'ĐÃ TỚI ĐÍCH'
            else:
                self.mode = 'ĐI theo A*' if self.path_world else 'đi thẳng tới đích'
        wz = float(np.clip(wz, WZ_MIN, WZ_MAX))  # tranh vung nga CW (xem WZ_MIN)
        self.cmd[:] = [vx, vy, wz]

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
        self._p2 = ImageTk.PhotoImage(self._map_img(pos))
        self.mapv.configure(image=self._p2)
        self.status.set(f'[{self.mode}]  vật gần={near:.2f}m  vx={self.cmd[0]:+.2f} wz={self.cmd[2]:+.2f}'
                        + (f'  đích=({self.goal[0]:+.1f},{self.goal[1]:+.1f})' if self.goal else ''))
        self.root.after(30, self._tick)

    def _map_img(self, pos):
        mp = np.full((MAP_PX, MAP_PX, 3), 130, np.uint8)
        mp[self.occ < -0.5] = (215, 220, 225)
        mp[self.occ > 0.5] = (40, 60, 90)
        img = Image.fromarray(mp[::-1])
        dr = ImageDraw.Draw(img)
        if self.path_world:
            pts = [c2disp(*w2c(wx, wy)) for wx, wy in self.path_world]
            dr.line(pts, fill=(60, 220, 120), width=2)
        if self.goal:
            gx, gy = c2disp(*w2c(*self.goal)); dr.ellipse([gx-5, gy-5, gx+5, gy+5], outline=(240, 80, 80), width=2)
        rx, ry = c2disp(*w2c(pos[0], pos[1])); dr.ellipse([rx-4, ry-4, rx+4, ry+4], fill=(80, 160, 255))
        return img

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    App().run()
