"""Go2 DI TOI TOA DO (goto-point) tren MAT PHANG (khong vat can), trong MuJoCo.

Chon dich bang NUT toa do dat san hoac CLICK ban do -> robot xoay ve huong dich roi
di thang toi, dung khi den noi. Dung logic thuan goto_point.py (unicycle: xoay ve
huong -> tien; da co self-test) -> (vx,wz) -> policy RL di. Khong LiDAR/A*/ne vat can.

Tai dung goto_point.py tu quadruped_navigation (co self-test).
"""
import os
import sys
import math
import tkinter as tk
from collections import deque

import numpy as np
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import onnxruntime as ort
from PIL import Image, ImageDraw, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'quadruped_navigation')))
from quadruped_navigation.goto_point import GotoPointParams, wrap_to_pi
from quadruped_navigation.obstacle_avoider import SlewLimiter

# Mat phang: dung scene.xml (khong vat can). Van cho doi canh qua tham so dong lenh.
_scene = sys.argv[1] if len(sys.argv) > 1 else None
XML = os.path.join(HERE, 'go2_model', f'scene_{_scene}.xml' if _scene else 'scene.xml')
ONNX = os.path.join(HERE, '..', 'models', 'policy.onnx')
DEFAULT = np.array([0.1, 0.9, -1.8, -0.1, 0.9, -1.8,
                    0.1, 0.9, -1.8, -0.1, 0.9, -1.8], np.float32)
KP = np.array([20, 20, 40] * 4, np.float32)
KD = np.array([1, 1, 2] * 4, np.float32)
SIM_DT = 0.002
DECIMATION = 10
STEPS_PER_TICK = 18
WZ_MIN, WZ_MAX = -0.75, 1.0          # clamp yaw (wz<=-0.80 lam robot NGA - DA DO)
SLEW_MAX_DELTA = [0.12, 0.12, 0.15]
VIEW_W, VIEW_H = 520, 400
MAP_PX = 300
MAP_M = 3.5                          # ban do +-3.5m
MAP_RES = MAP_M * 2 / MAP_PX
GOAL_TOL = 0.25                      # coi la "da toi" khi cach dich duoi nguong nay (m)
# kp_linear/max_linear cao hon mac dinh (Gazebo 0.3) vi policy MuJoCo di duoc nhanh hon
GOTO_PARAMS = GotoPointParams(max_linear=0.6, max_angular=0.9, kp_linear=1.2,
                              position_tolerance=GOAL_TOL)
PRESETS = [('↗ (2.5,2.5)', (2.5, 2.5)), ('↖ (-2.5,2.5)', (-2.5, 2.5)),
           ('↙ (-2.5,-2.5)', (-2.5, -2.5)), ('↘ (2.5,-2.5)', (2.5, -2.5)),
           ('• (0,0)', (0.0, 0.0))]


def grav(q):
    w, x, y, z = q
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)], np.float32)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def w2disp(wx, wy):
    px = int((wx + MAP_M) / MAP_RES)
    py = int(MAP_PX - 1 - (wy + MAP_M) / MAP_RES)   # anh lat truc y
    return px, py


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
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 120, -40, 4.5
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.slew = SlewLimiter(SLEW_MAX_DELTA)
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.counter = 0
        self.goal = None
        self.trail = deque(maxlen=400)
        self.mode = 'ĐỨNG'

        self.root = tk.Tk()
        self.root.title('Go2 Goto-Point — mặt phẳng (MuJoCo)')
        self.root.configure(bg='#1e1e1e')
        top = tk.Frame(self.root, bg='#1e1e1e'); top.pack(padx=10, pady=10)
        lf = tk.Frame(top, bg='#1e1e1e'); lf.grid(row=0, column=0, padx=6)
        tk.Label(lf, text='MuJoCo', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.view = tk.Label(lf, bg='#000'); self.view.pack()
        rf = tk.Frame(top, bg='#1e1e1e'); rf.grid(row=0, column=1, padx=6)
        tk.Label(rf, text='Bản đồ — CLICK hoặc chọn toạ độ', fg='#8fd', bg='#1e1e1e',
                 font=('sans', 9)).pack()
        self.mapv = tk.Label(rf, bg='#000'); self.mapv.pack()
        self.mapv.bind('<Button-1>', self._on_click)
        pf = tk.Frame(self.root, bg='#1e1e1e'); pf.pack(pady=(0, 4))
        tk.Label(pf, text='Đi tới:', fg='white', bg='#1e1e1e', font=('sans', 9)).pack(side='left')
        for label, xy in PRESETS:
            tk.Button(pf, text=label, bg='#37474f', fg='white',
                      command=lambda p=xy: self._set_goal(p)).pack(side='left', padx=3)
        tk.Button(pf, text='DỪNG', bg='#5a2d2d', fg='white', command=self._clear).pack(side='left', padx=8)
        self.status = tk.StringVar(value='Chọn toạ độ để đi tới')
        tk.Label(self.root, textvariable=self.status, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 6))
        self._p1 = self._p2 = None
        self.root.after(30, self._tick)

    def _set_goal(self, xy):
        self.goal = (float(xy[0]), float(xy[1])); self.slew.reset()

    def _on_click(self, e):
        wx = int(e.x) * MAP_RES - MAP_M
        wy = (MAP_PX - 1 - int(e.y)) * MAP_RES - MAP_M
        self._set_goal((wx, wy))

    def _clear(self):
        self.goal = None; self.slew.reset()

    def _tick(self):
        pos = self.d.qpos[:3].copy(); yaw = yaw_of(self.d.qpos[3:7])
        self.trail.append((pos[0], pos[1]))

        dist = herr = 0.0
        if self.goal is None:
            vx, wz = 0.0, 0.0; self.mode = 'ĐỨNG (chưa có đích)'
        else:
            dx = self.goal[0] - pos[0]; dy = self.goal[1] - pos[1]
            dist = math.hypot(dx, dy)
            if dist < GOAL_TOL:
                self.goal = None; vx, wz = 0.0, 0.0; self.mode = 'ĐÃ TỚI ĐÍCH ✓'
            else:
                p = GOTO_PARAMS
                herr = wrap_to_pi(math.atan2(dy, dx) - yaw)
                wz = max(-p.max_angular, min(p.max_angular, p.kp_angular * herr))
                # XOAY-KHI-DI (arc toi dich): policy MuJoCo xoay TAI CHO (nhat la CW) rat kem
                # (DA DO: xoay CW ~2 deg/s roi stall) nhung xoay khi DANG TIEN thi tot. Nen
                # luon giu it van toc tien (giam khi lech huong nhieu) de robot vong toi dich.
                fwd = max(0.25, math.cos(herr) + 0.55)
                vx = min(p.max_linear, p.max_linear * min(1.0, dist / 0.8) * fwd)
                self.mode = 'đi thẳng tới đích' if abs(herr) < 0.5 else 'vòng tới đích'
        wz = float(np.clip(wz, WZ_MIN, WZ_MAX))
        if vx == 0.0 and wz == 0.0:
            self.slew.reset(); cmd = np.zeros(3, np.float32)
        else:
            cmd = self.slew.step([vx, 0.0, wz]).astype(np.float32)

        for _ in range(STEPS_PER_TICK):
            tau = KP*(self.target - self.d.qpos[7:]) - KD*self.d.qvel[6:]
            self.d.ctrl[:] = np.clip(tau, -23.7, 23.7)
            mujoco.mj_step(self.m, self.d); self.counter += 1
            if self.counter % DECIMATION == 0:
                obs = np.concatenate([self.d.qvel[3:6], grav(self.d.qpos[3:7]), cmd,
                                      self.d.qpos[7:]-DEFAULT, self.d.qvel[6:], self.action]).astype(np.float32)
                self.action = self.sess.run(None, {self.inp: obs[None]})[0].flatten()
                self.target = self.action*0.5 + DEFAULT

        self.cam.lookat[:] = self.d.qpos[:3]
        self.ren.update_scene(self.d, self.cam)
        self._p1 = ImageTk.PhotoImage(Image.fromarray(self.ren.render()))
        self.view.configure(image=self._p1)
        self._p2 = ImageTk.PhotoImage(self._map_img(pos, yaw))
        self.mapv.configure(image=self._p2)
        self.status.set(f'[{self.mode}]'
                        + (f'  đích=({self.goal[0]:+.1f},{self.goal[1]:+.1f}) còn {dist:.2f}m '
                           f'lệch={math.degrees(herr):+.0f}°' if self.goal else ''))
        self.root.after(30, self._tick)

    def _map_img(self, pos, yaw):
        img = Image.new('RGB', (MAP_PX, MAP_PX), (38, 46, 56))
        dr = ImageDraw.Draw(img)
        for k in range(-3, 4):                       # luoi 1m
            c = int((k + MAP_M) / MAP_RES)
            dr.line([(c, 0), (c, MAP_PX)], fill=(55, 65, 78))
            dr.line([(0, c), (MAP_PX, c)], fill=(55, 65, 78))
        dr.line([w2disp(0, -MAP_M), w2disp(0, MAP_M)], fill=(70, 80, 95))
        dr.line([w2disp(-MAP_M, 0), w2disp(MAP_M, 0)], fill=(70, 80, 95))
        if len(self.trail) > 1:                      # duong da di
            dr.line([w2disp(x, y) for x, y in self.trail], fill=(60, 180, 110), width=2)
        if self.goal:
            gx, gy = w2disp(*self.goal)
            dr.ellipse([gx-6, gy-6, gx+6, gy+6], outline=(240, 80, 80), width=2)
            dr.line([(gx-8, gy), (gx+8, gy)], fill=(240, 80, 80))
            dr.line([(gx, gy-8), (gx, gy+8)], fill=(240, 80, 80))
        rx, ry = w2disp(pos[0], pos[1])              # robot + huong
        hx, hy = w2disp(pos[0]+0.35*math.cos(yaw), pos[1]+0.35*math.sin(yaw))
        dr.line([(rx, ry), (hx, hy)], fill=(120, 200, 255), width=2)
        dr.ellipse([rx-5, ry-5, rx+5, ry+5], fill=(80, 160, 255))
        return img

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    App().run()
