"""Lai robot Go2 bang joystick ao TRONG MuJoCo (sim goc cua policy).

1 app Tkinter: khung xem MuJoCo (render offscreen glfw) + 2 can gat keo chuot
(pad vx/vy, slider wz). Physics + policy chay trong vong Tk. Khong can ROS.

Policy: unitree-go2-velocity-flat (ONNX 45->12). MJCF Menagerie go2 thu tu khop
= thu tu policy [FL,FR,RL,RR] nen khong remap. Motor torque -> d.ctrl = PD tau.
"""
import os
os.environ['MUJOCO_GL'] = 'glfw'
import sys
import math
import tkinter as tk
from tkinter import ttk
import numpy as np
import mujoco
import onnxruntime as ort
from PIL import Image, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
# scene: 'flat' (mac dinh) hoac 'obstacles' (cau thang + chuong ngai), tham so dong lenh
_scene = sys.argv[1] if len(sys.argv) > 1 else 'flat'
XML = os.path.join(HERE, 'go2_model',
                   'scene_obstacles.xml' if _scene == 'obstacles' else 'scene.xml')
ONNX = os.path.join(HERE, '..', 'models', 'policy.onnx')
DEFAULT = np.array([0.1, 0.9, -1.8, -0.1, 0.9, -1.8,
                    0.1, 0.9, -1.8, -0.1, 0.9, -1.8], np.float32)
KP = np.array([20, 20, 40] * 4, np.float32)
KD = np.array([1, 1, 2] * 4, np.float32)
SIM_DT = 0.002
DECIMATION = 10                 # policy 50Hz
STEPS_PER_TICK = 10             # sim 0.02s moi tick Tk (~real-time)
MAX_VX, MAX_VY, MAX_WZ = 1.0, 0.5, 1.0
# gioi han lenh cua policy (deploy.yaml): vx[-1,2], vy[-1,1], wz[-1,1].
# Bien duoi wz = -0.75 (khong phai -1.0): DA DO trong MuJoCo, wz <= -0.80 (xoay CW
# gap) lam robot NGA (z tut 0.33->0.22, nghieng ~28 deg); -0.75 con vung. CCW giu +1.0.
CMD_LIM = np.array([[-1.0, 2.0], [-1.0, 1.0], [-0.75, 1.0]])
W, H = 640, 400

JOY_R, SLIDE_W, PUCK = 80, 80, 14


def grav(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)], np.float32)


class Pad2D:
    """Can 2 truc (vx tien/lui, vy trai/phai), tu ve giua."""
    def __init__(self, parent):
        s = 2 * (JOY_R + PUCK + 8)
        self.cx = self.cy = s / 2
        self.c = tk.Canvas(parent, width=s, height=s, bg='#1e1e1e', highlightthickness=0)
        self.c.create_oval(self.cx-JOY_R, self.cy-JOY_R, self.cx+JOY_R, self.cy+JOY_R, outline='#4caf50', width=2)
        self.c.create_text(self.cx, self.cy-JOY_R-10, text='TIẾN', fill='white', font=('sans', 9))
        self.c.create_text(self.cx, self.cy+JOY_R+10, text='LÙI', fill='white', font=('sans', 9))
        self.p = self.c.create_oval(self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK, fill='#4caf50', outline='')
        for e in ('<Button-1>', '<B1-Motion>'):
            self.c.bind(e, self._drag)
        self.c.bind('<ButtonRelease-1>', self._rel)
        self.vx = self.vy = 0.0
    def _drag(self, e):
        dx, dy = e.x-self.cx, e.y-self.cy
        d = math.hypot(dx, dy)
        if d > JOY_R:
            dx, dy = dx*JOY_R/d, dy*JOY_R/d
        self.c.coords(self.p, self.cx+dx-PUCK, self.cy+dy-PUCK, self.cx+dx+PUCK, self.cy+dy+PUCK)
        self.vx, self.vy = -dy/JOY_R, -dx/JOY_R
    def _rel(self, _):
        self.c.coords(self.p, self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK)
        self.vx = self.vy = 0.0


class Slider1D:
    """Can 1 truc (wz xoay), tu ve giua."""
    def __init__(self, parent):
        w, h = 2*(SLIDE_W+PUCK+8), 2*(PUCK+8)
        self.cx, self.cy = w/2, h/2
        self.c = tk.Canvas(parent, width=w, height=h, bg='#1e1e1e', highlightthickness=0)
        self.c.create_line(self.cx-SLIDE_W, self.cy, self.cx+SLIDE_W, self.cy, fill='#2196f3', width=3)
        self.p = self.c.create_oval(self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK, fill='#2196f3', outline='')
        for e in ('<Button-1>', '<B1-Motion>'):
            self.c.bind(e, self._drag)
        self.c.bind('<ButtonRelease-1>', self._rel)
        self.wz = 0.0
    def _drag(self, e):
        dx = max(-SLIDE_W, min(SLIDE_W, e.x-self.cx))
        self.c.coords(self.p, self.cx+dx-PUCK, self.cy-PUCK, self.cx+dx+PUCK, self.cy+PUCK)
        self.wz = -dx/SLIDE_W
    def _rel(self, _):
        self.c.coords(self.p, self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK)
        self.wz = 0.0


class App:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(XML)
        self.m.opt.timestep = SIM_DT
        self.d = mujoco.MjData(self.m)
        self.d.qpos[2] = 0.30
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[7:] = DEFAULT
        mujoco.mj_forward(self.m, self.d)
        self.ren = mujoco.Renderer(self.m, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 120, -18, 2.2
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.counter = 0

        self.root = tk.Tk()
        self.root.title('Go2 Joystick — MuJoCo')
        self.root.configure(bg='#1e1e1e')
        self.view = tk.Label(self.root, bg='#000')
        self.view.pack(padx=10, pady=(10, 6))
        pads = tk.Frame(self.root, bg='#1e1e1e'); pads.pack(pady=(0, 6))
        lf = tk.Frame(pads, bg='#1e1e1e'); lf.grid(row=0, column=0, padx=14)
        tk.Label(lf, text='Di chuyển (vx/vy)', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.pad = Pad2D(lf); self.pad.c.pack()
        rf = tk.Frame(pads, bg='#1e1e1e'); rf.grid(row=0, column=1, padx=14)
        tk.Label(rf, text='Xoay (wz)', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.slider = Slider1D(rf); self.slider.c.pack(pady=(24, 0))

        # Thanh chinh toc do (nhan vao lenh van toc) + nut +/- buoc 0.1
        sf = tk.Frame(self.root, bg='#1e1e1e'); sf.pack(fill='x', padx=16, pady=(2, 4))
        tk.Label(sf, text='Tốc độ', fg='white', bg='#1e1e1e', font=('sans', 9)).pack(side='left')
        self.speed = tk.DoubleVar(value=1.0)
        tk.Button(sf, text='−', width=2, command=lambda: self._bump(-0.1)).pack(side='left', padx=(8, 2))
        ttk.Scale(sf, from_=0.2, to=2.0, variable=self.speed, orient='horizontal'
                  ).pack(side='left', fill='x', expand=True, padx=2)
        tk.Button(sf, text='+', width=2, command=lambda: self._bump(0.1)).pack(side='left', padx=(2, 0))
        self.status = tk.StringVar(value='vx=0.00 vy=0.00 wz=0.00')
        tk.Label(self.root, textvariable=self.status, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 4))
        tk.Button(self.root, text='RESET tư thế', bg='#37474f', fg='white',
                  command=self._reset).pack(pady=(0, 10))
        self._photo = None
        self.root.after(20, self._tick)

    def _bump(self, delta):
        self.speed.set(max(0.2, min(2.0, round(self.speed.get() + delta, 2))))

    def _reset(self):
        self.d.qpos[:] = 0
        self.d.qpos[2] = 0.30; self.d.qpos[3:7] = [1, 0, 0, 0]; self.d.qpos[7:] = DEFAULT
        self.d.qvel[:] = 0
        self.action[:] = 0; self.target = DEFAULT.copy()
        mujoco.mj_forward(self.m, self.d)

    def _tick(self):
        sp = self.speed.get()
        cmd = np.array([self.pad.vx*MAX_VX*sp, self.pad.vy*MAX_VY*sp, self.slider.wz*MAX_WZ*sp], np.float32)
        cmd = np.clip(cmd, CMD_LIM[:, 0], CMD_LIM[:, 1]).astype(np.float32)  # trong pham vi policy
        for _ in range(STEPS_PER_TICK):
            tau = KP*(self.target - self.d.qpos[7:]) - KD*self.d.qvel[6:]
            self.d.ctrl[:] = np.clip(tau, -23.7, 23.7)
            mujoco.mj_step(self.m, self.d)
            self.counter += 1
            if self.counter % DECIMATION == 0:
                obs = np.concatenate([self.d.qvel[3:6], grav(self.d.qpos[3:7]), cmd,
                                      self.d.qpos[7:]-DEFAULT, self.d.qvel[6:], self.action]).astype(np.float32)
                self.action = self.sess.run(None, {self.inp: obs[None]})[0].flatten()
                self.target = self.action*0.5 + DEFAULT
        # render (camera bam theo robot)
        self.cam.lookat[:] = self.d.qpos[:3]
        self.ren.update_scene(self.d, self.cam)
        img = Image.fromarray(self.ren.render())
        self._photo = ImageTk.PhotoImage(img)
        self.view.configure(image=self._photo)
        self.status.set(f'vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f}   '
                        f'tốc độ×{sp:.1f}   z={self.d.qpos[2]:.2f}m')
        self.root.after(20, self._tick)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    App().run()
