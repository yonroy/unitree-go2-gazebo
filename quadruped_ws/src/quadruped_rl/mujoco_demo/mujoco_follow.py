"""Go2 BAM MUC TIEU (follow-object) trong MuJoCo, dung policy RL de di.

Muc tieu la 1 qua cau (mocap body) - lai bang pad ao hoac de tu bay vong. Robot
cam nhan vi tri muc tieu (ground-truth, quy ve khung robot) -> compute_cmd (PID
khoang cach+goc, tai dung tu quadruped_follow) -> (vx,wz) -> policy RL. Robot bam
theo va giu dung stop_distance (lui lai neu qua gan). Khong can ROS/camera/YOLO.

Thu tu khop MJCF = thu tu policy [FL,FR,RL,RR] nen khong remap (giong cac app khac).
"""
import os
import sys
import math
import tkinter as tk

import numpy as np
import mujoco
import onnxruntime as ort
from PIL import Image, ImageDraw, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
# tai dung logic follow thuan + slew/clamp tu cac package (deu co self-test)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'quadruped_follow')))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', 'quadruped_navigation')))
from quadruped_follow.follow_object import compute_cmd, FollowParams
from quadruped_navigation.obstacle_avoider import SlewLimiter

XML = os.path.join(HERE, 'go2_model', 'scene_follow.xml')
ONNX = os.path.join(HERE, '..', 'models', 'policy.onnx')

DEFAULT = np.array([0.1, 0.9, -1.8, -0.1, 0.9, -1.8,
                    0.1, 0.9, -1.8, -0.1, 0.9, -1.8], np.float32)
KP = np.array([20, 20, 40] * 4, np.float32)
KD = np.array([1, 1, 2] * 4, np.float32)
SIM_DT = 0.002
DECIMATION = 10
STEPS_PER_TICK = 10
W, H = 640, 420
JOY_R, PUCK = 80, 14

# Gioi han lenh yaw an toan (DA DO trong MuJoCo): wz <= -0.80 (CW gap) lam robot NGA;
# -0.75 con vung. CCW gioi han +1.0 (dai train policy).
WZ_MIN, WZ_MAX = -0.75, 1.0
SLEW_MAX_DELTA = [0.12, 0.12, 0.15]

TARGET_SPEED = 0.9      # m/s - toc do di chuyen muc tieu khi keo pad het co
TARGET_Z = 0.25
ARENA = 4.5             # gioi han muc tieu trong +-ARENA (m)
STOP_DIST = 0.8         # m - khoang cach robot muon giu voi muc tieu
TARGET_R = 0.15         # ban kinh qua cau (m) - de tinh bounding box

# Camera FPV gan tren than robot (them vao body "base" luc chay bang MjSpec, khong sua
# go2.xml goc). Dung de "detect" muc tieu: chieu vi tri 3D qua cau -> bounding box 2D.
CAM_W, CAM_H = 400, 300
CAM_FOVY = 72.0
CAM_TILT = math.radians(15)   # cui xuong (muc tieu duoi dat khong roi khoi khung khi lai gan)
CAM_POS = [0.40, 0.0, 0.06]   # RA TRUOC HEAD robot (base frame) - x=0.28 con thay ket cau
# head che khung (DA KIEM CHUNG bang render); x=0.40 khung thoang han, muc tieu ro.


def mat2quat(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s; x = (m[2, 1]-m[1, 2])/s; y = (m[0, 2]-m[2, 0])/s; z = (m[1, 0]-m[0, 1])/s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1 + m[0, 0]-m[1, 1]-m[2, 2]) * 2
        w = (m[2, 1]-m[1, 2])/s; x = 0.25*s; y = (m[0, 1]+m[1, 0])/s; z = (m[0, 2]+m[2, 0])/s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1 + m[1, 1]-m[0, 0]-m[2, 2]) * 2
        w = (m[0, 2]-m[2, 0])/s; x = (m[0, 1]+m[1, 0])/s; y = 0.25*s; z = (m[1, 2]+m[2, 1])/s
    else:
        s = math.sqrt(1 + m[2, 2]-m[0, 0]-m[1, 1]) * 2
        w = (m[1, 0]-m[0, 1])/s; x = (m[0, 2]+m[2, 0])/s; y = (m[1, 2]+m[2, 1])/s; z = 0.25*s
    return np.array([w, x, y, z])


def _robot_cam_quat():
    """Quat huong camera: nhin ra +x (truoc) cua base, cui xuong CAM_TILT."""
    fwd = np.array([math.cos(CAM_TILT), 0.0, -math.sin(CAM_TILT)])
    z_cam = -fwd
    x_cam = np.cross(fwd, [0, 0, 1.0]); x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(z_cam, x_cam)
    return mat2quat(np.column_stack([x_cam, y_cam, z_cam]))


def grav(q):
    w, x, y, z = q
    return np.array([2*(-z*x+w*y), -2*(z*y+w*x), 1-2*(w*w+z*z)], np.float32)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class Pad2D:
    """Can 2 truc lai MUC TIEU (x truoc/sau, y trai/phai the gioi), tu ve giua."""
    def __init__(self, parent):
        s = 2 * (JOY_R + PUCK + 8)
        self.cx = self.cy = s / 2
        self.c = tk.Canvas(parent, width=s, height=s, bg='#1e1e1e', highlightthickness=0)
        self.c.create_oval(self.cx-JOY_R, self.cy-JOY_R, self.cx+JOY_R, self.cy+JOY_R,
                           outline='#ff8c42', width=2)
        self.c.create_text(self.cx, self.cy-JOY_R-10, text='+X', fill='white', font=('sans', 9))
        self.c.create_text(self.cx+JOY_R+12, self.cy, text='-Y', fill='white', font=('sans', 9))
        self.p = self.c.create_oval(self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK,
                                    fill='#ff8c42', outline='')
        for e in ('<Button-1>', '<B1-Motion>'):
            self.c.bind(e, self._drag)
        self.c.bind('<ButtonRelease-1>', self._rel)
        self.dx = self.dy = 0.0
    def _drag(self, e):
        dx, dy = e.x-self.cx, e.y-self.cy
        d = math.hypot(dx, dy)
        if d > JOY_R:
            dx, dy = dx*JOY_R/d, dy*JOY_R/d
        self.c.coords(self.p, self.cx+dx-PUCK, self.cy+dy-PUCK, self.cx+dx+PUCK, self.cy+dy+PUCK)
        self.dx, self.dy = -dy/JOY_R, -dx/JOY_R   # len=+x the gioi, trai=+y the gioi
    def _rel(self, _):
        self.c.coords(self.p, self.cx-PUCK, self.cy-PUCK, self.cx+PUCK, self.cy+PUCK)
        self.dx = self.dy = 0.0


class App:
    def __init__(self):
        # Them camera FPV vao body "base" bang MjSpec (khong sua go2.xml goc)
        spec = mujoco.MjSpec.from_file(XML)
        cam = spec.body('base').add_camera()
        cam.name = 'robot_cam'; cam.pos = CAM_POS; cam.fovy = CAM_FOVY
        cam.quat = _robot_cam_quat()
        self.m = spec.compile()
        self.cam_id = self.m.camera('robot_cam').id
        self.m.opt.timestep = SIM_DT
        self.d = mujoco.MjData(self.m)
        self.d.qpos[2] = 0.30
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[7:] = DEFAULT
        self.mocap_id = self.m.body('target').mocapid[0]
        self.tpos = np.array([2.0, 0.0, TARGET_Z])       # vi tri muc tieu
        self.d.mocap_pos[self.mocap_id] = self.tpos
        mujoco.mj_forward(self.m, self.d)
        self.ren = mujoco.Renderer(self.m, H, W)
        self.camren = mujoco.Renderer(self.m, CAM_H, CAM_W)   # view FPV robot (detect)
        self.cam = mujoco.MjvCamera()
        self.cam.azimuth, self.cam.elevation, self.cam.distance = 120, -25, 4.0
        self.sess = ort.InferenceSession(ONNX, providers=['CPUExecutionProvider'])
        self.inp = self.sess.get_inputs()[0].name
        # kp_linear cao (2.2) de ep robot tien sat stop_distance - policy RL co vung chet
        # van toc thap, kp thap thi dung som (DA DO: kp 0.9 -> dung o 0.98m thay vi 0.8m;
        # kp 2.2 -> 0.85m, khong dao dong/nga).
        self.params = FollowParams(max_linear=0.6, max_angular=0.9, kp_linear=2.2)
        self.slew = SlewLimiter(SLEW_MAX_DELTA)
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT.copy()
        self.counter = 0
        self.auto = False
        self.auto_t = 0.0

        self.root = tk.Tk()
        self.root.title('Go2 Follow-Object — MuJoCo')
        self.root.configure(bg='#1e1e1e')
        views = tk.Frame(self.root, bg='#1e1e1e'); views.pack(padx=10, pady=(10, 4))
        vl = tk.Frame(views, bg='#1e1e1e'); vl.grid(row=0, column=0, padx=(0, 6))
        tk.Label(vl, text='Cảnh (bên thứ 3)', fg='white', bg='#1e1e1e', font=('sans', 9)).pack()
        self.view = tk.Label(vl, bg='#000'); self.view.pack()
        vr = tk.Frame(views, bg='#1e1e1e'); vr.grid(row=0, column=1, padx=(6, 0))
        tk.Label(vr, text='Camera robot — DETECT', fg='#8fd', bg='#1e1e1e', font=('sans', 9)).pack()
        self.detview = tk.Label(vr, bg='#000'); self.detview.pack()
        ctl = tk.Frame(self.root, bg='#1e1e1e'); ctl.pack(pady=(0, 6))
        lf = tk.Frame(ctl, bg='#1e1e1e'); lf.grid(row=0, column=0, padx=14)
        tk.Label(lf, text='Lái MỤC TIÊU (quả cầu cam)', fg='#ff8c42', bg='#1e1e1e',
                 font=('sans', 9)).pack()
        self.pad = Pad2D(lf); self.pad.c.pack()
        rf = tk.Frame(ctl, bg='#1e1e1e'); rf.grid(row=0, column=1, padx=14)
        self.auto_var = tk.IntVar(value=0)
        tk.Checkbutton(rf, text='Tự động bay vòng', variable=self.auto_var,
                       command=self._toggle_auto, fg='white', bg='#1e1e1e',
                       selectcolor='#333', activebackground='#1e1e1e').pack(anchor='w')
        tk.Button(rf, text='RESET', bg='#37474f', fg='white', command=self._reset).pack(pady=6)
        self.status = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.status, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 6))
        self._photo = None
        self.root.after(20, self._tick)

    def _toggle_auto(self):
        self.auto = bool(self.auto_var.get())

    def _reset(self):
        self.d.qpos[:] = 0
        self.d.qpos[2] = 0.30; self.d.qpos[3:7] = [1, 0, 0, 0]; self.d.qpos[7:] = DEFAULT
        self.d.qvel[:] = 0
        self.action[:] = 0; self.target = DEFAULT.copy(); self.slew.reset()
        self.tpos = np.array([2.0, 0.0, TARGET_Z]); self.auto_t = 0.0
        mujoco.mj_forward(self.m, self.d)

    def _move_target(self, dt):
        if self.auto:
            self.auto_t += dt
            R, w = 2.2, 0.5    # ban kinh + van toc goc quay vong
            self.tpos[0] = R * math.cos(w * self.auto_t)
            self.tpos[1] = R * math.sin(w * self.auto_t)
        else:
            self.tpos[0] += self.pad.dx * TARGET_SPEED * dt
            self.tpos[1] += self.pad.dy * TARGET_SPEED * dt
            self.tpos[:2] = np.clip(self.tpos[:2], -ARENA, ARENA)
        self.tpos[2] = TARGET_Z
        self.d.mocap_pos[self.mocap_id] = self.tpos

    def _project(self, P):
        """Chieu diem the gioi P -> pixel (px,py) tren anh camera robot; None neu sau/ngoai."""
        cp = self.d.cam_xpos[self.cam_id]
        X = self.d.cam_xmat[self.cam_id].reshape(3, 3)
        rel = np.asarray(P) - cp
        xc = rel @ X[:, 0]; yc = rel @ X[:, 1]; zc = rel @ X[:, 2]
        depth = -zc                                    # camera nhin theo -z
        if depth <= 1e-3:
            return None
        f = (CAM_H / 2) / math.tan(math.radians(CAM_FOVY) / 2)
        return (CAM_W/2 + f*xc/depth, CAM_H/2 - f*yc/depth, depth, f)

    def _render_detect(self):
        """Render camera robot + ve bounding box quanh muc tieu (detect bang chieu 3D)."""
        self.camren.update_scene(self.d, camera='robot_cam')
        img = Image.fromarray(self.camren.render())
        pr = self._project(self.tpos)
        detected = False
        if pr is not None:
            px, py, depth, f = pr
            half = f * TARGET_R / depth                 # ban kinh qua cau -> nua canh box (px)
            margin = half * 1.35
            if -margin < px < CAM_W+margin and -margin < py < CAM_H+margin:
                detected = True
                dr = ImageDraw.Draw(img)
                x0, y0, x1, y1 = px-margin, py-margin, px+margin, py+margin
                dr.rectangle([x0, y0, x1, y1], outline=(60, 230, 120), width=3)
                dist = math.hypot(self.tpos[0]-self.d.qpos[0], self.tpos[1]-self.d.qpos[1])
                dr.text((max(2, x0), max(2, y0-12)), f'target {dist:.1f}m', fill=(60, 230, 120))
        if not detected:
            dr = ImageDraw.Draw(img)
            dr.text((8, 8), 'khong thay muc tieu', fill=(240, 120, 120))
        self._detected = detected
        return img

    def _tick(self):
        dt_tick = STEPS_PER_TICK * SIM_DT
        self._move_target(dt_tick)

        pos = self.d.qpos[:3]
        yaw = yaw_of(self.d.qpos[3:7])
        dx, dy = self.tpos[0] - pos[0], self.tpos[1] - pos[1]
        # quy vi tri muc tieu ve khung robot (x truoc, y trai)
        px = dx * math.cos(yaw) + dy * math.sin(yaw)
        py = -dx * math.sin(yaw) + dy * math.cos(yaw)
        lin_x, ang_z, dist, angle, in_range = compute_cmd(px, py, STOP_DIST, self.params)
        wz = float(np.clip(ang_z, WZ_MIN, WZ_MAX))
        cmd = self.slew.step([lin_x, 0.0, wz]).astype(np.float32)

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

        self.cam.lookat[:] = self.d.qpos[:3]
        self.ren.update_scene(self.d, self.cam)
        self._photo = ImageTk.PhotoImage(Image.fromarray(self.ren.render()))
        self.view.configure(image=self._photo)
        self._detphoto = ImageTk.PhotoImage(self._render_detect())   # camera robot + bbox
        self.detview.configure(image=self._detphoto)
        det = 'DETECT ✓' if getattr(self, '_detected', False) else 'mất mục tiêu'
        hold = 'GIỮ ✓' if in_range else ('lùi lại' if dist < STOP_DIST else 'bám theo')
        self.status.set(f'[{det}] [{hold}] cách={dist:.2f}m (giữ {STOP_DIST:.1f}m) '
                        f'lệch={math.degrees(angle):+.0f}°  vx={cmd[0]:+.2f} wz={cmd[2]:+.2f}')
        self.root.after(20, self._tick)

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    App().run()
