"""Bang dieu khien joystick ao (Tkinter) - publish /cmd_vel de di chuyen Go2
theo moi huong (tien/lui, sang trai/phai, xoay trai/phai, cheo).

Khong can joystick/gamepad vat ly: keo chuot tren 2 "can" ao tren man hinh.
- Can trai (hinh tron): vi tri tuong doi -> vx (tien/lui), vy (trai/phai - strafe,
  loi the cua robot chan so voi robot banh xe, xem §3 tai lieu).
- Can phai (thanh ngang): vi tri tuong doi -> wz (xoay trai/phai).
- Tha chuot -> can tu dong ve giua -> /cmd_vel = 0 (an toan, giong lo xo hoi).
"""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

PUBLISH_RATE_HZ = 20
BASE_MAX_LINEAR = 0.3   # m/s - khop voi toc do da kiem chung on dinh trong Gazebo
BASE_MAX_ANGULAR = 0.5  # rad/s - khop voi toc do da kiem chung on dinh trong Gazebo

JOY_RADIUS = 90
SLIDER_HALF_WIDTH = 90
PUCK_RADIUS = 16


class _DragPad2D:
    """Can ao 2 truc (vx/vy), tu ve giua khi tha chuot."""

    def __init__(self, parent):
        size = 2 * (JOY_RADIUS + PUCK_RADIUS + 10)
        self.cx = self.cy = size / 2
        self.canvas = tk.Canvas(parent, width=size, height=size,
                                 bg='#1e1e1e', highlightthickness=0)
        self.canvas.create_oval(
            self.cx - JOY_RADIUS, self.cy - JOY_RADIUS,
            self.cx + JOY_RADIUS, self.cy + JOY_RADIUS,
            outline='#4caf50', width=2,
        )
        self.canvas.create_text(self.cx, self.cy - JOY_RADIUS - 14, text='TIẾN', fill='white')
        self.canvas.create_text(self.cx, self.cy + JOY_RADIUS + 14, text='LÙI', fill='white')
        self.canvas.create_text(self.cx - JOY_RADIUS - 16, self.cy, text='TRÁI', fill='white')
        self.canvas.create_text(self.cx + JOY_RADIUS + 16, self.cy, text='PHẢI', fill='white')
        self.puck = self.canvas.create_oval(
            self.cx - PUCK_RADIUS, self.cy - PUCK_RADIUS,
            self.cx + PUCK_RADIUS, self.cy + PUCK_RADIUS,
            fill='#4caf50', outline='',
        )
        self.canvas.bind('<Button-1>', self._drag)
        self.canvas.bind('<B1-Motion>', self._drag)
        self.canvas.bind('<ButtonRelease-1>', self._release)
        self.vx_norm = 0.0
        self.vy_norm = 0.0

    def widget(self):
        return self.canvas

    def _drag(self, event):
        dx = event.x - self.cx
        dy = event.y - self.cy
        dist = math.hypot(dx, dy)
        if dist > JOY_RADIUS:
            dx = dx * JOY_RADIUS / dist
            dy = dy * JOY_RADIUS / dist
        self.canvas.coords(
            self.puck,
            self.cx + dx - PUCK_RADIUS, self.cy + dy - PUCK_RADIUS,
            self.cx + dx + PUCK_RADIUS, self.cy + dy + PUCK_RADIUS,
        )
        self.vx_norm = -dy / JOY_RADIUS
        self.vy_norm = -dx / JOY_RADIUS

    def _release(self, _event):
        self.canvas.coords(
            self.puck,
            self.cx - PUCK_RADIUS, self.cy - PUCK_RADIUS,
            self.cx + PUCK_RADIUS, self.cy + PUCK_RADIUS,
        )
        self.vx_norm = 0.0
        self.vy_norm = 0.0


class _DragSlider1D:
    """Can ao 1 truc (wz), tu ve giua khi tha chuot."""

    def __init__(self, parent):
        width = 2 * (SLIDER_HALF_WIDTH + PUCK_RADIUS + 10)
        height = 2 * (PUCK_RADIUS + 10)
        self.cx = width / 2
        self.cy = height / 2
        self.canvas = tk.Canvas(parent, width=width, height=height,
                                 bg='#1e1e1e', highlightthickness=0)
        self.canvas.create_line(
            self.cx - SLIDER_HALF_WIDTH, self.cy,
            self.cx + SLIDER_HALF_WIDTH, self.cy,
            fill='#2196f3', width=3,
        )
        self.canvas.create_text(self.cx - SLIDER_HALF_WIDTH - 20, self.cy, text='⟲ TRÁI', fill='white')
        self.canvas.create_text(self.cx + SLIDER_HALF_WIDTH + 20, self.cy, text='PHẢI ⟳', fill='white')
        self.puck = self.canvas.create_oval(
            self.cx - PUCK_RADIUS, self.cy - PUCK_RADIUS,
            self.cx + PUCK_RADIUS, self.cy + PUCK_RADIUS,
            fill='#2196f3', outline='',
        )
        self.canvas.bind('<Button-1>', self._drag)
        self.canvas.bind('<B1-Motion>', self._drag)
        self.canvas.bind('<ButtonRelease-1>', self._release)
        self.wz_norm = 0.0

    def widget(self):
        return self.canvas

    def _drag(self, event):
        dx = event.x - self.cx
        dx = max(-SLIDER_HALF_WIDTH, min(SLIDER_HALF_WIDTH, dx))
        self.canvas.coords(
            self.puck,
            self.cx + dx - PUCK_RADIUS, self.cy - PUCK_RADIUS,
            self.cx + dx + PUCK_RADIUS, self.cy + PUCK_RADIUS,
        )
        self.wz_norm = -dx / SLIDER_HALF_WIDTH

    def _release(self, _event):
        self.canvas.coords(
            self.puck,
            self.cx - PUCK_RADIUS, self.cy - PUCK_RADIUS,
            self.cx + PUCK_RADIUS, self.cy + PUCK_RADIUS,
        )
        self.wz_norm = 0.0


class JoystickPanel:
    def __init__(self, node: Node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_scale = 1.0

        self.root = tk.Tk()
        self.root.title('Go2 Joystick Panel')
        self.root.configure(bg='#1e1e1e')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        pads = tk.Frame(self.root, bg='#1e1e1e')
        pads.pack(padx=16, pady=12)

        left_col = tk.Frame(pads, bg='#1e1e1e')
        left_col.grid(row=0, column=0, padx=12)
        tk.Label(left_col, text='Di chuyển (vx / vy)', fg='white', bg='#1e1e1e').pack()
        self.pad2d = _DragPad2D(left_col)
        self.pad2d.widget().pack()

        right_col = tk.Frame(pads, bg='#1e1e1e')
        right_col.grid(row=0, column=1, padx=12, sticky='n')
        tk.Label(right_col, text='Xoay (wz)', fg='white', bg='#1e1e1e').pack()
        self.slider1d = _DragSlider1D(right_col)
        self.slider1d.widget().pack(pady=(30, 0))

        speed_frame = tk.Frame(self.root, bg='#1e1e1e')
        speed_frame.pack(fill='x', padx=16, pady=(0, 8))
        tk.Label(speed_frame, text='Tốc độ', fg='white', bg='#1e1e1e').pack(side='left')
        self.speed_var = tk.DoubleVar(value=1.0)
        ttk.Scale(speed_frame, from_=0.2, to=1.5, variable=self.speed_var,
                  orient='horizontal').pack(side='left', fill='x', expand=True, padx=8)

        self.status_var = tk.StringVar(value='vx=0.00  vy=0.00  wz=0.00')
        tk.Label(self.root, textvariable=self.status_var, fg='#9e9e9e', bg='#1e1e1e',
                 font=('monospace', 10)).pack(pady=(0, 4))

        tk.Button(self.root, text='DỪNG KHẨN', bg='#c62828', fg='white',
                  font=('sans-serif', 11, 'bold'), command=self._emergency_stop
                  ).pack(fill='x', padx=16, pady=(0, 12))

        self._publish_loop()

    def _emergency_stop(self):
        self.pad2d._release(None)
        self.slider1d._release(None)
        self._send(0.0, 0.0, 0.0)

    def _send(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub.publish(msg)
        self.status_var.set(f'vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}')

    def _publish_loop(self):
        scale = self.speed_var.get()
        vx = self.pad2d.vx_norm * BASE_MAX_LINEAR * scale
        vy = self.pad2d.vy_norm * BASE_MAX_LINEAR * scale
        wz = self.slider1d.wz_norm * BASE_MAX_ANGULAR * scale
        self._send(vx, vy, wz)
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(int(1000 / PUBLISH_RATE_HZ), self._publish_loop)

    def _on_close(self):
        self._send(0.0, 0.0, 0.0)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = Node('joystick_panel')
    panel = JoystickPanel(node)
    try:
        panel.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
