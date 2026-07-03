"""Bang dieu khien joystick ao (Tkinter) - publish /cmd_vel de di chuyen Go2
theo moi huong (tien/lui, sang trai/phai, xoay trai/phai, cheo), kem khung
hien thi camera truc tiep (subscribe /camera/image).

Khong can joystick/gamepad vat ly: keo chuot tren 2 "can" ao tren man hinh.
- Can trai (hinh tron): vi tri tuong doi -> vx (tien/lui), vy (trai/phai - strafe,
  loi the cua robot chan so voi robot banh xe, xem §3 tai lieu).
- Can phai (thanh ngang): vi tri tuong doi -> wz (xoay trai/phai).
- Tha chuot -> can tu dong ve giua -> /cmd_vel = 0 (an toan, giong lo xo hoi).
- Khung camera phia tren: anh tu camera RGBD tren robot (dung chung view voi
  track_object). Neu chua co topic /camera/image thi hien "cho camera...".
"""
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# Camera hien thi la tuy chon: neu thieu cv_bridge/PIL thi panel van chay,
# chi bo khung camera (khong lam hong chuc nang joystick chinh).
try:
    from cv_bridge import CvBridge
    from PIL import Image as PILImage, ImageDraw, ImageFont, ImageTk
    from sensor_msgs.msg import Image as RosImage
    from vision_msgs.msg import Detection2DArray
    _CAMERA_AVAILABLE = True
    try:
        _BBOX_FONT = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 15)
    except OSError:
        _BBOX_FONT = ImageFont.load_default()  # font mac dinh (nho hon) neu thieu ttf
except ImportError as _exc:
    _CAMERA_IMPORT_ERROR = _exc
    _CAMERA_AVAILABLE = False

PUBLISH_RATE_HZ = 20
BASE_MAX_LINEAR = 0.3   # m/s - khop voi toc do da kiem chung on dinh trong Gazebo
BASE_MAX_ANGULAR = 0.5  # rad/s - khop voi toc do da kiem chung on dinh trong Gazebo

JOY_RADIUS = 90
SLIDER_HALF_WIDTH = 90
PUCK_RADIUS = 16

CAMERA_TOPIC = '/camera/image'
CAM_W = 400          # kich thuoc hien thi (camera goc 640x480, giu ty le 4:3)
CAM_H = 300
CAM_UPDATE_HZ = 15   # khop update_rate cua rgbd_camera sensor


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

        self._setup_camera()

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
        if self._camera_enabled:
            self._update_camera()

    def _setup_camera(self):
        """Tao khung camera + subscribe /camera/image (neu co du thu vien)."""
        cam_frame = tk.Frame(self.root, bg='#1e1e1e')
        cam_frame.pack(padx=16, pady=(12, 0))
        tk.Label(cam_frame, text='Camera (/camera/image)', fg='white', bg='#1e1e1e').pack()

        self._camera_enabled = _CAMERA_AVAILABLE
        self._latest_frame = None
        self._latest_dets = []  # list (x1,y1,x2,y2,label,score) - toa do pixel goc 640x480
        self._photo = None  # giu tham chieu tranh bi garbage-collect

        if not self._camera_enabled:
            # Khong co PIL/cv_bridge -> chi 1 dong text (width/height la don vi
            # ky tu khi Label khong co image, dat vua du).
            tk.Label(cam_frame, text='(thieu cv_bridge/PIL - khong hien camera)',
                     fg='#9e9e9e', bg='#000000', width=44, height=2).pack()
            self.node.get_logger().warn(f'Camera panel tat: {_CAMERA_IMPORT_ERROR}')
            return

        # Anh placeholder den CAM_W x CAM_H de giu dung kich thuoc (pixel) tu dau,
        # kem text "cho camera..." de len tren (compound='center').
        placeholder = PILImage.new('RGB', (CAM_W, CAM_H), (0, 0, 0))
        self._photo = ImageTk.PhotoImage(placeholder)
        self.cam_label = tk.Label(
            cam_frame, image=self._photo, text='cho camera...', compound='center',
            fg='#9e9e9e', bg='#000000', font=('sans-serif', 12),
        )
        self.cam_label.pack()

        self._bridge = CvBridge()
        self.node.create_subscription(RosImage, CAMERA_TOPIC, self._on_image, 1)
        # /detections (tu quadruped_perception/object_detector) de ve bounding box
        # len anh. Neu khong chay perception thi khong co box (panel van hien anh).
        self.node.create_subscription(Detection2DArray, '/detections', self._on_detections, 5)

    def _on_image(self, msg):
        # spin_once chay trong luong Tk (o _publish_loop) nen callback nay cung
        # o luong Tk - chi luu frame, viec ve len widget lam o _update_camera.
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_detections(self, msg):
        dets = []
        for d in msg.detections:
            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            sx = d.bbox.size_x
            sy = d.bbox.size_y
            label = d.results[0].hypothesis.class_id if d.results else '?'
            score = d.results[0].hypothesis.score if d.results else 0.0
            dets.append((cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2, label, score))
        self._latest_dets = dets

    def _update_camera(self):
        if self._latest_frame is not None:
            h, w = self._latest_frame.shape[:2]
            # BGR -> RGB (khong can cv2); .copy() de mang C-contiguous cho PIL.
            rgb = self._latest_frame[:, :, ::-1].copy()
            img = PILImage.fromarray(rgb)
            # Ve bounding box + nhan len anh GOC (truoc resize) de box scale dung.
            if self._latest_dets:
                draw = ImageDraw.Draw(img)
                for (x1, y1, x2, y2, label, score) in self._latest_dets:
                    draw.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=3)
                    text = f'{label} {score:.2f}'
                    tb = draw.textbbox((0, 0), text, font=_BBOX_FONT)
                    tw, th = tb[2] - tb[0], tb[3] - tb[1]
                    ty = max(0, y1 - th - 4)
                    # Nen xanh dam de chu trang deu doc duoc tren moi mau anh
                    draw.rectangle([x1, ty, x1 + tw + 6, ty + th + 4], fill=(0, 150, 0))
                    draw.text((x1 + 3, ty + 2), text, fill=(255, 255, 255), font=_BBOX_FONT)
            img = img.resize((CAM_W, CAM_H))
            self._photo = ImageTk.PhotoImage(img)
            self.cam_label.configure(image=self._photo, text='')
        self.root.after(int(1000 / CAM_UPDATE_HZ), self._update_camera)

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
