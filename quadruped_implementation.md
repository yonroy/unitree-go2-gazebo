# Triển khai Baseline Go2 — Tổng hợp những gì đã làm

> Tài liệu này giải thích **các file thực tế đã tạo** trong `~/Ros2/quadruped_ws/`, ứng với bước 1-3 của lộ trình ở `quadruped_ros_system.md` (§13): xác nhận URDF → gait cố định + IK → đứng/đi trong Gazebo, cộng thêm heading-hold và bảng điều khiển joystick ảo.

---

## 1. Cây thư mục

```
quadruped_ws/src/
├── quadruped_description/   # "Robot trông như thế nào" — URDF Go2 + Gazebo
│   ├── xacro/
│   │   ├── const.xacro          # Hằng số hình học/khối lượng Go2 (copy nguyên từ Unitree)
│   │   ├── materials.xacro      # Màu sắc mesh (copy nguyên)
│   │   ├── leg.xacro            # 1 chân (hip-thigh-calf-foot) x 4, adapt từ Unitree
│   │   ├── robot.xacro          # File gốc: ghép trunk + 4 chân + include các file trên
│   │   ├── ros2_control.xacro   # Khai báo 12 khớp cho ros2_control (MỚI, tự viết)
│   │   └── gazebo.xacro         # Plugin gz_ros2_control + cảm biến IMU (MỚI, tự viết)
│   ├── meshes/*.dae             # Mesh 3D (copy nguyên từ Unitree)
│   ├── worlds/flat_ground.sdf   # World Gazebo: mặt đất phẳng (MỚI)
│   ├── config/go2_controllers.yaml  # Cấu hình controller_manager (MỚI)
│   └── launch/gazebo.launch.py  # Spawn Gazebo + robot + controller (MỚI)
│
├── quadruped_gait/           # "Robot đi như thế nào" — não bộ điều khiển
│   └── quadruped_gait/
│       ├── leg_ik.py            # Inverse/Forward kinematics giải tích cho 1 chân
│       ├── trot_gait.py         # Gait planner: cmd_vel → quỹ đạo 4 chân → góc khớp
│       └── gait_node.py         # Node ROS 2: /cmd_vel → publish góc khớp 50Hz
│
├── quadruped_bringup/        # "Bật tất cả cùng lúc"
│   └── launch/quadruped_sim.launch.py
│
└── quadruped_teleop/         # "Người dùng điều khiển bằng gì"
    └── quadruped_teleop/joystick_panel.py  # GUI joystick ảo (Tkinter) → /cmd_vel
```

**Nguồn gốc URDF:** tải từ `unitreerobotics/unitree_ros` (giấy phép BSD-3-Clause, miễn phí) — đây là repo chính thức của Unitree Robotics chứa file `go2_description`.

---

## 2. `quadruped_description` — Hình dáng robot + mô phỏng vật lý

| File | Vai trò |
|---|---|
| `xacro/robot.xacro` | File "tổng": định nghĩa link `trunk` (thân), `imu_link`, rồi gọi macro `leg` 4 lần cho FR/FL/RR/RL |
| `xacro/leg.xacro` | Macro 1 chân: 3 khớp (`hip_joint` trục X, `thigh_joint`/`calf_joint` trục Y) + mesh + khối lượng/quán tính thật |
| `xacro/const.xacro` | Số đo thật của Go2: chiều dài hip=0.0955m, thigh=calf=0.213m, giới hạn góc khớp, khối lượng từng bộ phận |
| `xacro/ros2_control.xacro` | Khai báo 12 khớp dùng `command_interface="position"`, có `initial_value` để robot **spawn sẵn ở tư thế đứng** (không rơi tự do rồi giật) |
| `xacro/gazebo.xacro` | Nạp plugin `gz_ros2_control` (thay cho `gazebo_ros_control` bản ROS1 gốc) + sensor IMU |
| `worlds/flat_ground.sdf` | World Gazebo Harmonic: mặt phẳng + ánh sáng + physics step 1ms |
| `config/go2_controllers.yaml` | 2 controller: `joint_state_broadcaster` (đọc encoder) + `forward_position_controller` (nhận lệnh góc khớp) |
| `launch/gazebo.launch.py` | Mở Gazebo, spawn robot, bridge `/clock` và `/imu/data` sang ROS 2, gọi controller theo đúng thứ tự (tránh race condition) |

**Điều chỉnh so với bản gốc của Unitree (ROS1 + Gazebo Classic):**
- Thay toàn bộ `<transmission>` + `libgazebo_ros_control.so` (ROS1) bằng `<ros2_control>` + `gz_ros2_control` (ROS2 + Gazebo Harmonic).
- Sửa 1 lỗi có sẵn trong repo gốc: `robot.xacro` tham chiếu mesh `trunk.dae` nhưng file thật tên là `base.dae`.
- Bỏ các plugin Gazebo Classic không tương thích (contact sensor, force draw...) — ngoài phạm vi bước baseline.

---

## 3. `quadruped_gait` — Não bộ điều khiển (phần tự viết hoàn toàn)

### `leg_ik.py` — Động học ngược 1 chân
Cho vị trí bàn chân mong muốn `(x, y, z)` tính từ khớp hông, giải ra 3 góc khớp bằng công thức lượng giác (không dùng thư viện ngoài). Có `forward_kinematics()` để tự kiểm tra ngược (IK → FK phải ra đúng điểm ban đầu).

### `trot_gait.py` — Gait planner
- Chia 4 chân thành 2 cặp chéo (FR+RL, FL+RR), lệch pha nhau nửa chu kỳ → dáng đi **trot**.
- Mỗi chân: nửa chu kỳ *stance* (chân chạm đất, đẩy lùi để thân tiến), nửa chu kỳ *swing* (nhấc chân, đưa về phía trước).
- **Tần số bước tự thích nghi theo tốc độ**: tốc độ lệnh càng cao, chu kỳ bước càng ngắn (bước nhanh hơn thay vì bước dài hơn ở cùng nhịp) — tránh mất ổn định khi đi nhanh.

### `gait_node.py` — Node ROS 2
Subscribe `/cmd_vel` (50Hz) → gọi `trot_gait` tính góc khớp → publish `Float64MultiArray` tới `forward_position_controller`. Có thêm **heading-hold**: đọc `/imu/data`, tự bù `wz` để giữ đúng hướng khi đi thẳng lâu (bù lệch yaw tích lũy do gait không cân bằng chủ động).

---

## 4. `quadruped_bringup` — Launch tổng
`quadruped_sim.launch.py` chỉ đơn giản gộp `gazebo.launch.py` (ở trên) + chạy `gait_node`. Teleop chạy riêng ở terminal khác (không ép buộc trong launch, vì cần tương tác người dùng).

---

## 5. `quadruped_teleop` — Bảng điều khiển joystick ảo
`joystick_panel.py`: cửa sổ Tkinter với 2 "cần" kéo-thả bằng chuột (không cần joystick vật lý):
- **Cần trái** (hình tròn): kéo lên/xuống = tiến/lùi (`vx`), trái/phải = đi ngang (`vy`, lợi thế của robot chân).
- **Cần phải** (thanh ngang): kéo trái/phải = xoay (`wz`).
- Thả chuột → cần tự về giữa (lò xo ảo) → dừng ngay, an toàn.
- Thanh trượt tốc độ + nút **DỪNG KHẨN**.

Publish `/cmd_vel` ở 20Hz bằng `root.after()` kết hợp `rclpy.spin_once()`.

---

## 6. Các lỗi đã tìm ra và sửa (qua tự kiểm chứng, không chỉ "build pass")

| # | Lỗi | Cách phát hiện | Cách sửa |
|---|---|---|---|
| 1 | Sai dấu trong công thức IK | Self-test IK→FK round-trip | Sửa công thức `alpha` (dùng nhầm `knee_interior` thay vì `theta3`) |
| 2 | Quy ước trục "xuống" bị đảo (đứng lẽ ra 0° lại ra 144°) | Self-test round-trip tiếp tục phát hiện góc bất thường | Đảo dấu trong công thức `theta1`/FK cho khớp hip |
| 3 | Robot lật khi spawn (góc khớp "nhảy" đột ngột lúc controller kích hoạt) | Quan sát quaternion lật 180° trong Gazebo qua `gz topic` | Thêm `initial_value` để spawn sẵn ở tư thế đứng |
| 4 | **Đi tới lại thành đi lùi** — trục quay khớp thigh/calf (Y, right-hand rule) đẩy chân về -X chứ không phải +X như giả định | Đo trực tiếp vị trí world qua `gz topic`, so sánh `vx=+0.3` và `vx=-0.3` | Đảo dấu 1 chỗ trong `leg_ik.py` (biến `a = -x`) |
| 5 | Đi lệch hướng (drift) khi đi thẳng lâu | Đo yaw tích lũy theo thời gian | Thêm vòng heading-hold dùng IMU trong `gait_node.py` |
| 6 | Tăng tốc độ joystick → robot nghiêng/giật/hạ thấp người | Test `vx=0.45` trực tiếp trong Gazebo, thấy roll/pitch xuất hiện | Tần số bước tự thích nghi theo tốc độ (`trot_gait.py`) |

Lỗi #4 là nghiêm trọng nhất và khó thấy nhất: vì tư thế **đứng** đối xứng (x=0) nên không lộ ra qua test tĩnh — chỉ lộ khi đo chuyển động thật.

---

## 7. Cách chạy lại

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros2/quadruped_ws/install/setup.bash
ros2 launch quadruped_bringup quadruped_sim.launch.py
```

Điều khiển (chọn 1):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # bàn phím
ros2 run quadruped_teleop joystick_panel                # joystick ảo (GUI)
```

---

## 8. Giới hạn đã biết (đúng như §6 tài liệu gốc dự đoán)

- Đây là **baseline reactive**: gait cố định + IK, không cảm nhận lực chạm đất, không tái cân bằng chủ động khi bị xô đẩy.
- Heading-hold chỉ sửa hướng (yaw), chưa sửa trực tiếp vị trí ngang (Y) — vẫn còn trôi nhẹ.
- Chưa có perception/follow-object hay RL locomotion (bước 4-6 trong lộ trình gốc) — để dành cho giai đoạn sau nếu cần.
