# Hệ thống ROS cho Robot 4 chân — Tài liệu tổng hợp

> Tài liệu này tổng hợp toàn bộ kiến trúc, thành phần, và lộ trình xây dựng một robot 4 chân dùng ROS 2, từ nền tảng giao tiếp tới điều khiển học máy và lựa chọn phần cứng.

---

## Mục lục

1. [Nền tảng ROS](#1-nền-tảng-ros)
2. [Kiến trúc hệ thống 5 tầng](#2-kiến-trúc-hệ-thống-5-tầng)
3. [Điều khiển 12 motor và dáng đi](#3-điều-khiển-12-motor-và-dáng-đi)
4. [Pipeline follow object](#4-pipeline-follow-object)
5. [Từ vision tới chuyển động](#5-từ-vision-tới-chuyển-động)
6. [Nâng cấp điều khiển: reactive → planning → learning](#6-nâng-cấp-điều-khiển)
7. [Hướng học máy (RL) và cảm biến cần thiết](#7-hướng-học-máy-rl)
8. [Tốc độ phản hồi và độ trễ](#8-tốc-độ-phản-hồi-và-độ-trễ)
9. [Phần cứng đề xuất và chi phí](#9-phần-cứng-đề-xuất)
10. [Tận dụng model pretrained để tiết kiệm](#10-tận-dụng-model-pretrained)
11. [Các mảng dễ bị bỏ sót](#11-các-mảng-dễ-bị-bỏ-sót)
12. [Cấu trúc package và bảng node/topic](#12-cấu-trúc-package)
13. [Lộ trình triển khai](#13-lộ-trình-triển-khai)
14. [Nguồn tham khảo](#14-nguồn-tham-khảo)

---

## 1. Nền tảng ROS

ROS (Robot Operating System) không phải hệ điều hành thật, mà là **middleware + framework + hệ sinh thái** giúp biến robot thành tập hợp các tiến trình nhỏ (node) giao tiếp với nhau qua hệ thống truyền thông điệp chuẩn hóa, cộng với kho thư viện/công cụ sẵn có (SLAM, path planning, driver cảm biến, mô phỏng).

**Lý do cần ROS:** tránh viết tất cả trong một chương trình khổng lồ khó bảo trì, khó tái sử dụng, khó chạy song song/phân tán.

### Các khái niệm cốt lõi

| Khái niệm | Vai trò | Khi nào dùng |
|---|---|---|
| **Node** | Tiến trình độc lập làm một việc | Mọi thành phần logic |
| **Topic** (pub/sub) | Kênh bất đồng bộ, một chiều, nhiều-tới-nhiều | Dữ liệu liên tục: cảm biến, cmd_vel, trạng thái |
| **Service** (req/res) | Đồng bộ, một-tới-một, như gọi hàm từ xa | Cần một câu trả lời cụ thể, nhanh |
| **Action** (goal/feedback/result) | Tác vụ dài, báo tiến độ, có thể hủy | Di chuyển tới đích, gắp vật |
| **Parameter** | Giá trị cấu hình đọc/ghi lúc chạy | Tốc độ tối đa, ngưỡng cảm biến |
| **Message type** | Cấu trúc dữ liệu chuẩn hóa | "Hợp đồng" để các node hiểu nhau |

### ROS 1 vs ROS 2

| | ROS 1 | ROS 2 |
|---|---|---|
| Middleware | Master tập trung (roscore) | DDS phân tán, không cần master |
| Real-time | Không tốt | Có hỗ trợ |
| Đa nền tảng | Chủ yếu Linux | Linux, Windows, macOS, nhúng |
| Bảo mật | Yếu | Có DDS-Security |
| Tình trạng | Đã EOL (Noetic là bản cuối) | Đang phát triển tích cực |

**Kết luận:** bắt đầu mới → học thẳng ROS 2.

---

## 2. Kiến trúc hệ thống 5 tầng

Dữ liệu chảy từ trên xuống, trạng thái/phản hồi chảy ngược lên:

```
┌─────────────────────────────────────────────┐
│  Sensors        — Camera, lidar, IMU          │
├─────────────────────────────────────────────┤
│  Perception     — Object detection, SLAM      │
├─────────────────────────────────────────────┤
│  Planning       — Nav2 path planner + gait    │
├─────────────────────────────────────────────┤
│  Control        — Leg trajectory + joint PID  │
├─────────────────────────────────────────────┤
│  Actuation      — 12 servo motor (4 x 3 khớp) │
└─────────────────────────────────────────────┘
```

Đây là cấu trúc chuẩn cho hầu hết robot di động phức tạp, không riêng robot 4 chân.

---

## 3. Điều khiển 12 motor và dáng đi

Mỗi chân có 3 khớp: **hông (hip)** – **đùi (thigh)** – **gối (knee)**. 4 chân × 3 = **12 servo**.

### Bố trí

```
   Chân trước-trái          Chân trước-phải
        ●                         ●
         \                       /
          ●                     ●
           \                   /
        ┌───●───────────────●───┐
        │      THÂN ROBOT       │   ← IMU + máy tính
        │   (IMU, compute)      │
        └───●───────────────●───┘
           /                   \
          ●                     ●
         /                       \
        ●                         ●
   Chân sau-trái            Chân sau-phải
```

### Chuỗi điều khiển

1. **Body velocity command** (`cmd_vel`: vx, vy, ωz) — "muốn đi hướng nào, nhanh chậm ra sao". Lớp trên (Nav2, joystick, follow-object) chỉ cần gửi `cmd_vel`, không cần biết 12 khớp.
2. **Gait planner** — nhận cmd_vel, chọn kiểu dáng đi (trot phổ biến nhất), tính quỹ đạo bàn chân theo chu kỳ.
3. **Inverse kinematics (IK)** — chuyển vị trí bàn chân mong muốn thành 3 góc khớp cho mỗi chân.
4. **Joint controller** (PID/driver servo) — gửi lệnh xuống 12 motor, đọc encoder phản hồi.

**Đi trái/phải** = ωz khác 0 khi vx vẫn dương (vừa tiến vừa quay). **Đi ngang thuần** (strafe) = vy khác 0 — lợi thế robot chân so với robot bánh xe.

**Framework gợi ý:** `champ`, `unitree_ros2` — làm sẵn tầng gait + IK, tích hợp ROS 2 + Nav2.

---

## 4. Pipeline follow object

```
Camera ──▶ Object detector ──▶ Tracker ──▶ Bộ điều khiển bám mục tiêu
(image)    (YOLO → bbox)      (giữ 1 ID)    (PID vị trí + khoảng cách)
                                                    │
                              Lidar ────────────────┤ (tránh vật cản)
                                                    ▼
                                            Local planner
                                                    │
                                                    ▼
                                            /cmd_vel ──▶ Gait planner ──▶ 12 motor
```

### Logic cơ bản (reactive/rule-based)

```python
# error_x: lệch tâm bbox so với tâm ảnh (âm = mục tiêu bên trái)
# error_size: bbox mong muốn - bbox hiện tại (dương = cần tiến gần)

angular_z = kp_angular * error_x        # quay theo mục tiêu
linear_x  = kp_linear  * error_size     # tiến/lùi giữ khoảng cách

cmd_vel.linear.x  = clamp(linear_x, -MAX_SPEED, MAX_SPEED)
cmd_vel.angular.z = clamp(angular_z, -MAX_TURN, MAX_TURN)
publisher.publish(cmd_vel)
```

### Topic hay Action?

- **Follow liên tục, không điểm kết** → dùng **Topic** đơn giản (publish vị trí mục tiêu, subscribe tính cmd_vel mỗi frame).
- **"Đi tới vật X rồi dừng/gắp"** → dùng **Action** (có goal, feedback tiến độ, hủy được).

```
# FollowObject.action
int32 target_id
float32 stop_distance
---
bool success
string message
---
float32 current_distance
float32 current_angle
```

---

## 5. Từ vision tới chuyển động

Chuỗi 6 bước biến ánh sáng thành chuyển động:

```
1. Pixel ảnh (u,v)          ← bounding box từ detector
        ▼
2. Điểm 3D khung camera     ← depth + ma trận nội tại camera
        ▼
3. Điểm 3D khung robot       ← chuyển đổi qua tf2 (base_link)
        ▼
4. Vận tốc mong muốn         ← PID trên khoảng cách + góc → /cmd_vel
        ▼
5. Góc 12 khớp               ← gait planner + inverse kinematics
        ▼
6. Tín hiệu motor            ← ros2_control → PWM/serial
```

### Công thức pinhole camera (bước 1→2)

```python
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
# fx, fy, cx, cy lấy từ /camera/camera_info (đã calib)
```

### Chuyển khung qua tf2 (bước 2→3)

```python
from tf2_geometry_msgs import do_transform_point
point_in_camera = PointStamped(header=..., point=Point(x=X, y=Y, z=Z))
transform = tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
point_in_base = do_transform_point(point_in_camera, transform)
```

### Tính cmd_vel (bước 3→4)

```python
import math

def compute_cmd_vel(point_in_base, desired_distance=1.0):
    distance = math.hypot(point_in_base.point.x, point_in_base.point.y)
    angle    = math.atan2(point_in_base.point.y, point_in_base.point.x)

    linear_x  = kp_linear  * (distance - desired_distance)
    angular_z = kp_angular * angle

    cmd = Twist()
    cmd.linear.x  = clamp(linear_x, -MAX_SPEED, MAX_SPEED)
    cmd.angular.z = clamp(angular_z, -MAX_TURN, MAX_TURN)
    return cmd
```

Đây là **điểm nối vision ↔ motion**: trước đó là xử lý ảnh, từ đây là điều khiển chuyển động thuần túy.

---

## 6. Nâng cấp điều khiển

### Ba cấp độ

| Cấp | Bản chất | Đặc điểm |
|---|---|---|
| **Reactive (rule-based)** | PID nhìn lỗi hiện tại → phản ứng ngay | Đơn giản, nhanh, nhưng giật cục, không nhìn trước |
| **Model-based planning** | DWA/TEB mô phỏng trước nhiều quỹ đạo → chọn tốt nhất | Mượt hơn, tránh vật cản chủ động |
| **Learning (RL)** | Mạng neural học điều khiển từ mô phỏng | Mượt và tự nhiên nhất, thích nghi địa hình |

### DWA vs TEB (model-based)

- **DWA (Dynamic Window Approach)** — sampling-based: lấy mẫu nhiều cặp (v, ω) khả thi, mô phỏng cung đường mỗi cặp, chấm điểm, chọn tốt nhất.
- **TEB (Timed Elastic Band)** — optimization-based: biểu diễn quỹ đạo như dải cao su đàn hồi, liên tục tối ưu cho mượt + ngắn + tránh vật cản. Mượt hơn DWA.

> **Lưu ý:** "model" ở đây là **mô hình động học/động lực học của robot**, KHÔNG phải model machine learning.

### Vì sao "planning mượt" ≠ "dáng đi mượt như chó thật"

Nav2 (DWA/TEB) chỉ giải quyết lớp "đi đâu" (vx, vy, ωz), không đụng lớp "chân bước thế nào". Độ mượt như chó thật nằm ở các tầng nhanh hơn nhiều:

```
Path planning (Nav2 DWA/TEB)        ~1-10 Hz    ← quyết định đi đâu
        ▼
Gait planner                        ~30-50 Hz   ← chọn nhịp bước
        ▼
Whole-body / MPC controller         ~200-500 Hz ← cân bằng động, phân bổ lực
        ▼
Joint torque / impedance control    ~1000 Hz+   ← độ đàn hồi, hấp thụ va chạm
```

**Baseline gait cố định + PID vị trí khớp có 3 giới hạn:**
1. Không cảm nhận lực chạm đất (chân "cứng" theo quỹ đạo định sẵn).
2. Gait cố định, không tái cân bằng thời gian thực.
3. Không đọc địa hình để chọn điểm đặt chân.

**Hai hướng nâng cấp thật sự:**
- **MPC + impedance control** (MIT Cheetah, Unitree) — cần servo hỗ trợ torque control.
- **RL locomotion policy** (ANYmal, Unitree Go2 mới) — mượt nhất, cần hạ tầng train.

---

## 7. Hướng học máy (RL)

### Pipeline huấn luyện

```
1. Mô hình robot chính xác     — URDF/MJCF: khối lượng, quán tính, giới hạn khớp
        ▼
2. Mô phỏng song song          — Isaac Lab / MuJoCo: hàng nghìn robot cùng lúc trên GPU
        ▼
3. Reward function + PPO        — thưởng đi thẳng, ổn định, tiết kiệm năng lượng
        ▼
4. Domain randomization        — xáo trộn ma sát, khối lượng, độ trễ (giúp sim-to-real)
        ▼
5. Xuất policy (ONNX)          — chạy inference trong node ROS
        ▼
6. Chạy trên robot thật        — sim-to-real, tinh chỉnh (lặp lại nếu chưa mượt)
```

### Cảm biến cần thiết cho RL

**Bắt buộc (thiếu thì policy không chạy được):**
- **Encoder vị trí + vận tốc tại 12 khớp** — input quan trọng nhất (proprioception). Servo hobby chỉ nhận PWM không phản hồi → KHÔNG đủ.
- **IMU** — góc nghiêng, vận tốc góc, gia tốc tuyến tính.

→ Chỉ 2 loại này đủ train **"blind policy"** (đi vững không cần nhìn).

**Nên thêm (perceptive locomotion):**
- **Cảm biến lực/tiếp đất ở 4 bàn chân** (FSR) — biết chính xác thời điểm chạm đất.
- **Depth camera hướng xuống trước** — nhìn thấy địa hình trước khi đặt chân (lidar quét ngang không thấy mặt đất dưới chân).

**Lưu ý tần số:** encoder + IMU phải trả dữ liệu ≥ tần số inference (50-100Hz, IMU lý tưởng 100-200Hz). Driver motor 10-20Hz sẽ là nút thắt cổ chai.

### GPU: cần ở đâu

| | Training | Inference trên robot |
|---|---|---|
| Cần GPU? | **Có, gần như bắt buộc** | **Không** (cho locomotion) |
| Vì sao | Mô phỏng nghìn robot + backprop | Mạng nhỏ, forward <1ms, chạy trên CPU nhúng |

Nhưng **object detection (YOLO) + depth processing** cần GPU/NPU onboard → vẫn nên có Jetson.

---

## 8. Tốc độ phản hồi và độ trễ

> **Nguyên tắc vàng: latency quan trọng hơn throughput.** Camera 60fps trễ 100ms tệ hơn camera 30fps trễ 15ms.

### Tần số cần thiết mỗi vòng

| Vòng | Tần số | Ghi chú |
|---|---|---|
| Điều khiển khớp (encoder+IMU→motor) | 200-1000 Hz | Nhanh nhất, quyết định robot ngã hay không |
| Inference RL policy | 50-100 Hz | Bám kịp vòng khớp |
| Lidar → tránh vật cản | 10-40 Hz | Giới hạn bởi tốc độ quay lidar |
| Camera → detection → tracking | 15-30 Hz | Chậm nhất, mục tiêu di chuyển chậm hơn cơ thể |

**Không ép mọi thứ cùng tần số** — vừa lãng phí vừa nghẽn.

### 5 yếu tố quyết định tốc độ

1. **Giao thức bus giao tiếp** (quan trọng nhất): PWM (~50-333Hz, không feedback) < Serial/Dynamixel < CAN bus < EtherCAT.
2. **Kiến trúc xử lý**: cùng process < intra-process ROS 2 < qua mạng (giữ vòng khớp NỘI BỘ control board, không qua mạng).
3. **Cấu hình DDS/QoS**: `best_effort` cho cảm biến tốc độ cao; bật shared memory transport.
4. **Tải CPU & lịch trình**: tách vision sang board riêng để vòng khớp không bị cướp CPU; cân nhắc real-time kernel (PREEMPT_RT).
5. **Bản chất vật lý cảm biến**: tốc độ quay lidar, exposure camera, thời gian inference model.

### 3 quy tắc thiết kế

1. **Phân tầng theo tần số** — vòng khớp nhanh + cô lập, vòng vision chậm + ở Jetson.
2. **Giữ vòng tốc-độ-cao gần phần cứng nhất** — không qua mạng, không tranh CPU.
3. **Latency > tần số** — đo độ trễ end-to-end thật, đừng chỉ nhìn Hz trên datasheet.

---

## 9. Phần cứng đề xuất

### Bảng linh kiện + chi phí (tham khảo quốc tế, USD)

| Thành phần | Model đề xuất | Giá đơn vị | SL | Thành tiền |
|---|---|---|---|---|
| Máy tính vision | NVIDIA Jetson Orin Nano Super | ~$249 | 1 | ~$249 |
| Board điều khiển | Raspberry Pi 5 (8GB) | ~$80 | 1 | ~$80 |
| Motor khớp | Unitree GO-M8010-6 (CAN, encoder 22-bit) | ~$150-200 | 12 | ~$1,800-2,400 |
| Camera depth | Intel RealSense D435i | ~$300-380 | 1 | ~$300-380 |
| Lidar 2D | RPLidar A2/A3 | ~$300-450 | 1 | ~$300-450 |
| IMU rời | BNO085 | ~$20-30 | 1 | ~$25 |
| Pin | LiPo 6S dung lượng lớn, C-rating cao | ~$80-150 | 1-2 | ~$150 |
| PDB + BEC | Mạch phân phối điện cách ly | ~$40-80 | 1 bộ | ~$60 |
| Khung + chân | In 3D / nhôm | ~$100-300 | 1 bộ | ~$200 |
| Phụ kiện | Dây, đầu nối, E-stop, CAN adapter | — | — | ~$100 |
| **Tổng** | | | | **~$3,300-4,300** |

### Vì sao chọn motor CAN bus (Unitree GO-M8010-6)

- Torque liên tục 10 Nm, đỉnh 20 Nm, encoder 22-bit tuyệt đối (Dynamixel XM430 chỉ 4.8 Nm, 12-bit).
- Hỗ trợ torque control real-time qua CANopen tới **1 kHz** (servo hobby thường giới hạn 500 Hz).
- CAN bus xử lý nhiều motor tốt hơn serial half-duplex của Dynamixel (không gửi/nhận cùng lúc).

### So sánh DIY vs mua sẵn

| | DIY (bảng trên) | Unitree Go2 |
|---|---|---|
| Chi phí | ~$3,300-4,300 | Air ~$1,600 / Pro ~$2,800 |
| Ưu điểm | Kiểm soát toàn bộ, hiểu sâu | Rẻ hơn, sẵn phần cứng + policy RL + SDK |
| Phù hợp | Học sâu phần cứng, tự thiết kế | Tập trung phần mềm/perception |

> Với người mới: mua Go2 EDU thường "tiết kiệm tổng thể" hơn — đã giải quyết sẵn cơ khí, nguồn, an toàn, calib, tản nhiệt.
> **Giá tại VN cộng thêm thuế nhập khẩu + phí ship, đặc biệt motor Unitree và RealSense.**

---

## 10. Tận dụng model pretrained

> Pretrained tiết kiệm **công sức + thời gian + chi phí GPU training**, KHÔNG tiết kiệm chi phí phần cứng robot.

### Vision model — luôn nên dùng
- **YOLO pretrained (COCO)** dùng ngay cho vật thể phổ biến; fine-tune vài trăm ảnh cho vật thể đặc thù.
- **Tracker (ByteTrack)** — thuật toán thuần, không cần train.

### Locomotion policy — có điều kiện
- Policy bám cứng thông số vật lý robot → đổi hình học vài cm là vô dụng.
- **Robot DIY tùy ý** → gần như phải tự train, pretrained ít giúp.
- **Nền tảng chuẩn (Unitree A1/Go2, SpotMicro)** → có checkpoint sẵn, dùng lại hoặc fine-tune nhẹ.

### Tài nguyên đáng dùng
- **GenLoco** — controller tổng quát, train trên nhiều morphology, có pretrained policy + code deploy.
- **legged_gym / Isaac Lab** — pipeline train sẵn (reward, action/obs space) — tiết kiệm công thiết kế lớn nhất.
- **Motion Imitation (Google)** — có pretrained nhưng đã deprecated 11/2025.

### Kết luận chiến lược
1. Dùng vision pretrained — luôn luôn.
2. **Chọn phần cứng theo nền tảng chuẩn** — nơi tiết kiệm công train lớn nhất.
3. Khởi đầu từ pipeline train sẵn / checkpoint rồi fine-tune.
4. Phần cứng vẫn phải mua đủ — pretrained không giảm khoản này.

---

## 11. Các mảng dễ bị bỏ sót

Ngoài phần mềm, một robot 4 chân hoàn chỉnh cần:

### Nhóm tiên quyết (không có thì không chạy được)
- **Nguồn điện & pin** — 12 motor rút dòng đỉnh gấp nhiều lần danh định; cần pin C-rating cao + PDB tách nguồn motor/logic.
- **An toàn & E-stop** — nút dừng khẩn vật lý (cắt nguồn motor tức thì), watchdog phần mềm, giới hạn dòng/torque.
- **Calibration** — zero offset từng khớp, calib nội tại camera (fx,fy,cx,cy), calib IMU bias, đo TF thật giữa các bộ phận.

### Nhóm phần cứng
- **Thiết kế cơ khí** — gear ratio, vật liệu chân, độ cứng khung (ảnh hưởng nhiễu IMU).
- **Tản nhiệt** — servo nóng khi giữ tư thế tĩnh; Jetson chạy YOLO cần tản nhiệt chủ động.

### Nhóm vận hành
- **Giao tiếp & giám sát** — teleop dự phòng (joystick), telemetry + logging (rosbag) để debug sim-to-real.

**Thứ tự bổ sung:** cơ khí + servo → nguồn điện → an toàn/E-stop → calibration → phần mềm → giám sát/logging.

---

## 12. Cấu trúc package

```
quadruped_ws/src/
├── quadruped_description/     # URDF/xacro: 4 chân x 3 khớp, camera, lidar, TF tree
├── quadruped_bringup/         # launch file tổng
├── quadruped_gait/            # gait planner + IK (hoặc champ)
├── quadruped_hw_interface/    # driver 12 servo (ros2_control)
├── quadruped_perception/      # object detection + tracking
├── quadruped_follow/          # node PID follow object → /cmd_vel
├── quadruped_navigation/      # config Nav2: costmap, planner
├── quadruped_slam/            # slam_toolbox / RTAB-Map
└── quadruped_rl/              # RL policy node (ONNX inference)
```

### Bảng node/topic

| Node | Package | Input | Output | Board |
|---|---|---|---|---|
| `camera_driver` | realsense2_camera | phần cứng | /camera/color, /camera/depth | Jetson |
| `object_detector` | quadruped_perception | /camera/color | /detections | Jetson |
| `tracker` | quadruped_perception | /detections | /tracked_target | Jetson |
| `target_pose_tf` | quadruped_perception | /tracked_target, /camera/depth, tf | /target_pose | Jetson |
| `follow_behavior` | quadruped_follow | /target_pose, /scan | /cmd_vel | Jetson |
| `imu_driver` | vendor | phần cứng | /imu/data | Control board |
| `lidar_driver` | vendor | phần cứng | /scan | Control board |
| `joint_state_broadcaster` | ros2_control | 12 encoder | /joint_states | Control board |
| `rl_policy_node` | quadruped_rl | /cmd_vel, /imu/data, /joint_states | /joint_commands | Control board |
| `motor_driver` | quadruped_hw_interface | /joint_commands | PWM/serial | Control board |

### Phân bổ 2 board

- **Jetson Orin Nano** — vision: camera + YOLO + tracking (dùng GPU).
- **Control board (RPi 5 / Jetson Nano)** — IMU, lidar, 12 motor, RL policy (CPU, policy rất nhẹ).
- Giao tiếp qua ROS 2 DDS trên LAN — chỉ gửi `cmd_vel` nhỏ, không gửi cả ảnh.

---

## 13. Lộ trình triển khai

> Đừng làm hết cùng lúc — luôn giữ một hệ thống chạy được ở mỗi giai đoạn.

1. **Xác nhận phần cứng** — servo có phản hồi vị trí/tốc độ không, IMU đủ 100Hz+.
2. **Build baseline rule-based** — gait cố định + PID follow-object, dùng làm đối chứng.
3. **Setup mô phỏng chính xác** — URDF đúng khối lượng/quán tính, robot ảo đứng/đi được trong Gazebo/Isaac Sim.
4. **Train blind policy** — chỉ IMU + encoder, dễ debug hơn.
5. **Sim-to-real blind policy** — đưa lên robot thật, nền phẳng, tinh chỉnh domain randomization.
6. **Ghép follow-object** — đổi nguồn cmd_vel từ test sang node vision; RL policy phía dưới không đổi.

### Lưu ý môi trường phát triển
- Ổ đĩa: **tối thiểu 50GB**, thoải mái **100-120GB+** (ROS 2 full + Gazebo + PyTorch + Docker). Isaac Sim cần hàng chục GB. 20GB quá chật.
- Ubuntu là nền tảng khuyến nghị nhất cho ROS 2.
- Bắt đầu bằng mô phỏng TRƯỚC khi đụng robot thật (gait sai có thể gãy servo).

---

## 14. Nguồn tham khảo

### Mã nguồn mở nên xem
- **sim2real-3d-printed-quadruped** — pipeline end-to-end nhỏ gọn: Isaac Lab → PyTorch → ROS2 → robot in 3D thật (12-DOF, inference 20Hz).
- **quadruped_ros2_control** (legubiao) — ROS2-Control cho robot 4 chân, gồm sim2real; tham chiếu unitree_guide, legged_control, rl_sar.
- **champ** (chvmp/champ) — gait + IK sẵn cho ROS 2, tích hợp Nav2. Khởi đầu tốt nhất.
- **rex-gym / SpotMicro** — robot 4 chân in 3D mã nguồn mở, môi trường Gym + PPO, cộng đồng lớn.
- **GenLoco** — controller tổng quát đa morphology, có pretrained policy.
- **legged_gym / Isaac Lab** — pipeline train RL chuẩn.
- **TopHillRobotics/quadruped-robot** — MPC + WBC (ROS 1, tham chiếu whole-body control).

### Bài báo
- "Sim-to-Real Transfer for Mobile Robots with Reinforcement Learning: from NVIDIA Isaac Sim to Gazebo and Real ROS 2 Robots" — arxiv.org/pdf/2501.02902
- "Learning agile and dynamic motor skills for legged robots" (Hwangbo et al., Science Robotics 2019) — ANYmal.
- "RMA: Rapid Motor Adaptation for Legged Robots" (Kumar 2021).

### Công cụ ROS
- **RViz** — trực quan hóa 3D
- **Gazebo / Isaac Sim** — mô phỏng vật lý
- **tf2** — quản lý hệ tọa độ
- **Nav2** — điều hướng, DWA/TEB local planner
- **MoveIt** — kế hoạch chuyển động tay máy
- **ros2_control** — hardware interface chuẩn
- **rqt** — debug đồ họa
