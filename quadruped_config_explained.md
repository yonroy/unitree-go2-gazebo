# Giải thích chi tiết `quadruped_description` và `quadruped_gait`

> Đây là 2 package quyết định "hình dáng vật lý" và "cách suy nghĩ điều khiển" của robot. Hiểu 2 phần này là hiểu toàn bộ cấu hình Go2 trong dự án.

---

## PHẦN A — `quadruped_description`: robot trông như thế nào

### A.1 Kích thước thật của Go2 (lấy từ `const.xacro`, không tự bịa)

```
Thân (trunk):    dài 0.3762m  x  rộng 0.0935m  x  cao 0.114m   khối lượng 6.921kg
Chân (mỗi chân): hip 0.0955m → thigh 0.213m → calf 0.213m
```

```
                    ┌──────── 0.3762 m ────────┐
   FL ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● FR
       \                 TRUNK                 /
        \          (6.921 kg, IMU ở đây)       /
         \                                     /
   RL ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● RR
```

Mỗi chân treo vào thân tại một điểm offset:
- `leg_offset_x = 0.1934m` — chân trước (FR/FL) lệch về phía trước tâm thân 0.1934m, chân sau (RR/RL) lệch về phía sau cùng khoảng cách.
- `leg_offset_y = 0.0465m` — chân trái lệch sang trái tâm thân, chân phải lệch sang phải.

Đây chính là 4 điểm `hip_joint` — gốc toạ độ cục bộ mà mọi phép tính IK trong `quadruped_gait` đều quy về.

### A.2 Chuỗi khớp của 1 chân — đây là phần quan trọng nhất để hiểu cấu hình

Mỗi chân là **3 khớp nối tiếp**, mỗi khớp chỉ xoay quanh 1 trục (revolute, 1-DOF):

```
trunk
  │  (offset cố định: ±0.1934 theo X, ±0.0465 theo Y)
  ▼
hip_joint    trục xoay = X (trục dọc thân, hướng trước-sau)
  │           → đây là khớp "abduction/adduction": xoè chân ra/khép vào
  │  (offset cố định: 0.0955m theo Y, xem thigh_offset)
  ▼
thigh_joint  trục xoay = Y (trục ngang thân, hướng trái-phải)
  │           → khớp "đùi": đưa chân ra trước/sau (giống bước đi)
  │  (chiều dài link: 0.213m)
  ▼
calf_joint   trục xoay = Y (song song thigh_joint)
  │           → khớp "gối": co/duỗi cẳng chân, chỉ gập được 1 chiều
  │  (chiều dài link: 0.213m)
  ▼
foot (điểm chạm đất, hình cầu bán kính 0.02m)
```

**Vì sao hip xoay quanh X còn thigh/calf xoay quanh Y?** Đây là thiết kế robot 4 chân tiêu chuẩn (Unitree, Boston Dynamics, MIT Cheetah đều theo mẫu này):
- Trục X (hip) cho phép chân xoè ra ngoài/khép vào trong → tạo ra chuyển động **đi ngang (strafe)** và giữ thăng bằng khi nghiêng.
- Trục Y (thigh, calf) nằm trong **mặt phẳng đứng dọc thân** (mặt phẳng X-Z) → đây là mặt phẳng chứa chuyển động bước đi chính (tiến/lùi + nhấc chân).

### A.3 Giới hạn góc khớp thật (dùng để kiểm tra gait không vượt quá)

| Khớp | Góc nhỏ nhất | Góc lớn nhất | Ghi chú |
|---|---|---|---|
| `hip_joint` | -60° | +60° | (±1.0472 rad) |
| `thigh_joint` | -90° | +200° | (-1.5708 → 3.4907 rad) |
| `calf_joint` | **-156°** | **-48°** | Luôn âm! Đầu gối chỉ gập về sau, không bao giờ duỗi thẳng hoàn toàn (0°) |

Chi tiết `calf_joint` luôn âm là điểm dễ gây nhầm lẫn nhất: **0° không phải là "chân thẳng đứng bình thường"** mà là "cẳng chân duỗi thẳng theo đường nối dài của đùi" (một tư thế cực đoan, gần như không dùng). Tư thế đứng bình thường của robot có `calf ≈ -90.47°` (gập gần vuông góc).

### A.4 `ros2_control.xacro` — cầu nối giữa "khớp vật lý" và "phần mềm"

File này khai báo với Gazebo: "12 khớp này nhận lệnh vị trí (`command_interface="position"`), trả về vị trí + vận tốc đo được (`state_interface`)". Mỗi khớp có thêm 1 tham số `position_proportional_gain` — hệ số P để Gazebo tự tính lực cần thiết ép khớp về đúng góc lệnh (vì vật lý mô phỏng chỉ hiểu "lực", không hiểu "vị trí" trực tiếp):

```
hip:          gain = 100   (nhẹ hơn — chỉ chống chịu lực xoay ngang)
thigh, calf:  gain = 300   (nặng hơn — chống chịu trọng lượng cả thân robot)
```

Hai con số này **không phải mình tự chọn** — lấy đúng từ file cấu hình PID thật của Unitree (`config/robot_control.yaml` trong repo gốc), để mô phỏng gần với robot thật nhất có thể.

`initial_value` trong cùng file quy định robot **spawn sẵn ở tư thế đứng** (hip=0°, thigh=45.23°, calf=-90.47°) thay vì spawn ở góc 0/0/0 rồi giật về tư thế đứng — đây là chỗ đã sửa lỗi robot bị lật (xem tài liệu tổng hợp trước).

---

## PHẦN B — `quadruped_gait`: robot "suy nghĩ" thế nào để đi

### B.1 Hệ quy chiếu dùng trong toàn bộ code

```
x: hướng ra TRƯỚC thân robot
y: hướng sang TRÁI thân robot
z: hướng LÊN trên (z âm = xuống dưới, vì chân luôn ở dưới thân)
```

Mọi hàm trong `leg_ik.py` đều nhận toạ độ bàn chân **tính từ khớp hip của chính chân đó** (không phải từ tâm thân) — đơn giản hoá bài toán: mỗi chân tự giải độc lập, không cần biết 3 chân kia đang ở đâu.

### B.2 `leg_ik.py` — biến "muốn bàn chân ở đâu" thành "3 góc khớp"

Đầu vào: `(x, y, z)` — vị trí mong muốn của bàn chân, và `hip_sign` (+1 cho chân trái, -1 cho chân phải, vì 2 bên đối xứng gương).

Quá trình giải **2 bước độc lập**:

**Bước 1 — giải góc hip** (trong mặt phẳng Y-Z, nhìn thân robot từ phía trước):
```python
b = sqrt(y² + z² - l1²)          # l1 = 0.0955m (chiều dài link hip)
theta1 = atan2(z, y) + atan2(b, l1)
```
Về hình học: đây là bài toán "tìm góc xoay để 1 thanh cứng độ dài l1 cộng thêm 1 đoạn b vuông góc với nó chạm tới điểm (y,z)" — giải bằng lượng giác thuần tuý, không cần thư viện.

**Bước 2 — giải góc thigh + calf** (bài toán "cánh tay 2 đoạn" kinh điển trong robot học, dùng định lý hàm cos):
```python
r = sqrt(x² + b²)                              # khoảng cách thẳng từ khớp thigh tới bàn chân
cos_theta3 = (r² - l2² - l3²) / (2·l2·l3)      # l2 = l3 = 0.213m
theta3 = -acos(cos_theta3)                      # góc gối: 0=duỗi thẳng, âm=gập
theta2 = atan2(x, b) - atan2(l3·sin(theta3), l2 + l3·cos(theta3))
```
Nếu `r` lớn hơn `l2+l3` (v.d 0.426m) hoặc nhỏ hơn `|l2-l3|` (=0) → điểm đó **ngoài tầm với** của chân, hàm trả về `None`.

File có kèm `forward_kinematics()` — hàm ngược lại (góc khớp → vị trí bàn chân), dùng để **tự kiểm tra**: đưa 1 điểm vào IK, lấy góc ra, đưa góc đó vào FK, phải ra đúng điểm ban đầu. Đây là cách đã bắt được 2 lỗi dấu trong quá trình làm.

### B.3 `trot_gait.py` — nhịp bước đi

**Ý tưởng dáng đi "trot":** chia 4 chân thành 2 cặp chéo, mỗi cặp luôn di chuyển đồng bộ, 2 cặp lệch pha nhau nửa chu kỳ:

```
Cặp 1: FR + RL (chân trước-phải + chân sau-trái) → cùng pha (phase offset = 0)
Cặp 2: FL + RR (chân trước-trái + chân sau-phải) → lệch pha (phase offset = 0.5)
```

Tại mọi thời điểm, luôn có **đúng 2 chân chạm đất chéo nhau** (tạo thế tam giác vững với trọng tâm) và 2 chân đang bước — giống hệt cách chó/mèo đi ở tốc độ trung bình.

**Mỗi chu kỳ của 1 chân chia làm 2 nửa:**

```
        stance (0 → 0.5)              swing (0.5 → 1.0)
    chân CHẠM ĐẤT, đẩy lùi        chân NHẤC LÊN, đưa về trước
z:  cố định = -stance_height      z tăng theo hình sin, đỉnh ở giữa (+0.06m)
x:  từ +dx/2  →  -dx/2            từ -dx/2  →  +dx/2
```

Trực giác: khi chân chạm đất, nếu bàn chân "lùi dần" so với hông (từ trước ra sau), mà bàn chân không trượt trên mặt đất, thì thân robot phải "tiến dần" so với bàn chân — đó chính là cách robot bước tới.

**Biên độ bước `dx`, `dy` được tính từ `cmd_vel` cộng với vị trí hông từng chân so với tâm thân** (công thức vật lý chuẩn "vận tốc điểm trên vật rắn quay" `v = v_thân + ω × r`):
```python
dx = (vx - wz·ry) · step_length_gain
dy = (vy + wz·rx) · step_width_gain
```
`rx, ry` là toạ độ hông của TỪNG chân so với tâm thân (`leg_offset_x/y` ở phần A.1) — đây là lý do khi robot rẽ (`wz≠0`), chân trước và chân sau tự động bước với biên độ khác nhau (giống ô tô rẽ: bánh ngoài đi quãng đường dài hơn bánh trong).

**Tần số bước tự thích nghi:**
```python
speed_demand = hypot(vx, vy) + |wz|·leg_offset_x
cycle_period = 0.5s / max(speed_demand / 0.3, 1.0)   # giới hạn dưới 0.28s
```
Tốc độ lệnh càng lớn, `cycle_period` càng nhỏ (bước nhanh hơn) — thay vì giữ nguyên nhịp 0.5s/chu kỳ rồi kéo dài bước ra (dễ mất thăng bằng ở tốc độ cao, đã gặp và sửa).

### B.4 `gait_node.py` — vòng lặp điều khiển thời gian thực

```
mỗi 1/50 giây (50Hz):
    đọc /cmd_vel mới nhất (hết hạn sau 0.5s không nhận được lệnh → tự dừng)
    + đọc yaw thực tế từ IMU
    + tính độ lệch hướng so với hướng đã "khoá" lúc bắt đầu đi
    + cộng thêm 1 lượng wz bù nhỏ để tự sửa hướng (heading-hold)
    → gọi trot_gait.step() ra 12 góc khớp
    → publish sang forward_position_controller
```

Thứ tự 12 góc trong mảng publish luôn cố định: `FR(hip,thigh,calf) → FL(...) → RR(...) → RL(...)`, phải khớp chính xác với thứ tự khai báo trong `config/go2_controllers.yaml` (nếu lệch thứ tự, lệnh sẽ gửi nhầm khớp).

---

## Tóm tắt liên hệ giữa 2 phần

```
quadruped_description                    quadruped_gait
────────────────────                    ──────────────
Định nghĩa VẬT LÝ:                       Định nghĩa LOGIC:
- l1=0.0955, l2=l3=0.213      ────────▶  leg_ik.py dùng ĐÚNG 3 số này
- hip trục X, thigh/calf trục Y ──────▶  quy ước dấu trong công thức IK phải khớp
- leg_offset_x=0.1934, y=0.0465 ──────▶  trot_gait.py dùng để tính rx,ry khi rẽ
- giới hạn góc khớp             ──────▶  self-test trong trot_gait.py kiểm tra không vượt
- thứ tự 12 khớp trong yaml     ──────▶  gait_node.py publish đúng thứ tự đó
```

Nói ngắn gọn: **`quadruped_description` là "sự thật vật lý"**, còn **`quadruped_gait` là bộ não phải tôn trọng đúng sự thật đó** — mọi hằng số dùng trong gait (chiều dài link, offset hông, giới hạn góc) đều lấy trực tiếp từ file URDF/xacro, không có số nào tự bịa ra.
