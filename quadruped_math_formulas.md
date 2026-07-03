# Công thức toán đã dùng trong dự án

> Tổng hợp toàn bộ công thức toán học thực sự nằm trong code — không lý thuyết suông, mỗi công thức đều trỏ tới file/hàm cụ thể và đã được tự kiểm chứng bằng self-test hoặc đo trong Gazebo.

---

## 1. Động học ngược 1 chân (Inverse Kinematics) — `leg_ik.py`

### Bài toán
Cho trước vị trí bàn chân mong muốn `(x, y, z)` tính từ khớp hip, tìm 3 góc khớp `(θ1, θ2, θ3)` = (hip, thigh, calf).

**Quy ước trục:** x = trước, y = trái, z = lên. `hip_sign` = +1 (chân trái) / −1 (chân phải).

**Hằng số hình học Go2:** `l1 = 0.0955m` (hip), `l2 = 0.213m` (thigh), `l3 = 0.213m` (calf).

### Bước 1 — góc hip θ1 (giải trong mặt phẳng Y-Z)

Khớp hip xoay quanh trục X, đẩy khớp thigh ra một đoạn `l1` theo Y. Đặt:

```
b = √(y² + z² − l1²)
```

`b` là khoảng cách còn lại sau khi "trừ" đoạn `l1`. Vì `l1` (đoạn cố định) và `b` (đoạn còn lại) tạo thành 2 cạnh vuông góc của cùng 1 tam giác vuông xoay theo θ1, ta có hệ:

```
y = l1·cos(θ1) + b·sin(θ1)
z = l1·sin(θ1) − b·cos(θ1)
```

Giải bằng số phức (`y + iz = (l1 − ib)·e^(iθ1)`):

```
θ1 = atan2(z, y) + atan2(b, l1)
```

> **Lưu ý dấu (bug đã sửa):** ban đầu dùng `θ1 = atan2(z,y) − atan2(b,l1)` — sai vì trục "xuống" (foot ở dưới hip) tương ứng `D̂ = Rx(θ1)·(0,0,−1) = (0, sinθ1, −cosθ1)`, không phải `(0,−sinθ1,cosθ1)`. Sai dấu này làm góc hip tính ra ~144° thay vì đúng phải là 0° ở tư thế đứng thẳng — self-test round-trip IK→FK bắt được lỗi này.

### Bước 2 — góc thigh θ2, calf θ3 (bài toán tay máy 2 đoạn phẳng)

Trong mặt phẳng còn lại (trục a, b), với `a = −x` (dấu âm vì trục quay thigh/calf là +Y theo quy tắc bàn tay phải, góc dương lại đẩy chân về **−X**, ngược trực giác ban đầu — **bug nghiêm trọng nhất đã sửa**, phát hiện qua đo trực tiếp trong Gazebo: lệnh tiến `vx=+0.3` khiến robot đi lùi):

```
r² = a² + b²
cos(θ3) = (r² − l2² − l3²) / (2·l2·l3)        [định lý hàm cos]
θ3 = −acos(cos θ3)                              0 = duỗi thẳng, âm = gập
α  = atan2(l3·sin θ3, l2 + l3·cos θ3)
θ2 = atan2(a, b) − α
```

Nếu `r > l2+l3` hoặc `r < |l2−l3|` → điểm ngoài tầm với, trả `None`.

### Kiểm chứng (Forward Kinematics)
```
a = l2·sin(θ2) + l3·sin(θ2+θ3)
b = l2·cos(θ2) + l3·cos(θ2+θ3)
x = −a
y = l1·cos(θ1) + b·sin(θ1)
z = l1·sin(θ1) − b·cos(θ1)
```
Self-test: đưa điểm vào IK → lấy góc → đưa vào FK → phải ra đúng điểm ban đầu (sai số đo được: **~10⁻¹⁷ m**, tức bằng 0 về mặt số học).

---

## 2. Dáng đi Trot — `trot_gait.py`

### 2.1 Vận tốc thân → biên độ bước mỗi chân

Áp dụng công thức vật lý vật rắn quay: **vận tốc một điểm trên vật rắn = vận tốc tịnh tiến + vận tốc góc × bán kính**:

```
v_hip = v_thân + ω × r_hip
```

Với `r_hip = (rx, ry)` là vị trí hip so với tâm thân (`rx = ±0.1934` trước/sau, `ry = ±0.0465` trái/phải), khai triển tích có hướng 2D (`ω × r = ω·(−ry, rx)`):

```
dx = (vx − wz·ry) · step_length_gain     (step_length_gain = 0.35)
dy = (vy + wz·rx) · step_width_gain      (step_width_gain  = 0.20)
```

→ đây là lý do khi rẽ (`wz≠0`), 4 chân tự động bước biên độ khác nhau (giống ô tô rẽ: bánh ngoài đi xa hơn bánh trong).

### 2.2 Quỹ đạo bàn chân theo pha (phase ∈ [0,1))

**Pha stance** (chạm đất, đẩy lùi):
```
x(phase) = dx/2 − dx·phase
y(phase) = dy/2 − dy·phase
z = −stance_height                        (cố định, = −0.30m)
```

**Pha swing** (nhấc chân, đưa về trước):
```
x(phase) = −dx/2 + dx·phase
y(phase) = −dy/2 + dy·phase
z(phase) = −stance_height + swing_height·sin(π·phase)     (swing_height = 0.06m)
```

`stance` chiếm nửa đầu chu kỳ (`phase < 0.5`), `swing` nửa sau — trực giác: bàn chân "lùi dần" so với hip trong lúc chạm đất (mà không trượt trên mặt đất) ⇒ thân robot "tiến dần" so với bàn chân.

### 2.3 Lệch pha giữa 4 chân (dáng đi trot)
```
FR, RL:  phase_offset = 0.0     (2 chân chéo, cùng pha)
FL, RR:  phase_offset = 0.5     (2 chân chéo còn lại, lệch nửa chu kỳ)
```

### 2.4 Tần số bước tự thích nghi theo tốc độ

```
speed_demand = √(vx² + vy²) + |wz|·leg_offset_x
speed_ratio  = speed_demand / nominal_speed        (nominal_speed = 0.3 m/s)
cycle_period = base_cycle_period / max(speed_ratio, 1.0)      (base = 0.5s)
cycle_period = max(cycle_period, min_cycle_period)             (sàn = 0.28s)
```

Tốc độ lệnh càng cao → chu kỳ càng ngắn (bước nhanh hơn) thay vì giữ nguyên nhịp rồi kéo dài bước ra. **Bug đã sửa:** trước khi có công thức này, tăng tốc độ joystick làm robot nghiêng/giật vì biên độ bước tăng vô hạn theo `vx` trong khi thời gian stance cố định — chân phải di chuyển nhanh hơn khả năng bám của bộ điều khiển vị trí.

Pha tích lũy dạng tăng dần (không phải chia thời gian tuyệt đối cho chu kỳ) để đổi `cycle_period` giữa chừng không làm pha nhảy cóc:
```
phase(t+dt) = (phase(t) + dt/cycle_period) mod 1.0
```

---

## 3. Heading-hold (giữ hướng) — `gait_node.py`

### Trích yaw từ quaternion (IMU)
```
yaw = atan2( 2·(w·z + x·y),  1 − 2·(y² + z²) )
```
(công thức chuẩn chuyển quaternion → góc Euler yaw, chỉ dùng thành phần z/w vì robot hiếm khi roll/pitch lớn).

### Bộ điều khiển bù lệch hướng
```
target_yaw += wz_lệnh · dt                          (tích lũy hướng mong muốn)
yaw_error   = wrap_to_pi(target_yaw − yaw_đo_được)
wz_bù       = clamp(kp_yaw · yaw_error, −max_bù, +max_bù)      (kp_yaw = 1.5)
wz_thực_tế  = wz_lệnh + wz_bù
```
`wrap_to_pi(θ) = ((θ + π) mod 2π) − π` — đưa góc về khoảng [−π, π] để tránh nhảy góc khi qua ±180°.

Kết quả đo: yaw lệch giảm từ ~5.8°/3s xuống còn ~0.6°/8s (>10 lần).

---

## 4. Goto-point — `goto_point.py`

### Sai số vị trí và hướng
```
dx = target_x − x
dy = target_y − y
distance        = √(dx² + dy²)
desired_heading = atan2(dy, dx)
heading_error   = wrap_to_pi(desired_heading − yaw)
```

### Bộ điều khiển P kép (khoảng cách + góc), có ưu tiên
```
angular_z = clamp(kp_angular · heading_error, −max_angular, max_angular)     (kp_angular=1.5)

nếu |heading_error| < 30°:
    linear_x = clamp(kp_linear · distance, −max_linear, max_linear)          (kp_linear=0.8)
else:
    linear_x = 0        # lệch hướng quá nhiều -> quay tại chỗ trước, chưa tiến

nếu distance < 0.15m:
    dừng hẳn, báo "đã tới"
```

Đây chính là mẫu **"PID vị trí + khoảng cách → cmd_vel"** ở §5 tài liệu gốc (vốn thiết kế cho follow-object), áp dụng lại cho trường hợp mục tiêu là 1 toạ độ cố định thay vì vật thể di động.

Self-test: mô phỏng động học unicycle đơn giản (`yaw += ω·dt; x += v·cos(yaw)·dt; y += v·sin(yaw)·dt`) để kiểm tra vòng lặp **hội tụ thật** về mục tiêu, không chỉ kiểm tra công thức đứng yên — chạy qua 3 kịch bản (mục tiêu chéo trước, mục tiêu phía sau, robot đang quay ngược 180°), tất cả hội tụ trong sai số < 0.15m.

---

## Bảng tổng hợp hằng số dùng trong công thức

| Hằng số | Giá trị | Nguồn |
|---|---|---|
| `l1` (hip length) | 0.0955 m | Go2 thật, `const.xacro` |
| `l2, l3` (thigh, calf) | 0.213 m | Go2 thật, `const.xacro` |
| `leg_offset_x` | 0.1934 m | Go2 thật, `const.xacro` |
| `leg_offset_y` | 0.0465 m | Go2 thật, `const.xacro` |
| `stance_height` | 0.30 m | Tự chọn (đã kiểm chứng ổn định) |
| `swing_height` | 0.06 m | Tự chọn |
| `base_cycle_period` | 0.5 s | Tự chọn, đã kiểm chứng |
| `nominal_speed` | 0.3 m/s | Tốc độ đã kiểm chứng ổn định |
| `position_tolerance` (goto-point) | 0.15 m | Tự chọn |
