# Checklist tiến độ — Go2 ROS 2 Simulation

> Theo lộ trình 6 bước ở `quadruped_ros_system.md` §13. Cập nhật thủ công khi làm thêm việc mới.

## Bước 1 — Xác nhận phần cứng / môi trường
- [x] Cài ROS 2 Jazzy + Gazebo Harmonic + ros2_control (apt)
- [x] Tải URDF Go2 miễn phí (`unitreerobotics/unitree_ros`, BSD-3-Clause)
- [x] Port URDF từ ROS1/Gazebo Classic sang ROS 2/Gazebo Harmonic (`gz_ros2_control`)

## Bước 2 — Baseline rule-based (gait cố định + IK)
- [x] IK giải tích 3-DOF cho 1 chân (`leg_ik.py`) + self-test round-trip
- [x] Trot gait planner (`trot_gait.py`) — 2 cặp chân chéo, stance/swing
- [x] `gait_node.py`: `/cmd_vel` → 12 góc khớp, 50Hz
- [x] Tần số bước tự thích nghi theo tốc độ (tránh mất ổn định khi đi nhanh)
- [x] Heading-hold bằng IMU (chống lệch hướng khi đi thẳng lâu)

## Bước 3 — Setup mô phỏng chính xác trong Gazebo
- [x] World phẳng + physics + IMU sensor (`flat_ground.sdf`)
- [x] Controller config (`joint_state_broadcaster` + `forward_position_controller`)
- [x] Launch tổng (`quadruped_bringup`)
- [x] Robot đứng vững, không đổ khi spawn (`initial_value` đúng tư thế đứng)
- [x] Verify đi tới/lùi/ngang/xoay đúng hướng bằng số liệu thực (`gz topic`)
- [x] Bảng điều khiển joystick ảo (Tkinter, `quadruped_teleop`)

## Đã đẩy lên GitHub
- [x] `git init`, `.gitignore` (loại `build/`, `install/`, `log/`)
- [x] SSH key + push toàn bộ code lên `github.com/yonroy/unitree-go2-gazebo`

---

## Bước 4 — Train blind policy (RL)
- [ ] Setup Isaac Lab hoặc legged_gym trên máy (RTX 4050 6GB)
- [ ] Định nghĩa reward function (đi thẳng, ổn định, tiết kiệm năng lượng)
- [ ] Domain randomization (ma sát, khối lượng, độ trễ)
- [ ] Train policy chỉ dùng IMU + encoder (không cần camera)

## Bước 5 — Sim-to-real blind policy
- [ ] Xuất policy ONNX, viết node inference ROS 2
- [ ] Thay `gait_node` (rule-based) bằng RL policy node
- [ ] Test lại trong Gazebo trước khi lên robot thật

## Bước 6 — Ghép follow-object (vision)
- [ ] `quadruped_perception`: camera + YOLO detect + tracker (ByteTrack)
- [ ] `quadruped_follow`: PID bám mục tiêu → `/cmd_vel`
- [ ] Test trong Gazebo với object giả lập

---

## Việc lặt vặt / cải thiện thêm (không bắt buộc)
- [ ] Sửa dứt điểm Y-drift còn sót lại (hiện chỉ sửa yaw qua heading-hold)
- [ ] Thêm E-stop phần mềm (giới hạn tốc độ/gia tốc đột ngột)
- [ ] Ghi log rosbag để debug sim-to-real sau này
- [ ] Viết README.md tử tế cho repo GitHub (hiện chỉ có 1 dòng)
