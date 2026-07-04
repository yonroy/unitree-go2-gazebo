# Tóm tắt dự án — Unitree Go2 Quadruped Simulation

> Cập nhật: 2026-07-04 · Nhánh: `feature/gotopoint`

## 1. Tổng quan
Mô phỏng robot 4 chân **Unitree Go2**, xây **từ đầu** (không dùng champ/quadruped_ros2_control),
trên **ROS 2 Jazzy + Gazebo Harmonic**, kèm bộ **demo MuJoCo** độc lập (không cần ROS).
URDF lấy từ `unitreerobotics/unitree_ros` (BSD-3).

- Git root: `~/Ros2` · Workspace: `~/Ros2/quadruped_ws` (colcon)

## 2. Các package
| Package | Vai trò |
|---|---|
| `quadruped_description` | URDF/xacro, world Gazebo, config controller, LiDAR + camera sensor |
| `quadruped_gait` | IK giải tích 3-DOF + trot gait + node điều khiển |
| `quadruped_bringup` | Launch tổng |
| `quadruped_teleop` | Joystick ảo Tkinter + khung camera + bounding box |
| `quadruped_interfaces` | Action `GotoPoint`, `TrackObject`; msg `TrackedTarget` |
| `quadruped_navigation` | `goto_point_server` + `obstacle_avoider` (reactive) + `planner` (A*+pure pursuit) + SLAM launch |
| `quadruped_perception` | Camera RGBD → YOLO → tracker → điểm 3D khung trunk |
| `quadruped_follow` | `track_object_server` (bám mục tiêu, giữ khoảng cách) |
| `quadruped_rl` | Policy ONNX (locomotion) thay gait + `mujoco_demo/` |

## 3. Tính năng đã xong & kiểm chứng số liệu thật
- ✅ **Trot gait** — IK↔FK round-trip; xoay tại chỗ 360° (đo Gazebo: ~0.77 rad/s, drift <3cm, không ngã).
- ✅ **goto_point** — action đi tới (x,y) bằng odometry ground-truth.
- ✅ **track_object** — camera + YOLO + PID bám mục tiêu tới `stop_distance`.
- ✅ **LiDAR né vật cản + SLAM** (Gazebo) — reactive VFH-lite, `slam_toolbox` dựng `/map`, gần nhất ≥0.87m.
- ✅ **RL locomotion** (ONNX) — robot đứng + đi + xoay tại chỗ (sau khi fix, xem §4).
- ✅ **MuJoCo demos** — joystick, lidar_avoid, lidar_nav (A*), lidar_rviz, mê cung.

## 4. Điểm kỹ thuật quan trọng (bug chỉ lộ khi đo thật)
- **RL sai thứ tự khớp** — ONNX theo thứ tự POLICY (FL,FR,RL,RR) ≠ SDK (FR,FL,RR,RL);
  phải remap `joint_ids_map`, nếu không robot lộn nhào.
- **RL deadlock** — node hammer `switch_controller` ~200Hz làm nghẽn controller_manager →
  effort controller không nạp → policy không bao giờ chạy. Fix: throttle 1Hz + nạp effort sớm.
  Sau fix xoay được ~0.74 rad/s.
- **MuJoCo ngã khi xoay CW gấp** — `wz ≤ −0.80` làm robot ngã (nghiêng ~28°);
  đã clamp `wz ∈ [−0.75, +1.0]` cả 4 script mujoco_demo.
- **Cua giật ở góc** — reactive nhảy lệnh ~0.63 rad/s/tick → thêm `SlewLimiter` (giảm còn 0.15).
- **Kẹt ngõ cụt mê cung** — reactive không nhớ đường; thêm `StuckEscape` + đi chậm hơn
  (hết ngã, nhưng vẫn có thể kẹt — giới hạn bản chất của reactive).
- **Nav (A*) là công cụ đúng cho mê cung** — đo bằng App thật: không bao giờ ngã, tới 4/6
  đích góc xa (tinh chỉnh: roam khám phá khi chưa có đường, MAP_M 4→5, inflation 0.28, replan nhanh).

## 5. RL transfer: MuJoCo vs Gazebo
Policy `unitree-go2-velocity-flat` (HuggingFace, BSD-3) chạy chuẩn trong **MuJoCo**
(đứng yên khi lệnh=0; xoay tới ~22°/s ở wz=1.0) nhưng **trôi trong Gazebo** (~0.28 m/s
khi lệnh=0; xoay chỉ ~0.24 rad/s bão hoà) do gap sim-to-sim (actuator/contact khác).
Muốn hết drift → train lại policy bằng chính URDF.

## 6. Nguyên tắc làm việc của repo
- Comment/docstring tiếng Việt không dấu (ASCII).
- Mọi module logic thuần có **self-test** (`python3 -m <package>.<module>`).
- Không bịa hằng số hình học — lấy từ `const.xacro`.
- **Luôn kiểm chứng bằng số liệu thật** — nhiều bug nghiêm trọng chỉ lộ khi đo trong sim
  (Gazebo `gz topic`, hoặc chạy chính class App MuJoCo), không phát hiện qua đọc code.

## 7. Còn lại / hướng tiếp
- Tăng tin cậy 2 đích "phía bắc" của nav (roam có định hướng về đích).
- RL training / sim-to-real (bước 4-5 lộ trình) — chưa làm.

Xem chi tiết: `README.md`, `CLAUDE.md`, `quadruped_ws/src/quadruped_rl/mujoco_demo/README.md`, `TODO.md`.
