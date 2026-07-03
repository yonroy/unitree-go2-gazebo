# Unitree Go2 — ROS 2 + Gazebo + RL

Mô phỏng robot 4 chân **Unitree Go2** xây từ đầu bằng **ROS 2 Jazzy + Gazebo Harmonic**
(không dùng champ / quadruped_ros2_control), kèm **RL locomotion** (chạy policy neural
thay gait rule-based) và pipeline **vision follow-object** (camera + YOLO).

<p align="center">
  <img src="docs/images/go2_mujoco_walk.gif" width="480" alt="Go2 di bang RL policy trong MuJoCo"><br>
  <em>RL policy (ONNX) điều khiển Go2 — đi thẳng, không drift, trong MuJoCo</em>
</p>

## Tính năng chính

| | Mô tả |
|---|---|
| 🦿 **Gait rule-based** | IK giải tích 3-DOF + trot gait, `/cmd_vel` → 12 khớp @ 50Hz (`quadruped_gait`) |
| 🎯 **Đi tới toạ độ** | Action `goto_point` — tới (x,y) rồi dừng (`quadruped_navigation`) |
| 👁️ **Follow-object** | Camera RGBD → YOLO → tracker → action `track_object` bám mục tiêu (`quadruped_perception` + `quadruped_follow`) |
| 🕹️ **Teleop** | Bảng joystick ảo Tkinter + khung camera kèm bounding box (`quadruped_teleop`) |
| 🧠 **RL locomotion** | Chạy policy ONNX pretrained thay gait; obs 45 chiều → PD torque (`quadruped_rl`) |

## Cấu trúc package

| Package | Vai trò |
|---|---|
| `quadruped_description` | URDF Go2 (xacro), world, controller config, launch spawn |
| `quadruped_gait` | IK (`leg_ik.py`) + trot (`trot_gait.py`) + `gait_node.py` |
| `quadruped_navigation` | `goto_point_server` (đi tới x,y) |
| `quadruped_perception` | Camera → YOLO `object_detector` → `tracker` → `target_pose_node` |
| `quadruped_follow` | `track_object_server` (PID bám, giữ `stop_distance`) |
| `quadruped_teleop` | Joystick ảo + hiển thị camera/detection |
| `quadruped_rl` | RL policy ONNX → effort/PD; + `mujoco_demo/` (lái bằng joystick) |
| `quadruped_interfaces` | Action `GotoPoint`, `TrackObject`; msg `TrackedTarget` |

## Chạy nhanh

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros2/quadruped_ws/install/setup.bash

# Mô phỏng đầy đủ (gait + follow-object)
ros2 launch quadruped_bringup quadruped_sim.launch.py headless:=true

# RL locomotion (policy neural thay gait)
ros2 launch quadruped_rl rl_locomotion.launch.py headless:=true
```

Build: `cd ~/Ros2/quadruped_ws && colcon build --symlink-install`

## RL locomotion: MuJoCo vs Gazebo

Policy `unitree-go2-velocity-flat` (HuggingFace, BSD-3) chạy **đẹp trong MuJoCo**
(sim gốc của nó — đứng yên tuyệt đối khi lệnh=0, đi thẳng) nhưng **bị trôi trong
Gazebo** do gap sim-to-sim (mô hình actuator/contact khác). Pipeline tích hợp ROS2
(obs 45, remap khớp `joint_ids_map`, PD 2 vòng, effort + startup hand-off) tái dùng
nguyên vẹn — muốn hết drift thì train lại policy bằng chính URDF (Genesis).

### Lái Go2 bằng joystick trong MuJoCo + test chướng ngại

```bash
cd ~/Ros2/quadruped_ws/src/quadruped_rl/mujoco_demo
DISPLAY=:1 python3 mujoco_joystick.py            # nền phẳng
DISPLAY=:1 python3 mujoco_joystick.py obstacles  # cầu thang + chướng ngại
```

<p align="center">
  <img src="docs/images/obstacle_course.png" width="520" alt="Course chuong ngai: go -> buc -> cau thang">
</p>

Xem `quadruped_ws/src/quadruped_rl/mujoco_demo/README.md` để biết điều khiển chi tiết.

## Tài liệu

`quadruped_ros_system.md` (kiến trúc), `quadruped_implementation.md`,
`quadruped_config_explained.md`, `quadruped_math_formulas.md`, `CLAUDE.md`, `TODO.md`.

## Nguồn / license

- URDF Go2: [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) (BSD-3)
- RL policy: [diasAiMaster/unitree-go2-velocity-flat](https://huggingface.co/diasAiMaster/unitree-go2-velocity-flat) (BSD-3)
- MuJoCo model: [MuJoCo Menagerie unitree_go2](https://github.com/google-deepmind/mujoco_menagerie) (BSD-3)
- YOLO: [ultralytics](https://github.com/ultralytics/ultralytics)
