# CLAUDE.md

Hướng dẫn cho Claude Code khi làm việc trong repo này.

## Tổng quan

Mô phỏng robot 4 chân Unitree Go2 bằng ROS 2 Jazzy + Gazebo Harmonic, xây từ đầu (không dùng champ/quadruped_ros2_control) theo tài liệu kiến trúc `quadruped_ros_system.md`. URDF Go2 lấy từ `unitreerobotics/unitree_ros` (BSD-3-Clause, free).

- Repo GitHub: `git@github.com:yonroy/unitree-go2-gazebo.git`
- Git root = `~/Ros2` (chứa các file `.md` tài liệu + thư mục `quadruped_ws/`)
- ROS 2 workspace thật: `~/Ros2/quadruped_ws` (colcon)

## Môi trường & lệnh cơ bản

ROS 2 **không** tự source — luôn chạy trước mọi lệnh `ros2`/`colcon`/`gz`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros2/quadruped_ws/install/setup.bash
```

Build: `cd ~/Ros2/quadruped_ws && colcon build --symlink-install`

Chạy mô phỏng đầy đủ:
```bash
ros2 launch quadruped_bringup quadruped_sim.launch.py
```

**Lưu ý camera sensor trên máy GPU hybrid (Intel iGPU + NVIDIA dGPU):** mesh COLLADA (`.dae`) nhiều polygon của Go2 (~78k faces cho `base.dae`) làm **hỏng render context của camera sensor** → ảnh `/camera/image` ra màu xám đồng nhất (`std=0`). Đã kiểm chứng bằng thực nghiệm loại trừ: visual bằng box/mesh ít face → camera render đúng; visual bằng mesh nhiều face → camera hỏng (ngưỡng ~7700 faces/mesh, xảy ra với cả `ogre`/`ogre2`, cả GUI lẫn `--headless-rendering`). Đây là lỗi tầng gz-sim/driver, không phải lỗi model. **Giải pháp đang dùng: visual robot dùng box/cylinder primitive** (xem `xacro/robot.xacro`, `xacro/leg.xacro`) — collision/inertial vẫn giữ số thật, không ảnh hưởng vật lý/điều khiển. Mesh `.dae` gốc vẫn còn trong `meshes/` nhưng không tham chiếu trong xacro. (Mesh giảm poly ~7k faces render được nhưng nhìn xấu hơn box nên không dùng.)

Launch có tham số `headless:=true` (chạy `-s --headless-rendering`, không GUI) — không bắt buộc cho camera nữa nhưng giữ lại để chạy nhẹ khi không cần cửa sổ.

Điều khiển thủ công (chọn 1):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run quadruped_teleop joystick_panel     # GUI joystick ảo, can DISPLAY=:1
```

Debug/verify trực tiếp trong Gazebo (bỏ qua ROS, đọc thẳng ground-truth):
```bash
gz topic -e -t /world/flat_ground/pose/info -n 1 | grep -A12 'name: "go2"'
```

`sudo` trên máy này cần nhập mật khẩu thủ công — Claude không tự chạy được lệnh cần sudo, phải đưa script cho user tự chạy trong terminal của họ.

## Cấu trúc package

| Package | Vai trò |
|---|---|
| `quadruped_description` | URDF Go2 (xacro), world Gazebo, controller config, launch spawn robot |
| `quadruped_gait` | IK giải tích 3-DOF (`leg_ik.py`), trot gait (`trot_gait.py`), node điều khiển (`gait_node.py`) |
| `quadruped_bringup` | Launch tổng |
| `quadruped_teleop` | Bảng điều khiển joystick ảo (Tkinter) + khung hiển thị camera `/camera/image` kèm bounding box từ `/detections` (vẽ bằng PIL). Camera là tuỳ chọn: thiếu `cv_bridge`/`PIL`/`vision_msgs` thì panel vẫn chạy, chỉ bỏ khung camera. |
| `quadruped_interfaces` | Custom action `GotoPoint.action`, `TrackObject.action`, msg `TrackedTarget.msg` |
| `quadruped_navigation` | Action server `goto_point_server` (đi tới toạ độ x,y rồi dừng) |
| `quadruped_perception` | Camera RGBD (`camera_link`/`camera_link_optical` trong xacro) → YOLO (`object_detector.py`) → tracker đơn giản (`tracker.py`) → điểm 3D khung `trunk` (`target_pose_node.py`). Phụ thuộc `ultralytics`/`torch` cài qua pip (không phải rosdep), xem ghi chú bên dưới. |
| `quadruped_follow` | Action server `track_object_server` (bám mục tiêu, giữ `stop_distance`) + logic PID thuần `follow_object.py` |
| `quadruped_rl` | RL locomotion: chạy policy ONNX (`diasAiMaster/unitree-go2-velocity-flat`, BSD-3) thay gait rule-based. `policy_runner.py` (logic thuần + self-test) + `rl_policy_node.py` (obs 45 chiều → ONNX → PD torque → effort). Launch `rl_locomotion.launch.py`. Phụ thuộc `onnxruntime` (pip). |

Xem giải thích chi tiết: `quadruped_implementation.md`, `quadruped_config_explained.md`, `quadruped_math_formulas.md`. Checklist tiến độ: `TODO.md`.

### Phụ thuộc pip cho `quadruped_perception` (không qua rosdep/apt)

`ultralytics` (YOLO) + `torch` không phải rosdep key, cài thủ công 1 lần (máy có GPU NVIDIA nên torch sẽ tự dùng CUDA nếu có):
```bash
sudo apt install -y python3-pip ros-jazzy-vision-msgs   # can sudo, user tu chay
python3 -m pip install --user --break-system-packages ultralytics   # khong can sudo
```
**Lưu ý xung đột numpy:** `ultralytics` kéo numpy 2.x vào `~/.local` đè numpy 1.x hệ thống mà `cv_bridge` (biên dịch sẵn) cần → crash khi import. Đã ghim lại:
```bash
python3 -m pip install --user --break-system-packages "numpy==1.26.4"
```

Weight `yolov8n.pt` (~6MB) tự tải về CWD lần chạy đầu (cần internet, đã `.gitignore *.pt`).

### RL locomotion (`quadruped_rl`) — điểm đã kiểm chứng thực tế

Chạy: `ros2 launch quadruped_rl rl_locomotion.launch.py headless:=true`. Cài `onnxruntime` qua pip (`pip install --user --break-system-packages onnxruntime`). Policy ONNX + config đã kèm trong `quadruped_rl/models/` (tải từ HuggingFace).

Các bug **chỉ lộ khi đo số liệu thật trong Gazebo** (đúng nguyên tắc), theo thứ tự phát hiện:
1. **Tính PD 1 lần/policy-step (50Hz) rồi giữ torque suốt 20ms → bơm năng lượng → co giật** (khớp 20-30 rad/s, torque bão hoà). Fix: **2 vòng** — policy 50Hz cập nhật `target`, PD tính lại torque ở nhịp `/joint_states` (~200Hz) với q,qd mới.
2. **Robot limp ngã trong lúc chờ controller active lúc khởi động.** Fix: **startup hand-off** — `forward_position_controller` giữ robot đứng, node chờ ổn định (~3s) rồi gọi `switch_controller` (tắt position, bật `joint_effort_controller`) mới chạy policy. Cần `effort` command_interface trong `ros2_control.xacro` (đã thêm, giữ cả `position` cho gait).
3. **QUAN TRỌNG NHẤT — sai thứ tự khớp.** ONNX nhận obs/xuất action theo **thứ tự POLICY (sim MJCF = FL,FR,RL,RR)**, khác **thứ tự SDK Unitree (FR,FL,RR,RL)** mà `/joint_states`/controller dùng. `joint_ids_map=[3,4,5,0,1,2,9,10,11,6,7,8]` (deploy.yaml) là hoán vị FR↔FL, RR↔RL. **Không remap thì robot đứng yên OK nhưng lộn nhào ngay khi policy chạy** (chân trái/phải bị tráo). Fix: remap trong `policy_runner.py` (`JOINT_IDS_MAP`, là involution). Sau fix: robot **đứng vững + đi được** (vx=0.6 → ~0.27 m/s, thẳng đứng).

**Trạng thái transfer:** policy MuJoCo→Gazebo đi được nhưng chưa hoàn hảo — đi chậm hơn lệnh (~0.27 vs 0.6 m/s), có trôi/cong hướng và không đứng yên tuyệt đối khi lệnh=0 (residual sim-to-sim gap, có thể tinh chỉnh thêm).

**Điểm đã kiểm chứng thực tế của pipeline track_object (bug chỉ lộ khi đo số liệu thật):**
- `object_detector.py` **phải** truyền `conf=min_confidence` vào `model.predict()` — mặc định ultralytics lọc ở 0.25, cắt hết detection yếu hơn *trước* khi lọc theo tham số của mình (từng làm `/detections` rỗng dù camera vẫn thấy bóng).
- Quả cầu trơn **không texture** nằm ngoài phân bố COCO → YOLO gán nhãn không ổn định (orange/sports ball/kite/frisbee) và confidence thấp (~0.15-0.2); model lớn hơn (yolov8s/m) còn tệ hơn (bowl/mouse/toilet). Vì vậy `target_classes` nhận nhiều nhãn + `min_confidence=0.10`. Muốn detect chắc hơn thì cần object có texture thật.
- Camera nghiêng xuống ~15° (`camera_joint` rpy trong `robot.xacro`) để mục tiêu dưới đất không rơi khỏi khung khi robot tới gần.

## Quy ước code trong repo này

- **Comment/docstring bằng tiếng Việt không dấu** (ASCII) trong toàn bộ code — giữ nhất quán khi thêm code mới.
- **Mọi module logic thuần (không phụ thuộc ROS) phải có self-test** ở `if __name__ == "__main__":`, chạy độc lập bằng `python3 -m <package>.<module>` (không cần ROS/Gazebo). Ví dụ: `leg_ik.py` tự kiểm IK↔FK round-trip, `trot_gait.py` tự kiểm giới hạn góc khớp, `goto_point.py` tự kiểm hội tụ bằng mô phỏng động học unicycle.
- **Không tự bịa hằng số hình học** — mọi chiều dài link, offset hip, giới hạn góc khớp lấy trực tiếp từ `quadruped_description/xacro/const.xacro` (số thật của Go2), không đoán.
- Đặt tên joint/topic phải khớp chính xác thứ tự khai báo trong `quadruped_description/config/go2_controllers.yaml` (`FR→FL→RR→RL`, mỗi chân `hip,thigh,calf`) — sai thứ tự sẽ gửi lệnh nhầm khớp.

## Nguyên tắc quan trọng nhất: luôn kiểm chứng bằng số liệu thật

**Đừng tin suy luận/đạo hàm công thức là đúng chỉ vì code chạy không lỗi.** Trong quá trình xây dựng, nhiều bug nghiêm trọng (sai dấu công thức IK, robot đi lùi khi lệnh đi tới, mất ổn định ở tốc độ cao) đều **chỉ lộ ra khi đo số liệu thật trong Gazebo** (`gz topic`, `ros2 topic echo /joint_states`), không phát hiện được qua đọc code hay build pass. Khi thêm/sửa bất kỳ tính năng điều khiển/động học nào:
1. Viết self-test độc lập trước (không cần ROS).
2. Sau khi deploy, đo số liệu thật trong Gazebo đang chạy để xác nhận đúng hướng/độ lớn/ổn định — không kết luận "xong" chỉ từ đọc code.

## Trạng thái nhánh (cập nhật lần cuối 2026-07-02)

- `main`: baseline hoàn chỉnh (bước 1-3 lộ trình §13), đã push GitHub.
- `feature/gotopoint`: đã commit local (`557e625`), **chưa push** — thêm action `goto_point` (đi tới toạ độ x,y). Đã thêm tiếp feature `track_object` (camera + YOLO + action bám mục tiêu) trên cùng nhánh này, code + kiểm chứng số liệu thật xong nhưng **chưa commit/push** — hỏi user trước khi commit/push/merge.
- Chưa làm: bước 4-5 (RL training, sim-to-real) — xem `TODO.md`. Bước 6 (follow-object) **đã xong và kiểm chứng thực tế trong Gazebo** (robot bám tới `stop_distance`, bám được mục tiêu di động).
