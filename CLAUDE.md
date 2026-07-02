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
| `quadruped_teleop` | Bảng điều khiển joystick ảo (Tkinter) |
| `quadruped_interfaces` | Custom action `GotoPoint.action` |
| `quadruped_navigation` | Action server `goto_point_server` (đi tới toạ độ x,y rồi dừng) |

Xem giải thích chi tiết: `quadruped_implementation.md`, `quadruped_config_explained.md`, `quadruped_math_formulas.md`. Checklist tiến độ: `TODO.md`.

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
- `feature/gotopoint`: đã commit local (`557e625`), **chưa push** — thêm action `goto_point` (đi tới toạ độ x,y). Hỏi user trước khi push/merge nếu tiếp tục nhánh này.
- Chưa làm: bước 4-6 (RL training, sim-to-real, follow-object vision) — xem `TODO.md`.
