# MuJoCo joystick demo — lái Go2 bằng RL policy

Chạy policy `unitree-go2-velocity-flat` (ONNX) trong **MuJoCo** (sim gốc của nó, đi
chuẩn không drift, khác Gazebo) và **lái bằng cần gạt ảo**. Không cần ROS.

<p align="center">
  <img src="../../../../docs/images/go2_mujoco_montage.png" width="620" alt="Go2 trot gait trong MuJoCo"><br>
  <em>Go2 đi bằng RL policy (6 khung hình) — dáng trot, thân giữ ~0.32m</em>
</p>

![Course chuong ngai](../../../../docs/images/obstacle_course.png)

## Cài 1 lần (nếu chưa)

```bash
python3 -m pip install --user --break-system-packages mujoco onnxruntime pillow
```

Model Go2 (`go2_model/`) là MuJoCo Menagerie (BSD, ~30MB, đã `.gitignore`). Nếu thiếu:
```bash
cd /tmp && git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie && git sparse-checkout set unitree_go2
cp -r unitree_go2 ~/Ros2/quadruped_ws/src/quadruped_rl/mujoco_demo/go2_model
# copy lai scene_obstacles.xml vao go2_model/ (xem git)
```

## Mở

```bash
export DISPLAY=:1                      # neu chay tren may co man hinh :1
cd ~/Ros2/quadruped_ws/src/quadruped_rl/mujoco_demo

python3 mujoco_joystick.py             # nen phang
python3 mujoco_joystick.py obstacles   # course: go thap -> buc -> cau thang
```

Cửa sổ **"Go2 Joystick — MuJoCo"** hiện ra (Alt+Tab nếu bị che).

## Điều khiển

| Widget | Tác dụng |
|---|---|
| Cần trái (vòng tròn) | Kéo lên/xuống = tiến/lùi (vx); trái/phải = đi ngang (vy) |
| Cần phải (thanh ngang) | Xoay trái/phải (wz) |
| Thanh **Tốc độ** + nút −/+ | Nhân vận tốc 0.2×–2.0× (đà mạnh hơn để vượt gờ) |
| Nút **RESET tư thế** | Dựng lại robot khi ngã |
| Thả chuột | Cần về giữa → robot dừng |

## Lưu ý

Policy này **train trên nền phẳng + mù** (obs không có địa hình). Gờ thấp thường vượt
được; **cầu thang gần như chắc ngã** — đó là test giới hạn, không phải lỗi. Muốn leo
địa hình thật cần policy train rough-terrain + perception (hướng khác).
