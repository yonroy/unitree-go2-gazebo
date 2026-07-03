"""Logic thuan (khong phu thuoc ROS) cho track-object: tinh /cmd_vel tu vi tri
muc tieu TRONG KHUNG ROBOT (x-forward, y-left, da qua tf2 sang "trunk" boi
quadruped_perception/target_pose_node.py) + khoang cach muon giu (stop_distance).

Dung chung mau PID "khoang cach + goc -> cmd_vel" voi goto_point.py, khac o
2 diem: (1) muc tieu la 1 diem TRONG KHUNG ROBOT (khong can yaw de quy doi vi
da o khung tuong doi), (2) linear_x co the AM (lui lai neu qua gan, khong chi
tien toi 0 nhu goto_point).
"""
import math
from dataclasses import dataclass


@dataclass
class FollowParams:
    position_tolerance: float = 0.15                      # m - sai so quanh stop_distance coi la "dat"
    heading_tolerance_for_move: float = math.radians(30)   # lech qua nguong nay -> chi xoay tai cho
    max_linear: float = 0.3     # m/s - khop toc do da kiem chung on dinh (xem quadruped_gait)
    max_angular: float = 0.6    # rad/s
    kp_linear: float = 0.8
    kp_angular: float = 1.5


def compute_cmd(point_x: float, point_y: float, stop_distance: float, params: FollowParams = None):
    """Tra ve (linear_x, angular_z, distance, angle, in_range).

    point_x, point_y: vi tri muc tieu trong khung robot (vd "trunk").
    stop_distance: khoang cach muon giu voi muc tieu (m).
    """
    p = params or FollowParams()
    distance = math.hypot(point_x, point_y)
    angle = math.atan2(point_y, point_x)

    angular_z = max(-p.max_angular, min(p.max_angular, p.kp_angular * angle))
    if abs(angle) < p.heading_tolerance_for_move:
        linear_x = max(-p.max_linear, min(p.max_linear, p.kp_linear * (distance - stop_distance)))
    else:
        linear_x = 0.0  # lech huong qua nhieu -> xoay tai cho truoc, chua tien/lui

    in_range = abs(distance - stop_distance) < p.position_tolerance and abs(angle) < p.heading_tolerance_for_move
    return linear_x, angular_z, distance, angle, in_range


if __name__ == "__main__":
    # Self-test: mo phong dong hoc unicycle don gian cho CA robot lan muc tieu
    # (muc tieu di chuyen thang deu) - khac goto_point.py (muc tieu dung yen).
    # Vi diem dua vao compute_cmd phai o KHUNG ROBOT, moi buoc phai xoay vector
    # the-gioi (target - robot) ve khung robot bang -yaw truoc khi goi compute_cmd,
    # dung y het nhung gi target_pose_node.py lam qua tf2 trong he thong that.
    params = FollowParams()
    dt = 1.0 / 20.0
    stop_distance = 1.0

    test_cases = [
        # (robot_x0, y0, yaw0, target_x0, y0, target_vx, target_vy)
        (0.0, 0.0, 0.0, 3.0, 0.0, 0.1, 0.0),      # muc tieu ngay truoc mat, di xa cham
        (0.0, 0.0, 0.0, 2.0, 1.5, -0.05, 0.05),   # muc tieu cheo, dang tien lai gan
        (0.0, 0.0, math.pi, 3.0, 0.0, 0.0, 0.0),  # robot quay lung, muc tieu dung yen
    ]

    for (x0, y0, yaw0, tx0, ty0, tvx, tvy) in test_cases:
        x, y, yaw = x0, y0, yaw0
        tx, ty = tx0, ty0
        distances = []
        for _ in range(4000):
            dx = tx - x
            dy = ty - y
            point_x = dx * math.cos(yaw) + dy * math.sin(yaw)
            point_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

            linear_x, angular_z, distance, _angle, _in_range = compute_cmd(
                point_x, point_y, stop_distance, params
            )
            distances.append(distance)

            yaw += angular_z * dt
            x += linear_x * math.cos(yaw) * dt
            y += linear_x * math.sin(yaw) * dt
            tx += tvx * dt
            ty += tvy * dt

        final_errors = distances[-200:]
        avg_err = sum(abs(d - stop_distance) for d in final_errors) / len(final_errors)
        assert avg_err < params.position_tolerance * 3, (
            f"Khong bam duoc muc tieu tu ({tx0},{ty0}): sai so trung binh {avg_err}"
        )
        print(f"target0=({tx0},{ty0}) v=({tvx},{tvy}): sai so khoang cach TB 200 buoc cuoi = {avg_err:.4f}m")

    print("OK - tat ca test case bam duoc muc tieu (dung yen/di dong) trong pham vi sai so")
