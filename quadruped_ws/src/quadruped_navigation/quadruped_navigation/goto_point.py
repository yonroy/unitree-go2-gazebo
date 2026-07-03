"""Logic thuan (khong phu thuoc ROS) cho goto-point: tinh /cmd_vel tu vi tri
hien tai + muc tieu (x,y), dung PID don gian tren khoang cach + goc lech
huong - cung mo hinh voi §5 tai lieu (PID vi tri + khoang cach -> cmd_vel),
ap dung cho truong hop muc tieu la 1 toa do co dinh thay vi 1 doi tuong dang
di chuyen.
"""
import math
from dataclasses import dataclass


@dataclass
class GotoPointParams:
    position_tolerance: float = 0.15                      # m - coi la "toi noi"
    heading_tolerance_for_move: float = math.radians(30)   # lech qua nguong nay -> chi xoay tai cho
    max_linear: float = 0.3     # m/s - khop toc do da kiem chung on dinh (xem quadruped_gait)
    max_angular: float = 0.6    # rad/s
    kp_linear: float = 0.8
    kp_angular: float = 1.5


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_cmd(
    x: float, y: float, yaw: float,
    target_x: float, target_y: float,
    params: GotoPointParams = None,
):
    """Tra ve (linear_x, angular_z, distance, heading_error, reached).

    x, y, yaw: vi tri/huong hien tai cua robot (khung odom).
    target_x, target_y: toa do muc tieu (cung khung odom).
    """
    p = params or GotoPointParams()
    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    if distance < p.position_tolerance:
        return 0.0, 0.0, distance, 0.0, True

    desired_heading = math.atan2(dy, dx)
    heading_error = wrap_to_pi(desired_heading - yaw)

    angular_z = max(-p.max_angular, min(p.max_angular, p.kp_angular * heading_error))
    if abs(heading_error) < p.heading_tolerance_for_move:
        linear_x = max(-p.max_linear, min(p.max_linear, p.kp_linear * distance))
    else:
        linear_x = 0.0  # lech huong qua nhieu -> xoay tai cho truoc, chua tien

    return linear_x, angular_z, distance, heading_error, False


if __name__ == "__main__":
    # Self-test: mo phong dong hoc unicycle don gian (khong ma sat/truot, khong
    # can ROS/Gazebo) de kiem tra vong dieu khien thuc su HOI TU ve muc tieu.
    params = GotoPointParams()
    dt = 1.0 / 20.0

    test_cases = [
        (0.0, 0.0, 0.0, 2.0, 1.0),     # muc tieu cheo truoc-phai
        (0.0, 0.0, 0.0, -1.5, 0.5),    # muc tieu o phia sau -> phai quay dau
        (0.0, 0.0, math.pi, 1.0, 1.0),  # robot dang quay 180 do so voi muc tieu
    ]

    for (x0, y0, yaw0, tx, ty) in test_cases:
        x, y, yaw = x0, y0, yaw0
        reached_at = None
        for step in range(3000):
            linear_x, angular_z, distance, heading_error, reached = compute_cmd(
                x, y, yaw, tx, ty, params
            )
            if reached:
                reached_at = step
                break
            yaw += angular_z * dt
            x += linear_x * math.cos(yaw) * dt
            y += linear_x * math.sin(yaw) * dt

        assert reached_at is not None, f"Khong hoi tu ve ({tx},{ty}) tu ({x0},{y0},{yaw0})"
        final_distance = math.hypot(tx - x, ty - y)
        assert final_distance < params.position_tolerance * 1.5, (
            f"Sai so cuoi qua lon: {final_distance}"
        )
        print(f"start=({x0},{y0},yaw={math.degrees(yaw0):.0f}deg) target=({tx},{ty}): "
              f"toi sau {reached_at} buoc ({reached_at*dt:.2f}s), sai so cuoi={final_distance:.4f}m")

    print("OK - tat ca test case hoi tu ve muc tieu")
