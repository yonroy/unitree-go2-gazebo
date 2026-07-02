"""Giai tich IK/FK cho 1 chan robot 4 chan (hip abduction + thigh + knee).

Quy uoc he toa do gan tai khop hip/abduction cua moi chan:
  x: huong ra truoc than robot
  y: huong sang trai than robot
  z: huong len tren (z am = ban chan o duoi than)

  Link 1 (hip_length L1):   khop hip xoay quanh truc X (abduction/adduction),
                             dich chuyen khop thigh ra theo truc Y.
  Link 2 (thigh_length L2): khop thigh xoay quanh truc pitch cuc bo (song song Y
                             sau khi da xoay theo hip).
  Link 3 (calf_length L3):  khop calf (dau goi) noi tiep, cung truc pitch.

theta3 (calf) quy uoc: 0 = chan duoi thang, am = dau goi gap ve sau -
dung voi thiet ke khop goi that cua Go2 (chi gap mot chieu).

Chieu dai link mac dinh lay tu hinh hoc that cua Unitree Go2, xem
quadruped_description/urdf/go2.urdf.xacro (khong bia so).
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class LegGeometry:
    hip_length: float    # L1: chieu dai link abduction (offset hip -> thigh joint)
    thigh_length: float  # L2
    calf_length: float   # L3


def inverse_kinematics(
    x: float, y: float, z: float, geom: LegGeometry, hip_sign: float
) -> Optional[Tuple[float, float, float]]:
    """Tinh (theta_hip, theta_thigh, theta_calf) de ban chan toi (x,y,z)
    trong he toa do gan tai khop hip cua chan do.

    hip_sign: +1 cho chan ben trai, -1 cho chan ben phai (dau cua offset L1
    theo truc Y vi hai ben chan doi xung guong qua mat phang XZ cua than).

    Tra ve None neu diem ngoai tam voi (unreachable).
    """
    l1 = geom.hip_length * hip_sign
    l2 = geom.thigh_length
    l3 = geom.calf_length

    # --- Khop hip (abduction), giai trong mat phang Y-Z ---
    yz_sq = y * y + z * z - l1 * l1
    if yz_sq < 0:
        return None
    b = math.sqrt(yz_sq)  # khoang cach hinh chieu con lai sau khi tru offset l1
    theta1 = math.atan2(z, y) + math.atan2(b, l1)

    # --- Khop thigh + calf: IK phang 2R trong mat phang (a, b) ---
    # a = -x: truc quay thigh/calf la +Y (right-hand rule) -> goc duong day
    # ban chan ve -X, nguoc voi quy uoc "a doc +X" dung de giai 2R o duoi.
    a = -x
    r_sq = a * a + b * b
    r = math.sqrt(r_sq)
    if r > (l2 + l3) or r < abs(l2 - l3):
        return None  # ngoai tam voi

    cos_theta3 = (r_sq - l2 * l2 - l3 * l3) / (2 * l2 * l3)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3 = -math.acos(cos_theta3)          # 0 khi duoi thang, am khi gap ve sau

    alpha = math.atan2(l3 * math.sin(theta3), l2 + l3 * math.cos(theta3))
    theta2 = math.atan2(a, b) - alpha

    return theta1, theta2, theta3


def forward_kinematics(
    theta1: float, theta2: float, theta3: float, geom: LegGeometry, hip_sign: float
) -> Tuple[float, float, float]:
    """FK — dung de tu kiem tra IK bang round-trip trong self-test."""
    l1 = geom.hip_length * hip_sign
    l2 = geom.thigh_length
    l3 = geom.calf_length

    a = l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)
    b = l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = -a
    y = l1 * math.cos(theta1) + b * math.sin(theta1)
    z = l1 * math.sin(theta1) - b * math.cos(theta1)
    return x, y, z


if __name__ == "__main__":
    # Self-test: IK -> FK round-trip tren mot so diem trong tam voi, dung so
    # do hinh hoc that cua Go2 (xem §9 tai lieu / thong so Unitree Go2).
    geom = LegGeometry(hip_length=0.0955, thigh_length=0.213, calf_length=0.213)

    test_targets = [
        (0.0, 0.0955, -0.30),
        (0.05, 0.0955, -0.28),
        (-0.05, 0.0955, -0.32),
        (0.0, 0.0955, -0.40),
        (0.08, 0.0955, -0.25),
    ]

    max_err = 0.0
    for (tx, ty, tz) in test_targets:
        sol = inverse_kinematics(tx, ty, tz, geom, hip_sign=+1)
        assert sol is not None, f"Target {(tx, ty, tz)} bao unreachable nhung phai reachable"
        t1, t2, t3 = sol
        fx, fy, fz = forward_kinematics(t1, t2, t3, geom, hip_sign=+1)
        err = math.dist((tx, ty, tz), (fx, fy, fz))
        max_err = max(max_err, err)
        print(f"target={tx:+.3f},{ty:+.3f},{tz:+.3f}  "
              f"theta=({math.degrees(t1):+7.2f},{math.degrees(t2):+7.2f},{math.degrees(t3):+7.2f}) deg  "
              f"fk={fx:+.4f},{fy:+.4f},{fz:+.4f}  err={err:.6f}")

    assert max_err < 1e-6, f"IK/FK round-trip sai so qua lon: {max_err}"
    print(f"OK - max round-trip error = {max_err:.2e} m")
