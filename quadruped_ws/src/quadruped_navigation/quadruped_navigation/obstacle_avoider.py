"""Ne vat can reactive bang LiDAR 2D (/scan) -> /cmd_vel (khong can train/hoc may).

Thuat toan VFH-lite: chia scan thanh quat truoc/trai/phai; di thang khi truoc
thoang, be lai ve phia THOANG hon khi gap vat can, xoay tai cho khi bi chan hoan
toan. Locomotion (gait) giu nguyen - chi nhan cmd_vel. Robot tu di quanh -> vua
ne vat can vua giup slam_toolbox dung ban do (/map).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class AvoidParams:
    cruise_vx: float = 0.22       # m/s - toc do tien khi thoang (khop gait on dinh)
    max_wz: float = 0.5           # rad/s - toc do xoay toi da
    clear_dist: float = 1.3       # m - truoc xa hon nay -> full speed
    stop_dist: float = 0.55       # m - truoc gan hon nay -> dung tien, xoay tai cho
    front_half: float = 0.5       # rad (~29 deg) - nua goc quat "truoc"
    side_lo: float = 0.35         # rad - quat ben tu goc nay...
    side_hi: float = 1.6          # rad - ...den goc nay


def _sector_min(ranges, angles, lo, hi, range_max):
    """Khoang cach nho nhat trong quat goc [lo,hi] (rad)."""
    mask = (angles >= lo) & (angles <= hi)
    if not np.any(mask):
        return range_max
    return float(np.min(ranges[mask]))


def compute_avoidance(ranges, angle_min, angle_increment, range_max, params=None):
    """Tra ve (vx, wz). ranges: list/array; inf/nan/0 -> range_max (thoang)."""
    p = params or AvoidParams()
    r = np.asarray(ranges, dtype=np.float64)
    r[~np.isfinite(r)] = range_max
    r[r <= 0.0] = range_max
    angles = angle_min + np.arange(len(r)) * angle_increment

    front = _sector_min(r, np.abs(angles), 0.0, p.front_half, range_max)
    left = _sector_min(r, angles, p.side_lo, p.side_hi, range_max)
    right = _sector_min(r, angles, -p.side_hi, -p.side_lo, range_max)

    # Huong xoay: ve phia THOANG hon (khoang cach lon hon)
    turn_sign = 1.0 if left >= right else -1.0   # +1 = xoay trai

    if front >= p.clear_dist:
        # Thoang: di thang, be nhe ve ben thoang hon de tranh tu tu
        vx = p.cruise_vx
        wz = 0.15 * turn_sign * (1.0 if abs(left - right) > 0.5 else 0.0)
    elif front > p.stop_dist:
        # Co vat can phia truoc: giam toc + be lai
        frac = (front - p.stop_dist) / (p.clear_dist - p.stop_dist)  # 0..1
        vx = p.cruise_vx * frac
        wz = p.max_wz * turn_sign * (1.0 - 0.5 * frac)
    else:
        # Bi chan sat: LUI NHE + xoay manh de thoat goc (tranh ket dao dong tai cho)
        vx = -0.08
        wz = p.max_wz * turn_sign

    wz = max(-p.max_wz, min(p.max_wz, wz))
    return vx, wz


if __name__ == '__main__':
    # Self-test (khong can ROS): kiem hanh vi tren cac scan gia lap.
    N = 360
    amin = -math.pi
    ainc = 2 * math.pi / N
    RMAX = 8.0
    ang = amin + np.arange(N) * ainc

    def scan_clear():
        return np.full(N, RMAX)

    # 1) Thoang het -> di thang (vx>0, wz~0)
    vx, wz = compute_avoidance(scan_clear(), amin, ainc, RMAX)
    assert vx > 0.15 and abs(wz) < 0.05, (vx, wz)
    print(f'thoang: vx={vx:.2f} wz={wz:.2f}')

    # 2) Tuong sat truoc mat -> dung tien, xoay (vx~0, |wz| lon)
    r = scan_clear(); r[np.abs(ang) < 0.5] = 0.4
    vx, wz = compute_avoidance(r, amin, ainc, RMAX)
    assert vx < 0.05 and abs(wz) > 0.3, (vx, wz)
    print(f'tuong truoc: vx={vx:.2f} wz={wz:.2f}')

    # 3) Vat can ben PHAI (goc am), trai thoang -> xoay TRAI (wz>0)
    r = scan_clear(); r[(ang < -0.35) & (ang > -1.6)] = 0.5; r[np.abs(ang) < 0.5] = 0.9
    vx, wz = compute_avoidance(r, amin, ainc, RMAX)
    assert wz > 0, f'phai bi chan -> phai xoay trai, wz={wz}'
    print(f'vat ben phai: vx={vx:.2f} wz={wz:.2f} (xoay trai)')

    # 4) Vat can ben TRAI -> xoay PHAI (wz<0)
    r = scan_clear(); r[(ang > 0.35) & (ang < 1.6)] = 0.5; r[np.abs(ang) < 0.5] = 0.9
    vx, wz = compute_avoidance(r, amin, ainc, RMAX)
    assert wz < 0, f'trai bi chan -> phai xoay phai, wz={wz}'
    print(f'vat ben trai: vx={vx:.2f} wz={wz:.2f} (xoay phai)')

    print('OK - obstacle_avoider self-test PASS')
