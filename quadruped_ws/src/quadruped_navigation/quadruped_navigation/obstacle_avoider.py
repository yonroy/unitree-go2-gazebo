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
    escape_vx: float = -0.08      # m/s - lui khi bi chan sat (goc cut). Lui manh hon
    # giup thoat goc cut trong me cung (tranh ket + lao dao), xem regime "bi chan".


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
        # Bi chan sat: LUI + xoay manh de thoat goc (tranh ket dao dong tai cho)
        vx = p.escape_vx
        wz = p.max_wz * turn_sign

    wz = max(-p.max_wz, min(p.max_wz, wz))
    return vx, wz


class SlewLimiter:
    """Gioi han toc do doi lenh (slew-rate) de lam muot chuyen dong.

    `compute_avoidance` la reactive, doi lenh dot ngot khi chuyen regime (vd cua o
    goc me cung: wz nhay ~0.6 rad/s trong 1 tick -> robot giat). Limiter nay ep lenh
    thay doi toi da `max_delta` moi tick nen wz/vx tang/giam dan -> cua muot. Trang
    thai (lenh truoc) giu trong doi tuong; goi `step(cmd)` moi tick.
    """

    def __init__(self, max_delta):
        # max_delta: scalar hoac vector (thay doi toi da cho phep moi tick, moi phan tu)
        self.max_delta = np.asarray(max_delta, dtype=np.float64)
        self.prev = None

    def step(self, cmd):
        cmd = np.asarray(cmd, dtype=np.float64)
        if self.prev is None:
            self.prev = cmd.copy()
        else:
            delta = np.clip(cmd - self.prev, -self.max_delta, self.max_delta)
            self.prev = self.prev + delta
        return self.prev.copy()

    def reset(self):
        self.prev = None


class StuckEscape:
    """Phat hien robot bi KET (khong tien duoc) va ra dong tac thoat CAM KET.

    Ne reactive khong nho duong -> ket o ngo cut me cung: lui ra 1 chut thi front
    mo, lai lao vao, dao dong/ti tuong/lao dao tai cho (DA DO bang App that: vi tri
    dong bang, cmd ket o (-0.08,-0.75) suot). Class nay theo doi vi tri: neu di
    chuyen < `move_thresh` trong `stuck_time` giay -> vao che do ESCAPE trong
    `escape_time` giay: LUI + xoay 1 chieu CO DINH (khong doi dau, tranh lai lao vao)
    de lui han ra + quay dau, roi tra dieu khien lai cho reactive.
    """

    def __init__(self, move_thresh=0.18, stuck_time=2.5, escape_time=2.2,
                 escape_vx=-0.22, escape_wz=0.7):
        self.move_thresh = move_thresh
        self.stuck_time = stuck_time
        self.escape_time = escape_time
        self.escape_vx = escape_vx
        self.escape_wz = escape_wz
        self.reset()

    def reset(self):
        self.ref = None       # vi tri moc gan nhat con tien duoc
        self.ref_t = 0.0
        self.escape_end = -1e9
        self.dir = 1.0        # chieu xoay cam ket khi thoat (+1 = trai)
        self.wedge = None     # vi tri bat dau ket - giu chieu xoay cho toi khi roi xa

    def step(self, t, pos_xy, vx, wz, turn_hint=1.0):
        """t: thoi gian (s); pos_xy: (x,y); vx,wz: lenh reactive; turn_hint: chieu
        xoay uu tien (>0 trai). Tra (vx,wz) - ghi de khi dang/bat dau thoat ket."""
        pos = np.asarray(pos_xy, dtype=np.float64)
        # Roi xa han cho ket -> ket thuc dot thoat, cho phep chon lai chieu lan sau
        if self.wedge is not None and np.hypot(*(pos - self.wedge)) > 0.6:
            self.wedge = None
        if t < self.escape_end:                       # dang trong dong tac thoat
            return self.escape_vx, self.dir * self.escape_wz
        if self.ref is None or np.hypot(*(pos - self.ref)) > self.move_thresh:
            self.ref = pos.copy(); self.ref_t = t     # con tien -> cap nhat moc
            return vx, wz
        if t - self.ref_t > self.stuck_time:          # ket qua lau -> bat dau thoat
            if self.wedge is None:                    # dot ket MOI -> chon chieu, ghi moc ket
                self.wedge = pos.copy()
                self.dir = 1.0 if turn_hint >= 0 else -1.0
            # neu van gan wedge cu (chained) -> GIU nguyen dir de quay du 1 vong thoat ra
            self.escape_end = t + self.escape_time
            self.ref = pos.copy(); self.ref_t = t
            return self.escape_vx, self.dir * self.escape_wz
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

    # 5) SlewLimiter: lenh nhay tu 0 -> [0,-0.75] bi ep tang dan max_delta moi tick,
    #    va sau du tick phai hoi tu ve dung lenh dich.
    sl = SlewLimiter([0.12, 0.12, 0.15])
    out0 = sl.step([0.0, 0.0, 0.0])
    assert np.allclose(out0, 0.0)
    out1 = sl.step([0.55, 0.0, -0.75])       # lenh nhay lon
    assert abs(out1[0] - 0.12) < 1e-9 and abs(out1[2] - (-0.15)) < 1e-9, out1
    # sau nhieu tick giu nguyen lenh dich -> hoi tu
    for _ in range(20):
        out = sl.step([0.55, 0.0, -0.75])
    assert np.allclose(out, [0.55, 0.0, -0.75]), out
    # moi buoc khong doi qua max_delta
    sl.reset(); prev = sl.step([0, 0, 0])
    for tgt in [[0.5, 0, -0.7], [-0.3, 0, 0.6], [0.2, 0, 0.0]]:
        for _ in range(30):
            cur = sl.step(tgt)
            assert np.all(np.abs(cur - prev) <= np.array([0.12, 0.12, 0.15]) + 1e-9)
            prev = cur
    print('SlewLimiter OK (ep slew-rate, hoi tu dung lenh dich)')

    # 6) StuckEscape: robot dung yen (khong tien) -> sau stuck_time phai vao ESCAPE
    #    (ghi de vx<0 + xoay cam ket), va giu suot escape_time.
    se = StuckEscape(move_thresh=0.18, stuck_time=2.5, escape_time=2.2,
                     escape_vx=-0.22, escape_wz=0.7)
    p = (5.0, 5.0)  # dung yen 1 cho
    t = 0.0; dt = 0.05; escaped = False
    while t < 2.4:  # chua du stuck_time -> van la lenh reactive
        vxo, wzo = se.step(t, p, 0.3, 0.1, turn_hint=1.0); t += dt
    assert (vxo, wzo) == (0.3, 0.1), (vxo, wzo)
    vxo, wzo = se.step(t + 0.2, p, 0.3, 0.1, turn_hint=1.0)  # da qua stuck_time
    assert vxo < 0 and wzo > 0, (vxo, wzo)   # dang thoat: lui + xoay
    escaped = True
    # trong escape_time van giu lenh thoat du reactive doi
    vxo2, wzo2 = se.step(t + 0.5, p, 0.5, -0.4, turn_hint=-1.0)
    assert vxo2 < 0 and wzo2 > 0, (vxo2, wzo2)
    # robot di chuyen ro -> reset moc, khong con ket
    se2 = StuckEscape()
    for k in range(60):
        vxo, wzo = se2.step(k * 0.05, (k * 0.05, 0.0), 0.3, 0.0)  # di deu 1 m/s
    assert (vxo, wzo) == (0.3, 0.0), (vxo, wzo)
    assert escaped
    print('StuckEscape OK (phat hien ket -> dong tac thoat cam ket)')

    print('OK - obstacle_avoider self-test PASS')
