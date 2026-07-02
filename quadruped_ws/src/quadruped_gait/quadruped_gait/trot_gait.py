"""Trot gait planner: cmd_vel -> quy dao ban chan 4 chan -> goc khop qua leg_ik.

Dang di trot: 2 cap chan cheo (FR+RL, FL+RR) luan phien swing/stance. Day la
baseline "gait co dinh" (§6 tai lieu): khong cam nhan luc cham dat, khong tai
can bang - chi dam bao dung hinh hoc + chu ky buoc theo cmd_vel.
"""
import math
from dataclasses import dataclass
from typing import Dict, Tuple

from .leg_ik import LegGeometry, inverse_kinematics

LEG_NAMES = ('FR', 'FL', 'RR', 'RL')

# hip_sign: +1 chan trai (FL, RL), -1 chan phai (FR, RR) - dung quy uoc trong
# leg_ik.py va khop voi mirror trong go2_description/xacro/robot.xacro.
HIP_SIGN = {'FR': -1.0, 'FL': 1.0, 'RR': -1.0, 'RL': 1.0}

# Cap cheo: FR+RL cung pha, FL+RR lech pha 0.5 chu ky (trot dieu chuan).
PHASE_OFFSET = {'FR': 0.0, 'RL': 0.0, 'FL': 0.5, 'RR': 0.5}

# front_hind_sign: +1 chan truoc (FR, FL), -1 chan sau (RR, RL) - khop voi
# front_hind trong go2_description/xacro/robot.xacro.
FRONT_HIND_SIGN = {'FR': 1.0, 'FL': 1.0, 'RR': -1.0, 'RL': -1.0}

# Offset hip so voi tam than (m) - xem leg_offset_x/y trong const.xacro.
LEG_OFFSET_X = 0.1934
LEG_OFFSET_Y = 0.0465

# Hinh hoc chan that cua Go2 (m) - xem quadruped_description/xacro/const.xacro.
GO2_LEG_GEOMETRY = LegGeometry(hip_length=0.0955, thigh_length=0.213, calf_length=0.213)


@dataclass
class GaitParams:
    stance_height: float = 0.30     # do cao than so voi hip khi dung (m)
    step_length_gain: float = 0.35  # m moi (m/s) linear_x -> bien do buoc doc x
    step_width_gain: float = 0.20   # m moi (m/s) linear_y hoac (rad/s) angular_z -> bien do buoc ngang y
    swing_height: float = 0.06      # do nang chan khi swing (m)
    cycle_period: float = 0.5       # giay / 1 chu ky buoc, o toc do danh nghia (~2 Hz)
    min_cycle_period: float = 0.28  # gioi han duoi (tan so toi da ~3.6 Hz), tranh rung/mat on dinh
    nominal_speed: float = 0.3      # m/s - toc do da kiem chung on dinh voi cycle_period mac dinh
    stance_duty: float = 0.5        # ty le thoi gian stance trong 1 chu ky
    hip_y_offset: float = 0.0955    # y mac dinh cua ban chan so voi hip (= hip_length,
    # ban chan thang duoi khop thigh, khong lech them sang ben)


def _swing_trajectory(phase: float, dx: float, dy: float, swing_height: float) -> Tuple[float, float]:
    """phase trong [0,1): vi tri x,z cua ban chan trong pha swing (nang chan).

    Dung nua hinh sin cho quy dao nang chan (mo phong cung parabol don gian),
    di chuyen tuyen tinh tu -dx/2 -> +dx/2 (tuong tu cho dy).
    """
    x = -dx / 2.0 + dx * phase
    y = -dy / 2.0 + dy * phase
    z = swing_height * math.sin(math.pi * phase)
    return x, y, z


def _stance_trajectory(phase: float, dx: float, dy: float) -> Tuple[float, float]:
    """phase trong [0,1): ban chan tiep dat, day nguoc huong di chuyen than."""
    x = dx / 2.0 - dx * phase
    y = dy / 2.0 - dy * phase
    return x, y


class TrotGait:
    def __init__(self, geom: LegGeometry = GO2_LEG_GEOMETRY, params: GaitParams = None):
        self.geom = geom
        self.params = params or GaitParams()
        self._phase = 0.0  # [0,1), tich luy truc tiep (khong phai t/cycle_period)
        # de doi tan buoc (cycle_period) khi toc do doi khong lam pha nhay cuc.

    def reset_time(self):
        self._phase = 0.0

    def standing_pose(self) -> Dict[str, Tuple[float, float, float]]:
        """Goc 12 khop khi dung yen (cmd_vel = 0), khong trot."""
        p = self.params
        angles = {}
        for name in LEG_NAMES:
            hip_sign = HIP_SIGN[name]
            target = (0.0, p.hip_y_offset * hip_sign, -p.stance_height)
            sol = inverse_kinematics(*target, self.geom, hip_sign)
            if sol is None:
                raise ValueError(f'Standing pose unreachable cho chan {name}: {target}')
            angles[name] = sol
        return angles

    def step(self, dt: float, vx: float, vy: float, wz: float) -> Dict[str, Tuple[float, float, float]]:
        """Tien mot buoc thoi gian dt (s), tra ve goc 12 khop.

        vx, vy: m/s (than toa do), wz: rad/s - dung tu /cmd_vel.
        Neu ca 3 gan 0 -> giu standing_pose (khong reset pha, de lan sau di lai
        muot, khong giat).
        """
        p = self.params
        moving = abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(wz) > 1e-3
        if not moving:
            return self.standing_pose()

        # Tan so buoc thich nghi theo toc do lenh: toc do cao hon -> chu ky
        # ngan hon (buoc nhanh hon), giu bien do buoc/thoi gian stance o muc
        # da kiem chung on dinh, thay vi buoc dai hon o cung nhip co dinh
        # (nguyen nhan gay giat/mat on dinh khi tang toc do - xem quadruped_teleop).
        speed_demand = math.hypot(vx, vy) + abs(wz) * LEG_OFFSET_X
        speed_ratio = speed_demand / p.nominal_speed if p.nominal_speed > 0 else 1.0
        cycle_period = p.cycle_period / max(speed_ratio, 1.0)
        cycle_period = max(cycle_period, p.min_cycle_period)

        self._phase = (self._phase + dt / cycle_period) % 1.0
        cycle_phase = self._phase

        angles = {}
        for name in LEG_NAMES:
            hip_sign = HIP_SIGN[name]
            front_hind = FRONT_HIND_SIGN[name]
            ry = LEG_OFFSET_Y * hip_sign
            rx = LEG_OFFSET_X * front_hind

            # Van toc than tai diem hip cua chan nay = v_body + wz x r_hip
            # (r_hip = (rx, ry)), quy doi sang bien do buoc (m) qua gain.
            dx = (vx - wz * ry) * p.step_length_gain
            dy = (vy + wz * rx) * p.step_width_gain

            leg_phase = (cycle_phase + PHASE_OFFSET[name]) % 1.0

            if leg_phase < p.stance_duty:
                stance_phase = leg_phase / p.stance_duty
                fx, fy = _stance_trajectory(stance_phase, dx, dy)
                fz = -p.stance_height
            else:
                swing_phase = (leg_phase - p.stance_duty) / (1.0 - p.stance_duty)
                fx, fy, dz = _swing_trajectory(swing_phase, dx, dy, p.swing_height)
                fz = -p.stance_height + dz

            target = (fx, p.hip_y_offset * hip_sign + fy, fz)
            sol = inverse_kinematics(*target, self.geom, hip_sign)
            if sol is None:
                # ngoai tam voi (hiem khi xay ra voi bien do nho) -> giu tu the dung
                sol = inverse_kinematics(0.0, p.hip_y_offset * hip_sign, -p.stance_height,
                                          self.geom, hip_sign)
            angles[name] = sol

        return angles


if __name__ == "__main__":
    # Self-test: chay vai giay mo phong (khong can ROS2), kiem tra khong loi
    # va goc khop nam trong gioi han vat ly cua Go2 (const.xacro).
    HIP_LIMIT = math.radians(60.0)      # +-1.0472 rad (~60 deg), xem const.xacro
    THIGH_LIMIT = (math.radians(-90.0), math.radians(200.0))
    CALF_LIMIT = (math.radians(-156.0), math.radians(-48.0))

    gait = TrotGait()
    dt = 0.02
    n_steps = int(2.0 / dt)

    print("Standing pose:")
    for name, (t1, t2, t3) in gait.standing_pose().items():
        print(f"  {name}: hip={math.degrees(t1):+7.2f} thigh={math.degrees(t2):+7.2f} calf={math.degrees(t3):+7.2f} deg")

    violations = 0
    for vx_test, wz_test in [(0.3, 0.2), (0.45, 0.75)]:  # (0.45,0.75) = max joystick (scale 1.5)
        gait.reset_time()
        for i in range(n_steps):
            angles = gait.step(dt, vx=vx_test, vy=0.0, wz=wz_test)
            for name, (t1, t2, t3) in angles.items():
                if not (-HIP_LIMIT <= t1 <= HIP_LIMIT):
                    print(f"[vx={vx_test},wz={wz_test}][{i}] {name} hip vuot gioi han: {math.degrees(t1):.2f} deg")
                    violations += 1
                if not (CALF_LIMIT[0] <= t3 <= CALF_LIMIT[1]):
                    print(f"[vx={vx_test},wz={wz_test}][{i}] {name} calf vuot gioi han: {math.degrees(t3):.2f} deg")
                    violations += 1
        print(f"vx={vx_test} wz={wz_test}: da chay {n_steps} buoc, vi pham gioi han: {violations}")

    assert violations == 0, "Co goc khop vuot gioi han vat ly cua Go2"
    print("OK - tat ca goc khop trong gioi han")
