"""Logic thuan (khong phu thuoc ROS) chay policy locomotion RL cho Go2.

Policy: diasAiMaster/unitree-go2-velocity-flat (HuggingFace, BSD-3), train bang
PPO/RSL-RL trong unitree_rl_mjlab (MuJoCo). ONNX: obs[45] -> actions[12].

Cac hang so lay TRUC TIEP tu params/deploy.yaml cua model (khong bia):
  - obs (45) theo dung thu tu:
        base_ang_vel(3) + projected_gravity(3) + velocity_commands(3)
      + joint_pos_rel(12) + joint_vel_rel(12) + last_action(12)   (tat ca scale 1.0)
  - action: target = action*0.5 + default_joint_pos
  - default_joint_pos (thu tu khop FR,FL,RR,RL - hip,thigh,calf):
        [-0.1,0.9,-1.8, 0.1,0.9,-1.8, -0.1,0.9,-1.8, 0.1,0.9,-1.8]
  - PD (dung khi actuate bang effort): Kp=[20,20,40]/chan, Kd=[1,1,2]/chan
  - step_dt = 0.02 s  (50 Hz)

Thu tu khop FR,FL,RR,RL TRUNG voi forward_position_controller trong
quadruped_description/config/go2_controllers.yaml -> khong can hoan vi.
"""
import os
import numpy as np

# --- Hang so tu deploy.yaml, theo thu tu SDK Unitree (FR,FL,RR,RL) ---
# Node doc/ghi khop theo thu tu nay (khop /joint_states theo ten + joint_effort_controller).
DEFAULT_JOINT_POS = np.array(
    [-0.1, 0.9, -1.8,  0.1, 0.9, -1.8,  -0.1, 0.9, -1.8,  0.1, 0.9, -1.8],
    dtype=np.float32,
)
ACTION_SCALE = 0.5
KP = np.array([20, 20, 40,  20, 20, 40,  20, 20, 40,  20, 20, 40], dtype=np.float32)
KD = np.array([1, 1, 2,  1, 1, 2,  1, 1, 2,  1, 1, 2], dtype=np.float32)
STEP_DT = 0.02
OBS_DIM = 45
ACT_DIM = 12

# joint_ids_map (deploy.yaml): anh xa giua thu tu POLICY (sim MJCF = FL,FR,RL,RR)
# va thu tu SDK (FR,FL,RR,RL). ONNX nhan obs / xuat action theo thu tu POLICY,
# nen phai remap khop tu SDK <-> POLICY. Day la involution (tu nghich) - hoan vi
# FR<->FL, RR<->RL. (DA KIEM CHUNG: khong remap thi robot lon nhao khi policy chay.)
JOINT_IDS_MAP = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])

# Ten khop theo thu tu SDK (FR,FL,RR,RL) = thu tu joint_effort_controller
JOINT_ORDER = [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
]

GRAVITY_W = np.array([0.0, 0.0, -1.0], dtype=np.float32)


def projected_gravity(quat_xyzw):
    """Vector trong luc [0,0,-1] cua world chieu ve khung base (dung IMU orientation).
    quat_xyzw: (x,y,z,w) orientation cua base trong world.
    Tra ve R_wb^T @ g_world (chinh la trong luc trong khung base)."""
    x, y, z, w = quat_xyzw
    # R_wb^T @ v  =  quay v boi quaternion nghich dao (conjugate)
    # cong thuc quay vector boi quaternion nghich (q* v q)
    gx, gy, gz = GRAVITY_W
    # v' = v + 2*w*(q_v x v) ... dung dang rotate boi conjugate:
    # Cach don gian: dung ma tran R_wb roi transpose.
    R = _quat_to_matrix(x, y, z, w)
    return (R.T @ GRAVITY_W).astype(np.float32)


def _quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-9:
        return np.eye(3, dtype=np.float32)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ], dtype=np.float32)


def build_obs(base_ang_vel, proj_gravity, velocity_cmd,
              joint_pos, joint_vel, last_action):
    """Dung vector observation 45 chieu.
    joint_pos, joint_vel theo thu tu SDK (JOINT_ORDER) -> remap sang thu tu POLICY.
    last_action da theo thu tu POLICY (nhu ONNX xuat ra)."""
    joint_pos_rel = np.asarray(joint_pos, np.float32) - DEFAULT_JOINT_POS
    obs = np.concatenate([
        np.asarray(base_ang_vel, np.float32),        # 3
        np.asarray(proj_gravity, np.float32),        # 3
        np.asarray(velocity_cmd, np.float32),        # 3
        joint_pos_rel[JOINT_IDS_MAP],                # 12  SDK -> POLICY
        np.asarray(joint_vel, np.float32)[JOINT_IDS_MAP],  # 12  SDK -> POLICY
        np.asarray(last_action, np.float32),         # 12  (da POLICY order)
    ]).astype(np.float32)
    return obs


def action_to_target(action_policy):
    """action ONNX (thu tu POLICY) -> goc khop muc tieu (rad, thu tu SDK).
    action_sdk = action_policy[map] (map la involution), roi *scale + default."""
    action_sdk = np.asarray(action_policy, np.float32)[JOINT_IDS_MAP]
    return action_sdk * ACTION_SCALE + DEFAULT_JOINT_POS


def pd_torque(target_pos, joint_pos, joint_vel):
    """PD giong deploy that: tau = Kp*(target - q) - Kd*qd (dung khi effort interface)."""
    q = np.asarray(joint_pos, np.float32)
    qd = np.asarray(joint_vel, np.float32)
    return (KP * (np.asarray(target_pos, np.float32) - q) - KD * qd).astype(np.float32)


class PolicyRunner:
    """Boc ONNX session + giu last_action giua cac buoc."""

    def __init__(self, onnx_path):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        in_shape = self.sess.get_inputs()[0].shape
        out_shape = self.sess.get_outputs()[0].shape
        assert in_shape[-1] == OBS_DIM, f'ONNX obs dim {in_shape} != {OBS_DIM}'
        assert out_shape[-1] == ACT_DIM, f'ONNX act dim {out_shape} != {ACT_DIM}'
        self.last_action = np.zeros(ACT_DIM, np.float32)

    def infer(self, base_ang_vel, proj_gravity, velocity_cmd, joint_pos, joint_vel):
        obs = build_obs(base_ang_vel, proj_gravity, velocity_cmd,
                        joint_pos, joint_vel, self.last_action)
        action = self.sess.run(None, {self.input_name: obs[None, :]})[0].flatten()
        self.last_action = action.astype(np.float32)
        return action, action_to_target(action)

    def reset(self):
        self.last_action = np.zeros(ACT_DIM, np.float32)


if __name__ == '__main__':
    # Self-test (khong can ROS): kiem cong thuc + nap ONNX + inference on dinh.
    # 1) projected_gravity: robot dung thang (quat = identity) -> [0,0,-1]
    g = projected_gravity((0.0, 0.0, 0.0, 1.0))
    assert np.allclose(g, [0, 0, -1], atol=1e-6), g
    # nghieng 90 deg quanh Y (pitch) -> truc x cua base huong xuong => g_base ~ [1,0,0]? kiem dau
    import math
    qy = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # pitch +90
    g2 = projected_gravity(qy)
    assert abs(np.linalg.norm(g2) - 1.0) < 1e-5
    print('projected_gravity thang =', np.round(g, 3), ' pitch90 =', np.round(g2, 3))

    # 2) joint_ids_map la involution (remap 2 lan = goc)
    assert np.array_equal(JOINT_IDS_MAP[JOINT_IDS_MAP], np.arange(12)), 'map khong phai involution'
    # remap dua khop SDK sang POLICY: FR(sdk 0-2) -> policy slot 3-5
    probe = np.arange(12)
    assert list(probe[JOINT_IDS_MAP][3:6]) == [0, 1, 2], 'FR phai o policy slot 3-5'
    assert list(probe[JOINT_IDS_MAP][0:3]) == [3, 4, 5], 'FL phai o policy slot 0-2'
    print('joint_ids_map OK (involution, FR<->FL RR<->RL)')

    # 3) build_obs dung do dai + dung vi tri cac khoi
    obs = build_obs([0, 0, 0], g, [0.5, 0, 0],
                    DEFAULT_JOINT_POS, np.zeros(12), np.zeros(12))
    assert obs.shape == (OBS_DIM,), obs.shape
    assert np.allclose(obs[6:9], [0.5, 0, 0])           # velocity_cmd
    assert np.allclose(obs[9:21], 0.0)                  # joint_pos_rel = 0 (q=default)
    print('build_obs OK, shape', obs.shape)

    # 4) action_to_target: action=0 -> target = default (moi thu tu deu 0)
    assert np.allclose(action_to_target(np.zeros(12)), DEFAULT_JOINT_POS)
    assert np.allclose(pd_torque(DEFAULT_JOINT_POS, DEFAULT_JOINT_POS, np.zeros(12)), 0.0)
    print('action_to_target + pd_torque OK')

    # 4) Nap ONNX that + inference vai buoc, kiem huu han + on dinh
    onnx_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'policy.onnx')
    onnx_path = os.path.abspath(onnx_path)
    if os.path.exists(onnx_path):
        runner = PolicyRunner(onnx_path)
        for i in range(5):
            act, tgt = runner.infer([0, 0, 0], g, [0.5, 0, 0],
                                    DEFAULT_JOINT_POS, np.zeros(12))
            assert np.all(np.isfinite(act)) and act.shape == (ACT_DIM,)
        print('ONNX inference OK: action[0..2] =', np.round(act[:3], 3),
              ' target[0..2] =', np.round(tgt[:3], 3))
        print('OK - policy_runner self-test PASS')
    else:
        print('(bo qua test ONNX: khong thay', onnx_path, ')')
