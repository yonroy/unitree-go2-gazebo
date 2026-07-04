"""Sinh me cung ngau nhien (recursive backtracker - moi ngo deu thong nhau) va
xuat ra go2_model/scene_maze.xml de cac app MuJoCo dung (them tham so 'maze').

Chay:  python3 gen_maze.py [N] [seed]     (mac dinh N=5 o, seed=7)
Robot bat dau o o giua (goc toa do), luon la o trong.
"""
import os
import sys
import random

HERE = os.path.dirname(os.path.abspath(__file__))
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5      # so o moi chieu
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 7
S = 1.5                                                # o rong 1.5m -> hanh lang ~1.4m
T, H = 0.1, 0.6                                        # day + cao tuong (LiDAR z=0.5 bat duoc)
random.seed(SEED)

x0 = -(N - 1) / 2 * S
mid = N // 2                                           # o giua -> tai goc toa do (N le)


def cx(c): return x0 + c * S
def cy(r): return x0 + r * S


# recursive backtracker: mo tuong giua cac o de tao me cung thong nhau
removed = set()
def key(a, b): return frozenset((a, b))
vis = {(mid, mid)}; stack = [(mid, mid)]
while stack:
    c, r = stack[-1]
    nb = [(c+dc, r+dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
          if 0 <= c+dc < N and 0 <= r+dr < N and (c+dc, r+dr) not in vis]
    if nb:
        n = random.choice(nb); removed.add(key((c, r), n)); vis.add(n); stack.append(n)
    else:
        stack.pop()

walls = []  # (px, py, sx, sy)
for c in range(N - 1):
    for r in range(N):
        if key((c, r), (c+1, r)) not in removed:
            walls.append((cx(c)+S/2, cy(r), T/2, S/2))
for c in range(N):
    for r in range(N - 1):
        if key((c, r), (c, r+1)) not in removed:
            walls.append((cx(c), cy(r)+S/2, S/2, T/2))
half = (N - 1) / 2 * S + S / 2
walls += [(0, half, half, T/2), (0, -half, half, T/2),
          (half, 0, T/2, half), (-half, 0, T/2, half)]

geoms = '\n'.join(
    f'    <geom name="mw{i}" type="box" group="3" material="wall" '
    f'size="{sx:.3f} {sy:.3f} {H/2}" pos="{px:.3f} {py:.3f} {H/2}"/>'
    for i, (px, py, sx, sy) in enumerate(walls))

xml = f'''<mujoco model="go2 maze">
  <include file="go2.xml"/>
  <statistic center="0 0 0.1" extent="0.8"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-30"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="wall" rgba="0.55 0.45 0.4 1"/>
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <!-- Me cung {N}x{N} o (hanh lang {S-T:.1f}m), recursive backtracker seed={SEED}. Robot bat dau o giua (0,0). -->
{geoms}
  </worldbody>
</mujoco>'''
out = os.path.join(HERE, 'go2_model', 'scene_maze.xml')
open(out, 'w').write(xml)
print(f'{out}: {len(walls)} tuong, me cung {N}x{N} o, hanh lang {S-T:.1f}m (seed {SEED})')
