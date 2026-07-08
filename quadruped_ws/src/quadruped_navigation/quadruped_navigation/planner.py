"""Lap duong (A*) tren luoi occupancy + bam duong (pure pursuit) -> cmd_vel.

Logic THUAN (chi numpy), khong ROS/MuJoCo, co self-test. Dung cho ca app MuJoCo
(mujoco_demo) lan node Gazebo. Cho robot mot DICH -> tu tim duong ne vat can toi noi.
"""
import heapq
import math

import numpy as np

# 8 huong di chuyen (dx, dy, cost)
_NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, 1.41421356), (1, -1, 1.41421356),
              (-1, 1, 1.41421356), (1, 1, 1.41421356)]


def inflate(blocked, radius_cells):
    """Phinh vat can them radius_cells o (an toan cho ban kinh robot).
    blocked: 2D bool (True=vat can). Tra ve bool grid da phinh."""
    b = np.asarray(blocked, dtype=bool)
    if radius_cells <= 0:
        return b.copy()
    out = b.copy()
    ys, xs = np.where(b)
    r = int(math.ceil(radius_cells))
    H, W = b.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            ny, nx = ys + dy, xs + dx
            m = (ny >= 0) & (ny < H) & (nx >= 0) & (nx < W)
            out[ny[m], nx[m]] = True
    return out


def astar(blocked, start, goal):
    """A* 8-huong tren luoi. blocked: 2D bool (True=chan). start/goal: (x,y) cell.
    Tra ve list [(x,y),...] tu start den goal, hoac None neu khong co duong."""
    b = np.asarray(blocked, dtype=bool)
    H, W = b.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
        return None
    if b[gy, gx]:
        return None  # dich nam trong vat can

    def h(x, y):
        return math.hypot(x - gx, y - gy)

    openq = [(h(sx, sy), 0.0, sx, sy)]
    came = {}
    gcost = {(sx, sy): 0.0}
    seen = set()
    while openq:
        _, g, x, y = heapq.heappop(openq)
        if (x, y) in seen:
            continue
        seen.add((x, y))
        if (x, y) == (gx, gy):
            path = [(x, y)]
            while (x, y) in came:
                x, y = came[(x, y)]
                path.append((x, y))
            return path[::-1]
        for dx, dy, c in _NEIGHBORS:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H) or b[ny, nx]:
                continue
            ng = g + c
            if ng < gcost.get((nx, ny), 1e18):
                gcost[(nx, ny)] = ng
                came[(nx, ny)] = (x, y)
                heapq.heappush(openq, (ng + h(nx, ny), ng, nx, ny))
    return None


def pure_pursuit(path_world, rx, ry, ryaw, lookahead=0.8, cruise=0.35,
                 max_wz=0.6, goal_tol=0.3, max_vy=0.6):
    """Bam duong kieu HOLONOMIC (robot 4 chan di ngang duoc): di thang toi diem
    lookahead bang (vx,vy) trong khung robot + xoay nhe de huong dan theo.
    path_world: list [(wx,wy),...] the gioi. Tra ve (vx, vy, wz, reached)."""
    if not path_world:
        return 0.0, 0.0, 0.0, True
    gx, gy = path_world[-1]
    if math.hypot(gx - rx, gy - ry) < goal_tol:
        return 0.0, 0.0, 0.0, True

    target = path_world[-1]
    for wx, wy in path_world:
        if math.hypot(wx - rx, wy - ry) >= lookahead:
            target = (wx, wy)
            break

    dx, dy = target[0] - rx, target[1] - ry
    # quay vector (the gioi) ve khung robot: fx=truoc, fy=trai
    fx = dx * math.cos(ryaw) + dy * math.sin(ryaw)
    fy = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
    norm = math.hypot(fx, fy)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, False
    vx = cruise * fx / norm
    vy = max(-max_vy, min(max_vy, cruise * fy / norm))
    # xoay nhe de dan huong ve phia di chuyen (giam strafe lau dai)
    move_ang = math.atan2(fy, fx)
    wz = max(-max_wz, min(max_wz, 0.8 * move_ang))
    return vx, vy, wz, False


if __name__ == '__main__':
    # Self-test (khong can ROS/MuJoCo)
    # 1) inflate: 1 o -> lan ra ban kinh
    g = np.zeros((7, 7), bool); g[3, 3] = True
    gi = inflate(g, 1.5)
    assert gi[3, 3] and gi[3, 4] and gi[4, 3] and not gi[0, 0]
    print('inflate OK:', int(gi.sum()), 'o bi chan')

    # 2) A*: tuong doc giua, chua 1 khe -> tim duong vong qua
    grid = np.zeros((11, 11), bool)
    grid[1:10, 5] = True; grid[5, 5] = False  # tuong co khe o giua
    path = astar(grid, (1, 5), (9, 5))
    assert path is not None and path[0] == (1, 5) and path[-1] == (9, 5)
    assert all(not grid[y, x] for x, y in path), 'duong di qua vat can!'
    print(f'A* OK: duong {len(path)} o, ne tuong qua khe')

    # 3) A*: tuong ngang co khe -> co duong; chan het -> None
    grid2 = np.zeros((5, 5), bool); grid2[2, :] = True; grid2[2, 4] = False  # khe o bien
    assert astar(grid2, (0, 0), (0, 4)) is not None
    grid3 = np.zeros((5, 5), bool); grid3[2, :] = True  # chan het
    assert astar(grid3, (0, 0), (0, 4)) is None
    print('A* co khe -> co duong; chan het -> None OK')

    # 4) pure_pursuit holonomic: duong thang truoc mat -> vx>0, vy~0
    pp = [(0, 0), (1, 0), (2, 0)]
    vx, vy, wz, done = pure_pursuit(pp, 0, 0, 0.0)
    assert vx > 0.15 and abs(vy) < 0.1 and not done, (vx, vy, wz)
    # dich ben TRAI robot (robot huong +x, dich o +y) -> vy>0 (di ngang trai)
    vx, vy, wz, done = pure_pursuit([(0, 0), (0, 2)], 0, 0, 0.0)
    assert vy > 0.15 and wz > 0, f'dich ben trai -> strafe trai + xoay trai, vy={vy} wz={wz}'
    # toi dich -> reached
    *_, done = pure_pursuit([(0, 0), (0.1, 0)], 0.1, 0, 0.0)
    assert done
    print('pure_pursuit holonomic OK (thang / ngang trai / toi dich)')

    print('OK - planner self-test PASS')
