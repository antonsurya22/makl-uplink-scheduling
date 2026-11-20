#!/usr/bin/env python3
"""
worldgen.py -- procedural Gazebo Classic worlds and matching Nav2 occupancy maps
for the network_aware_nav experiments.

Generates three kinds of environment from one code path, so that every map in the
study is produced by the same documented procedure:

  1. train_XX       : procedurally varied corridor networks, 6x5 m .. 20x15 m.
                      Used for TRAINING only.
  2. eval_building  : the large held-out evaluation map (default 20x15 m).
                      Never used for training or for baseline tuning.
  3. arena_lab      : an SDF replica of the physical laboratory arena
                      (2.73 x 2.37 m, S-course with interior divider).
                      Held out spatially; used for sim-to-real comparison.

For each environment it writes:
    <name>.world      Gazebo Classic SDF
    <name>.pgm        occupancy grid for Nav2 map_server
    <name>.yaml       map metadata for Nav2
    <name>.png        human-readable preview (start green, goal red)
    <name>.json       start/goal poses, obstacles, seed, geodesic path length

Every map is validated: start and goal are guaranteed connected through free
space after robot-radius inflation, so no generated map can silently be
unsolvable.

Usage:
    python3 worldgen.py --out ./worlds
    python3 worldgen.py --out ./worlds --n-train 16 --seed 7
    python3 worldgen.py --out ./worlds --only eval_building

Requires: numpy, pillow
"""

import argparse
import json
import math
import os
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

# ----------------------------------------------------------------------------
# Robot / map constants. Change these if you move off the TurtleBot3 Burger.
# ----------------------------------------------------------------------------
ROBOT_RADIUS = 0.14          # m, Burger footprint radius (+ small margin)

# B2 safety-aware static-route validation.
# Frozen A3 hard guard: d_hard = 0.31 m.
# This value is center-to-static-surface clearance;
# it is NOT added to ROBOT_RADIUS.
SAFETY_PATH_CLEARANCE = 0.45
MIN_CORRIDOR_W = 1.10
MAP_RES = 0.05               # m per pixel, standard for Nav2
WALL_T = 0.10                # m, wall thickness
WALL_H = 0.80                # m, wall height (well above the LDS-01 plane)
OBST_H = 0.50                # m, obstacle height
DOOR_W = 1.10                # m, safety-feasible doorway gap width


# ----------------------------------------------------------------------------
# Geometry primitives
# ----------------------------------------------------------------------------
class Box:
    """Axis-aligned box, centre (cx, cy), full size (sx, sy)."""

    __slots__ = ("cx", "cy", "sx", "sy", "h")

    def __init__(self, cx, cy, sx, sy, h=WALL_H):
        self.cx, self.cy, self.sx, self.sy, self.h = cx, cy, sx, sy, h

    def as_tuple(self):
        return (self.cx, self.cy, self.sx, self.sy, self.h)


def boundary_walls(W, H, t=WALL_T):
    """Four outer walls for a world spanning [-W/2, W/2] x [-H/2, H/2]."""
    hw, hh = W / 2.0, H / 2.0
    return [
        Box(0.0, hh + t / 2, W + 2 * t, t),
        Box(0.0, -hh - t / 2, W + 2 * t, t),
        Box(-hw - t / 2, 0.0, t, H + 2 * t),
        Box(hw + t / 2, 0.0, t, H + 2 * t),
    ]


# ----------------------------------------------------------------------------
# Layout: grid-of-blocks corridor network
# ----------------------------------------------------------------------------
def corridor_network(W, H, nx, ny, corridor_w, rng,
                     block_dropout=0.25, doorway_prob=0.35):
    """
    Build a corridor network by placing solid rectangular blocks on an nx x ny
    grid; the gaps between blocks are the corridors. Some blocks are dropped to
    create open areas, and some corridors get a partial wall with a doorway gap
    so that the map contains narrow passages where clearance is genuinely tight.
    """
    walls = boundary_walls(W, H)

    bw = (W - (nx + 1) * corridor_w) / nx
    bh = (H - (ny + 1) * corridor_w) / ny
    if bw <= 0.4 or bh <= 0.4:
        return None  # geometry infeasible for this (nx, ny, corridor_w)

    x0 = -W / 2.0
    y0 = -H / 2.0

    kept = []
    for i in range(nx):
        for j in range(ny):
            if rng.random() < block_dropout:
                continue
            cx = x0 + corridor_w + i * (bw + corridor_w) + bw / 2.0
            cy = y0 + corridor_w + j * (bh + corridor_w) + bh / 2.0
            walls.append(Box(cx, cy, bw, bh))
            kept.append((i, j))

    kept_set = set(kept)

    def anchored(a, b):
        """True if the doorway would attach to at least one retained block."""
        return (a in kept_set) or (b in kept_set)

    # Doorways across vertical corridors (travel along y).
    for i in range(nx + 1):
        cx = x0 + corridor_w / 2.0 + i * (bw + corridor_w)
        for j in range(ny):
            if rng.random() > doorway_prob:
                continue
            if not anchored((i - 1, j), (i, j)):
                continue
            cy = y0 + corridor_w + j * (bh + corridor_w) + bh * rng.uniform(0.25, 0.75)
            gap_lo = cx - corridor_w / 2.0 + rng.uniform(0.0, max(0.0, corridor_w - DOOR_W))
            _add_split_wall(walls, cx, cy, corridor_w, gap_lo, axis="x")

    # Doorways across horizontal corridors (travel along x).
    for j in range(ny + 1):
        cy = y0 + corridor_w / 2.0 + j * (bh + corridor_w)
        for i in range(nx):
            if rng.random() > doorway_prob:
                continue
            if not anchored((i, j - 1), (i, j)):
                continue
            cx = x0 + corridor_w + i * (bw + corridor_w) + bw * rng.uniform(0.25, 0.75)
            gap_lo = cy - corridor_w / 2.0 + rng.uniform(0.0, max(0.0, corridor_w - DOOR_W))
            _add_split_wall(walls, cx, cy, corridor_w, gap_lo, axis="y")

    return walls


def _add_split_wall(walls, cx, cy, span, gap_lo, axis):
    """Wall of length `span` across a corridor, with a DOOR_W gap starting at gap_lo."""
    gap_hi = gap_lo + DOOR_W
    if axis == "x":
        lo, hi = cx - span / 2.0, cx + span / 2.0
        if gap_lo - lo > 0.05:
            walls.append(Box((lo + gap_lo) / 2.0, cy, gap_lo - lo, WALL_T))
        if hi - gap_hi > 0.05:
            walls.append(Box((gap_hi + hi) / 2.0, cy, hi - gap_hi, WALL_T))
    else:
        lo, hi = cy - span / 2.0, cy + span / 2.0
        if gap_lo - lo > 0.05:
            walls.append(Box(cx, (lo + gap_lo) / 2.0, WALL_T, gap_lo - lo))
        if hi - gap_hi > 0.05:
            walls.append(Box(cx, (gap_hi + hi) / 2.0, WALL_T, hi - gap_hi))


def lab_arena():
    """
    Replica of the physical laboratory arena: 2.73 x 2.37 m, S-shaped course
    formed by two offset interior dividers. Dimensions match the hardware setup.
    """
    W, H = 2.73, 2.37
    walls = boundary_walls(W, H)
    # Lower divider: shifted downward to preserve clearance
    # required by the frozen A3 hard-safety envelope.
    walls.append(Box(-0.41, -0.41, 1.91, WALL_T))

    # Upper divider: shifted upward symmetrically.
    walls.append(Box(0.41, 0.41, 1.91, WALL_T))

    start = (-1.05, -0.92)
    goal = (1.05, 0.92)

    # No static obstacle in the narrow traversed corridor.
    # Dynamic pedestrian interaction is handled separately.
    obstacles = []
    return W, H, walls, obstacles, start, goal


# ----------------------------------------------------------------------------
# Rasterisation and connectivity
# ----------------------------------------------------------------------------
def rasterize(boxes, W, H, res=MAP_RES, pad=0.30):
    """Occupancy grid. True = occupied. Grid spans the world plus `pad` margin."""
    ox, oy = -W / 2.0 - pad, -H / 2.0 - pad
    nw = int(math.ceil((W + 2 * pad) / res))
    nh = int(math.ceil((H + 2 * pad) / res))
    grid = np.zeros((nh, nw), dtype=bool)
    for b in boxes:
        i0 = int(math.floor((b.cx - b.sx / 2.0 - ox) / res))
        i1 = int(math.ceil((b.cx + b.sx / 2.0 - ox) / res))
        j0 = int(math.floor((b.cy - b.sy / 2.0 - oy) / res))
        j1 = int(math.ceil((b.cy + b.sy / 2.0 - oy) / res))
        i0, i1 = max(0, i0), min(nw, i1)
        j0, j1 = max(0, j0), min(nh, j1)
        if i1 > i0 and j1 > j0:
            grid[j0:j1, i0:i1] = True
    return grid, (ox, oy)


def inflate(grid, radius=ROBOT_RADIUS, res=MAP_RES):
    """Binary dilation by the robot radius, so free cells are truly reachable."""
    r = int(math.ceil(radius / res))
    if r <= 0:
        return grid.copy()
    out = grid.copy()
    for dj in range(-r, r + 1):
        for di in range(-r, r + 1):
            if di * di + dj * dj > r * r:
                continue
            out |= np.roll(np.roll(grid, dj, axis=0), di, axis=1)
    out[:r, :] = True
    out[-r:, :] = True
    out[:, :r] = True
    out[:, -r:] = True
    return out


def bfs_dist(free, src):
    """Grid BFS distance in cells from src=(j,i). -1 where unreachable."""
    h, w = free.shape
    dist = -np.ones((h, w), dtype=np.int32)
    if not free[src]:
        return dist
    dist[src] = 0
    q = deque([src])
    while q:
        j, i = q.popleft()
        d = dist[j, i] + 1
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and free[nj, ni] and dist[nj, ni] < 0:
                dist[nj, ni] = d
                q.append((nj, ni))
    return dist


def farthest_pair(free):
    """Approximate diameter endpoints of the free space (double BFS)."""
    cells = np.argwhere(free)
    if len(cells) == 0:
        return None
    a = tuple(cells[len(cells) // 2])
    d1 = bfs_dist(free, a)
    if d1.max() <= 0:
        return None
    b = tuple(np.unravel_index(np.argmax(d1), d1.shape))
    d2 = bfs_dist(free, b)
    c = tuple(np.unravel_index(np.argmax(d2), d2.shape))
    return b, c, int(d2[c])


def cell_to_world(cell, origin, res=MAP_RES):
    j, i = cell
    return (origin[0] + (i + 0.5) * res, origin[1] + (j + 0.5) * res)


# ----------------------------------------------------------------------------
# Obstacle placement
# ----------------------------------------------------------------------------
def place_obstacles(walls, W, H, n, rng, min_clear=0.45):
    """Drop n boxes into corridor space, keeping the map connected."""
    placed = []
    for _ in range(n * 25):
        if len(placed) >= n:
            break
        size = rng.uniform(0.22, 0.40)
        grid, origin = rasterize(walls + placed, W, H)
        free = ~inflate(grid, max(ROBOT_RADIUS, min_clear))
        cells = np.argwhere(free)
        if len(cells) == 0:
            break
        cell = tuple(cells[rng.integers(len(cells))])
        x, y = cell_to_world(cell, origin)
        cand = Box(x, y, size, size, OBST_H)
        trial = walls + placed + [cand]
        g2, o2 = rasterize(trial, W, H)
        f2 = ~inflate(g2, max(ROBOT_RADIUS, min_clear))
        fp = farthest_pair(f2)
        if fp is None or fp[2] < 10:
            continue
        placed.append(cand)
    return placed


# ----------------------------------------------------------------------------
# Writers
# ----------------------------------------------------------------------------
SDF_HEAD = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
      <real_time_update_rate>{rate}</real_time_update_rate>
    </physics>
    <gui>
      <camera name="user_camera">
        <pose>0 0 {camz} 0 1.5707 1.5707</pose>
      </camera>
    </gui>
"""

SDF_TAIL = """  </world>
</sdf>
"""


def _box_xml(tag, idx, b, material):
    return f"""        <{tag} name="{tag}_{idx}">
          <pose>{b.cx:.4f} {b.cy:.4f} {b.h/2:.4f} 0 0 0</pose>
          <geometry><box><size>{b.sx:.4f} {b.sy:.4f} {b.h:.4f}</size></box></geometry>{material}
        </{tag}>
"""


def write_sdf(path, name, walls, obstacles, W, H, fast=True):
    step = 0.004 if fast else 0.001
    rate = 0 if fast else 1000
    rtf = 1.0
    camz = max(W, H) * 1.2
    wall_mat = ("\n          <material><script>"
                "<uri>file://media/materials/scripts/gazebo.material</uri>"
                "<name>Gazebo/Grey</name></script></material>")
    obst_mat = ("\n          <material><script>"
                "<uri>file://media/materials/scripts/gazebo.material</uri>"
                "<name>Gazebo/Orange</name></script></material>")

    parts = [SDF_HEAD.format(name=name, step=step, rtf=rtf, rate=rate, camz=camz)]
    parts.append('    <model name="walls">\n      <static>true</static>\n      <link name="link">\n')
    for k, b in enumerate(walls):
        parts.append(_box_xml("collision", k, b, ""))
        parts.append(_box_xml("visual", k, b, wall_mat))
    parts.append("      </link>\n    </model>\n")

    for k, b in enumerate(obstacles):
        parts.append(f'    <model name="obstacle_{k}">\n      <static>true</static>\n'
                     f'      <link name="link">\n')
        parts.append(_box_xml("collision", 0, Box(0, 0, b.sx, b.sy, b.h), ""))
        parts.append(_box_xml("visual", 0, Box(0, 0, b.sx, b.sy, b.h), obst_mat))
        parts.append(f"      </link>\n      <pose>{b.cx:.4f} {b.cy:.4f} 0 0 0 0</pose>\n"
                     f"    </model>\n")

    parts.append(SDF_TAIL)
    with open(path, "w") as f:
        f.write("".join(parts))


def write_map(pgm_path, yaml_path, grid, origin, res=MAP_RES):
    """PGM + YAML in the convention expected by nav2_map_server."""
    img = np.where(grid, 0, 254).astype(np.uint8)
    img = np.flipud(img)  # PGM row 0 is the top (max y)
    h, w = img.shape
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"# generated by worldgen.py\n{w} {h}\n255\n".encode())
        f.write(img.tobytes())
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def write_preview(path, grid, origin, start, goal, res=MAP_RES, scale=6):
    h, w = grid.shape
    img = Image.fromarray(np.flipud(np.where(grid, 40, 245).astype(np.uint8)), mode="L")
    img = img.convert("RGB").resize((w * scale, h * scale), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def px(p):
        i = (p[0] - origin[0]) / res
        j = (p[1] - origin[1]) / res
        return (i * scale, (h - j) * scale)

    for pt, col in ((start, (0, 170, 70)), (goal, (210, 40, 40))):
        x, y = px(pt)
        r = 4 * scale
        d.ellipse([x - r, y - r, x + r, y + r], fill=col)
    img.save(path)


# ----------------------------------------------------------------------------
# Environment assembly
# ----------------------------------------------------------------------------
def build_env(
    name,
    W,
    H,
    walls,
    obstacles,
    rng,
    start=None,
    goal=None,
    path_clearance=ROBOT_RADIUS,
):
    grid, origin = rasterize(walls + obstacles, W, H)

    clearance = max(
        ROBOT_RADIUS,
        float(path_clearance),
    )

    free = ~inflate(
        grid,
        radius=clearance,
    )
    fp = farthest_pair(free)
    if fp is None:
        return None
    a_cell, b_cell, dist_cells = fp
    if start is None or goal is None:
        start = cell_to_world(a_cell, origin)
        goal = cell_to_world(b_cell, origin)
        path_len = dist_cells * MAP_RES
    else:
        sj = int((start[1] - origin[1]) / MAP_RES)
        si = int((start[0] - origin[0]) / MAP_RES)
        gj = int((goal[1] - origin[1]) / MAP_RES)
        gi = int((goal[0] - origin[0]) / MAP_RES)
        if not (free[sj, si] and free[gj, gi]):
            return None
        dmap = bfs_dist(free, (sj, si))
        if dmap[gj, gi] < 0:
            return None
        path_len = float(dmap[gj, gi]) * MAP_RES
    return dict(name=name, W=W, H=H, walls=walls, obstacles=obstacles,
                grid=grid, origin=origin, start=start, goal=goal,
                path_len=path_len, path_clearance=clearance)


def emit(env, outdir, fast=True, seed=None, kind="train"):
    base = os.path.join(outdir, env["name"])
    write_sdf(base + ".world", env["name"], env["walls"], env["obstacles"],
              env["W"], env["H"], fast=fast)
    write_map(base + ".pgm", base + ".yaml", env["grid"], env["origin"])
    write_preview(base + ".png", env["grid"], env["origin"], env["start"], env["goal"])
    meta = dict(
        name=env["name"], kind=kind, seed=seed,
        size_m=[round(env["W"], 3), round(env["H"], 3)],
        start=[round(env["start"][0], 3), round(env["start"][1], 3), 0.0],
        goal=[round(env["goal"][0], 3), round(env["goal"][1], 3), 0.0],
        geodesic_path_length_m=round(env["path_len"], 2),
        n_obstacles=len(env["obstacles"]),
        map_resolution=MAP_RES,
        robot_radius=ROBOT_RADIUS,
        safety_path_clearance_m=round(
            env.get("path_clearance", ROBOT_RADIUS),
            3,
        ),
    )
    with open(base + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return meta


# ----------------------------------------------------------------------------
def gen_training_maps(outdir, n, seed, fast):
    metas = []
    made = 0
    attempt = 0
    while made < n and attempt < n * 60:
        attempt += 1
        s = seed * 10000 + attempt
        rng = np.random.default_rng(s)
        # Scale is randomised across three regimes so that the policy sees
        # clearances ranging from laboratory-arena tightness to full building
        # corridors. Without the "tight" regime, the physical arena would be far
        # outside the training distribution and hardware transfer would fail for
        # reasons that have nothing to do with the sim-to-real gap.
        regime = rng.choice(["tight", "medium", "large"], p=[0.30, 0.35, 0.35])
        if regime == "tight":
            W, H = rng.uniform(2.5, 6.0), rng.uniform(2.5, 5.0)
            nx = ny = 1
            cw, n_obst = rng.uniform(1.10, 1.35), int(rng.integers(0, 3))
        elif regime == "medium":
            W, H = rng.uniform(6.0, 12.0), rng.uniform(5.0, 10.0)
            nx, ny = int(rng.integers(1, 3)), int(rng.integers(1, 3))
            cw, n_obst = rng.uniform(1.10, 1.60), int(rng.integers(1, 5))
        else:
            W, H = rng.uniform(12.0, 20.0), rng.uniform(9.0, 15.0)
            nx, ny = int(rng.integers(2, 4)), int(rng.integers(2, 4))
            cw, n_obst = rng.uniform(1.30, 2.00), int(rng.integers(3, 8))
        W, H = float(W), float(H)
        cw_max = min((W - nx * 0.45) / (nx + 1), (H - ny * 0.45) / (ny + 1))
        cw = float(min(cw, cw_max))
        if cw < MIN_CORRIDOR_W:
            continue
        walls = corridor_network(W, H, nx, ny, cw, rng)
        if walls is None:
            continue
        obstacles = place_obstacles(walls, W, H, n_obst, rng)
        env = build_env(
            f"train_{made:02d}",
            W,
            H,
            walls,
            obstacles,
            rng,
            path_clearance=SAFETY_PATH_CLEARANCE,
        )
        if env is None or env["path_len"] < 3.0:
            continue
        metas.append(emit(env, outdir, fast=fast, seed=s, kind="train"))
        made += 1
    return metas


def gen_eval_building(outdir, seed, fast, W=20.0, H=15.0):
    for k in range(200):
        s = seed * 777 + k
        rng = np.random.default_rng(s)
        walls = corridor_network(W, H, 3, 3, 1.8, rng,
                                 block_dropout=0.20, doorway_prob=0.45)
        if walls is None:
            continue
        obstacles = place_obstacles(walls, W, H, 6, rng)
        env = build_env(
            "eval_building",
            W,
            H,
            walls,
            obstacles,
            rng,
            path_clearance=SAFETY_PATH_CLEARANCE,
        )
        if env is None or env["path_len"] < 18.0:
            continue
        return emit(env, outdir, fast=fast, seed=s, kind="eval_heldout")
    raise RuntimeError("could not generate eval_building meeting the length target")


def gen_arena(outdir, fast):
    W, H, walls, obstacles, start, goal = lab_arena()
    env = build_env(
        "arena_lab",
        W,
        H,
        walls,
        obstacles,
        np.random.default_rng(0),
        start=start,
        goal=goal,
        path_clearance=ROBOT_RADIUS,
    )
    if env is None:
        raise RuntimeError("arena_lab start/goal not connected -- check dividers")
    return emit(env, outdir, fast=fast, seed=None, kind="eval_hardware")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="./worlds")
    ap.add_argument("--n-train", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--only", choices=["train", "eval_building", "arena_lab"])
    ap.add_argument("--no-fast", action="store_true",
                    help="use 1 kHz physics instead of unthrottled (slower, for GUI use)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    fast = not args.no_fast
    metas = []

    if args.only in (None, "train"):
        metas += gen_training_maps(args.out, args.n_train, args.seed, fast)
    if args.only in (None, "eval_building"):
        metas.append(gen_eval_building(args.out, args.seed, fast))
    if args.only in (None, "arena_lab"):
        metas.append(gen_arena(args.out, fast))

    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(metas, f, indent=2)

    print(f"{'name':<16} {'kind':<15} {'size (m)':<14} {'path (m)':>9}  obst")
    print("-" * 62)
    for m in metas:
        size = f"{m['size_m'][0]:.1f} x {m['size_m'][1]:.1f}"
        print(f"{m['name']:<16} {m['kind']:<15} {size:<14} "
              f"{m['geodesic_path_length_m']:>9.1f}  {m['n_obstacles']}")
    print(f"\nwrote {len(metas)} environments to {args.out}")


if __name__ == "__main__":
    main()
