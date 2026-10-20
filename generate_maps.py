#!/usr/bin/env python3
"""generate_maps.py - random unseen layouts for the NIDAR AirMouse pipeline.

One generated layout produces ALL of these from the same seed, so the pygame
simulator, Gazebo and RViz always describe the identical building:

    <name>.json        grid + start cell + hidden survivors (pygame / tests)
    <name>.sdf         Gazebo (gz sim) world: walls, furniture, survivors
    <name>.pgm/.yaml   ground-truth occupancy map for nav2 map_server / RViz
    <name>_preview.png quick look at the layout
    rescue_mapping.rviz  RViz2 config (written once per output folder)

Grid encoding is identical to main.py: 0 = free, 1 = wall, 2 = furniture.
Row 0 of the grid is the TOP of the picture; in world coordinates it is the
largest Y, so the PGM and the pygame view line up with RViz without flipping.

Examples:
    python generate_maps.py --count 5 --mode house --seed 42
    python generate_maps.py --mode maze --persons 6 --cols 31 --rows 21
    python generate_maps.py --count 3 --person-uri "model://casual_man"

Use in main.py:
    from generate_maps import load_map
    grid, start, people = load_map("maps/house_00042.json")
"""
from __future__ import annotations

import argparse
import json
import random
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

FREE, WALL, FURN = 0, 1, 2
MAX_PERSONS = 6


# --------------------------------------------------------------------------- #
# Grid utilities
# --------------------------------------------------------------------------- #
def label_components(grid: np.ndarray):
    rows, cols = grid.shape
    label = -np.ones(grid.shape, dtype=int)
    n = 0
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] == FREE and label[y, x] < 0:
                q = deque([(x, y)])
                label[y, x] = n
                while q:
                    cx, cy = q.popleft()
                    for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                        if 0 <= nx < cols and 0 <= ny < rows and grid[ny, nx] == FREE and label[ny, nx] < 0:
                            label[ny, nx] = n
                            q.append((nx, ny))
                n += 1
    return label, n


def ensure_connected(grid: np.ndarray, rng: random.Random) -> None:
    """Open single wall cells until every free cell is reachable from every other."""
    rows, cols = grid.shape
    while True:
        label, n = label_components(grid)
        if n <= 1:
            return
        cands = []
        for y in range(1, rows - 1):
            for x in range(1, cols - 1):
                if grid[y, x] == WALL:
                    labs = {label[ny, nx] for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1))
                            if label[ny, nx] >= 0}
                    if len(labs) >= 2:
                        cands.append((x, y))
        if not cands:
            return
        x, y = rng.choice(cands)
        grid[y, x] = FREE


def bfs_dist(grid: np.ndarray, start):
    rows, cols = grid.shape
    dist = -np.ones(grid.shape, dtype=int)
    dist[start[1], start[0]] = 0
    q = deque([start])
    while q:
        cx, cy = q.popleft()
        for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
            if 0 <= nx < cols and 0 <= ny < rows and grid[ny, nx] == FREE and dist[ny, nx] < 0:
                dist[ny, nx] = dist[cy, cx] + 1
                q.append((nx, ny))
    return dist


# --------------------------------------------------------------------------- #
# Layout generators
# --------------------------------------------------------------------------- #
def gen_house(cols: int, rows: int, rng: random.Random, min_room: int = 4) -> np.ndarray:
    """Rooms by binary space partition, each dividing wall gets 1-2 doorways."""
    g = np.zeros((rows, cols), dtype=np.uint8)
    g[0, :] = g[-1, :] = WALL
    g[:, 0] = g[:, -1] = WALL

    def split(x0, y0, x1, y1, depth):
        w, h = x1 - x0 + 1, y1 - y0 + 1
        can_v = w >= 2 * min_room + 1
        can_h = h >= 2 * min_room + 1
        if not (can_v or can_h):
            return
        if depth >= 1 and rng.random() < 0.12:  # leave an occasional big room
            return
        if can_v and can_h:
            vertical = rng.random() < 0.5 if abs(w - h) < 4 else w > h
        else:
            vertical = can_v
        if vertical:
            sx = rng.randint(x0 + min_room, x1 - min_room)
            g[y0:y1 + 1, sx] = WALL
            for _ in range(2 if h >= 10 and rng.random() < 0.6 else 1):
                dw = rng.choice((1, 2))
                dy = rng.randint(y0, max(y0, y1 - dw + 1))
                g[dy:dy + dw, sx] = FREE
            split(x0, y0, sx - 1, y1, depth + 1)
            split(sx + 1, y0, x1, y1, depth + 1)
        else:
            sy = rng.randint(y0 + min_room, y1 - min_room)
            g[sy, x0:x1 + 1] = WALL
            for _ in range(2 if w >= 10 and rng.random() < 0.6 else 1):
                dw = rng.choice((1, 2))
                dx = rng.randint(x0, max(x0, x1 - dw + 1))
                g[sy, dx:dx + dw] = FREE
            split(x0, y0, x1, sy - 1, depth + 1)
            split(x0, sy + 1, x1, y1, depth + 1)

    split(1, 1, cols - 2, rows - 2, 0)
    ensure_connected(g, rng)
    return g


def gen_maze(cols: int, rows: int, rng: random.Random, braid: float = 0.12) -> np.ndarray:
    """Recursive-backtracker maze. braid > 0 knocks out some walls to make loops."""
    cols |= 1
    rows |= 1
    g = np.ones((rows, cols), dtype=np.uint8)
    g[1, 1] = FREE
    stack = [(1, 1)]
    while stack:
        x, y = stack[-1]
        opts = [(x + dx * 2, y + dy * 2, dx, dy)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 1 <= x + dx * 2 < cols - 1 and 1 <= y + dy * 2 < rows - 1
                and g[y + dy * 2, x + dx * 2] == WALL]
        if opts:
            nx, ny, dx, dy = rng.choice(opts)
            g[y + dy, x + dx] = FREE
            g[ny, nx] = FREE
            stack.append((nx, ny))
        else:
            stack.pop()
    for y in range(1, rows - 1):
        for x in range(1, cols - 1):
            if g[y, x] == WALL and (x + y) % 2 == 1 and rng.random() < braid:
                if (g[y, x - 1] == FREE and g[y, x + 1] == FREE) or (g[y - 1, x] == FREE and g[y + 1, x] == FREE):
                    g[y, x] = FREE
    return g


def add_obstacles(grid: np.ndarray, rng: random.Random, count: int, avoid, max_w=3, max_h=2) -> None:
    """Drop furniture blocks, rejecting any that would split the free space."""
    rows, cols = grid.shape
    placed, tries = 0, 0
    while placed < count and tries < count * 40:
        tries += 1
        w, h = rng.randint(1, max_w), rng.randint(1, max_h)
        if rng.random() < 0.5:
            w, h = h, w
        x, y = rng.randint(1, cols - 1 - w), rng.randint(1, rows - 1 - h)
        block = grid[y:y + h, x:x + w]
        if not np.all(block == FREE):
            continue
        if abs(x - avoid[0]) <= w + 1 and abs(y - avoid[1]) <= h + 1:
            continue
        block[:] = FURN
        if label_components(grid)[1] != 1:
            block[:] = FREE
            continue
        placed += 1


def pick_start(grid: np.ndarray, prefer):
    rows, cols = grid.shape
    free = [(x, y) for y in range(rows) for x in range(cols) if grid[y, x] == FREE]
    return min(free, key=lambda c: abs(c[0] - prefer[0]) + abs(c[1] - prefer[1]))


def place_people(grid: np.ndarray, start, n: int, rng: random.Random):
    dist = bfs_dist(grid, start)
    rows, cols = grid.shape
    for min_start, min_sep in ((7, 5), (5, 4), (4, 3), (2, 2), (1, 1)):
        cells = [(x, y) for y in range(rows) for x in range(cols) if dist[y, x] >= min_start]
        rng.shuffle(cells)
        chosen = []
        for c in cells:
            if all(np.hypot(c[0] - o[0], c[1] - o[1]) >= min_sep for o in chosen):
                chosen.append(c)
                if len(chosen) == n:
                    return chosen
    return chosen


# --------------------------------------------------------------------------- #
# Export helpers
# --------------------------------------------------------------------------- #
def cell_to_world(x, y, rows, s):
    """Cell centre -> world metres. Grid row 0 is the top, i.e. the largest Y."""
    return (x + 0.5) * s, (rows - y - 0.5) * s


def merge_rects(grid: np.ndarray, value: int):
    """Greedy decomposition into as few boxes as possible (keeps SDF small)."""
    rows, cols = grid.shape
    used = np.zeros(grid.shape, dtype=bool)
    out = []
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] != value or used[y, x]:
                continue
            w = 1
            while x + w < cols and grid[y, x + w] == value and not used[y, x + w]:
                w += 1
            h = 1
            while (y + h < rows and np.all(grid[y + h, x:x + w] == value)
                   and not used[y + h, x:x + w].any()):
                h += 1
            used[y:y + h, x:x + w] = True
            out.append((x, y, w, h))
    return out


def box_model(name, cx, cy, sx, sy, sz, rgba):
    r, g, b = rgba
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def person_model(name, wx, wy, uri=None):
    if uri:
        return (f'    <include><uri>{uri}</uri><name>{name}</name>'
                f'<static>true</static><pose>{wx:.3f} {wy:.3f} 0 0 0 0</pose></include>\n')
    # Simple capsule-like stand-in. For YOLO tests in Gazebo pass --person-uri
    # with a real human mesh (e.g. a Gazebo Fuel "Casual Man" model).
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><pose>0 0 0.85 0 0 0</pose><geometry><cylinder><radius>0.22</radius><length>1.7</length></cylinder></geometry></collision>
        <visual name="body"><pose>0 0 0.6 0 0 0</pose><geometry><cylinder><radius>0.22</radius><length>1.2</length></cylinder></geometry>
          <material><ambient>0.85 0.2 0.15 1</ambient><diffuse>0.85 0.2 0.15 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 1.45 0 0 0</pose><geometry><sphere><radius>0.13</radius></sphere></geometry>
          <material><ambient>0.9 0.72 0.6 1</ambient><diffuse>0.9 0.72 0.6 1</diffuse></material></visual>
      </link>
    </model>
"""


def build_sdf(grid, people, s, name, wall_h=2.5, person_uri=None) -> str:
    rows, cols = grid.shape
    parts = [f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.2 0.2 0.22 1</background><shadows>true</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 15 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <pose>{cols * s / 2:.2f} {rows * s / 2:.2f} 0 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{cols * s + 10:.0f} {rows * s + 10:.0f}</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{cols * s + 10:.0f} {rows * s + 10:.0f}</size></plane></geometry>
          <material><ambient>0.75 0.75 0.72 1</ambient><diffuse>0.75 0.75 0.72 1</diffuse></material></visual>
      </link>
    </model>
"""]
    for i, (x, y, w, h) in enumerate(merge_rects(grid, WALL)):
        cx, cy = (x + w / 2) * s, (rows - (y + h / 2)) * s
        parts.append(box_model(f"wall_{i}", cx, cy, w * s, h * s, wall_h, (0.35, 0.38, 0.45)))
    for i, (x, y, w, h) in enumerate(merge_rects(grid, FURN)):
        cx, cy = (x + w / 2) * s, (rows - (y + h / 2)) * s
        parts.append(box_model(f"furniture_{i}", cx, cy, w * s, h * s, 0.8, (0.5, 0.4, 0.28)))
    for i, (px, py) in enumerate(people, 1):
        wx, wy = cell_to_world(px, py, rows, s)
        parts.append(person_model(f"survivor_{i}", wx, wy, person_uri))
    parts.append("  </world>\n</sdf>\n")
    return "".join(parts)


RVIZ_CONFIG = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: map
    Frame Rate: 30
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/Map
      Name: Ground truth (generated)
      Enabled: true
      Alpha: 0.35
      Topic:
        Value: /ground_truth_map
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/Map
      Name: Drone map (SLAM / occupancy)
      Enabled: true
      Alpha: 0.9
      Topic:
        Value: /map
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
    - Class: rviz_default_plugins/Path
      Name: Drone path
      Enabled: true
      Topic:
        Value: /drone/path
    - Class: rviz_default_plugins/Path
      Name: Planned path
      Enabled: true
      Topic:
        Value: /drone/plan
    - Class: rviz_default_plugins/MarkerArray
      Name: Survivors
      Enabled: true
      Topic:
        Value: /survivors
    - Class: rviz_default_plugins/Image
      Name: Camera
      Enabled: true
      Topic:
        Value: /camera/image_annotated
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Scale: 18
"""


def save_preview(grid, start, people, path, px=20):
    rows, cols = grid.shape
    img = np.full((rows * px, cols * px, 3), 235, dtype=np.uint8)
    for y in range(rows):
        for x in range(cols):
            c = {WALL: (72, 62, 54), FURN: (94, 112, 126)}.get(int(grid[y, x]))
            if c:
                cv2.rectangle(img, (x * px, y * px), ((x + 1) * px - 1, (y + 1) * px - 1), c, -1)
    cv2.circle(img, (start[0] * px + px // 2, start[1] * px + px // 2), px // 2 - 2, (245, 130, 35), -1)
    for i, (x, y) in enumerate(people, 1):
        cv2.circle(img, (x * px + px // 2, y * px + px // 2), px // 2 - 2, (72, 72, 224), -1)
        cv2.putText(img, str(i), (x * px + 5, y * px + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.imwrite(str(path), img)


def export_map(grid, start, people, seed, mode, s, out: Path, name: str, person_uri=None, ppc=5):
    rows, cols = grid.shape
    out.mkdir(parents=True, exist_ok=True)
    meta = {
        "name": name, "seed": seed, "mode": mode, "rows": rows, "cols": cols,
        "cell_size_m": s, "encoding": {"free": 0, "wall": 1, "furniture": 2},
        "start": list(start), "start_world": list(cell_to_world(*start, rows, s)),
        "people": [{"id": i, "cell": [x, y], "world": list(cell_to_world(x, y, rows, s))}
                   for i, (x, y) in enumerate(people, 1)],
        "grid": grid.tolist(),
    }
    (out / f"{name}.json").write_text(json.dumps(meta))
    (out / f"{name}.sdf").write_text(build_sdf(grid, people, s, name, person_uri=person_uri))
    # Ground-truth occupancy image for nav2 map_server (occupied=0, free=254).
    pic = np.where(grid == FREE, 254, 0).astype(np.uint8)
    cv2.imwrite(str(out / f"{name}.pgm"), np.kron(pic, np.ones((ppc, ppc), dtype=np.uint8)))
    (out / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {s / ppc:.5f}\n"
        f"origin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    save_preview(grid, start, people, out / f"{name}_preview.png")
    rviz = out / "rescue_mapping.rviz"
    if not rviz.exists():
        rviz.write_text(RVIZ_CONFIG)


def load_map(path):
    """Return (grid ndarray, start (x, y), people [(x, y), ...]) from a generated JSON."""
    d = json.loads(Path(path).read_text())
    return (np.array(d["grid"], dtype=np.uint8), tuple(d["start"]),
            [tuple(p["cell"]) for p in d["people"]])


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="Generate random house/maze layouts with survivors.")
    ap.add_argument("--mode", choices=("house", "maze"), default="house")
    ap.add_argument("--count", type=int, default=1, help="number of maps to generate")
    ap.add_argument("--seed", type=int, default=None, help="base seed (map i uses seed+i)")
    ap.add_argument("--persons", type=int, default=None, help="1-6 survivors (default: random 1-6)")
    ap.add_argument("--cols", type=int, default=28)
    ap.add_argument("--rows", type=int, default=20)
    ap.add_argument("--cell-size", type=float, default=1.0, help="metres per grid cell")
    ap.add_argument("--obstacles", type=int, default=None, help="furniture blocks (default 5 house / 3 maze)")
    ap.add_argument("--person-uri", default=None, help="Gazebo model URI for survivors (optional)")
    ap.add_argument("--out", default="maps")
    a = ap.parse_args()

    if a.persons is not None and not 1 <= a.persons <= MAX_PERSONS:
        ap.error(f"--persons must be 1..{MAX_PERSONS}")
    base = a.seed if a.seed is not None else int(time.time()) % 100000
    out = Path(a.out)

    for i in range(a.count):
        seed = base + i
        rng = random.Random(seed)
        if a.mode == "house":
            grid = gen_house(a.cols, a.rows, rng)
            prefer = (2, 2)
        else:
            grid = gen_maze(a.cols, a.rows, rng)
            prefer = (1, 1)
        start = pick_start(grid, prefer)
        n_obs = a.obstacles if a.obstacles is not None else (5 if a.mode == "house" else 3)
        add_obstacles(grid, rng, n_obs, avoid=start)
        n_people = a.persons if a.persons is not None else rng.randint(1, MAX_PERSONS)
        people = place_people(grid, start, n_people, rng)
        name = f"{a.mode}_{seed:05d}"
        export_map(grid, start, people, seed, a.mode, a.cell_size, out, name, a.person_uri)
        print(f"[{name}] {grid.shape[1]}x{grid.shape[0]} cells, {len(people)} survivors, start={start} -> {out}/")


if __name__ == "__main__":
    main()
