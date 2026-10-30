#!/usr/bin/env python3
"""
Build a random maze ("puzzle") on the 9 pillars of turtlebot3_world.

The pillars sit on a 3 x 3 grid with 1.1 m spacing. That grid has 12 gaps
between neighbouring pillars, which border 4 square cells:

      P---P---P
      | A | B |          P = pillar
      P---P---P          A..D = cells inside the grid
      | C | D |          every gap (--- or |) is a possible wall
      P---P---P

Each gap connects two areas (two cells, or a cell and the outside). The
script keeps a random spanning tree of those connections open and fills
every other gap with a wall from wall.sdf. That leaves exactly one way into
each cell, and every cell stays reachable. --openness opens extra gaps to
make the maze easier.

--spokes adds short walls pointing outward from the outer pillars, so the
outside path around the grid isn't a plain loop. They never touch the arena
wall, so nothing gets cut off.

Examples:
  python3 gazebo_obstacles/spawn_pillar_maze.py
  python3 gazebo_obstacles/spawn_pillar_maze.py --seed 3 --spokes 4
  python3 gazebo_obstacles/spawn_pillar_maze.py --replace --openness 0.3
  python3 gazebo_obstacles/spawn_pillar_maze.py --clear
"""

import argparse
import math
import os
import random
import re
import sys

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from spawn_random_walls import (  # noqa: E402
    PILLAR_RADIUS,
    WallSpawner,
    delete_existing,
    point_segment_distance,
)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SPACING = 1.1
GRID = (-1.1, 0.0, 1.1)
OUTSIDE = "outside"


# ---------------------------------------------------------
# MAZE GRAPH
# ---------------------------------------------------------

def pillar(i, j):
    """Pillar at column i, row j (0..2, row 0 at the bottom)."""
    return (GRID[i], GRID[j])


def cell_name(i, j):
    """Cell whose bottom-left pillar is (i, j); i, j in 0..1."""
    return f"cell_{i}{j}"


def build_gaps():
    """Return the 12 gaps as (pillar_a, pillar_b, area_1, area_2)."""
    gaps = []

    # Horizontal gaps: between pillar (i, j) and (i + 1, j).
    for i in range(2):
        for j in range(3):
            below = cell_name(i, j - 1) if j > 0 else OUTSIDE
            above = cell_name(i, j) if j < 2 else OUTSIDE
            gaps.append((pillar(i, j), pillar(i + 1, j), below, above))

    # Vertical gaps: between pillar (i, j) and (i, j + 1).
    for i in range(3):
        for j in range(2):
            left = cell_name(i - 1, j) if i > 0 else OUTSIDE
            right = cell_name(i, j) if i < 2 else OUTSIDE
            gaps.append((pillar(i, j), pillar(i, j + 1), left, right))

    return gaps


def choose_open_gaps(gaps, openness, rng):
    """Random spanning tree over the areas (Kruskal), plus extra openings."""
    parent = {}

    def find(a):
        parent.setdefault(a, a)
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    order = list(range(len(gaps)))
    rng.shuffle(order)

    open_gaps = set()
    for k in order:
        ra, rb = find(gaps[k][2]), find(gaps[k][3])
        if ra != rb:
            parent[ra] = rb
            open_gaps.add(k)

    for k in range(len(gaps)):
        if k not in open_gaps and rng.random() < openness:
            open_gaps.add(k)

    return open_gaps


def spoke_candidates():
    """Outward directions from each outer pillar: (pillar, (dx, dy))."""
    candidates = []
    for i in range(3):
        for j in range(3):
            directions = []
            if i == 0:
                directions.append((-1, 0))
            if i == 2:
                directions.append((1, 0))
            if j == 0:
                directions.append((0, -1))
            if j == 2:
                directions.append((0, 1))
            for d in directions:
                candidates.append((pillar(i, j), d))
    return candidates


# ---------------------------------------------------------
# WALL GEOMETRY
# ---------------------------------------------------------

def wall_between(a, b, gap):
    """Wall from the surface of pillar a to the surface of pillar b."""
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    length = math.hypot(b[0] - a[0], b[1] - a[1]) - 2.0 * (PILLAR_RADIUS + gap)
    x = 0.5 * (a[0] + b[0])
    y = 0.5 * (a[1] + b[1])
    return x, y, yaw, length


def spoke_wall(p, direction, length, gap):
    """Wall starting at a pillar surface and pointing outward."""
    dx, dy = direction
    start = PILLAR_RADIUS + gap
    x = p[0] + dx * (start + 0.5 * length)
    y = p[1] + dy * (start + 0.5 * length)
    end_a = (p[0] + dx * start, p[1] + dy * start)
    end_b = (p[0] + dx * (start + length), p[1] + dy * (start + length))
    return x, y, math.atan2(dy, dx), length, (end_a, end_b)


def square_block_xml(xml, side):
    """Make every box in the SDF a square block: side x side, same height."""
    def replace(match):
        _, _, z = match.group(1).split()
        return f"<size>{side:.4f} {side:.4f} {z}</size>"

    return re.sub(r"<size>\s*([^<]+?)\s*</size>", replace, xml)


def block_side(xml):
    """Default block side: the larger of x and y in the SDF box."""
    sizes = re.findall(r"<size>\s*([^<]+?)\s*</size>", xml)
    return max(max(float(v) for v in s.split()[:2]) for s in sizes)


def wall_height(xml):
    sizes = re.findall(r"<size>\s*([^<]+?)\s*</size>", xml)
    if not sizes:
        sys.exit("No <size> element found in the wall SDF")
    return max(float(s.split()[2]) for s in sizes)


# ---------------------------------------------------------
# TEXT PREVIEW
# ---------------------------------------------------------

def print_layout(gaps, open_gaps, spokes):
    """Print the maze as text, top row first. '#' = wall, ' ' = open."""
    def closed(a, b):
        for k, (pa, pb, _, _) in enumerate(gaps):
            if {pa, pb} == {a, b}:
                return k not in open_gaps
        return False

    spoke_set = {(p, d) for p, d in spokes}

    def vertical_spokes(j, direction):
        # Pillars are printed at text columns 4, 8 and 12.
        chars = [" "] * 13
        for i in range(3):
            if (pillar(i, j), direction) in spoke_set:
                chars[4 + 4 * i] = "|"
        return "".join(chars).rstrip()

    lines = []
    for j in (2, 1, 0):
        if j == 2:
            lines.append(vertical_spokes(2, (0, 1)))

        row = " -- " if (pillar(0, j), (-1, 0)) in spoke_set else "    "
        for i in range(3):
            row += "O"
            if i < 2:
                row += "###" if closed(pillar(i, j), pillar(i + 1, j)) else "   "
        row += " --" if (pillar(2, j), (1, 0)) in spoke_set else ""
        lines.append(row)

        if j > 0:
            row = "    "
            for i in range(3):
                row += "#" if closed(pillar(i, j - 1), pillar(i, j)) else " "
                if i < 2:
                    row += "   "
            lines.append(row)
        else:
            lines.append(vertical_spokes(0, (0, -1)))

    print("Maze layout (top = +y, O = pillar, # = wall, -- / | = spoke):")
    for line in lines:
        print(line)


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Spawn a random maze on the 9 turtlebot3_world pillars."
    )
    p.add_argument("--sdf", default=os.path.join(SCRIPT_DIR, "wall.sdf"),
                   help="wall model (its height and colour are used)")
    p.add_argument("--seed", type=int, default=None,
                   help="random seed for a repeatable maze")
    p.add_argument("--walls", type=int, default=4,
                   help="exact total number of walls (default 4). Up to 8 go "
                        "between pillars, the rest point outward. "
                        "Use -1 to use --openness and --spokes instead")
    p.add_argument("--openness", type=float, default=0.0,
                   help="with --walls -1: chance (0-1) to open each extra gap")
    p.add_argument("--spokes", type=int, default=3,
                   help="with --walls -1: number of outward walls (0-12)")
    p.add_argument("--size", type=float, default=None,
                   help="side of each square block in metres "
                        "(default: box size in wall.sdf)")
    p.add_argument("--pillar-gap", type=float, default=0.01,
                   help="space left between a wall end and a pillar surface")
    p.add_argument("--robot-x", type=float, default=-2.0)
    p.add_argument("--robot-y", type=float, default=-0.5)
    p.add_argument("--robot-clearance", type=float, default=0.5,
                   help="outward walls stay this far from the robot start")
    p.add_argument("--prefix", default="maze_wall",
                   help="entity name prefix; also used by --clear")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="seconds to wait for each Gazebo service")
    p.add_argument("--clear", action="store_true",
                   help="delete all maze walls, then exit")
    p.add_argument("--replace", action="store_true",
                   help="delete the existing maze before building a new one")
    p.add_argument("--dry-run", action="store_true",
                   help="print the layout without contacting Gazebo")
    return p.parse_args()


def main():
    args = parse_args()
    rng = random.Random(args.seed)

    if args.clear:
        rclpy.init()
        node = WallSpawner(args.timeout)
        try:
            delete_existing(node, args.prefix)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    with open(args.sdf, "r", encoding="utf-8") as f:
        base_xml = f.read()
    height = wall_height(base_xml)
    side = args.size if args.size is not None else block_side(base_xml)
    block_xml = square_block_xml(base_xml, side)

    gaps = build_gaps()

    if args.walls >= 0:
        # Exact total: fill gaps between pillars first (at most 8, so every
        # cell stays reachable), then put the rest on outward walls.
        open_gaps = choose_open_gaps(gaps, 0.0, rng)
        closed = [k for k in range(len(gaps)) if k not in open_gaps]
        rng.shuffle(closed)
        gap_walls = min(args.walls, len(closed))
        open_gaps |= set(closed[gap_walls:])
        spoke_count = args.walls - gap_walls
    else:
        open_gaps = choose_open_gaps(gaps, args.openness, rng)
        spoke_count = args.spokes

    robot = (args.robot_x, args.robot_y)
    candidates = []
    for p, d in spoke_candidates():
        *_, (a, b) = spoke_wall(p, d, side, args.pillar_gap)
        if point_segment_distance(robot, a, b) >= args.robot_clearance:
            candidates.append((p, d))
    rng.shuffle(candidates)
    spokes = candidates[:max(0, min(spoke_count, len(candidates)))]

    if len(spokes) < spoke_count:
        print(f"Warning: only room for {len(spokes)} outward walls; "
              f"spawning {len(gaps) - len(open_gaps) + len(spokes)} walls "
              f"instead of {args.walls}.")

    # One square block per wall: in the middle of a closed gap, or just
    # outside an outer pillar.
    walls = []
    for k, (a, b, _, _) in enumerate(gaps):
        if k not in open_gaps:
            x, y, yaw, gap_length = wall_between(a, b, args.pillar_gap)
            if side > gap_length:
                sys.exit(f"--size {side:.2f} m is wider than the "
                         f"{gap_length:.2f} m gap between pillars.")
            walls.append((x, y, yaw))
    for p, d in spokes:
        x, y, yaw, _, _ = spoke_wall(p, d, side, args.pillar_gap)
        walls.append((x, y, yaw))

    print_layout(gaps, open_gaps, spokes)
    print(f"{len(gaps) - len(open_gaps)} gap blocks, {len(spokes)} outward "
          f"blocks, each {side:.2f} x {side:.2f} m")

    if args.dry_run:
        return

    rclpy.init()
    node = WallSpawner(args.timeout)
    try:
        if args.replace:
            delete_existing(node, args.prefix)

        existing = set(node.model_names())
        index = 0
        for x, y, yaw in walls:
            while f"{args.prefix}_{index}" in existing:
                index += 1
            name = f"{args.prefix}_{index}"
            existing.add(name)

            ok, msg = node.spawn(name, block_xml, x, y, 0.5 * height, yaw)
            if ok:
                print(f"Spawned {name}: x={x:.2f} y={y:.2f} "
                      f"size={side:.2f} m")
            else:
                print(f"Failed to spawn {name}: {msg}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
