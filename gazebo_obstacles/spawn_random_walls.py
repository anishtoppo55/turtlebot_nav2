#!/usr/bin/env python3
"""
Spawn several copies of wall.sdf at random poses in a running Gazebo world.

Walls are placed so that they:
  - stay inside the arena (endpoints within --arena-radius of --arena-center),
  - keep --robot-clearance metres away from the robot start pose,
  - keep --pillar-clearance metres away from the turtlebot3_world pillars
    (disable with --no-pillars, e.g. for empty_world),
  - keep --min-gap metres away from each other.

Requires Gazebo started through gazebo_ros (the turtlebot3_gazebo launch files
do this), which provides /spawn_entity, /delete_entity and /get_model_list.

Examples:
  python3 gazebo_obstacles/spawn_random_walls.py --count 4
  python3 gazebo_obstacles/spawn_random_walls.py --count 3 --length 1.0 --seed 42
  python3 gazebo_obstacles/spawn_random_walls.py --clear
"""

import argparse
import math
import os
import random
import re
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Pillar centres in turtlebot3_world (3 x 3 grid, cylinder radius 0.15 m).
TURTLEBOT3_WORLD_PILLARS = [
    (x, y) for x in (-1.1, 0.0, 1.1) for y in (-1.1, 0.0, 1.1)
]
PILLAR_RADIUS = 0.15


# ---------------------------------------------------------
# GEOMETRY
# ---------------------------------------------------------

def wall_endpoints(x, y, yaw, length):
    hx = 0.5 * length * math.cos(yaw)
    hy = 0.5 * length * math.sin(yaw)
    return (x - hx, y - hy), (x + hx, y + hy)


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def segments_intersect(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    d1, d2 = cross(c, d, a), cross(c, d, b)
    d3, d4 = cross(a, b, c), cross(a, b, d)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def segment_segment_distance(a, b, c, d):
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    )


# ---------------------------------------------------------
# SDF HANDLING
# ---------------------------------------------------------

def load_wall_sdf(path, length, thickness):
    """Load the SDF and reshape every box into a wall.

    Returns (sdf_xml, length, thickness, height). The box gets x = length
    and y = thickness; the height (z) and material come from the file.
    """
    with open(path, "r", encoding="utf-8") as f:
        xml = f.read()

    sizes = re.findall(r"<size>\s*([^<]+?)\s*</size>", xml)
    if not sizes:
        sys.exit(f"No <size> element found in {path}")
    if length <= 0 or thickness <= 0:
        sys.exit("--length and --thickness must be positive.")

    height = max(float(s.split()[2]) for s in sizes)

    def replace(match):
        _, _, z = match.group(1).split()
        return f"<size>{length} {thickness} {z}</size>"

    xml = re.sub(r"<size>\s*([^<]+?)\s*</size>", replace, xml)
    return xml, length, thickness, height


# ---------------------------------------------------------
# NODE
# ---------------------------------------------------------

class WallSpawner(Node):

    def __init__(self, timeout):
        super().__init__("random_wall_spawner")
        self.timeout = timeout
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.list_client = self.create_client(GetModelList, "/get_model_list")

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=self.timeout):
            sys.exit(
                f"Service {client.srv_name} not available. "
                "Is Gazebo running (e.g. turtlebot3_world.launch.py)?"
            )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done():
            sys.exit(f"Timed out calling {client.srv_name}")
        return future.result()

    def model_names(self):
        return list(self.call(self.list_client, GetModelList.Request()).model_names)

    def spawn(self, name, xml, x, y, z, yaw):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.reference_frame = "world"
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z = math.sin(0.5 * yaw)
        pose.orientation.w = math.cos(0.5 * yaw)
        request.initial_pose = pose
        response = self.call(self.spawn_client, request)
        return response.success, response.status_message

    def delete(self, name):
        request = DeleteEntity.Request()
        request.name = name
        response = self.call(self.delete_client, request)
        return response.success, response.status_message


# ---------------------------------------------------------
# PLACEMENT
# ---------------------------------------------------------

def generate_poses(args, length, thickness, rng):
    pillars = [] if args.no_pillars else TURTLEBOT3_WORLD_PILLARS
    robot = (args.robot_x, args.robot_y)
    cx, cy = args.arena_center
    placed = []  # (x, y, yaw, (a, b))

    for _ in range(args.max_attempts):
        if len(placed) >= args.count:
            break

        x = rng.uniform(cx - args.arena_radius, cx + args.arena_radius)
        y = rng.uniform(cy - args.arena_radius, cy + args.arena_radius)
        yaw = rng.uniform(-math.pi, math.pi)
        a, b = wall_endpoints(x, y, yaw, length)

        # Both endpoints (and the corners) must stay inside the arena.
        margin = 0.5 * thickness
        if any(
            math.hypot(p[0] - cx, p[1] - cy) > args.arena_radius - margin
            for p in (a, b)
        ):
            continue

        if point_segment_distance(robot, a, b) < args.robot_clearance + margin:
            continue

        if any(
            point_segment_distance(p, a, b)
            < PILLAR_RADIUS + args.pillar_clearance + margin
            for p in pillars
        ):
            continue

        if any(
            segment_segment_distance(a, b, *other[3])
            < args.min_gap + thickness
            for other in placed
        ):
            continue

        placed.append((x, y, yaw, (a, b)))

    return [(x, y, yaw) for x, y, yaw, _ in placed]


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Spawn copies of wall.sdf at random poses in Gazebo."
    )
    p.add_argument("--count", type=int, default=3, help="number of walls to spawn")
    p.add_argument("--sdf", default=os.path.join(SCRIPT_DIR, "wall.sdf"),
                   help="wall model file (default: wall.sdf next to this script)")
    p.add_argument("--length", type=float, default=0.8,
                   help="wall length in metres (default 0.8)")
    p.add_argument("--thickness", type=float, default=0.12,
                   help="wall thickness in metres (default 0.12)")
    p.add_argument("--prefix", default="random_wall",
                   help="entity name prefix; also used by --clear")
    p.add_argument("--seed", type=int, default=None,
                   help="random seed for a repeatable layout")
    p.add_argument("--arena-center", type=float, nargs=2, default=(0.0, 0.0),
                   metavar=("X", "Y"))
    p.add_argument("--arena-radius", type=float, default=2.0,
                   help="walls stay within this radius of the arena centre")
    p.add_argument("--robot-x", type=float, default=-2.0,
                   help="robot start x (turtlebot3_world default -2.0)")
    p.add_argument("--robot-y", type=float, default=-0.5,
                   help="robot start y (turtlebot3_world default -0.5)")
    p.add_argument("--robot-clearance", type=float, default=0.6,
                   help="minimum distance from walls to the robot start")
    p.add_argument("--min-gap", type=float, default=0.5,
                   help="minimum free gap between two walls")
    p.add_argument("--pillar-clearance", type=float, default=0.3,
                   help="minimum free gap between a wall and a pillar")
    p.add_argument("--no-pillars", action="store_true",
                   help="ignore turtlebot3_world pillars (e.g. for empty_world)")
    p.add_argument("--z", type=float, default=None,
                   help="spawn height of the wall centre "
                        "(default: half the box height, so it sits on the ground)")
    p.add_argument("--max-attempts", type=int, default=5000)
    p.add_argument("--timeout", type=float, default=10.0,
                   help="seconds to wait for each Gazebo service")
    p.add_argument("--clear", action="store_true",
                   help="delete all entities whose name starts with --prefix, then exit")
    p.add_argument("--replace", action="store_true",
                   help="delete existing --prefix walls before spawning new ones")
    return p.parse_args()


def delete_existing(node, prefix):
    names = [n for n in node.model_names() if n.startswith(prefix + "_")]
    for name in names:
        ok, msg = node.delete(name)
        print(f"{'Deleted' if ok else 'Failed to delete'} {name}"
              + ("" if ok else f": {msg}"))
    if not names:
        print(f"No entities starting with '{prefix}_' found.")


def main():
    args = parse_args()
    rclpy.init()
    node = WallSpawner(args.timeout)

    try:
        if args.clear:
            delete_existing(node, args.prefix)
            return

        if args.replace:
            delete_existing(node, args.prefix)

        xml, length, thickness, height = load_wall_sdf(
            args.sdf, args.length, args.thickness
        )
        z = 0.5 * height if args.z is None else args.z
        rng = random.Random(args.seed)
        poses = generate_poses(args, length, thickness, rng)

        if len(poses) < args.count:
            print(
                f"Warning: only found room for {len(poses)} of {args.count} "
                f"walls (length {length:.2f} m). Try a shorter --length, a "
                "smaller --min-gap, or a larger --arena-radius."
            )

        existing = set(node.model_names())
        index = 0
        for x, y, yaw in poses:
            while f"{args.prefix}_{index}" in existing:
                index += 1
            name = f"{args.prefix}_{index}"
            existing.add(name)

            ok, msg = node.spawn(name, xml, x, y, z, yaw)
            if ok:
                print(f"Spawned {name}: x={x:.2f} y={y:.2f} "
                      f"yaw={math.degrees(yaw):.0f} deg "
                      f"length={length:.2f} m")
            else:
                print(f"Failed to spawn {name}: {msg}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
