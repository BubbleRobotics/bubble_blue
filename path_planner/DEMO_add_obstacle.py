#!/usr/bin/env python3
import argparse
import subprocess, shlex, tempfile, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from path_planner_interfaces.srv import InitiatePath
from path_planner_interfaces.srv import SetObstacles
from path_planner_interfaces.msg import AABB, Sphere, OrientedBox

# Defaults (can be overridden via CLI)
DEFAULT_WORLD = "underwater_world"
DEFAULT_NAME = "obstacle_box"


def build_sdf_box(name: str, sx: float, sy: float, sz: float) -> str:
    """Return an SDF string for a static box model with the given side lengths."""
    return f"""
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <visual name="vis">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </visual>
      <collision name="col">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def spawn(world: str, name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float):
    """Remove any existing model named `name`, then spawn a box of size (sx,sy,sz) at (x,y,z)."""
    remove(world, name)

    sdf = build_sdf_box(name, sx, sy, sz)
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as f:
        f.write(sdf)
        path = f.name

    payload = (
        f'sdf_filename: "{path}", '
        f'name: "{name}", '
        f'allow_renaming: false, '
        f'pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}'
    )

    cmd = (
        f"gz service -s /world/{world}/create "
        f"--reqtype gz.msgs.EntityFactory "
        f"--reptype gz.msgs.Boolean "
        f"--timeout 1000 "
        f"--req '{payload}'"
    )
    subprocess.run(shlex.split(cmd), check=True)
    print(f"Spawned {name} (size {sx}x{sy}x{sz}) at ({x},{y},{z}) in gazebo world '{world}'")


def remove(world: str, name: str = DEFAULT_NAME):
    # Preferred: remove by name + type
    cmd = (
        f"gz service -s /world/{world}/remove "
        f"--reqtype gz.msgs.Entity "
        f"--reptype gz.msgs.Boolean "
        f"--timeout 1000 "
        f'--req \'name: "{name}", type: MODEL\''
    )
    try:
        subprocess.run(shlex.split(cmd), check=True)
        print(f"Removed {name} from '{world}'")
        return
    except subprocess.CalledProcessError:
        pass  # fall through to robust fallback

    # Fallback: if it was auto-renamed earlier, remove by ID (find ID via `gz model`)
    list_cmd = f"gz model --list -w {world}"
    out = subprocess.check_output(shlex.split(list_cmd), text=True)
    candidates = [line.strip() for line in out.splitlines() if line.strip().startswith(name)]
    if not candidates:
        raise RuntimeError(f"No model starting with '{name}' found in world '{world}'")

    target = candidates[0]
    info_cmd = f"gz model -m {target} -w {world} -i"
    info = subprocess.check_output(shlex.split(info_cmd), text=True)
    ent_id = None
    for line in info.splitlines():
        if "id:" in line:
            try:
                ent_id = int(line.split("id:")[1].strip())
                break
            except ValueError:
                pass
    if ent_id is None:
        raise RuntimeError(f"Could not parse ID for model '{target}'")

    by_id = (
        f"gz service -s /world/{world}/remove "
        f"--reqtype gz.msgs.Entity "
        f"--reptype gz.msgs.Boolean "
        f"--timeout 1000 "
        f"--req 'id: {ent_id}'"
    )
    subprocess.run(shlex.split(by_id), check=True)
    print(f"Removed model id {ent_id} ('{target}') from '{world}'")


def make_aabb_from_center_and_size(cx: float, cy: float, cz: float, sx: float, sy: float, sz: float) -> AABB:
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    return AABB(
        minx=cx - hx, miny=cy - hy, minz=cz - hz,
        maxx=cx + hx, maxy=cy + hy, maxz=cz + hz,
    )


def call_set_obstacles(aabbs, spheres=None, oriented_boxes=None):
    rclpy.init()
    node = Node('initiate_path_client')
    try:
        cli = node.create_client(SetObstacles, '/bluerov2/path_planner/add_obstacles')
        cli.wait_for_service()
        req = SetObstacles.Request()
        req.aabbs = aabbs or []
        req.spheres = spheres or []
        req.oriented_boxes = oriented_boxes or []
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        resp = fut.result()
        print('planner response:', resp.success)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args():
    p = argparse.ArgumentParser(description="Spawn a box in Gazebo and register it with the path planner as an AABB.")
    p.add_argument('--world', default=DEFAULT_WORLD, help='Gazebo world name')
    p.add_argument('--name', default=DEFAULT_NAME, help='Model name to spawn/remove')
    p.add_argument('--x', type=float, default=2.0, help='Center X of the box')
    p.add_argument('--y', type=float, default=0.0, help='Center Y of the box')
    p.add_argument('--z', type=float, default=-3.5, help='Center Z of the box')
    p.add_argument('--sx', type=float, default=0.75, help='Box length along X (must be > 0)')
    p.add_argument('--sy', type=float, default=0.75, help='Box length along Y (must be > 0)')
    p.add_argument('--sz', type=float, default=0.75, help='Box length along Z (must be > 0)')
    p.add_argument('action', nargs='?', choices=['spawn', 'remove'], default='spawn', help='Action to perform')
    return p.parse_args()


def main():
    args = parse_args()

    if args.action == 'remove':
        remove(args.world, args.name)
        return

    # Validate sizes
    for n, v in [('sx', args.sx), ('sy', args.sy), ('sz', args.sz)]:
        if v <= 0:
            raise ValueError(f"{n} must be > 0, got {v}")

    # 1) Spawn the exact-size box in Gazebo
    spawn(args.world, args.name, args.x, args.y, args.z, args.sx, args.sy, args.sz)

    # 2) Inform the planner using a matching AABB
    obstacle = make_aabb_from_center_and_size(args.x, args.y, args.z, args.sx, args.sy, args.sz)
    call_set_obstacles([obstacle])


if __name__ == "__main__":
    main()
