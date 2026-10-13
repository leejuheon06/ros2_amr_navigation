#!/usr/bin/env python3
"""Generate a matched Gazebo maze world and ROS occupancy map.

The same wall rectangles are used for both assets so that:
- Gazebo collision geometry
- RViz / map_server occupancy
- ICP reference map
all describe the same environment.

This script uses only the Python standard library.
"""

from __future__ import annotations

import math
from pathlib import Path

WORLD_NAME = "maze_world"
RESOLUTION = 0.05  # m / pixel
ORIGIN_X = -6.0
ORIGIN_Y = -5.0
WIDTH_M = 12.0
HEIGHT_M = 10.0
WIDTH_PX = int(round(WIDTH_M / RESOLUTION))
HEIGHT_PX = int(round(HEIGHT_M / RESOLUTION))
WALL_HEIGHT = 1.0
WALL_THICKNESS = 0.20

# (name, center_x, center_y, size_x, size_y)
# The three long internal walls create an S-shaped maze while keeping corridors
# wide enough for the 0.60 x 0.40 m AMR footprint plus Nav2 inflation.
WALLS = [
    ("boundary_bottom", 0.0, -4.9, 12.0, WALL_THICKNESS),
    ("boundary_top", 0.0, 4.9, 12.0, WALL_THICKNESS),
    ("boundary_left", -5.9, 0.0, WALL_THICKNESS, 10.0),
    ("boundary_right", 5.9, 0.0, WALL_THICKNESS, 10.0),
    ("maze_wall_1", -2.75, -2.50, 5.50, WALL_THICKNESS),
    ("maze_wall_2", 1.50, 0.00, WALL_THICKNESS, 5.00),
    ("maze_wall_3", 2.75, 2.50, 5.50, WALL_THICKNESS),
]


def wall_sdf(name: str, cx: float, cy: float, sx: float, sy: float) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {WALL_HEIGHT / 2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {WALL_HEIGHT}</size></box></geometry>
          <material>
            <ambient>0.25 0.30 0.35 1</ambient>
            <diffuse>0.35 0.40 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def write_world(path: Path) -> None:
    walls_xml = "\n".join(wall_sdf(*wall) for wall in WALLS)
    text = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>12 10</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>12 10</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{walls_xml}
  </world>
</sdf>
"""
    path.write_text(text, encoding="utf-8")


def world_to_pixel(x: float, y: float) -> tuple[int, int]:
    col = int(math.floor((x - ORIGIN_X) / RESOLUTION))
    row_from_bottom = int(math.floor((y - ORIGIN_Y) / RESOLUTION))
    row = HEIGHT_PX - 1 - row_from_bottom
    return col, row


def fill_rect(image: bytearray, cx: float, cy: float, sx: float, sy: float) -> None:
    x_min = cx - sx / 2.0
    x_max = cx + sx / 2.0
    y_min = cy - sy / 2.0
    y_max = cy + sy / 2.0

    c0, r1 = world_to_pixel(x_min, y_min)
    c1, r0 = world_to_pixel(x_max, y_max)

    c0 = max(0, min(WIDTH_PX - 1, c0))
    c1 = max(0, min(WIDTH_PX - 1, c1))
    r0 = max(0, min(HEIGHT_PX - 1, r0))
    r1 = max(0, min(HEIGHT_PX - 1, r1))

    if c0 > c1:
        c0, c1 = c1, c0
    if r0 > r1:
        r0, r1 = r1, r0

    for row in range(r0, r1 + 1):
        start = row * WIDTH_PX + c0
        end = row * WIDTH_PX + c1 + 1
        image[start:end] = bytes([0]) * (end - start)


def write_pgm(path: Path) -> None:
    # ROS map_server with negate=0 interprets black as occupied and white as free.
    image = bytearray([254] * (WIDTH_PX * HEIGHT_PX))
    for _, cx, cy, sx, sy in WALLS:
        fill_rect(image, cx, cy, sx, sy)

    header = f"P5\n{WIDTH_PX} {HEIGHT_PX}\n255\n".encode("ascii")
    path.write_bytes(header + image)


def write_yaml(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "image: maze_map.pgm",
                "mode: trinary",
                f"resolution: {RESOLUTION}",
                f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    package_dir = script_dir.parent
    world_dir = package_dir / "worlds"
    map_dir = package_dir / "maps"
    world_dir.mkdir(parents=True, exist_ok=True)
    map_dir.mkdir(parents=True, exist_ok=True)

    write_world(world_dir / "maze_world.sdf")
    write_pgm(map_dir / "maze_map.pgm")
    write_yaml(map_dir / "maze_map.yaml")

    print("Generated:")
    print(world_dir / "maze_world.sdf")
    print(map_dir / "maze_map.pgm")
    print(map_dir / "maze_map.yaml")
    print("Suggested benchmark start: (-5.0, -4.0)")
    print("Suggested benchmark goal : ( 5.0,  4.0)")


if __name__ == "__main__":
    main()
