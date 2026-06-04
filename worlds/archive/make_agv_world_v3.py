import math
from pathlib import Path

WORLD_PATH = Path.home() / "gazebo_agv_ws/src/agv_sim/worlds/agv_factory_v3.world"

LINE_WIDTH = 0.22
LINE_HEIGHT = 0.035
LINE_Z = LINE_HEIGHT / 2

FLOOR_Z = -0.015


def material(name, r, g, b, a=1.0):
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""


def box_model(name, x, y, z, sx, sy, sz, color, yaw=0):
    r, g, b, a = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
{material(name + "_mat", r, g, b, a)}
        </visual>
      </link>
    </model>
"""


def cylinder_model(name, x, y, z, radius, length, color):
    r, g, b, a = color
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius:.4f}</radius>
              <length>{length:.4f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius:.4f}</radius>
              <length>{length:.4f}</length>
            </cylinder>
          </geometry>
{material(name + "_mat", r, g, b, a)}
        </visual>
      </link>
    </model>
"""


def segment_model(name, p1, p2, width=LINE_WIDTH, height=LINE_HEIGHT, color=(0, 0, 0, 1)):
    x1, y1 = p1
    x2, y2 = p2

    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    dx = x2 - x1
    dy = y2 - y1

    length = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)

    return box_model(
        name=name,
        x=mx,
        y=my,
        z=LINE_Z,
        sx=length,
        sy=width,
        sz=height,
        color=color,
        yaw=yaw
    )


def polyline_models(prefix, points, width=LINE_WIDTH):
    result = ""
    for i in range(len(points) - 1):
        result += segment_model(
            name=f"{prefix}_{i+1:02d}",
            p1=points[i],
            p2=points[i + 1],
            width=width
        )
    return result


BLACK = (0.0, 0.0, 0.0, 1)
FLOOR = (0.78, 0.78, 0.78, 1)

RED = (1.0, 0.0, 0.0, 1)
BLUE = (0.0, 0.1, 1.0, 1)
YELLOW = (1.0, 0.9, 0.0, 1)
ORANGE = (1.0, 0.45, 0.0, 1)
PURPLE = (0.55, 0.0, 0.9, 1)
GREEN = (0.0, 0.8, 0.1, 1)
GRAY = (0.45, 0.45, 0.45, 1)
DARK = (0.08, 0.08, 0.08, 1)
WHITE = (1.0, 1.0, 1.0, 1)

# ===============================
# v3 MAP center line coordinates
# ===============================

# Top line: red, blue
top_line = [
    (-2.55, 2.80),
    (-1.90, 2.80),
    (-1.20, 2.80),
    (-0.40, 2.80),
    (0.40, 2.80),
    (1.20, 2.80),
    (1.80, 2.80),
    (2.45, 2.80),
]

# Top-right curve: smoother connection
top_right_curve = [
    (2.45, 2.80),
    (2.70, 2.75),
    (2.95, 2.60),
    (3.12, 2.35),
    (3.24, 2.05),
    (3.30, 1.70),
]

# Right line: purple, green, container
right_line = [
    (3.30, 1.70),
    (3.30, 1.30),
    (3.30, 0.80),
    (3.30, 0.30),
    (3.30, 0.00),
    (3.30, -0.30),
    (3.30, -0.80),
    (3.30, -1.30),
    (3.30, -1.80),
]

# Bottom-right curve
bottom_right_curve = [
    (3.30, -1.80),
    (3.22, -2.10),
    (3.05, -2.35),
    (2.78, -2.55),
    (2.45, -2.70),
    (2.20, -2.80),
]

# Bottom line: orange
bottom_line = [
    (2.20, -2.80),
    (1.50, -2.80),
    (0.80, -2.80),
    (0.00, -2.80),
    (-0.80, -2.80),
    (-1.50, -2.80),
    (-2.20, -2.80),
]

# Bottom-left curve into real intersection
bottom_left_curve = [
    (-2.20, -2.80),
    (-2.45, -2.70),
    (-2.68, -2.50),
    (-2.88, -2.22),
    (-3.05, -1.88),
    (-3.18, -1.45),
    (-3.25, -1.05),
]

# Left line: yellow
left_line = [
    (-3.25, -1.05),
    (-3.25, -0.60),
    (-3.25, 0.00),
    (-3.25, 0.10),
    (-3.25, 0.70),
    (-3.25, 1.30),
    (-3.25, 1.90),
]

# Top-left curve
top_left_curve = [
    (-3.25, 1.90),
    (-3.18, 2.20),
    (-3.02, 2.45),
    (-2.78, 2.65),
    (-2.55, 2.80),
]

# Center cross line
center_cross = [
    (-3.25, 0.00),
    (-2.40, 0.00),
    (-1.60, 0.00),
    (-0.80, 0.00),
    (0.00, 0.00),
    (0.80, 0.00),
    (1.60, 0.00),
    (2.40, 0.00),
    (3.30, 0.00),
]

# Actual lower-left intersection shape
left_lower_intersection_vertical = [
    (-3.25, -1.40),
    (-3.25, -1.05),
    (-3.25, -0.70),
]

left_lower_intersection_branch = [
    (-3.25, -1.05),
    (-2.95, -1.15),
    (-2.65, -1.35),
    (-2.35, -1.65),
    (-2.20, -2.00),
]

# AGV start positions on line center
AGV01_START = (2.20, -2.80)
AGV02_START = (3.30, 1.70)

# Color square centers on black line center
RED_ZONE = (-1.90, 2.80)
BLUE_ZONE = (1.80, 2.80)
YELLOW_ZONE = (-3.25, 0.10)
ORANGE_ZONE = (0.00, -2.80)
PURPLE_ZONE = (3.30, 1.30)
GREEN_ZONE = (3.30, -1.30)

# Container center on right black line center
CONTAINER_ZONE = (3.30, 0.00)

world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="agv_factory_v3_curve_clean_map">

    <include>
      <uri>model://sun</uri>
    </include>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -0.2 10.8 0 1.5708 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    {box_model("factory_floor", 0, 0, FLOOR_Z, 9.5, 8.0, 0.03, FLOOR)}
"""

# Black path
world += polyline_models("line_top", top_line)
world += polyline_models("curve_top_right", top_right_curve)
world += polyline_models("line_right", right_line)
world += polyline_models("curve_bottom_right", bottom_right_curve)
world += polyline_models("line_bottom", bottom_line)
world += polyline_models("curve_bottom_left", bottom_left_curve)
world += polyline_models("line_left", left_line)
world += polyline_models("curve_top_left", top_left_curve)
world += polyline_models("line_center_cross", center_cross)

# Lower-left actual intersection components
world += polyline_models("left_lower_intersection_vertical", left_lower_intersection_vertical)
world += polyline_models("left_lower_intersection_branch", left_lower_intersection_branch)

# Intersection cover blocks to make connection clean
world += box_model("intersection_left_center", -3.25, 0.00, LINE_Z + 0.002, 0.62, 0.62, LINE_HEIGHT, BLACK)
world += box_model("intersection_right_center", 3.30, 0.00, LINE_Z + 0.002, 0.62, 0.62, LINE_HEIGHT, BLACK)
world += box_model("intersection_left_lower", -3.25, -1.05, LINE_Z + 0.002, 0.70, 0.70, LINE_HEIGHT, BLACK)

# Color squares: centers match black line center
SQUARE_SIZE = 0.58
SQUARE_HEIGHT = 0.06
SQUARE_Z = LINE_HEIGHT + SQUARE_HEIGHT / 2

world += box_model("red_square", RED_ZONE[0], RED_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, RED)
world += box_model("blue_square", BLUE_ZONE[0], BLUE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, BLUE)
world += box_model("yellow_square", YELLOW_ZONE[0], YELLOW_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, YELLOW)
world += box_model("orange_square", ORANGE_ZONE[0], ORANGE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, ORANGE)
world += box_model("purple_square", PURPLE_ZONE[0], PURPLE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, PURPLE)
world += box_model("green_square", GREEN_ZONE[0], GREEN_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, GREEN)

# Container / conveyor area: center matches right line center
world += box_model(
    "container_area",
    CONTAINER_ZONE[0],
    CONTAINER_ZONE[1],
    LINE_HEIGHT + 0.08,
    0.78,
    2.45,
    0.10,
    GRAY
)

# AGV models on line center
world += cylinder_model(
    "AGV01",
    AGV01_START[0],
    AGV01_START[1],
    0.16,
    0.28,
    0.22,
    DARK
)

world += cylinder_model(
    "AGV02",
    AGV02_START[0],
    AGV02_START[1],
    0.16,
    0.28,
    0.22,
    DARK
)

# Small white center dots on AGVs
world += cylinder_model(
    "AGV01_center_dot",
    AGV01_START[0],
    AGV01_START[1],
    0.285,
    0.07,
    0.03,
    WHITE
)

world += cylinder_model(
    "AGV02_center_dot",
    AGV02_START[0],
    AGV02_START[1],
    0.285,
    0.07,
    0.03,
    WHITE
)

world += """
  </world>
</sdf>
"""

WORLD_PATH.write_text(world, encoding="utf-8")
print(f"Created: {WORLD_PATH}")
