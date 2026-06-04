import math
from pathlib import Path

WORLD_PATH = Path.home() / "gazebo_agv_ws/src/agv_sim/worlds/agv_factory_v6.world"

LINE_WIDTH = 0.18
LINE_HEIGHT = 0.03
LINE_Z = LINE_HEIGHT / 2
SEGMENT_OVERLAP = 0.05
FLOOR_Z = -0.015


def material(r, g, b, a=1.0):
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""


def box_model(name, x, y, z, sx, sy, sz, color, yaw=0.0, static=True):
    r, g, b, a = color
    static_text = "true" if static else "false"

    return f"""
    <model name="{name}">
      <static>{static_text}</static>
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
{material(r, g, b, a)}
        </visual>
      </link>
    </model>
"""


def cylinder_model(name, x, y, z, radius, length, color, static=False):
    r, g, b, a = color
    static_text = "true" if static else "false"

    return f"""
    <model name="{name}">
      <static>{static_text}</static>
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
{material(r, g, b, a)}
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

    length = math.sqrt(dx * dx + dy * dy) + SEGMENT_OVERLAP
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
        yaw=yaw,
        static=True
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


BLACK = (0.0, 0.0, 0.0, 1.0)
FLOOR = (0.78, 0.78, 0.78, 1.0)

RED = (1.0, 0.0, 0.0, 1.0)
BLUE = (0.0, 0.1, 1.0, 1.0)
YELLOW = (1.0, 0.9, 0.0, 1.0)
ORANGE = (1.0, 0.45, 0.0, 1.0)
PURPLE = (0.55, 0.0, 0.9, 1.0)
GREEN = (0.0, 0.9, 0.15, 1.0)
GRAY = (0.6, 0.6, 0.6, 1.0)
DARK = (0.08, 0.08, 0.08, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)


# -----------------------------
# 기본 외곽 메인 루프
# -----------------------------

top_line = [
    (-2.20, 2.80),
    (-1.80, 2.80),
    (-1.20, 2.80),
    (-0.40, 2.80),
    (0.40, 2.80),
    (1.20, 2.80),
    (1.80, 2.80),
    (2.20, 2.80),
]

top_right_curve = [
    (2.20, 2.80),
    (2.45, 2.76),
    (2.65, 2.66),
    (2.82, 2.50),
    (2.93, 2.30),
    (3.00, 2.05),
    (3.02, 1.80),
]

right_line = [
    (3.02, 1.80),
    (3.02, 1.30),
    (3.02, 0.80),
    (3.02, 0.30),
    (3.02, -0.20),
    (3.02, -0.70),
    (3.02, -1.20),
    (3.02, -1.70),
]

bottom_right_curve = [
    (3.02, -1.70),
    (2.98, -1.98),
    (2.88, -2.22),
    (2.72, -2.42),
    (2.50, -2.58),
    (2.25, -2.72),
    (2.00, -2.80),
]

bottom_line = [
    (2.00, -2.80),
    (1.30, -2.80),
    (0.60, -2.80),
    (0.00, -2.80),
    (-0.60, -2.80),
    (-1.30, -2.80),
    (-2.00, -2.80),
]

bottom_left_curve = [
    (-2.00, -2.80),
    (-2.28, -2.74),
    (-2.52, -2.60),
    (-2.72, -2.40),
    (-2.86, -2.15),
    (-2.95, -1.88),
    (-3.00, -1.55),
]

left_main_vertical = [
    (-3.00, -1.55),
    (-3.00, -1.00),
    (-3.00, -0.45),
    (-3.00, 0.10),
    (-3.00, 0.65),
    (-3.00, 1.20),
    (-3.00, 1.75),
]

top_left_curve = [
    (-3.00, 1.75),
    (-2.95, 2.05),
    (-2.82, 2.32),
    (-2.62, 2.54),
    (-2.38, 2.70),
    (-2.20, 2.80),
]

# -----------------------------
# v6 왼쪽 교차로 수정 부분 1
# 빨간색~노란색 사이 끊긴 부분 연결
# -----------------------------

left_upper_cross = [
    (-2.20, 2.80),
    (-2.45, 2.72),
    (-2.72, 2.52),
    (-3.00, 2.20),
    (-3.20, 1.82),
    (-3.32, 1.40),
    (-3.32, 0.96),
    (-3.20, 0.52),
    (-3.00, 0.10),
]

# -----------------------------
# v6 왼쪽 교차로 수정 부분 2
# 노란색~주황색 사이 엉킨 부분 제거
# -----------------------------

left_lower_cross = [
    (-3.00, 0.10),
    (-2.80, -0.28),
    (-2.68, -0.70),
    (-2.68, -1.12),
    (-2.78, -1.54),
    (-2.95, -1.92),
    (-3.00, -1.98),
]

# -----------------------------
# 오른쪽 안쪽 곡선
# -----------------------------

right_inner_curve = [
    (3.02, 1.05),
    (2.82, 0.86),
    (2.68, 0.60),
    (2.62, 0.30),
    (2.64, -0.02),
    (2.76, -0.32),
    (2.94, -0.58),
    (3.02, -0.78),
]

# -----------------------------
# 색상 구역 / 컨테이너 / AGV
# -----------------------------

RED_ZONE = (-1.80, 2.80)
BLUE_ZONE = (1.20, 2.80)
YELLOW_ZONE = (-3.00, 0.10)
ORANGE_ZONE = (0.00, -2.80)
PURPLE_ZONE = (3.02, 1.05)
GREEN_ZONE = (3.02, -1.25)

CONTAINER_ZONE = (3.02, -0.10)

AGV01_START = (1.80, -2.80)
AGV02_START = (3.02, 1.55)


world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="agv_factory_v6">

    <include>
      <uri>model://sun</uri>
    </include>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -0.2 10.5 0 1.5708 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    {box_model("factory_floor", 0, 0, FLOOR_Z, 9.5, 8.0, 0.03, FLOOR)}
"""

# 메인 루프
world += polyline_models("top_line", top_line)
world += polyline_models("top_right_curve", top_right_curve)
world += polyline_models("right_line", right_line)
world += polyline_models("bottom_right_curve", bottom_right_curve)
world += polyline_models("bottom_line", bottom_line)
world += polyline_models("bottom_left_curve", bottom_left_curve)
world += polyline_models("left_main_vertical", left_main_vertical)
world += polyline_models("top_left_curve", top_left_curve)

# v6 왼쪽 교차로
world += polyline_models("left_upper_cross", left_upper_cross)
world += polyline_models("left_lower_cross", left_lower_cross)

# 오른쪽 안쪽 곡선
world += polyline_models("right_inner_curve", right_inner_curve)

# 색상 정사각형
SQUARE_SIZE = 0.58
SQUARE_HEIGHT = 0.06
SQUARE_Z = LINE_HEIGHT + SQUARE_HEIGHT / 2

world += box_model("red_square", RED_ZONE[0], RED_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, RED)
world += box_model("blue_square", BLUE_ZONE[0], BLUE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, BLUE)
world += box_model("yellow_square", YELLOW_ZONE[0], YELLOW_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, YELLOW)
world += box_model("orange_square", ORANGE_ZONE[0], ORANGE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, ORANGE)
world += box_model("purple_square", PURPLE_ZONE[0], PURPLE_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, PURPLE)
world += box_model("green_square", GREEN_ZONE[0], GREEN_ZONE[1], SQUARE_Z, SQUARE_SIZE, SQUARE_SIZE, SQUARE_HEIGHT, GREEN)

# 컨테이너
world += box_model(
    "container_area",
    CONTAINER_ZONE[0],
    CONTAINER_ZONE[1],
    LINE_HEIGHT + 0.08,
    0.72,
    2.20,
    0.10,
    GRAY
)

# AGV
world += cylinder_model("AGV01", AGV01_START[0], AGV01_START[1], 0.16, 0.22, 0.20, DARK)
world += cylinder_model("AGV02", AGV02_START[0], AGV02_START[1], 0.16, 0.22, 0.20, DARK)

world += cylinder_model("AGV01_dot", AGV01_START[0], AGV01_START[1], 0.265, 0.05, 0.03, WHITE)
world += cylinder_model("AGV02_dot", AGV02_START[0], AGV02_START[1], 0.265, 0.05, 0.03, WHITE)

world += """
  </world>
</sdf>
"""

WORLD_PATH.write_text(world, encoding="utf-8")
print(f"Created: {WORLD_PATH}")