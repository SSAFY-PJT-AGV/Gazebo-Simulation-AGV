import math
from pathlib import Path

WORLD_PATH = Path.home() / "gazebo_agv_ws/src/agv_sim/worlds/agv_factory_v4.world"

# =========================
# v4 핵심 설정값 - 라인 기본값
# =========================
LINE_WIDTH = 0.26          # v3보다 넓게 수정
LINE_HEIGHT = 0.035
LINE_Z = LINE_HEIGHT / 2

SEGMENT_OVERLAP = 0.08     # 선분 사이 빈틈 방지용 겹침 길이
FLOOR_Z = -0.015

# 색을 만들어주는 함수
def material(r, g, b, a=1.0):
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""

#직육면체 모델 박스를 만들어주는 함수
def box_model(name, x, y, z, sx, sy, sz, color, yaw=0, static=True):
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

#원기둥 모델 만들어주는 함수
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

#두 좌표 p1, p2를 검은색 직사각형 라인으로 이어주는 함수
def segment_model(name, p1, p2, width=LINE_WIDTH, height=LINE_HEIGHT, color=(0, 0, 0, 1)):
    x1, y1 = p1
    x2, y2 = p2

    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    dx = x2 - x1
    dy = y2 - y1

    length = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)

    # v4 핵심: 선분을 조금 더 길게 만들어 연결부 빈틈을 줄임
    length = length + SEGMENT_OVERLAP

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

#여러 좌표를 순서대로 이어서 라인을 여러 개 만들기
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


# =========================
# 색상값 설정
# =========================
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


# =========================
# v4 MAP 좌표 기준
# =========================

# 상단 라인: 빨강 / 파랑
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

# 오른쪽 위 곡선: 중간 좌표 추가
top_right_curve = [
    (2.45, 2.80),
    (2.58, 2.79),
    (2.72, 2.75),
    (2.86, 2.68),
    (2.98, 2.57),
    (3.08, 2.43),
    (3.17, 2.25),
    (3.24, 2.05),
    (3.28, 1.86),
    (3.30, 1.70),
]

# 오른쪽 라인: 보라 / 초록 / 컨테이너
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

# 오른쪽 아래 곡선: 중간 좌표 추가
bottom_right_curve = [
    (3.30, -1.80),
    (3.28, -1.98),
    (3.22, -2.16),
    (3.12, -2.32),
    (2.98, -2.46),
    (2.82, -2.58),
    (2.62, -2.68),
    (2.40, -2.75),
    (2.20, -2.80),
]

# 하단 라인: 주황
bottom_line = [
    (2.20, -2.80),
    (1.50, -2.80),
    (0.80, -2.80),
    (0.00, -2.80),
    (-0.80, -2.80),
    (-1.50, -2.80),
    (-2.15, -2.80),
]

# 왼쪽 아래 곡선: 중간 좌표 추가
bottom_left_curve = [
    (-2.15, -2.80),
    (-2.35, -2.78),
    (-2.55, -2.70),
    (-2.73, -2.58),
    (-2.88, -2.42),
    (-3.00, -2.22),
    (-3.10, -1.98),
    (-3.18, -1.70),
    (-3.23, -1.40),
    (-3.25, -1.10),
]

# 왼쪽 세로 메인 라인
left_line = [
    (-3.25, -1.10),
    (-3.25, -0.70),
    (-3.25, -0.30),
    (-3.25, 0.10),
    (-3.25, 0.50),
    (-3.25, 0.90),
    (-3.25, 1.30),
    (-3.25, 1.70),
    (-3.25, 1.95),
]

# 왼쪽 위 곡선: 중간 좌표 추가
top_left_curve = [
    (-3.25, 1.95),
    (-3.22, 2.15),
    (-3.15, 2.32),
    (-3.03, 2.48),
    (-2.88, 2.61),
    (-2.72, 2.71),
    (-2.55, 2.80),
]

# 중앙 가로 라인
center_cross = [
    (-3.25, 0.10),
    (-2.70, 0.10),
    (-2.10, 0.10),
    (-1.50, 0.10),
    (-0.90, 0.10),
    (-0.30, 0.10),
    (0.30, 0.10),
    (0.90, 0.10),
    (1.50, 0.10),
    (2.10, 0.10),
    (2.70, 0.10),
    (3.30, 0.10),
]

# =========================
# v4 왼쪽 교차로 구현
# 왼쪽에 S자형 교차 라인 추가
# =========================

# 왼쪽 바깥쪽에서 상단 라인과 연결되는 곡선
left_outer_intersection_upper = [
    (-2.95, 2.55),
    (-3.20, 2.45),
    (-3.45, 2.25),
    (-3.60, 1.95),
    (-3.62, 1.60),
    (-3.52, 1.25),
    (-3.34, 0.95),
    (-3.25, 0.65),
]

# 노란색 근처에서 안쪽으로 S자처럼 내려오는 교차 라인
left_s_intersection = [
    (-3.25, 0.65),
    (-3.08, 0.45),
    (-2.95, 0.20),
    (-2.92, -0.08),
    (-2.98, -0.36),
    (-3.12, -0.66),
    (-3.25, -0.95),
]

# 왼쪽 아래에서 하단 곡선으로 합류하는 구간
left_outer_intersection_lower = [
    (-3.25, -0.95),
    (-3.43, -1.25),
    (-3.55, -1.60),
    (-3.52, -1.95),
    (-3.37, -2.25),
    (-3.10, -2.48),
    (-2.75, -2.62),
]


# =========================
# AGV, 정사각형, 컨테이너 좌표
# =========================

AGV01_START = (2.20, -2.80)
AGV02_START = (3.30, 1.70)

RED_ZONE = (-1.90, 2.80)
BLUE_ZONE = (1.80, 2.80)
YELLOW_ZONE = (-3.25, 0.10)
ORANGE_ZONE = (0.00, -2.80)
PURPLE_ZONE = (3.30, 1.30)
GREEN_ZONE = (3.30, -1.30)

CONTAINER_ZONE = (3.30, 0.00)


# =========================
# World 파일 생성하기
# =========================

world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="agv_factory_v4_smooth_intersection">
  
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

# 기본 외곽 라인
world += polyline_models("line_top", top_line)
world += polyline_models("curve_top_right", top_right_curve)
world += polyline_models("line_right", right_line)
world += polyline_models("curve_bottom_right", bottom_right_curve)
world += polyline_models("line_bottom", bottom_line)
world += polyline_models("curve_bottom_left", bottom_left_curve)
world += polyline_models("line_left", left_line)
world += polyline_models("curve_top_left", top_left_curve)

# 중앙 라인
world += polyline_models("line_center_cross", center_cross)

# v4 왼쪽 교차로 라인
world += polyline_models("left_outer_intersection_upper", left_outer_intersection_upper)
world += polyline_models("left_s_intersection", left_s_intersection)
world += polyline_models("left_outer_intersection_lower", left_outer_intersection_lower)

# 교차점 연결부만 작게 보강
# v3처럼 큰 검정 정사각형을 두지 않고, 연결이 끊기는 부분만 작은 원형 느낌으로 보강
world += cylinder_model("left_upper_joint", -3.25, 0.65, LINE_Z + 0.003, 0.18, LINE_HEIGHT, BLACK, static=True)
world += cylinder_model("left_lower_joint", -3.25, -0.95, LINE_Z + 0.003, 0.18, LINE_HEIGHT, BLACK, static=True)
world += cylinder_model("left_center_joint", -3.25, 0.10, LINE_Z + 0.003, 0.18, LINE_HEIGHT, BLACK, static=True)
world += cylinder_model("right_center_joint", 3.30, 0.10, LINE_Z + 0.003, 0.18, LINE_HEIGHT, BLACK, static=True)

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
    0.78,
    2.45,
    0.10,
    GRAY
)

# AGV 모델
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

# AGV 중심 확인용 흰색 점
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