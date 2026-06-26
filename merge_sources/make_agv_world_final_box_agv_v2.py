import math
from pathlib import Path

# 최종 사진 기준 Gazebo MAP 재생성 파일
# 실행하면 ~/gazebo_agv_ws/src/agv_sim/worlds/agv_factory_v7_rebuild.world 생성
WORLD_PATH = Path.home() / "gazebo_agv_ws/src/agv_sim/worlds/agv_factory_final_box_agv_v2.world"

LINE_WIDTH = 0.22
LINE_HEIGHT = 0.035
LINE_Z = LINE_HEIGHT / 2
SEGMENT_OVERLAP = 0.06
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



def box_model_rpy(name, x, y, z, sx, sy, sz, color, roll=0.0, pitch=0.0, yaw=0.0, static=True):
    r, g, b, a = color
    static_text = "true" if static else "false"
    return f"""
    <model name="{name}">
      <static>{static_text}</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
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




def tray_storage_box_model(name, x, y, material_color, filled_slots=None, yaw=0.0, static=True):
    """실제 자재보관상자 형태의 트레이 모델.

    - 트레이 자체는 회색으로 표시
    - 내부 원형 자리 4개를 어두운 원형 디스크로 표시
    - filled_slots에 지정된 자리만 작은 큐브 자재로 표시
    - 중앙 사각 기둥은 AGV가 잡고 드는 손잡이 역할로 표시

    slot 이름:
      NW: 왼쪽 위, NE: 오른쪽 위, SW: 왼쪽 아래, SE: 오른쪽 아래
    """
    if filled_slots is None:
        filled_slots = []

    static_text = "true" if static else "false"
    tr, tg, tb, ta = TRAY_GRAY
    er, eg, eb, ea = TRAY_EDGE
    wr, wg, wb, wa = TRAY_WELL
    pr, pg, pb, pa = TRAY_POST
    mr, mg, mb, ma = material_color

    tray_size = 0.5200
    tray_base_h = 0.0450
    rim_h = 0.0850
    rim_w = 0.0400
    well_radius = 0.0750
    well_h = 0.0100
    slot_offset = 0.1500
    material_cube_size = 0.1050
    material_cube_h = 0.1050
    post_size = 0.1400
    post_h = 0.1700

    # 상대 좌표 기준: +x 오른쪽, +y 위쪽
    slot_positions = {
        "NW": (-slot_offset, slot_offset),
        "NE": (slot_offset, slot_offset),
        "SW": (-slot_offset, -slot_offset),
        "SE": (slot_offset, -slot_offset),
    }

    visuals = f"""
        <collision name="base_collision">
          <pose>0 0 {tray_base_h / 2:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{tray_size:.4f} {tray_size:.4f} {tray_base_h:.4f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="tray_base">
          <pose>0 0 {tray_base_h / 2:.4f} 0 0 0</pose>
          <geometry>
            <box>
              <size>{tray_size:.4f} {tray_size:.4f} {tray_base_h:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{tr} {tg} {tb} {ta}</ambient>
            <diffuse>{tr} {tg} {tb} {ta}</diffuse>
          </material>
        </visual>

        <visual name="rim_top">
          <pose>0 {tray_size / 2 - rim_w / 2:.4f} {tray_base_h + rim_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{tray_size:.4f} {rim_w:.4f} {rim_h:.4f}</size></box></geometry>
          <material><ambient>{er} {eg} {eb} {ea}</ambient><diffuse>{er} {eg} {eb} {ea}</diffuse></material>
        </visual>
        <visual name="rim_bottom">
          <pose>0 {-tray_size / 2 + rim_w / 2:.4f} {tray_base_h + rim_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{tray_size:.4f} {rim_w:.4f} {rim_h:.4f}</size></box></geometry>
          <material><ambient>{er} {eg} {eb} {ea}</ambient><diffuse>{er} {eg} {eb} {ea}</diffuse></material>
        </visual>
        <visual name="rim_left">
          <pose>{-tray_size / 2 + rim_w / 2:.4f} 0 {tray_base_h + rim_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{rim_w:.4f} {tray_size:.4f} {rim_h:.4f}</size></box></geometry>
          <material><ambient>{er} {eg} {eb} {ea}</ambient><diffuse>{er} {eg} {eb} {ea}</diffuse></material>
        </visual>
        <visual name="rim_right">
          <pose>{tray_size / 2 - rim_w / 2:.4f} 0 {tray_base_h + rim_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{rim_w:.4f} {tray_size:.4f} {rim_h:.4f}</size></box></geometry>
          <material><ambient>{er} {eg} {eb} {ea}</ambient><diffuse>{er} {eg} {eb} {ea}</diffuse></material>
        </visual>

        <collision name="center_post_collision">
          <pose>0 0 {tray_base_h + post_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{post_size:.4f} {post_size:.4f} {post_h:.4f}</size></box></geometry>
        </collision>
        <visual name="center_post">
          <pose>0 0 {tray_base_h + post_h / 2:.4f} 0 0 0</pose>
          <geometry><box><size>{post_size:.4f} {post_size:.4f} {post_h:.4f}</size></box></geometry>
          <material><ambient>{pr} {pg} {pb} {pa}</ambient><diffuse>{pr} {pg} {pb} {pa}</diffuse></material>
        </visual>
"""

    for slot_name, (sx, sy) in slot_positions.items():
        visuals += f"""
        <visual name="well_{slot_name}">
          <pose>{sx:.4f} {sy:.4f} {tray_base_h + well_h / 2 + 0.0020:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{well_radius:.4f}</radius>
              <length>{well_h:.4f}</length>
            </cylinder>
          </geometry>
          <material><ambient>{wr} {wg} {wb} {wa}</ambient><diffuse>{wr} {wg} {wb} {wa}</diffuse></material>
        </visual>
"""

        if slot_name in filled_slots:
            visuals += f"""
        <collision name="material_{slot_name}_collision">
          <pose>{sx:.4f} {sy:.4f} {tray_base_h + material_cube_h / 2 + 0.0150:.4f} 0 0 0</pose>
          <geometry><box><size>{material_cube_size:.4f} {material_cube_size:.4f} {material_cube_h:.4f}</size></box></geometry>
        </collision>
        <visual name="material_{slot_name}">
          <pose>{sx:.4f} {sy:.4f} {tray_base_h + material_cube_h / 2 + 0.0150:.4f} 0 0 0</pose>
          <geometry><box><size>{material_cube_size:.4f} {material_cube_size:.4f} {material_cube_h:.4f}</size></box></geometry>
          <material><ambient>{mr} {mg} {mb} {ma}</ambient><diffuse>{mr} {mg} {mb} {ma}</diffuse></material>
        </visual>
"""

    return f"""
    <model name="{name}">
      <static>{static_text}</static>
      <pose>{x:.4f} {y:.4f} 0.0000 0 0 {yaw:.4f}</pose>
      <link name="link">
{visuals}
      </link>
    </model>
"""

def agv_box_model(name, x, y, z, body_color, top_color, yaw=0.0):
    """간단한 박스형 AGV 모델.
    모델 이름은 AGV01 / AGV02 그대로 유지해서 이동 스크립트에서 바로 제어할 수 있게 함.
    +x 방향에 흰색 전면 표시를 둬서 이동 방향을 구분함.
    """
    br, bg, bb, ba = body_color
    tr, tg, tb, ta = top_color
    wr, wg, wb, wa = WHITE
    kr, kg, kb, ka = BLACK
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="base_link">
        <collision name="body_collision">
          <geometry>
            <box>
              <size>0.8500 0.6000 0.1800</size>
            </box>
          </geometry>
        </collision>

        <visual name="body_visual">
          <geometry>
            <box>
              <size>0.8500 0.6000 0.1800</size>
            </box>
          </geometry>
          <material>
            <ambient>{br} {bg} {bb} {ba}</ambient>
            <diffuse>{br} {bg} {bb} {ba}</diffuse>
          </material>
        </visual>

        <visual name="front_direction_marker">
          <pose>0.3800 0 0.1050 0 0 0</pose>
          <geometry>
            <box>
              <size>0.0900 0.4600 0.0300</size>
            </box>
          </geometry>
          <material>
            <ambient>{wr} {wg} {wb} {wa}</ambient>
            <diffuse>{wr} {wg} {wb} {wa}</diffuse>
          </material>
        </visual>

        <visual name="top_id_marker">
          <pose>0 0 0.1150 0 0 0</pose>
          <geometry>
            <box>
              <size>0.4000 0.2800 0.0300</size>
            </box>
          </geometry>
          <material>
            <ambient>{tr} {tg} {tb} {ta}</ambient>
            <diffuse>{tr} {tg} {tb} {ta}</diffuse>
          </material>
        </visual>

        <visual name="left_front_wheel">
          <pose>0.2900 0.3450 -0.0550 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1700 0.0750 0.0700</size>
            </box>
          </geometry>
          <material>
            <ambient>{kr} {kg} {kb} {ka}</ambient>
            <diffuse>{kr} {kg} {kb} {ka}</diffuse>
          </material>
        </visual>

        <visual name="left_rear_wheel">
          <pose>-0.2900 0.3450 -0.0550 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1700 0.0750 0.0700</size>
            </box>
          </geometry>
          <material>
            <ambient>{kr} {kg} {kb} {ka}</ambient>
            <diffuse>{kr} {kg} {kb} {ka}</diffuse>
          </material>
        </visual>

        <visual name="right_front_wheel">
          <pose>0.2900 -0.3450 -0.0550 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1700 0.0750 0.0700</size>
            </box>
          </geometry>
          <material>
            <ambient>{kr} {kg} {kb} {ka}</ambient>
            <diffuse>{kr} {kg} {kb} {ka}</diffuse>
          </material>
        </visual>

        <visual name="right_rear_wheel">
          <pose>-0.2900 -0.3450 -0.0550 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1700 0.0750 0.0700</size>
            </box>
          </geometry>
          <material>
            <ambient>{kr} {kg} {kb} {ka}</ambient>
            <diffuse>{kr} {kg} {kb} {ka}</diffuse>
          </material>
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


def bezier_points(p0, p1, p2, p3, steps=16):
    pts = []
    for i in range(steps + 1):
        t = i / steps
        x = ((1 - t) ** 3) * p0[0] + 3 * ((1 - t) ** 2) * t * p1[0] + 3 * (1 - t) * (t ** 2) * p2[0] + (t ** 3) * p3[0]
        y = ((1 - t) ** 3) * p0[1] + 3 * ((1 - t) ** 2) * t * p1[1] + 3 * (1 - t) * (t ** 2) * p2[1] + (t ** 3) * p3[1]
        pts.append((x, y))
    return pts


def join_paths(*paths):
    result = []
    for path in paths:
        if not path:
            continue
        if result and result[-1] == path[0]:
            result.extend(path[1:])
        else:
            result.extend(path)
    return result


# -----------------------------
# 색상
# -----------------------------
BLACK = (0.0, 0.0, 0.0, 1.0)
FLOOR = (0.86, 0.86, 0.86, 1.0)
RED = (1.0, 0.0, 0.0, 1.0)
BLUE = (0.0, 0.12, 1.0, 1.0)
YELLOW = (1.0, 0.92, 0.0, 1.0)
ORANGE = (1.0, 0.45, 0.0, 1.0)
PURPLE = (0.45, 0.1, 0.75, 1.0)
GREEN = (0.0, 0.9, 0.15, 1.0)
TEAL = (0.10, 0.52, 0.48, 1.0)
RAMP_BLUE = (0.38, 0.48, 1.0, 0.85)
GRAY = (0.45, 0.45, 0.45, 1.0)
DARK = (0.08, 0.08, 0.08, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)

TRAY_GRAY = (0.55, 0.57, 0.54, 1.0)
TRAY_EDGE = (0.18, 0.18, 0.18, 1.0)
TRAY_WELL = (0.30, 0.32, 0.30, 1.0)
TRAY_POST = (0.48, 0.49, 0.46, 1.0)

# -----------------------------
# 마커/경로 좌표표
# 서버는 current_marker / next_marker만 주고,
# Gazebo 이동 스크립트는 이 좌표표를 사용하면 됨.
# -----------------------------
MARKER_POSITIONS = {
    # 상단 라인
    "QR_TOP_01": (-2.65, 2.25),
    "QR_TOP_02": (-1.55, 2.25),
    "QR_TOP_03": (-0.35, 2.25),
    "QR_TOP_04": (0.85, 2.25),
    "QR_TOP_05": (2.05, 2.25),

    # 오른쪽 세로 라인 / 입출고 쪽
    "QR_RIGHT_01": (3.05, 2.55),
    "QR_RIGHT_02": (3.05, 1.45),
    "QR_RIGHT_03": (3.05, 0.35),
    "QR_RIGHT_04": (3.05, -0.75),
    "QR_RIGHT_05": (3.05, -1.85),

    # 하단 라인
    "QR_BOTTOM_01": (2.20, -2.55),
    "QR_BOTTOM_02": (1.10, -2.55),
    "QR_BOTTOM_03": (0.00, -2.55),
    "QR_BOTTOM_04": (-1.10, -2.55),
    "QR_BOTTOM_05": (-2.20, -2.55),

    # 왼쪽 컨베이어/경사로 라인
    "QR_LEFT_01": (-3.35, 1.45),
    "QR_LEFT_02": (-3.35, 0.55),
    "QR_LEFT_03": (-3.35, -0.35),
    "QR_LEFT_04": (-3.35, -1.25),
    "QR_LEFT_05": (-3.20, -2.05),

    # 오른쪽 S자 곡선 보조 경로
    "QR_S_01": (2.45, 1.70),
    "QR_S_02": (2.55, 0.50),
    "QR_S_03": (3.35, -1.25),
    "QR_S_04": (2.70, -2.25),
}

# -----------------------------
# 최종 사진 기준 주행 라인
# -----------------------------

# 왼쪽 큰 루프: 하단 → 왼쪽 컨베이어 → 상단 → 오른쪽 S자 → 하단
bottom_line = [
    (2.20, -2.55),
    (1.55, -2.55),
    (0.80, -2.55),
    (0.00, -2.55),
    (-0.80, -2.55),
    (-1.55, -2.55),
    (-2.20, -2.55),
]

bottom_left_curve = bezier_points(
    (-2.20, -2.55),
    (-3.10, -2.50),
    (-3.35, -2.05),
    (-3.35, -1.45),
    steps=14
)

left_vertical = [
    (-3.35, -1.45),
    (-3.35, -0.95),
    (-3.35, -0.45),
    (-3.35, 0.05),
    (-3.35, 0.55),
    (-3.35, 1.05),
    (-3.35, 1.45),
]

top_left_curve = bezier_points(
    (-3.35, 1.45),
    (-3.35, 2.00),
    (-3.05, 2.25),
    (-2.65, 2.25),
    steps=12
)

top_line = [
    (-2.65, 2.25),
    (-1.90, 2.25),
    (-1.15, 2.25),
    (-0.35, 2.25),
    (0.45, 2.25),
    (1.25, 2.25),
    (2.05, 2.25),
]

# 오른쪽 교차로 곡선
# v6의 교차로 방식처럼 “기준 세로 라인 + 별도 곡선 라인”이 함께 보이도록 구성.
# 노란색 박스가 중앙을 가리고, 위쪽에는 세로 라인/곡선 라인 2개,
# 아래쪽에도 세로 라인/곡선 라인 2개가 드러나도록 좌표를 분리함.
# cross_fixed2: 주황색/초록색 동그라미 부분에서 세로 라인과 곡선 라인이 만나도록 연결선 추가.
right_s_curve_upper = [
    # fixed10: 사용자가 빨간색으로 표시한 상단 곡선 가이드만 반영.
    # 다른 구조는 건드리지 않고, 상단 라인에서 오른쪽 세로 라인으로 들어가는 곡선만 수정.
    (2.05, 2.25),
    (2.24, 2.25),
    (2.42, 2.22),
    (2.58, 2.15),
    (2.72, 2.03),
    (2.84, 1.88),
    (2.94, 1.72),
    (3.05, 1.62),
    (2.91, 1.56),
    (2.76, 1.40),
    (2.64, 1.20),
    (2.58, 0.98),
    (2.60, 0.76),
    (2.72, 0.54),
    (2.88, 0.32),
    (3.00, 0.08),
    (3.05, -0.55),
]

right_s_curve_lower = [
    (3.05, -0.95),
    (3.25, -1.10),
    (3.46, -1.35),
    (3.55, -1.68),
    (3.50, -1.98),
    (3.32, -2.22),
    (3.04, -2.40),
    (2.68, -2.50),
    (2.20, -2.55),
]

main_loop = join_paths(
    bottom_line,
    bottom_left_curve,
    left_vertical,
    top_left_curve,
    top_line,
    right_s_curve_upper,
    right_s_curve_lower,
)

# 입출고구역으로 올라가는 오른쪽 세로 라인
right_vertical = [
    (3.05, -1.90),
    (3.05, -1.35),
    (3.05, -0.80),
    (3.05, -0.25),
    (3.05, 0.30),
    (3.05, 0.85),
    (3.05, 1.40),
    (3.05, 1.95),
    (3.05, 2.50),
    (3.05, 3.10),
    (3.05, 3.55),
    (3.05, 3.90),
    (3.05, 4.45),
]

# 주황색 동그라미 부분: 상단 라인과 오른쪽 세로 라인이 만나도록 연결
right_top_join = [
    (2.05, 2.25),
    (3.05, 2.25),
]

# 초록색 동그라미 부분: 오른쪽 세로 라인 하단과 하단 곡선이 만나도록 연결
right_bottom_join = [
    (3.05, -1.90),
    (3.05, -2.40),
]

# 왼쪽 컨베이어 쪽에 사진처럼 살짝 튀어나오는 짧은 곡선
left_inner_curve = bezier_points(
    (-3.35, -0.30),
    (-2.90, -0.45),
    (-2.90, -1.20),
    (-3.35, -1.45),
    steps=12
)

# -----------------------------
# 주요 구역 좌표
# -----------------------------
INOUT_ZONE = (3.05, 4.35)
CONVEYOR_ZONE = (-3.35, -0.45)
RAMP_ZONE = (-3.35, 0.75)

# 큰 색상 구역: 사진 속 라인 중심 기준
ORANGE_ZONE = (-0.35, 2.25)    # AGV02_START 근처
BLUE_LARGE_ZONE = (-1.45, -2.55)
RED_LARGE_ZONE = (1.25, -2.55)
PURPLE_LARGE_ZONE = (-3.35, -1.65)
YELLOW_LARGE_ZONE = (3.05, -0.30)

# 중앙 보관 구역: 사진 기준, 라인과 분리
FINISHED_BOXES = [
    ("finished_blue_box", -1.45, 1.45, BLUE),
    ("finished_yellow_box", -0.15, 1.45, YELLOW),
    ("finished_red_box", 1.15, 1.45, RED),
]

MATERIAL_BOXES = [
    ("material_blue_box", -1.45, -1.55, BLUE),
    ("material_yellow_box", -0.15, -1.55, YELLOW),
    ("material_red_box", 1.15, -1.55, RED),
]

# 입출고구역 부근 출고 박스 3개
INOUT_STACK_BOXES = [
    ("inout_red_box", 3.70, 3.35, RED),
    ("inout_yellow_box", 3.70, 2.05, YELLOW),
    ("inout_blue_box", 3.70, 2.70, BLUE),
]

# 출고 박스 3개 건너편에 둘 입고 박스 1개 위치
INBOUND_BOX_CENTER = (2.25, 3.35)

AGV01_START = (2.20, -2.55)
AGV02_START = (-1.35, 2.25)


world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="agv_factory_final_box_agv_v2">

    <include>
      <uri>model://sun</uri>
    </include>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 0 10.8 0 1.5708 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    {box_model("factory_floor", 0, 0.55, FLOOR_Z, 9.8, 9.3, 0.03, FLOOR)}
"""

# 주행 라인
world += polyline_models("main_loop", main_loop, width=LINE_WIDTH)
world += polyline_models("right_vertical", right_vertical, width=LINE_WIDTH)
# pink-marked area: right_top_join removed so this part is not connected
# world += polyline_models("right_top_join", right_top_join, width=LINE_WIDTH)
world += polyline_models("right_bottom_join", right_bottom_join, width=LINE_WIDTH)
world += polyline_models("left_inner_curve", left_inner_curve, width=LINE_WIDTH)

# 큰 구역 박스
BIG_SQUARE = 1.05
COLOR_BOX_HEIGHT = 0.020
COLOR_BOX_Z = COLOR_BOX_HEIGHT / 2

# 색상 정사각형은 검은색 라인보다 낮게 배치해서 라인이 위에 보이도록 처리
world += box_model("orange_agv02_start_area", ORANGE_ZONE[0], ORANGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, ORANGE)
world += box_model("blue_bottom_area", BLUE_LARGE_ZONE[0], BLUE_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, BLUE)
world += box_model("red_agv01_start_area", RED_LARGE_ZONE[0], RED_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, RED)
world += box_model("purple_left_area", PURPLE_LARGE_ZONE[0], PURPLE_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, PURPLE)
world += box_model("yellow_buffer_area", YELLOW_LARGE_ZONE[0], YELLOW_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, YELLOW)
world += box_model("green_left_square", RAMP_ZONE[0], RAMP_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, GREEN)

# 입출고구역: 검정색 라인보다 위에 배치해서 겹칠 경우 라인이 가려지도록 처리
INOUT_BOX_HEIGHT = 0.060
INOUT_BOX_Z = LINE_HEIGHT + INOUT_BOX_HEIGHT / 2
world += box_model("inout_area", INOUT_ZONE[0], INOUT_ZONE[1], INOUT_BOX_Z, 1.25, 1.25, INOUT_BOX_HEIGHT, TEAL)

# 컨베이어 및 경사로
# 컨베이어 색상은 최종 MAP 요청에 맞춰 회색으로 변경
CONVEYOR_HEIGHT = 0.10
CONVEYOR_Z = LINE_HEIGHT + CONVEYOR_HEIGHT / 2
world += box_model("conveyor_area", CONVEYOR_ZONE[0], CONVEYOR_ZONE[1], CONVEYOR_Z, 0.72, 2.20, CONVEYOR_HEIGHT, GRAY)

# 경사로: 컨베이어와 닿는 아래쪽(-y)은 높고, 반대편 위쪽(+y)은 바닥 높이에 가까워지도록 기울임
RAMP_LENGTH_Y = 0.65
RAMP_THICKNESS = 0.045
RAMP_HIGH_TOP_Z = LINE_HEIGHT + CONVEYOR_HEIGHT
RAMP_LOW_TOP_Z = LINE_HEIGHT
RAMP_ROLL = -math.asin((RAMP_HIGH_TOP_Z - RAMP_LOW_TOP_Z) / RAMP_LENGTH_Y)
RAMP_Z = ((RAMP_HIGH_TOP_Z + RAMP_LOW_TOP_Z) / 2) - (RAMP_THICKNESS / 2)
world += box_model_rpy("ramp_area", RAMP_ZONE[0], RAMP_ZONE[1], RAMP_Z, 0.80, RAMP_LENGTH_Y, RAMP_THICKNESS, RAMP_BLUE, roll=RAMP_ROLL)

# 중앙 보관 구역 / 입출고구역의 작은 네모 박스만 실제 트레이형 자재보관상자로 교체
# 요청사항 외의 검은 라인, AGV, 컨베이어, 경사로, 큰 구역 박스는 수정하지 않음.

# 완제품 상자 보관 구역
# - 파란색 위치: AGV02와 가장 가까운 NE 자리만 비우고 3/4 채움
# - 노란색/빨간색 위치: 0/4 비움
world += tray_storage_box_model("finished_blue_box", -1.45, 1.45, BLUE, filled_slots=["NW", "SW", "SE"])
world += tray_storage_box_model("finished_yellow_box", -0.15, 1.45, YELLOW, filled_slots=[])
world += tray_storage_box_model("finished_red_box", 1.15, 1.45, RED, filled_slots=[])

# 자재 상자 보관 구역
# - 파란색/노란색 위치: 4/4 채움
# - 빨간색 위치: AGV01과 가장 가까운 SE 자리만 1/4 채움
world += tray_storage_box_model("material_blue_box", -1.45, -1.55, BLUE, filled_slots=["NW", "NE", "SW", "SE"])
world += tray_storage_box_model("material_yellow_box", -0.15, -1.55, YELLOW, filled_slots=["NW", "NE", "SW", "SE"])
world += tray_storage_box_model("material_red_box", 1.15, -1.55, RED, filled_slots=["SE"])

# 입출고구역 출고 부분: 빨간색/노란색/파란색 위치 모두 4/4 채움
world += tray_storage_box_model("inout_red_box", 3.70, 3.35, RED, filled_slots=["NW", "NE", "SW", "SE"])
world += tray_storage_box_model("inout_yellow_box", 3.70, 2.05, YELLOW, filled_slots=["NW", "NE", "SW", "SE"])
world += tray_storage_box_model("inout_blue_box", 3.70, 2.70, BLUE, filled_slots=["NW", "NE", "SW", "SE"])

# 입출고구역 입고 부분: 0/4 비어 있는 트레이형 상자
world += tray_storage_box_model("inbound_box", INBOUND_BOX_CENTER[0], INBOUND_BOX_CENTER[1], RED, filled_slots=[])

# AGV 모델: 박스형 AGV 크기 확대
# 모델 이름은 AGV01 / AGV02 그대로 유지함.
AGV_BODY = (0.18, 0.18, 0.18, 1.0)
world += agv_box_model("AGV01", AGV01_START[0], AGV01_START[1], 0.16, AGV_BODY, RED)
world += agv_box_model("AGV02", AGV02_START[0], AGV02_START[1], 0.16, AGV_BODY, BLUE)

world += """
  </world>
</sdf>
"""

WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
WORLD_PATH.write_text(world, encoding="utf-8")
print(f"Created: {WORLD_PATH}")
