import math
from pathlib import Path

# 최종 시작 MAP 기준 Gazebo world 재생성 파일
# 실행하면 ~/gazebo_agv_ws/src/agv_sim/worlds/agv_factory_final_box_agv_v2.world 생성
WORLD_PATH = Path.home() / "gazebo_agv_ws/src/agv_sim/worlds/agv_factory_final_box_agv_v2.world"

LINE_WIDTH = 0.22
LINE_HEIGHT = 0.035
LINE_Z = LINE_HEIGHT / 2
SEGMENT_OVERLAP = 0.06
FLOOR_Z = -0.015

# 메인 MAP 가로 길이를 줄이기 위한 x좌표 압축 비율.
# 입출고구역으로 올라가는 세로 라인 길이는 y축 방향이라 이 값의 영향을 받지 않음.
X_SCALE = 0.86

def scale_x_coord(x):
    return x * X_SCALE


# -----------------------------
# 기본 SDF 모델 생성 함수
# -----------------------------
def material(r, g, b, a=1.0):
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""


def box_model(name, x, y, z, sx, sy, sz, color, yaw=0.0, static=True, apply_x_scale=True):
    r, g, b, a = color
    static_text = "true" if static else "false"
    x_out = scale_x_coord(x) if apply_x_scale else x
    return f"""
    <model name="{name}">
      <static>{static_text}</static>
      <pose>{x_out:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
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
    x_out = scale_x_coord(x)
    return f"""
    <model name="{name}">
      <static>{static_text}</static>
      <pose>{x_out:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
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


def cylinder_model(name, x, y, z, radius, length, color, static=True):
    r, g, b, a = color
    static_text = "true" if static else "false"
    x_out = scale_x_coord(x)
    return f"""
    <model name="{name}">
      <static>{static_text}</static>
      <pose>{x_out:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
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


def border_square_models(name, x, y, z, size, thickness, height, color):
    """빈 위치 표시용 사각 테두리."""
    return (
        box_model(f"{name}_top", x, y + size / 2, z, size, thickness, height, color)
        + box_model(f"{name}_bottom", x, y - size / 2, z, size, thickness, height, color)
        + box_model(f"{name}_left", x - size / 2, y, z, thickness, size, height, color)
        + box_model(f"{name}_right", x + size / 2, y, z, thickness, size, height, color)
    )


# -----------------------------
# AGV + 카메라 모델
# -----------------------------
def camera_block(agv_name):
    """AGV01/AGV02 공통 카메라 블록.
    ROS2 topic:
      /AGV01/agv01_camera/image_raw
      /AGV02/agv02_camera/image_raw
    """
    lower = agv_name.lower()
    return f"""

      <!-- RGB camera for {agv_name} -->
      <link name="{lower}_camera_link">
        <pose>0.45 0 0.50 0 0.7854 0</pose>

        <visual name="{lower}_camera_visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>

        <sensor name="{lower}_rgb_camera" type="camera">
          <always_on>true</always_on>
          <update_rate>20</update_rate>
          <visualize>true</visualize>

          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.01</near>
              <far>100</far>
            </clip>
          </camera>

          <plugin name="{lower}_camera_ros_plugin" filename="libgazebo_ros_camera.so">
            <ros>
              <namespace>/{agv_name}</namespace>
            </ros>
            <camera_name>{lower}_camera</camera_name>
            <frame_name>{lower}_camera_link</frame_name>
          </plugin>
        </sensor>
      </link>

      <joint name="{lower}_camera_joint" type="fixed">
        <parent>base_link</parent>
        <child>{lower}_camera_link</child>
      </joint>
"""


def agv_box_model(name, x, y, z, body_color, top_color, yaw=0.0):
    """박스형 AGV 모델. 모델 이름은 AGV01/AGV02 그대로 유지."""
    br, bg, bb, ba = body_color
    tr, tg, tb, ta = top_color
    wr, wg, wb, wa = WHITE
    kr, kg, kb, ka = BLACK
    x_out = scale_x_coord(x)
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{x_out:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
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
      {camera_block(name)}
    </model>
"""


# -----------------------------
# 라인 생성 함수
# -----------------------------
def segment_model(name, p1, p2, width=LINE_WIDTH, height=LINE_HEIGHT, color=(0, 0, 0, 1)):
    # 라인 자체는 x좌표를 압축한 뒤 길이와 yaw를 다시 계산해야 함.
    x1, y1 = scale_x_coord(p1[0]), p1[1]
    x2, y2 = scale_x_coord(p2[0]), p2[1]
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx * dx + dy * dy) + SEGMENT_OVERLAP
    yaw = math.atan2(dy, dx)
    return box_model(name, mx, my, LINE_Z, length, width, height, color, yaw=yaw, static=True, apply_x_scale=False)


def polyline_models(prefix, points, width=LINE_WIDTH):
    result = ""
    for i in range(len(points) - 1):
        result += segment_model(
            name=f"{prefix}_{i + 1:02d}",
            p1=points[i],
            p2=points[i + 1],
            width=width,
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
# 트레이 / 보관 상자 모델
# -----------------------------
def tray_slot_centers(cx, cy, gap=0.22):
    # 좌상, 우상, 좌하, 우하 순서
    return [
        (cx - gap, cy + gap),
        (cx + gap, cy + gap),
        (cx - gap, cy - gap),
        (cx + gap, cy - gap),
    ]


def material_tray_model(name, x, y, filled_slots=None, cube_color=None, yaw=0.0):
    """자재 보관 상자: 낮은 회색 트레이 + 4개 원형 슬롯 + 중앙 기둥 + 선택 큐브."""
    filled_slots = set(filled_slots or [])
    if cube_color is None:
        cube_color = BLUE
    result = ""

    # 트레이 바닥 및 외곽
    result += box_model(f"{name}_base", x, y, 0.055, 0.86, 0.70, 0.08, LIGHT_GRAY, yaw=yaw)
    result += box_model(f"{name}_rim_top", x, y + 0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_rim_bottom", x, y - 0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_rim_left", x - 0.46, y, 0.125, 0.055, 0.76, 0.12, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_rim_right", x + 0.46, y, 0.125, 0.055, 0.76, 0.12, DARK_GRAY, yaw=yaw)

    # 중앙 기둥
    result += box_model(f"{name}_center_post", x, y, 0.245, 0.22, 0.22, 0.32, GRAY)

    # 원형 슬롯 표시 + 큐브
    for idx, (sx, sy) in enumerate(tray_slot_centers(x, y), start=1):
        result += cylinder_model(f"{name}_slot_{idx}", sx, sy, 0.115, 0.105, 0.018, SLOT_GRAY)
        if idx in filled_slots:
            result += box_model(f"{name}_cube_{idx}", sx, sy, 0.245, 0.15, 0.15, 0.15, cube_color, static=True)

    return result


def finished_storage_box_model(name, x, y, yaw=0.0):
    """완제품 보관 상자: 위는 열려 있고 4개의 옆면은 모두 막힌 박스 형태."""
    result = ""
    result += box_model(f"{name}_base", x, y, 0.055, 0.90, 0.70, 0.08, LIGHT_GRAY, yaw=yaw)
    result += box_model(f"{name}_back_wall", x, y + 0.36, 0.220, 0.90, 0.06, 0.34, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_front_wall", x, y - 0.36, 0.220, 0.90, 0.06, 0.34, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_left_wall", x - 0.45, y, 0.220, 0.06, 0.70, 0.34, DARK_GRAY, yaw=yaw)
    result += box_model(f"{name}_right_wall", x + 0.45, y, 0.220, 0.06, 0.70, 0.34, DARK_GRAY, yaw=yaw)
    return result


# -----------------------------
# 움직이는 큐브 / 상자 모델
# -----------------------------
def link_box_element(name, rel_x, rel_y, rel_z, sx, sy, sz, color, yaw=0.0):
    """하나의 movable model 안에 들어갈 box visual 조각.
    시연 안정성을 위해 movable object는 collision을 만들지 않는다.
    """
    r, g, b, a = color
    return f"""
        <visual name="{name}_visual">
          <pose>{rel_x:.4f} {rel_y:.4f} {rel_z:.4f} 0 0 {yaw:.4f}</pose>
          <geometry>
            <box>
              <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
            </box>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>"""


def link_cylinder_element(name, rel_x, rel_y, rel_z, radius, length, color):
    """하나의 movable model 안에 들어갈 cylinder visual 조각.
    시연 안정성을 위해 movable object는 collision을 만들지 않는다.
    """
    r, g, b, a = color
    return f"""
        <visual name="{name}_visual">
          <pose>{rel_x:.4f} {rel_y:.4f} {rel_z:.4f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{radius:.4f}</radius>
              <length>{length:.4f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>"""


def movable_model(name, x, y, z, link_body, yaw=0.0, apply_x_scale=True):
    """gz model 명령으로 통째로 이동 가능한 단일 model."""
    x_out = scale_x_coord(x) if apply_x_scale else x
    return f"""
    <model name="{name}">
      <static>false</static>
      <pose>{x_out:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <gravity>false</gravity>
        <kinematic>true</kinematic>
        <self_collide>false</self_collide>
        {link_body}
      </link>
    </model>
"""


def movable_cube_model(name, x, y, z, color, size=0.15, yaw=0.0):
    """실제 시나리오에서 이동할 큐브. 반드시 obj_ prefix 사용."""
    body = link_box_element("cube", 0, 0, 0, size, size, size, color)
    return movable_model(name, x, y, z, body, yaw=yaw)


def movable_material_box_model(name, x, y, yaw=0.0):
    """이동 가능한 자재 상자/트레이. 큐브는 별도 obj_ 모델로 생성한다."""
    body = ""
    body += link_box_element("base", 0, 0, 0.055, 0.86, 0.70, 0.08, LIGHT_GRAY)
    body += link_box_element("rim_top", 0, 0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY)
    body += link_box_element("rim_bottom", 0, -0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY)
    body += link_box_element("rim_left", -0.46, 0, 0.125, 0.055, 0.76, 0.12, DARK_GRAY)
    body += link_box_element("rim_right", 0.46, 0, 0.125, 0.055, 0.76, 0.12, DARK_GRAY)
    body += link_box_element("center_post", 0, 0, 0.245, 0.22, 0.22, 0.32, GRAY)

    for idx, (sx, sy) in enumerate(tray_slot_centers(0, 0), start=1):
        body += link_cylinder_element(f"slot_{idx}", sx, sy, 0.115, 0.105, 0.018, SLOT_GRAY)

    return movable_model(name, x, y, 0.0, body, yaw=yaw)


def movable_finished_box_model(name, x, y, yaw=0.0):
    """이동 가능한 완제품/빈 완제품 상자. 내용물 큐브는 별도 obj_ 모델로 생성한다."""
    body = ""
    body += link_box_element("base", 0, 0, 0.055, 0.90, 0.70, 0.08, LIGHT_GRAY)
    body += link_box_element("back_wall", 0, 0.36, 0.220, 0.90, 0.06, 0.34, DARK_GRAY)
    body += link_box_element("front_wall", 0, -0.36, 0.220, 0.90, 0.06, 0.34, DARK_GRAY)
    body += link_box_element("left_wall", -0.45, 0, 0.220, 0.06, 0.70, 0.34, DARK_GRAY)
    body += link_box_element("right_wall", 0.45, 0, 0.220, 0.06, 0.70, 0.34, DARK_GRAY)
    return movable_model(name, x, y, 0.0, body, yaw=yaw)


def movable_supply_box_model(name, x, y, yaw=0.0):
    """보급 상자: 자재 상자 + 빨간 큐브 4개를 하나의 movable model로 묶는다."""
    body = ""
    body += link_box_element("base", 0, 0, 0.055, 0.86, 0.70, 0.08, LIGHT_GRAY)
    body += link_box_element("rim_top", 0, 0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY)
    body += link_box_element("rim_bottom", 0, -0.36, 0.125, 0.92, 0.055, 0.12, DARK_GRAY)
    body += link_box_element("rim_left", -0.46, 0, 0.125, 0.055, 0.76, 0.12, DARK_GRAY)
    body += link_box_element("rim_right", 0.46, 0, 0.125, 0.055, 0.76, 0.12, DARK_GRAY)
    body += link_box_element("center_post", 0, 0, 0.245, 0.22, 0.22, 0.32, GRAY)

    for idx, (sx, sy) in enumerate(tray_slot_centers(0, 0), start=1):
        body += link_cylinder_element(f"slot_{idx}", sx, sy, 0.115, 0.105, 0.018, SLOT_GRAY)
        body += link_box_element(f"red_cube_{idx}", sx, sy, 0.245, 0.15, 0.15, 0.15, RED)

    return movable_model(name, x, y, 0.0, body, yaw=yaw)


def add_movable_material_box_with_cubes(box_name, x, y, cube_color, filled_slots, cube_prefix):
    """상자와 큐브를 분리 생성한다. 상자와 큐브 모두 static=false."""
    result = movable_material_box_model(box_name, x, y)
    for idx, (sx, sy) in enumerate(tray_slot_centers(x, y), start=1):
        if idx in set(filled_slots or []):
            result += movable_cube_model(f"{cube_prefix}_{idx:02d}", sx, sy, 0.245, cube_color)
    return result


def empty_area_marker(name, x, y):
    """나중에 상자 이동 코드에서 사용할 위치를 바닥 테두리로만 표시."""
    return border_square_models(name, x, y, COLOR_BOX_Z, 0.72, 0.045, COLOR_BOX_HEIGHT, GRAY)


# -----------------------------
# 색상
# -----------------------------
BLACK = (0.0, 0.0, 0.0, 1.0)
FLOOR = (0.86, 0.86, 0.86, 1.0)
RED = (1.0, 0.0, 0.0, 1.0)
BLUE = (0.0, 0.12, 1.0, 1.0)
YELLOW = (1.0, 0.92, 0.0, 1.0)
ORANGE = (1.0, 0.30, 0.0, 1.0)
PURPLE = (0.45, 0.1, 0.75, 1.0)
GREEN = (0.0, 0.9, 0.15, 1.0)
TEAL = (0.0, 0.70, 0.12, 1.0)
RAMP_BLUE = (0.38, 0.48, 1.0, 0.85)
GRAY = (0.45, 0.45, 0.45, 1.0)
LIGHT_GRAY = (0.68, 0.68, 0.64, 1.0)
DARK_GRAY = (0.34, 0.34, 0.32, 1.0)
SLOT_GRAY = (0.32, 0.34, 0.34, 1.0)
DARK = (0.08, 0.08, 0.08, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
MARKER_RED = (0.45, 0.00, 0.04, 1.0)

# 큰 구역 표시용 높이
BIG_SQUARE = 1.05
COLOR_BOX_HEIGHT = 0.020
COLOR_BOX_Z = COLOR_BOX_HEIGHT / 2


# -----------------------------
# 경로/마커 좌표표
# -----------------------------
MARKER_POSITIONS = {
    "QR_TOP_01": (-2.65, 2.25),
    "QR_TOP_02": (-1.55, 2.25),
    "QR_TOP_03": (-0.35, 2.25),
    "QR_TOP_04": (0.85, 2.25),
    "QR_TOP_05": (2.05, 2.25),
    "QR_RIGHT_01": (3.05, 2.55),
    "QR_RIGHT_02": (3.05, 1.45),
    "QR_RIGHT_03": (3.05, 0.35),
    "QR_RIGHT_04": (3.05, -0.75),
    "QR_RIGHT_05": (3.05, -1.85),
    "QR_BOTTOM_01": (2.20, -2.55),
    "QR_BOTTOM_02": (1.10, -2.55),
    "QR_BOTTOM_03": (0.00, -2.55),
    "QR_BOTTOM_04": (-1.10, -2.55),
    "QR_BOTTOM_05": (-2.20, -2.55),
    "QR_LEFT_01": (-3.35, 1.45),
    "QR_LEFT_02": (-3.35, 0.55),
    "QR_LEFT_03": (-3.35, -0.35),
    "QR_LEFT_04": (-3.35, -1.25),
    "QR_LEFT_05": (-3.20, -2.05),
}

# 발표용 ArUco marker_900 위치: 빨간색/파란색 하단 정사각형 사이, 검정 라인에 더 가깝게 배치
ARUCO_MARKER_900 = (0.00, -2.78)

# 나중에 상자 이동 코드에서만 사용할 교차구역 좌표. 지금 world에는 실제 상자를 만들지 않음.
CROSS_EMPTY_BOX_PLACE = (3.95, -0.20)
CROSS_FULL_BOX_PLACE = (3.95, -1.35)


# -----------------------------
# 최종 시작 MAP 기준 주행 라인
# -----------------------------
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
    steps=14,
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
    steps=12,
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

# 오른쪽 교차 구역 우회로 라인 제거 버전.
# 시작 MAP처럼 상단 라인 -> 오른쪽 세로 라인 -> 하단 라인으로 이어지는 메인 라인만 남김.
right_top_curve = bezier_points(
    (2.05, 2.25),
    (2.55, 2.25),
    (3.05, 2.18),
    (3.05, 1.95),
    steps=14,
)

right_bottom_curve = bezier_points(
    (3.05, -1.90),
    (3.05, -2.32),
    (2.68, -2.55),
    (2.20, -2.55),
    steps=14,
)

# 왼쪽/상단/하단 루프만 main_loop로 만들고,
# 오른쪽 라인은 별도로 그려서 우회 S자 라인이 생기지 않게 함.
main_loop = join_paths(
    bottom_line,
    bottom_left_curve,
    left_vertical,
    top_left_curve,
    top_line,
)

# 입출고구역으로 올라가는 오른쪽 세로 라인.
# 입고 쪽 상자 2개와 검정 라인이 겹치지 않도록 위쪽으로 충분히 연장.
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
    (3.05, 3.70),
    (3.05, 4.30),
    (3.05, 4.90),
    (3.05, 5.50),
    (3.05, 6.10),
    (3.05, 6.65),
]

# 왼쪽 컨베이어 쪽 짧은 곡선은 유지
left_inner_curve = bezier_points(
    (-3.35, -0.30),
    (-2.90, -0.45),
    (-2.90, -1.20),
    (-3.35, -1.45),
    steps=12,
)


# -----------------------------
# 주요 구역 좌표
# -----------------------------
INOUT_ZONE = (3.05, 6.20)
CONVEYOR_ZONE = (-3.35, -0.45)
RAMP_ZONE = (-3.35, 0.75)

# 큰 색상 구역
# X_SCALE로 전체 메인 MAP의 가로 폭을 줄인 상태에서도 색상 구역은 같은 기준으로 배치됨
ORANGE_ZONE = (ARUCO_MARKER_900[0], 2.25)    # AGV02 위치는 유지하고, 주황색 정사각형 x좌표를 marker_900과 맞춤
BLUE_LARGE_ZONE = (-1.45, -2.55)
RED_LARGE_ZONE = (1.25, -2.55)
PURPLE_LARGE_ZONE = (-3.35, -1.65)
GREEN_LARGE_ZONE = (RAMP_ZONE[0], RAMP_ZONE[1])

# 보관 구역: 시작 MAP 기준
# 노란색 원 안의 상자/큐브는 모두 실제 시나리오에서 움직일 수 있어야 하므로 obj_ 모델로 분리한다.
# 자리표시용 고정 프레임은 추가하지 않는다.
FINISHED_STORAGE_BOXES = [
    # 완제품 보관 구역에 있는 이동 가능한 상자 1개
    ("obj_finished_box_01", -1.05, 0.95),
]

MATERIAL_STORAGE_BOXES = [
    # 자재 보관 구역: 이동 가능한 파란 자재 상자 + 파란 큐브 3개
    ("obj_material_blue_box_01", -1.05, -1.12, BLUE, [1, 2, 3], "obj_blue_cube"),
    # 자재 보관 구역: 이동 가능한 빨간 자재 상자 + 빨간 큐브 1개
    # 시작 MAP 수정: 빨간 큐브 시작 위치를 오른쪽 자재 상자 안의 좌하단 슬롯(3번)으로 이동
    ("obj_material_red_box_01", 0.30, -1.12, RED, [3], "obj_red_cube"),
]

# 입고구역: 검정색 라인과 초록색 입출고구역에 겹치지 않도록 왼쪽으로 분리 배치
# 시작 MAP 기준 입고 쪽에 이동 가능한 상자 3개를 둔다.
INBOUND_OBJECT_BOXES = [
    # 시나리오에서 AGV02가 운반하는 보급 상자. 상자+빨간 큐브 4개를 하나의 object로 묶는다.
    ("obj_supply_box_01", 2.05, 4.85, "supply", None, [], None),
    # 시나리오에서 움직이지 않는 빈 완제품 상자는 static=true로 둔다.
    ("static_inbound_empty_finished_box_01", 2.05, 3.88, "static_finished", None, [], None),
    # 시나리오에서 AGV02가 운반하는 빈 완제품 상자.
    ("obj_inbound_empty_finished_box_02", 2.05, 2.91, "finished", None, [], None),
]

# 시작 MAP에서 출고 쪽은 비워둠. 최종 시나리오에서 상자 이동 코드가 사용할 좌표만 남김.
OUTBOUND_BOX_PLACE = (3.95, 5.50)

AGV01_START = (2.20, -2.55)
AGV02_START = (-1.35, 2.25)


# -----------------------------
# World 생성
# -----------------------------
world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="agv_factory_final_box_agv_v2">

    <include>
      <uri>model://sun</uri>
    </include>

    <light name="wide_map_light" type="point">
      <pose>0 1.20 8.50 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <attenuation>
        <range>22.0</range>
        <constant>0.55</constant>
        <linear>0.02</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <cast_shadows>false</cast_shadows>
    </light>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 0.90 13.2 0 1.5708 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    {box_model("factory_floor", 0, 1.05, FLOOR_Z, 8.4, 13.0, 0.03, FLOOR)}
"""

# 주행 라인
world += polyline_models("main_loop", main_loop, width=LINE_WIDTH)
world += polyline_models("right_top_curve", right_top_curve, width=LINE_WIDTH)
world += polyline_models("right_vertical", right_vertical, width=LINE_WIDTH)
world += polyline_models("right_bottom_curve", right_bottom_curve, width=LINE_WIDTH)
world += polyline_models("left_inner_curve", left_inner_curve, width=LINE_WIDTH)

# 큰 색상 구역
# X_SCALE로 전체 메인 MAP의 가로 폭을 줄인 상태에서도 색상 구역은 같은 기준으로 배치됨: 노란색 교차 구역 박스는 제거
world += box_model("orange_agv02_start_area", ORANGE_ZONE[0], ORANGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, ORANGE)
world += box_model("blue_bottom_area", BLUE_LARGE_ZONE[0], BLUE_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, BLUE)
world += box_model("red_agv01_start_area", RED_LARGE_ZONE[0], RED_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, RED)
world += box_model("purple_left_area", PURPLE_LARGE_ZONE[0], PURPLE_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, PURPLE)
world += box_model("green_left_square", GREEN_LARGE_ZONE[0], GREEN_LARGE_ZONE[1], COLOR_BOX_Z, BIG_SQUARE, BIG_SQUARE, COLOR_BOX_HEIGHT, GREEN)

# 입출고구역 영역
INOUT_BOX_HEIGHT = 0.060
INOUT_BOX_Z = LINE_HEIGHT + INOUT_BOX_HEIGHT / 2
world += box_model("inout_area", INOUT_ZONE[0], INOUT_ZONE[1], INOUT_BOX_Z, 1.25, 1.25, INOUT_BOX_HEIGHT, TEAL)

# 컨베이어 및 경사로
CONVEYOR_HEIGHT = 0.10
CONVEYOR_Z = LINE_HEIGHT + CONVEYOR_HEIGHT / 2
world += box_model("conveyor_area", CONVEYOR_ZONE[0], CONVEYOR_ZONE[1], CONVEYOR_Z, 0.72, 2.20, CONVEYOR_HEIGHT, GRAY)

RAMP_LENGTH_Y = 0.65
RAMP_THICKNESS = 0.045
RAMP_HIGH_TOP_Z = LINE_HEIGHT + CONVEYOR_HEIGHT
RAMP_LOW_TOP_Z = LINE_HEIGHT
RAMP_ROLL = -math.asin((RAMP_HIGH_TOP_Z - RAMP_LOW_TOP_Z) / RAMP_LENGTH_Y)
RAMP_Z = ((RAMP_HIGH_TOP_Z + RAMP_LOW_TOP_Z) / 2) - (RAMP_THICKNESS / 2)
world += box_model_rpy("ramp_area", RAMP_ZONE[0], RAMP_ZONE[1], RAMP_Z, 0.80, RAMP_LENGTH_Y, RAMP_THICKNESS, RAMP_BLUE, roll=RAMP_ROLL)

# 완제품 보관 구역: 이동 가능한 상자 1개
for name, x, y in FINISHED_STORAGE_BOXES:
    world += movable_finished_box_model(name, x, y)

# 자재 보관 구역: 이동 가능한 상자 2개 + 각 상자 안의 큐브를 별도 object로 생성
for name, x, y, cube_color, filled_slots, cube_prefix in MATERIAL_STORAGE_BOXES:
    world += add_movable_material_box_with_cubes(name, x, y, cube_color, filled_slots, cube_prefix)

# 입고 구역: 이동 가능한 상자 3개. 상자 내부 큐브는 별도 object로 생성한다.
for name, x, y, box_type, cube_color, filled_slots, cube_prefix in INBOUND_OBJECT_BOXES:
    if box_type == "material":
        world += add_movable_material_box_with_cubes(name, x, y, cube_color, filled_slots, cube_prefix)
    elif box_type == "supply":
        world += movable_supply_box_model(name, x, y)
    elif box_type == "finished":
        world += movable_finished_box_model(name, x, y)
    elif box_type == "static_finished":
        world += finished_storage_box_model(name, x, y)
    else:
        raise ValueError(f"Unknown inbound box_type: {box_type}")

# OUTBOUND 쪽은 시작 시 실제 상자를 두지 않음. 필요 시 상자 이동 코드에서 OUTBOUND_BOX_PLACE 좌표 사용.

# 발표용 ArUco marker_900: 빨간색/파란색 정사각형 사이에 1개만 배치
marker_x, marker_y = ARUCO_MARKER_900
world += box_model("aruco_marker_900_white_plate", marker_x, marker_y, 0.045, 0.44, 0.44, 0.025, WHITE)
world += f"""
    <include>
      <uri>model://marker_900</uri>
      <name>marker_900</name>
      <pose>{marker_x:.4f} {marker_y:.4f} 0.0700 0 0 0</pose>
    </include>
"""

# AGV 모델: 이름은 이동 스크립트와 맞춰 AGV01 / AGV02 유지
AGV_BODY = (0.18, 0.18, 0.18, 1.0)
world += agv_box_model("AGV01", AGV01_START[0], AGV01_START[1], 0.16, AGV_BODY, RED, yaw=math.pi)
world += agv_box_model("AGV02", AGV02_START[0], AGV02_START[1], 0.16, AGV_BODY, BLUE)

world += """
  </world>
</sdf>
"""

WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
WORLD_PATH.write_text(world, encoding="utf-8")
print(f"Created: {WORLD_PATH}")
