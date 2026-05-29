import math
from pathlib import Path

WORLD_PATH = Path("agv_factory.world")

COLORS = {
    "black": (0, 0, 0, 1),
    "white": (1, 1, 1, 1),
    "red": (1, 0, 0, 1),
    "blue": (0, 0, 1, 1),
    "yellow": (1, 1, 0, 1),
    "orange": (1, 0.45, 0, 1),
    "green": (0, 0.8, 0, 1),
    "purple": (0.55, 0, 1, 1),
    "gray": (0.6, 0.6, 0.6, 1),
    "light_gray": (0.8, 0.8, 0.8, 1),
    "pink": (1, 0, 1, 0.65),
    "dark_gray": (0.12, 0.12, 0.12, 1),
}

QR = {
    "QR_TOP_01": (-2.6, 2.2),
    "QR_TOP_02": (-1.4, 2.2),
    "QR_TOP_03": (0.0, 2.2),
    "QR_TOP_04": (1.3, 2.2),
    "QR_TOP_05": (2.4, 2.0),

    "QR_RIGHT_01": (2.8, 1.6),
    "QR_RIGHT_02": (3.2, 0.8),
    "QR_RIGHT_03": (3.2, 0.0),
    "QR_RIGHT_04": (3.2, -0.8),
    "QR_RIGHT_05": (2.6, -2.2),

    "QR_BOTTOM_01": (-2.4, -2.6),
    "QR_BOTTOM_02": (-1.2, -2.6),
    "QR_BOTTOM_03": (0.0, -2.6),
    "QR_BOTTOM_04": (1.2, -2.6),
    "QR_BOTTOM_05": (2.4, -2.6),

    "QR_LEFT_01": (-2.7, 1.9),
    "QR_LEFT_02": (-3.1, 0.9),
    "QR_LEFT_03": (-3.0, 0.1),
    "QR_LEFT_04": (-2.8, -1.0),
    "QR_LEFT_05": (-2.4, -2.2),

    "QR_CROSS_ENTRY": (-2.8, -0.6),
    "QR_CROSS_EXIT": (-2.5, -1.8),
}

def color_xml(color_name):
    r, g, b, a = COLORS[color_name]
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""

def box_model(name, x, y, z, sx, sy, sz, color, yaw=0, static=True):
    static_text = "true" if static else "false"
    return f"""
    <model name=\"{name}\">
      <static>{static_text}</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name=\"link\">
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
            </box>
          </geometry>
          {color_xml(color)}
        </visual>
        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{sx:.3f} {sy:.3f} {sz:.3f}</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
"""

def road_between(name, p1, p2, width=0.13):
    x1, y1 = p1
    x2, y2 = p2
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    return box_model(name, mx, my, 0.055, length, width, 0.035, "black", yaw, True)

def qr_marker(name, x, y):
    base = box_model(name, x, y, 0.095, 0.28, 0.28, 0.025, "white", 0, True)
    inner = box_model(name + "_black", x, y, 0.115, 0.12, 0.12, 0.025, "black", 0, True)
    return base + inner

def build_world():
    parts = []

    parts.append("""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"agv_factory_world\">

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>
""")

    parts.append(box_model("map_base", 0, 0, 0.005, 8.2, 7.2, 0.01, "light_gray", 0, True))

    parts.append(box_model("inout_zone", -3.3, 2.7, 0.025, 0.9, 0.55, 0.035, "purple", 0, True))
    parts.append(box_model("final_box_zone", -0.7, 3.0, 0.025, 1.0, 0.55, 0.035, "gray", 0, True))
    parts.append(box_model("part_pickup_zone", 2.2, 2.7, 0.025, 0.9, 0.55, 0.035, "green", 0, True))

    parts.append(box_model("red_zone", -0.9, 2.1, 0.025, 0.55, 0.55, 0.035, "red", 0, True))
    parts.append(box_model("blue_zone", 2.2, 2.2, 0.025, 0.55, 0.55, 0.035, "blue", 0, True))
    parts.append(box_model("yellow_zone", -3.0, 0.2, 0.025, 0.55, 0.55, 0.035, "yellow", 0, True))
    parts.append(box_model("orange_zone", 0.0, -2.5, 0.025, 0.55, 0.55, 0.035, "orange", 0, True))

    parts.append(box_model("a_part_zone", -0.8, -1.7, 0.025, 0.65, 0.45, 0.035, "red", 0, True))
    parts.append(box_model("b_part_zone", 0.9, -1.7, 0.025, 0.65, 0.45, 0.035, "blue", 0, True))
    parts.append(box_model("c_part_zone", -0.8, -3.0, 0.025, 0.65, 0.45, 0.035, "yellow", 0, True))
    parts.append(box_model("d_part_zone", 0.9, -3.0, 0.025, 0.65, 0.45, 0.035, "orange", 0, True))

    parts.append(box_model("container_zone", 3.1, 0.0, 0.025, 0.75, 2.8, 0.035, "blue", 0, True))
    parts.append(box_model("cross_zone", -2.7, -1.2, 0.04, 1.3, 2.2, 0.03, "pink", 0, True))

    road_paths = {
        "top": ["QR_TOP_01", "QR_TOP_02", "QR_TOP_03", "QR_TOP_04", "QR_TOP_05", "QR_RIGHT_01"],
        "right": ["QR_RIGHT_01", "QR_RIGHT_02", "QR_RIGHT_03", "QR_RIGHT_04", "QR_RIGHT_05", "QR_BOTTOM_05"],
        "bottom": ["QR_BOTTOM_05", "QR_BOTTOM_04", "QR_BOTTOM_03", "QR_BOTTOM_02", "QR_BOTTOM_01", "QR_LEFT_05"],
        "left": ["QR_LEFT_05", "QR_CROSS_EXIT", "QR_LEFT_04", "QR_CROSS_ENTRY", "QR_LEFT_03", "QR_LEFT_02", "QR_LEFT_01", "QR_TOP_01"],
    }

    idx = 1
    for path_name, ids in road_paths.items():
        for a, b in zip(ids, ids[1:]):
            parts.append(road_between(f"road_{path_name}_{idx}", QR[a], QR[b]))
            idx += 1

    for name, (x, y) in QR.items():
        parts.append(qr_marker(name, x, y))

    parts.append(box_model("agv01", 2.4, -2.6, 0.22, 0.55, 0.38, 0.28, "dark_gray", 3.14, False))
    parts.append(box_model("agv02", 2.4, 2.0, 0.22, 0.55, 0.38, 0.28, "dark_gray", -1.57, False))

    parts.append("""
  </world>
</sdf>
""")

    return "".join(parts)

WORLD_PATH.write_text(build_world(), encoding="utf-8")
print(f"Created {WORLD_PATH.resolve()}")
print("QR count:", len(QR))
