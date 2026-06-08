import os
import shutil

BASE_DIR = "models/aruco_markers"
IMAGE_DIR = os.path.join(BASE_DIR, "images")

MARKER_IDS = [900, 901, 902]

# Gazebo 바닥에 놓을 마커 크기
# 너무 작으면 카메라에서 인식이 어려우므로 처음에는 0.5m 정도로 크게 테스트
MARKER_SIZE = 0.25
MARKER_THICKNESS = 0.005


def create_model_config(marker_id, model_dir):
    content = f"""<?xml version="1.0"?>
<model>
  <name>aruco_marker_{marker_id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>MJ</name>
    <email>none</email>
  </author>
  <description>ArUco marker {marker_id} for Gazebo camera simulation</description>
</model>
"""
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(content)


def create_model_sdf(marker_id, model_dir):
    texture_name = f"marker_{marker_id}.png"

    content = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="aruco_marker_{marker_id}">
    <static>true</static>

    <link name="link">
      <pose>0 0 0 0 0 0</pose>

      <collision name="collision">
        <geometry>
          <box>
            <size>{MARKER_SIZE} {MARKER_SIZE} {MARKER_THICKNESS}</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <pose>0 0 0.006 0 0 0</pose>
        <geometry>
          <box>
            <size>{MARKER_SIZE} {MARKER_SIZE} 0.001</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://aruco_marker_{marker_id}/materials/scripts</uri>
            <uri>model://aruco_marker_{marker_id}/materials/textures</uri>
            <name>ArUco/Marker{marker_id}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(content)


def create_material_script(marker_id, script_dir):
    content = f"""material ArUco/Marker{marker_id}
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture marker_{marker_id}.png
      }}
    }}
  }}
}}
"""
    with open(os.path.join(script_dir, f"marker_{marker_id}.material"), "w") as f:
        f.write(content)


def main():
    for marker_id in MARKER_IDS:
        image_path = os.path.join(IMAGE_DIR, f"marker_{marker_id}.png")

        if not os.path.exists(image_path):
            print(f"Missing image: {image_path}")
            continue

        model_dir = os.path.join(BASE_DIR, f"marker_{marker_id}")
        texture_dir = os.path.join(model_dir, "materials", "textures")
        script_dir = os.path.join(model_dir, "materials", "scripts")

        os.makedirs(texture_dir, exist_ok=True)
        os.makedirs(script_dir, exist_ok=True)

        shutil.copy(image_path, os.path.join(texture_dir, f"marker_{marker_id}.png"))

        create_model_config(marker_id, model_dir)
        create_model_sdf(marker_id, model_dir)
        create_material_script(marker_id, script_dir)

        print(f"Created Gazebo model: {model_dir}")

    print("ArUco Gazebo marker model generation complete.")


if __name__ == "__main__":
    main()
