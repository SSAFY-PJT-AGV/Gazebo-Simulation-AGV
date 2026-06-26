import json
import math
import time
import argparse
import subprocess
from pathlib import Path


DEFAULT_LINE_PATH_MAP = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/line_path_map.json"


def load_json(json_path):
    path = Path(json_path)

    if not path.exists():
        raise FileNotFoundError(f"JSON file not found: {json_path}")

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def calc_yaw(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


def set_model_pose(model_name, x, y, z, yaw):
    cmd = [
        "gz", "model",
        "-m", model_name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(z),
        "-Y", str(yaw),
    ]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        print("[ERROR] Failed to move model")
        print(result.stderr)


def move_between_points(model_name, start_pos, end_pos, speed, step_time):
    x1, y1, z1 = start_pos
    x2, y2, z2 = end_pos

    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    if distance == 0:
        return

    duration = distance / speed
    steps = max(int(duration / step_time), 1)

    yaw = calc_yaw(x1, y1, x2, y2)

    for i in range(steps + 1):
        ratio = i / steps

        x = x1 + (x2 - x1) * ratio
        y = y1 + (y2 - y1) * ratio
        z = z1 + (z2 - z1) * ratio

        set_model_pose(model_name, x, y, z, yaw)
        time.sleep(step_time)


def get_path_key(start_marker, end_marker):
    return f"{start_marker}_to_{end_marker}"


def get_reverse_path_key(start_marker, end_marker):
    return f"{end_marker}_to_{start_marker}"


def get_path_points(line_path_map, start_marker, end_marker):
    paths = line_path_map["paths"]

    path_key = get_path_key(start_marker, end_marker)

    if path_key in paths:
        return paths[path_key]

    reverse_key = get_reverse_path_key(start_marker, end_marker)

    if reverse_key in paths:
        reverse_points = paths[reverse_key]
        return list(reversed(reverse_points))

    raise KeyError(f"No path found: {path_key} or {reverse_key}")


def parse_route(route_text):
    return [int(x.strip()) for x in route_text.split(",") if x.strip()]


def move_along_path_points(model_name, path_points, speed, step_time):
    if len(path_points) < 2:
        return

    for i in range(len(path_points) - 1):
        start_pos = tuple(float(v) for v in path_points[i])
        end_pos = tuple(float(v) for v in path_points[i + 1])

        move_between_points(
            model_name=model_name,
            start_pos=start_pos,
            end_pos=end_pos,
            speed=speed,
            step_time=step_time
        )


def main():
    parser = argparse.ArgumentParser(
        description="Move Gazebo AGV along predefined black line path."
    )

    parser.add_argument(
        "--agv",
        default="AGV01",
        help="Gazebo model name. Example: AGV01 or AGV02"
    )

    parser.add_argument(
        "--line-path-map",
        default=DEFAULT_LINE_PATH_MAP,
        help="Path to line_path_map.json"
    )

    parser.add_argument(
        "--route",
        default="1,2,3,4,5",
        help="Marker route. Example: 1,2,3,4,5"
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Simulation speed. Larger value means faster movement."
    )

    parser.add_argument(
        "--step-time",
        type=float,
        default=0.05,
        help="Movement update interval."
    )

    args = parser.parse_args()

    line_path_map = load_json(args.line_path_map)
    route = parse_route(args.route)

    if len(route) < 2:
        raise ValueError("Route must contain at least 2 marker IDs.")

    print("====================================")
    print("Gazebo AGV Black Line Path Demo")
    print(f"AGV model: {args.agv}")
    print(f"line_path_map: {args.line_path_map}")
    print(f"route: {route}")
    print("====================================")

    for i in range(len(route) - 1):
        start_marker = route[i]
        end_marker = route[i + 1]

        print(f"[{args.agv}] Follow black line: marker {start_marker} -> marker {end_marker}")

        path_points = get_path_points(
            line_path_map=line_path_map,
            start_marker=start_marker,
            end_marker=end_marker
        )

        move_along_path_points(
            model_name=args.agv,
            path_points=path_points,
            speed=args.speed,
            step_time=args.step_time
        )

    print("====================================")
    print("Line path route complete")
    print("====================================")


if __name__ == "__main__":
    main()