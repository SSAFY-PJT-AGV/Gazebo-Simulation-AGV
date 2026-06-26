import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path


DEFAULT_SCENARIO_PLAN = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/scenario_plan.json"
DEFAULT_ZONE_ROUTE_MAP = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/zone_route_map.json"
DEFAULT_LINE_PATH_MAP = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/line_path_map.json"
DEFAULT_OBJECT_MOTION_CONFIG = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/object_motion_config.json"


VALID_STATUSES = {
    "IDLE",
    "ASSIGNED",
    "MOVING",
    "WAITING",
    "ARRIVED",
    "LOADING",
    "UNLOADING",
    "DONE",
    "ERROR",
    "OFFLINE",
}

VALID_ZONES = {
    "CONVEYOR_START",
    "CONVEYOR_END",
    "AGV01_START",
    "AGV02_START",
    "FINISHED_BOX_STORAGE",
    "MATERIAL_BOX_STORAGE",
    "INBOUND",
    "OUTBOUND",
    "CROSS_ZONE",
    "CROSS_FULL_BOX_STOP",
    "CROSS_EMPTY_BOX_STOP",
}

VALID_COMMANDS = {
    "PICK_FROM_STORAGE",
    "DROP_TO_STORAGE",
    "DROP_TO_CONVEYOR",
    "RETURN_TO_BASE",
    "PICK_EMPTY_BOX",
    "DROP_EMPTY_BOX",
    "PICK_FROM_CONVEYOR",
    "DROP_TO_FINISHED_BOX_STORAGE",
    "PICK_FROM_INBOUND",
    "DROP_TO_OUTBOUND",
    "DROP_TO_INBOUND",
    "PICK_FROM_CROSS",
    "DROP_TO_CROSS",
    "WAIT",
    "STOP",
    "RESUME",

    # Temporary command for local Gazebo demo.
    # Server has not confirmed this command yet.
    "PICK_FROM_FINISHED_BOX_STORAGE",
}


# =============================
# Object motion runtime state
# =============================
# 어떤 object가 어떤 AGV 위에 있는지 저장한다.
# 예: payload_on_agv["AGV01"] = ["obj_blue_cube_01"]
payload_on_agv = {
    "AGV01": [],
    "AGV02": [],
}

# object별 AGV 기준 상대 offset을 저장한다.
# 예: payload_relative_offsets["AGV02"]["obj_finished_box_01"] = (0.0, 0.0, 0.08)
payload_relative_offsets = {
    "AGV01": {},
    "AGV02": {},
}

# 컨베이어 끝에 도착한 object를 저장한다.
# 나중에 PICK_FROM_CONVEYOR에서 사용할 수 있다.
conveyor_output_queue = []

# AGV 위에 없는 object를 강제로 고정할 pose.
# static=false object가 Gazebo 물리엔진 영향으로 혼자 밀리는 것을 막기 위해 사용한다.
fixed_object_poses = {}

# AGV 이동 thread와 object 이동 thread가 동시에 접근할 수 있으므로 lock을 둔다.
object_state_lock = threading.Lock()




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
        print(f"[ERROR] Failed to move model: {model_name}")
        if result.stderr.strip():
            print(result.stderr.strip())
        return False

    return True


def get_optional_json(json_path):
    if not json_path:
        return {}

    path = Path(json_path)
    if not path.exists():
        print(f"[OBJECT-CONFIG] config file not found. Object motion rules will be skipped: {json_path}")
        return {}

    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def distance_2d(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def get_position_from_config(object_config, value):
    """
    value가 [x, y, z]이면 그대로 좌표로 사용하고,
    문자열이면 object_motion_config.json의 positions에서 찾아 좌표로 변환한다.
    """
    if value is None:
        return None

    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return tuple(float(v) for v in value[:3])

    if isinstance(value, str):
        positions = object_config.get("positions", {})
        if value not in positions:
            print(f"[OBJECT-WARN] position key not found in object config: {value}")
            return None
        return get_position_from_config(object_config, positions[value])

    print(f"[OBJECT-WARN] invalid position value: {value}")
    return None


def get_payload_offsets(object_config, agv_name):
    offsets = object_config.get("payload_offsets", {})
    default_offset = object_config.get("default_payload_offset", [0.0, 0.0, 0.34])
    offset = offsets.get(agv_name, default_offset)

    if not isinstance(offset, (list, tuple)) or len(offset) < 3:
        offset = [0.0, 0.0, 0.34]

    return tuple(float(v) for v in offset[:3])


def set_object_pose(model_name, x, y, z, yaw=0.0):
    return set_model_pose(model_name, x, y, z, yaw)


def remember_fixed_pose(model_name, x, y, z, yaw=0.0):
    with object_state_lock:
        fixed_object_poses[model_name] = (float(x), float(y), float(z), float(yaw))


def forget_fixed_pose(model_name):
    with object_state_lock:
        fixed_object_poses.pop(model_name, None)


def init_fixed_object_poses(object_config):
    initial_poses = object_config.get("fixed_object_poses", {})
    for model_name, value in initial_poses.items():
        if isinstance(value, dict):
            pos_value = value.get("position") or value.get("position_key")
            yaw = float(value.get("yaw", 0.0))
        else:
            pos_value = value
            yaw = 0.0

        pos = get_position_from_config(object_config, pos_value)
        if pos is None:
            continue
        remember_fixed_pose(model_name, pos[0], pos[1], pos[2], yaw)
        set_object_pose(model_name, pos[0], pos[1], pos[2], yaw)


def enforce_fixed_object_poses():
    with object_state_lock:
        agv_objects = set()
        for objects in payload_on_agv.values():
            agv_objects.update(objects)
        items = list(fixed_object_poses.items())

    for model_name, pose in items:
        if model_name in agv_objects:
            continue
        x, y, z, yaw = pose
        set_object_pose(model_name, x, y, z, yaw)


def animate_object(model_name, points, duration=None, speed=0.6, step_time=0.03):
    """
    object를 여러 좌표를 거쳐 이동시킨다.
    실제 집게/물리 제어가 아니라 Gazebo model pose를 직접 바꾸는 방식이다.
    """
    if not points or len(points) < 2:
        return

    clean_points = [tuple(float(v) for v in p[:3]) for p in points]

    if duration is not None and duration > 0:
        total_distance = sum(
            distance_2d(clean_points[i], clean_points[i + 1])
            for i in range(len(clean_points) - 1)
        )
        speed = max(total_distance / duration, 0.001)

    for i in range(len(clean_points) - 1):
        start_pos = clean_points[i]
        end_pos = clean_points[i + 1]
        yaw = calc_yaw(start_pos[0], start_pos[1], end_pos[0], end_pos[1])
        distance = distance_2d(start_pos, end_pos)

        if distance == 0:
            set_object_pose(model_name, end_pos[0], end_pos[1], end_pos[2], yaw)
            continue

        segment_duration = distance / max(speed, 0.001)
        steps = max(int(segment_duration / step_time), 1)

        for step_idx in range(steps + 1):
            ratio = step_idx / steps
            x = start_pos[0] + (end_pos[0] - start_pos[0]) * ratio
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * ratio
            z = start_pos[2] + (end_pos[2] - start_pos[2]) * ratio
            set_object_pose(model_name, x, y, z, yaw)
            time.sleep(step_time)


def get_objects_on_agv(agv_name):
    with object_state_lock:
        return list(payload_on_agv.get(agv_name, []))


def add_object_to_agv(agv_name, object_name, relative_offset=None):
    with object_state_lock:
        payload_on_agv.setdefault(agv_name, [])
        payload_relative_offsets.setdefault(agv_name, {})
        if object_name not in payload_on_agv[agv_name]:
            payload_on_agv[agv_name].append(object_name)
        if relative_offset is not None:
            payload_relative_offsets[agv_name][object_name] = tuple(float(v) for v in relative_offset[:3])


def remove_object_from_agv(agv_name, object_name):
    with object_state_lock:
        if object_name in payload_on_agv.get(agv_name, []):
            payload_on_agv[agv_name].remove(object_name)
        payload_relative_offsets.setdefault(agv_name, {}).pop(object_name, None)


def get_agv_payload_pose(agv_name, agv_x, agv_y, agv_z, object_index, object_config, object_name=None):
    with object_state_lock:
        saved_offset = None
        if object_name is not None:
            saved_offset = payload_relative_offsets.get(agv_name, {}).get(object_name)

    if saved_offset is not None:
        offset_x, offset_y, offset_z = saved_offset
    else:
        offset_x, offset_y, offset_z = get_payload_offsets(object_config, agv_name)
        # 여러 object가 한 AGV 위에 있을 때 완전히 겹치지 않도록 아주 작은 y offset을 준다.
        stack_gap = float(object_config.get("payload_stack_gap", 0.18))
        y_shift = (object_index - 0.5) * stack_gap if object_index > 0 else 0.0
        offset_y += y_shift

    return (agv_x + offset_x, agv_y + offset_y, agv_z + offset_z)


def follow_agv_payload(agv_name, agv_x, agv_y, agv_z, yaw, object_config=None):
    """AGV가 이동할 때 AGV 위에 등록된 object를 같이 이동시킨다."""
    object_config = object_config or {}
    objects = get_objects_on_agv(agv_name)

    for idx, object_name in enumerate(objects):
        obj_x, obj_y, obj_z = get_agv_payload_pose(
            agv_name, agv_x, agv_y, agv_z, idx, object_config, object_name=object_name
        )
        set_object_pose(object_name, obj_x, obj_y, obj_z, yaw)


def pick_object_to_agv(agv_name, object_name, source_pos, agv_pose=None, object_config=None, duration=0.0, step_time=0.03, object_index=None, relative_offset=None):
    """object를 즉시 AGV 위 좌표로 이동시킨다. 중간 애니메이션은 시연 안정성을 위해 사용하지 않는다."""
    object_config = object_config or {}

    print(f"[OBJECT-PICK] {agv_name}: {object_name} -> AGV payload (instant pose)")

    forget_fixed_pose(object_name)

    if object_index is None:
        object_index = len(get_objects_on_agv(agv_name))

    if agv_pose is not None:
        agv_x, agv_y, agv_z, yaw = agv_pose
        if relative_offset is not None:
            target_pos = (
                agv_x + float(relative_offset[0]),
                agv_y + float(relative_offset[1]),
                agv_z + float(relative_offset[2]),
            )
        else:
            target_pos = get_agv_payload_pose(
                agv_name,
                agv_x,
                agv_y,
                agv_z,
                object_index,
                object_config,
                object_name=object_name,
            )
        set_object_pose(object_name, target_pos[0], target_pos[1], target_pos[2], yaw)

    add_object_to_agv(agv_name, object_name, relative_offset=relative_offset)


def drop_object_from_agv(agv_name, object_name, target_pos, agv_pose=None, object_config=None, duration=0.0, step_time=0.03):
    """AGV 위 object를 즉시 target_pos에 내려놓고 해당 pose에 고정한다."""
    object_config = object_config or {}

    print(f"[OBJECT-DROP] {agv_name}: {object_name} -> {target_pos} (instant pose)")

    yaw = 0.0
    if agv_pose is not None:
        yaw = float(agv_pose[3])

    if target_pos is not None:
        set_object_pose(object_name, target_pos[0], target_pos[1], target_pos[2], yaw)
        remember_fixed_pose(object_name, target_pos[0], target_pos[1], target_pos[2], yaw)

    remove_object_from_agv(agv_name, object_name)


def move_object_on_conveyor(object_name, conveyor_points, duration=0.6, step_time=0.03):
    """컨베이어 위 object 이동. 짧게 이동시키고 도착 위치에 강제 고정한다."""
    if not conveyor_points or len(conveyor_points) < 2:
        print(f"[OBJECT-CONVEYOR-READY] {object_name}: conveyor path is not configured")
        return

    print(f"[OBJECT-CONVEYOR] {object_name}: moving on conveyor duration={duration:.2f}s")
    forget_fixed_pose(object_name)
    animate_object(
        model_name=object_name,
        points=conveyor_points,
        duration=duration,
        step_time=step_time,
    )

    end_pos = tuple(float(v) for v in conveyor_points[-1][:3])
    set_object_pose(object_name, end_pos[0], end_pos[1], end_pos[2], 0.0)
    remember_fixed_pose(object_name, end_pos[0], end_pos[1], end_pos[2], 0.0)

    with object_state_lock:
        if object_name not in conveyor_output_queue:
            conveyor_output_queue.append(object_name)


def get_rule(object_config, rule_type, agv_name, command, payload, from_zone, to_zone, step_number=None):
    """
    object_motion_config.json의 rules에서 현재 AGV 상태와 맞는 rule을 찾는다.
    rule에서 step/steps/agv/command/payload/from/to는 생략 가능하고, 명시된 값만 검사한다.
    """
    rules = object_config.get(rule_type, [])

    for rule in rules:
        # step 조건 검사
        if step_number is not None:
            if "step" in rule and int(rule["step"]) != int(step_number):
                continue
            if "steps" in rule:
                try:
                    valid_steps = {int(s) for s in rule.get("steps", [])}
                except Exception:
                    valid_steps = set()
                if int(step_number) not in valid_steps:
                    continue

        conditions = {
            "agv": agv_name,
            "command": command,
            "payload": payload,
            "from": from_zone,
            "to": to_zone,
        }

        matched = True
        for key, current_value in conditions.items():
            if key in rule and rule[key] != current_value:
                matched = False
                break

        if matched:
            return rule

    return None

def resolve_rule_objects(rule, object_config):
    if not rule:
        return []

    if "objects" in rule:
        return list(rule.get("objects") or [])

    if "object" in rule:
        return [rule["object"]]

    object_group = rule.get("object_group")
    if object_group:
        return list(object_config.get("object_groups", {}).get(object_group, []))

    return []


def handle_loading_object(agv_name, agv_state, object_config, step_time, action_delay, step_number=None, rule_type="loading_rules"):
    command = agv_state.get("command")
    payload = agv_state.get("payload", "EMPTY")
    from_zone = agv_state.get("from")
    to_zone = agv_state.get("to")

    rule = get_rule(object_config, rule_type, agv_name, command, payload, from_zone, to_zone, step_number=step_number)

    if not rule:
        label = "OBJECT-POST-MOVE-LOAD-READY" if rule_type == "post_move_loading_rules" else "OBJECT-LOAD-READY"
        print(f"[{label}] step={step_number} {agv_name}: command={command}, payload={payload}, from={from_zone}, to={to_zone}. Rule not configured yet.")
        return

    objects = resolve_rule_objects(rule, object_config)
    if not objects:
        print(f"[OBJECT-WARN] step={step_number} {agv_name}: loading rule has no object(s)")
        return

    # source_positions/source_position_keys가 있으면 object별 source 좌표를 각각 사용한다.
    source_values = rule.get("source_positions") or rule.get("source_position_keys")
    if source_values:
        source_positions = [get_position_from_config(object_config, value) for value in source_values]
    else:
        source_pos = get_position_from_config(object_config, rule.get("source_position") or rule.get("source_position_key"))
        source_positions = [source_pos for _ in objects]

    agv_pose_value = rule.get("agv_pose") or rule.get("agv_pose_key")
    agv_pose_pos = get_position_from_config(object_config, agv_pose_value)
    agv_pose = None
    if agv_pose_pos is not None:
        agv_pose = (agv_pose_pos[0], agv_pose_pos[1], agv_pose_pos[2], float(rule.get("agv_yaw", 0.0)))

    payload_offset_values = rule.get("payload_offsets") or rule.get("payload_offset_keys")
    relative_offsets = []
    if payload_offset_values:
        for value in payload_offset_values:
            pos = get_position_from_config(object_config, value)
            relative_offsets.append(pos)

    duration = float(rule.get("duration", 0.0))

    for idx, object_name in enumerate(objects):
        source_pos = source_positions[min(idx, len(source_positions) - 1)] if source_positions else None
        rel_offset = relative_offsets[min(idx, len(relative_offsets) - 1)] if relative_offsets else None
        pick_object_to_agv(
            agv_name=agv_name,
            object_name=object_name,
            source_pos=source_pos,
            agv_pose=agv_pose,
            object_config=object_config,
            duration=duration,
            step_time=step_time,
            object_index=idx,
            relative_offset=rel_offset,
        )


def handle_post_move_loading_object(agv_name, agv_state, object_config, step_time, action_delay, step_number=None):
    """MOVING step이 끝난 직후 수행해야 하는 픽업 처리. 예: Step 13 PICK_FROM_CROSS."""
    return handle_loading_object(
        agv_name=agv_name,
        agv_state=agv_state,
        object_config=object_config,
        step_time=step_time,
        action_delay=action_delay,
        step_number=step_number,
        rule_type="post_move_loading_rules",
    )

def handle_unloading_object(agv_name, agv_state, object_config, step_time, action_delay, step_number=None):
    command = agv_state.get("command")
    payload = agv_state.get("payload", "EMPTY")
    from_zone = agv_state.get("from")
    to_zone = agv_state.get("to")

    rule = get_rule(object_config, "unloading_rules", agv_name, command, payload, from_zone, to_zone, step_number=step_number)

    if not rule:
        print(f"[OBJECT-UNLOAD-READY] {agv_name}: command={command}, payload={payload}, to={to_zone}. Rule not configured yet.")
        return

    objects = resolve_rule_objects(rule, object_config)
    if not objects:
        objects = get_objects_on_agv(agv_name)

    target_positions_value = rule.get("target_positions")
    if target_positions_value:
        target_positions = [get_position_from_config(object_config, pos) for pos in target_positions_value]
    else:
        target_pos = get_position_from_config(object_config, rule.get("target_position") or rule.get("target_position_key"))
        target_positions = [target_pos for _ in objects]

    agv_pose_value = rule.get("agv_pose") or rule.get("agv_pose_key")
    agv_pose_pos = get_position_from_config(object_config, agv_pose_value)
    agv_pose = None
    if agv_pose_pos is not None:
        agv_pose = (agv_pose_pos[0], agv_pose_pos[1], agv_pose_pos[2], float(rule.get("agv_yaw", 0.0)))

    duration = float(rule.get("duration", 0.0))

    for idx, object_name in enumerate(objects):
        target_pos = target_positions[min(idx, len(target_positions) - 1)] if target_positions else None
        drop_object_from_agv(
            agv_name=agv_name,
            object_name=object_name,
            target_pos=target_pos,
            agv_pose=agv_pose,
            object_config=object_config,
            duration=duration,
            step_time=step_time,
        )

        if rule.get("conveyor_after_drop"):
            path_key = rule.get("conveyor_path", "default")
            conveyor_path = object_config.get("conveyor_paths", {}).get(path_key, [])
            move_object_on_conveyor(
                object_name=object_name,
                conveyor_points=conveyor_path,
                duration=float(rule.get("conveyor_duration", 1.5)),
                step_time=step_time,
            )


def move_between_points(model_name, start_pos, end_pos, speed, step_time, object_config=None):
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
        if model_name in payload_on_agv:
            follow_agv_payload(model_name, x, y, z, yaw, object_config=object_config)
        time.sleep(step_time)


def move_along_path_points(model_name, path_points, speed, step_time, object_config=None):
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
            step_time=step_time,
            object_config=object_config
        )


def get_path_points(line_path_map, start_marker, end_marker):
    paths = line_path_map["paths"]

    forward_key = f"{start_marker}_to_{end_marker}"
    reverse_key = f"{end_marker}_to_{start_marker}"

    if forward_key in paths:
        return paths[forward_key]

    if reverse_key in paths:
        return list(reversed(paths[reverse_key]))

    raise KeyError(f"No line path found: {forward_key} or {reverse_key}")


def validate_agv_state(step_number, agv_name, agv_state):
    command = agv_state.get("command")
    status = agv_state.get("status")
    from_zone = agv_state.get("from")
    to_zone = agv_state.get("to")
    route = agv_state.get("route")

    if command not in VALID_COMMANDS:
        print(f"[WARN] Step {step_number} {agv_name}: unknown command = {command}")

    if status not in VALID_STATUSES:
        print(f"[WARN] Step {step_number} {agv_name}: unknown status = {status}")

    if from_zone not in VALID_ZONES:
        print(f"[WARN] Step {step_number} {agv_name}: unknown from zone = {from_zone}")

    if to_zone not in VALID_ZONES:
        print(f"[WARN] Step {step_number} {agv_name}: unknown to zone = {to_zone}")

    if status == "MOVING" and not route:
        print(f"[WARN] Step {step_number} {agv_name}: MOVING status but route is empty")

    if status in {"LOADING", "UNLOADING", "WAITING", "IDLE"} and route:
        print(f"[WARN] Step {step_number} {agv_name}: {status} status usually does not need route")


def move_by_route(model_name, route_key, zone_route_map, line_path_map, speed, step_time, object_config=None):
    routes = zone_route_map["routes"]

    if route_key not in routes:
        print(f"[SKIP] {model_name}: route key not found in zone_route_map.json -> {route_key}")
        return

    marker_route = routes[route_key]

    if not marker_route:
        print(f"[SKIP] {model_name}: route is empty -> {route_key}")
        return

    if len(marker_route) < 2:
        print(f"[SKIP] {model_name}: route must contain at least 2 markers -> {route_key}")
        return

    print(f"[MOVE] {model_name}: {route_key} / markers={marker_route}")

    for i in range(len(marker_route) - 1):
        start_marker = marker_route[i]
        end_marker = marker_route[i + 1]

        print(f"[{model_name}] marker {start_marker} -> {end_marker}")

        path_points = get_path_points(
            line_path_map=line_path_map,
            start_marker=start_marker,
            end_marker=end_marker
        )

        move_along_path_points(
            model_name=model_name,
            path_points=path_points,
            speed=speed,
            step_time=step_time,
            object_config=object_config
        )


def get_effective_speed(step_number, agv_name, base_speed, object_config=None):
    object_config = object_config or {}
    multiplier = float(object_config.get("default_speed_multiplier", 1.0))

    override = object_config.get("speed_overrides", {}).get(str(step_number), {}).get(agv_name)
    if isinstance(override, dict):
        if "speed" in override:
            return float(override["speed"])
        if "multiplier" in override:
            multiplier = float(override["multiplier"])
    elif isinstance(override, (int, float)):
        multiplier = float(override)

    return max(base_speed * multiplier, 0.001)


def get_effective_action_delay(base_delay, object_config=None):
    object_config = object_config or {}
    if "action_delay" in object_config:
        return float(object_config.get("action_delay", base_delay))
    return base_delay


def run_agv_action(step_number, agv_name, agv_state, zone_route_map, line_path_map, speed, step_time, action_delay, object_config=None, enable_object_motion=False):
    command = agv_state["command"]
    status = agv_state["status"]
    from_zone = agv_state["from"]
    to_zone = agv_state["to"]
    route = agv_state.get("route")
    payload = agv_state.get("payload", "EMPTY")

    effective_speed = get_effective_speed(step_number, agv_name, speed, object_config if enable_object_motion else {})
    effective_action_delay = get_effective_action_delay(action_delay, object_config if enable_object_motion else {})

    print(
        f"[{agv_name}] command={command}, status={status}, "
        f"from={from_zone}, to={to_zone}, payload={payload}, speed={effective_speed:.3f}"
    )

    if status == "MOVING":
        print(f"[AGV-MOVE-START] step={step_number} {agv_name}: route={route}")
        move_by_route(
            model_name=agv_name,
            route_key=route,
            zone_route_map=zone_route_map,
            line_path_map=line_path_map,
            speed=effective_speed,
            step_time=step_time,
            object_config=object_config if enable_object_motion else None
        )
        print(f"[AGV-MOVE-DONE] step={step_number} {agv_name}: route={route}")
        if enable_object_motion:
            post_rule = get_rule(
                object_config or {},
                "post_move_loading_rules",
                agv_name,
                command,
                payload,
                from_zone,
                to_zone,
                step_number=step_number,
            )
            if post_rule is not None:
                handle_post_move_loading_object(agv_name, agv_state, object_config or {}, step_time, action_delay, step_number=step_number)

    elif status == "LOADING":
        print(f"[LOAD] {agv_name}: loading payload={payload}")
        if enable_object_motion:
            handle_loading_object(agv_name, agv_state, object_config or {}, step_time, action_delay, step_number=step_number)
        time.sleep(effective_action_delay)

    elif status == "UNLOADING":
        print(f"[UNLOAD] {agv_name}: unloading payload={payload}")
        if enable_object_motion:
            handle_unloading_object(agv_name, agv_state, object_config or {}, step_time, action_delay, step_number=step_number)
        time.sleep(effective_action_delay)

    elif status == "WAITING":
        print(f"[WAIT] {agv_name}: waiting")
        time.sleep(effective_action_delay)

    elif status == "IDLE":
        print(f"[IDLE] {agv_name}: idle")
        time.sleep(effective_action_delay)

    else:
        print(f"[INFO] {agv_name}: status={status}, no physical action implemented")
        time.sleep(effective_action_delay)


def get_step_duration(step, object_config=None):
    object_config = object_config or {}
    transition = step.get("transition", {})
    if transition.get("type") == "after_duration":
        duration = float(transition.get("duration_sec", 1.0))
        duration *= float(object_config.get("transition_duration_scale", 1.0))
        if "max_transition_delay" in object_config:
            duration = min(duration, float(object_config["max_transition_delay"]))
        return max(duration, 0.0)
    return 0.0



def reset_runtime_object_state():
    """preset 테스트를 위해 object runtime 상태를 초기화한다."""
    with object_state_lock:
        for agv_name in list(payload_on_agv.keys()):
            payload_on_agv[agv_name] = []
        for agv_name in list(payload_relative_offsets.keys()):
            payload_relative_offsets[agv_name] = {}
        conveyor_output_queue.clear()
        fixed_object_poses.clear()


def get_preset_position(object_config, preset_item):
    """preset 항목에서 position_key 또는 position 값을 읽어 좌표로 변환한다."""
    if isinstance(preset_item, dict):
        value = preset_item.get("position") or preset_item.get("position_key")
    else:
        value = preset_item
    return get_position_from_config(object_config, value)


def apply_preset(preset_name, object_config):
    """특정 step 구간 테스트용 시작 상태를 강제로 세팅한다."""
    if not preset_name:
        return

    presets = object_config.get("presets", {})
    if preset_name not in presets:
        available = ", ".join(sorted(presets.keys())) if presets else "none"
        raise KeyError(f"Preset not found: {preset_name}. Available presets: {available}")

    preset = presets[preset_name]
    print("")
    print("====================================")
    print(f"[PRESET] applying preset: {preset_name}")
    print(f"[PRESET] {preset.get('description', '')}")
    print("====================================")

    reset_runtime_object_state()

    # 1) AGV 위치 세팅
    agv_poses = {}
    for agv_name, item in preset.get("agvs", {}).items():
        pos = get_preset_position(object_config, item)
        if pos is None:
            print(f"[PRESET-WARN] {preset_name}: invalid AGV position for {agv_name}")
            continue
        yaw = float(item.get("yaw", 0.0)) if isinstance(item, dict) else 0.0
        set_model_pose(agv_name, pos[0], pos[1], pos[2], yaw)
        agv_poses[agv_name] = (pos[0], pos[1], pos[2], yaw)
        print(f"[PRESET-AGV] {agv_name} -> ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    # 2) 고정 object 위치 세팅
    for object_name, item in preset.get("fixed_objects", {}).items():
        pos = get_preset_position(object_config, item)
        if pos is None:
            print(f"[PRESET-WARN] {preset_name}: invalid object position for {object_name}")
            continue
        yaw = float(item.get("yaw", 0.0)) if isinstance(item, dict) else 0.0
        set_object_pose(object_name, pos[0], pos[1], pos[2], yaw)
        remember_fixed_pose(object_name, pos[0], pos[1], pos[2], yaw)
        print(f"[PRESET-OBJECT] {object_name} fixed -> ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    # 3) AGV 위 payload 세팅이 필요한 preset을 위한 기능
    for agv_name, payload_items in preset.get("payload_on_agv", {}).items():
        agv_pose = agv_poses.get(agv_name)
        if agv_pose is None:
            print(f"[PRESET-WARN] {preset_name}: payload preset ignored because AGV pose is missing: {agv_name}")
            continue
        for idx, item in enumerate(payload_items):
            if isinstance(item, str):
                object_name = item
                rel_offset = None
            else:
                object_name = item.get("object")
                rel_offset_value = item.get("offset") or item.get("offset_key")
                rel_offset = get_position_from_config(object_config, rel_offset_value) if rel_offset_value is not None else None
            if not object_name:
                continue
            forget_fixed_pose(object_name)
            add_object_to_agv(agv_name, object_name, relative_offset=rel_offset)
            obj_pos = get_agv_payload_pose(agv_name, agv_pose[0], agv_pose[1], agv_pose[2], idx, object_config, object_name=object_name)
            set_object_pose(object_name, obj_pos[0], obj_pos[1], obj_pos[2], agv_pose[3])
            print(f"[PRESET-PAYLOAD] {object_name} on {agv_name} -> ({obj_pos[0]:.3f}, {obj_pos[1]:.3f}, {obj_pos[2]:.3f})")

    # 4) 컨베이어 queue 세팅
    with object_state_lock:
        for object_name in preset.get("conveyor_output_queue", []):
            if object_name not in conveyor_output_queue:
                conveyor_output_queue.append(object_name)
    if preset.get("conveyor_output_queue"):
        print(f"[PRESET-CONVEYOR-QUEUE] {conveyor_output_queue}")

    enforce_fixed_object_poses()
    print(f"[PRESET-DONE] {preset_name}")


def main():
    parser = argparse.ArgumentParser(
        description="Run two AGVs from scenario_plan.json using temporary zone_route_map.json."
    )

    parser.add_argument(
        "--scenario-plan",
        default=DEFAULT_SCENARIO_PLAN,
        help="Path to scenario_plan.json"
    )

    parser.add_argument(
        "--zone-route-map",
        default=DEFAULT_ZONE_ROUTE_MAP,
        help="Path to zone_route_map.json"
    )

    parser.add_argument(
        "--line-path-map",
        default=DEFAULT_LINE_PATH_MAP,
        help="Path to line_path_map.json"
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=4.0,
        help="Base AGV movement speed. Object config can apply speed multipliers."
    )

    parser.add_argument(
        "--step-time",
        type=float,
        default=0.03,
        help="Movement update interval."
    )

    parser.add_argument(
        "--action-delay",
        type=float,
        default=0.25,
        help="Delay for LOADING, UNLOADING, WAITING actions."
    )

    parser.add_argument(
        "--enable-object-motion",
        action="store_true",
        help="Enable common object motion handlers for LOADING/UNLOADING and payload follow."
    )

    parser.add_argument(
        "--object-motion-config",
        default=DEFAULT_OBJECT_MOTION_CONFIG,
        help="Path to object_motion_config.json. If missing, object motion rules are skipped."
    )

    parser.add_argument(
        "--preset",
        default=None,
        help="Preset name for direct range testing. Examples: step5, step6, step10, step13, step20"
    )

    parser.add_argument(
        "--start-step",
        type=int,
        default=1,
        help="Start step number"
    )

    parser.add_argument(
        "--end-step",
        type=int,
        default=23,
        help="End step number. Default is 23 because this final object-motion demo intentionally ignores steps 24+"
    )

    args = parser.parse_args()

    scenario_plan = load_json(args.scenario_plan)
    zone_route_map = load_json(args.zone_route_map)
    line_path_map = load_json(args.line_path_map)
    object_config = get_optional_json(args.object_motion_config) if args.enable_object_motion else {}
    if args.enable_object_motion:
        init_fixed_object_poses(object_config)
        if args.preset:
            apply_preset(args.preset, object_config)
    elif args.preset:
        print("[PRESET-WARN] --preset was ignored because --enable-object-motion is not enabled.")

    steps = scenario_plan.get("steps", [])

    print("====================================")
    print("Gazebo Two AGV Scenario Runner")
    print(f"scenario_plan: {args.scenario_plan}")
    print(f"zone_route_map: {args.zone_route_map}")
    print(f"line_path_map: {args.line_path_map}")
    print(f"speed: {args.speed}")
    print(f"object_motion: {args.enable_object_motion}")
    print(f"preset: {args.preset}")
    print(f"step_range: {args.start_step}~{args.end_step}")
    if args.enable_object_motion:
        print(f"object_motion_config: {args.object_motion_config}")
    print("====================================")

    for step in steps:
        step_number = int(step["step"])

        if step_number < args.start_step or step_number > args.end_step:
            continue

        description = step.get("description", "")
        agvs = step.get("agvs", {})

        print("")
        print("====================================")
        print(f"[STEP {step_number}] {description}")
        print("====================================")
        if args.enable_object_motion:
            enforce_fixed_object_poses()

        for agv_name, agv_state in agvs.items():
            validate_agv_state(step_number, agv_name, agv_state)

        threads = []

        for agv_name, agv_state in agvs.items():
            thread = threading.Thread(
                target=run_agv_action,
                args=(
                    step_number,
                    agv_name,
                    agv_state,
                    zone_route_map,
                    line_path_map,
                    args.speed,
                    args.step_time,
                    args.action_delay,
                    object_config,
                    args.enable_object_motion
                )
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        if args.enable_object_motion:
            enforce_fixed_object_poses()

        extra_delay = get_step_duration(step, object_config if args.enable_object_motion else {})
        if extra_delay > 0:
            time.sleep(extra_delay)
            if args.enable_object_motion:
                enforce_fixed_object_poses()

    print("")
    print("====================================")
    print("Scenario complete")
    print("====================================")


if __name__ == "__main__":
    main()