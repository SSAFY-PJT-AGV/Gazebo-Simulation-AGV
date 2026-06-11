#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
move_agv_from_api.py

서버 GET API를 읽어서 Gazebo 안의 AGV 모델을 이동시키는 테스트 코드입니다.

사용 전 확인할 것:
1. Gazebo가 먼저 실행되어 있어야 합니다.
2. Gazebo 모델 이름이 아래 AGV_MODEL_NAMES와 같아야 합니다.
3. 서버 응답에서 AGV01의 current_position, next_position, debug=OK가 내려와야 이동 테스트가 됩니다.

실행:
    python3 move_agv_from_api.py

중지:
    Ctrl + C
"""

import json
import math
import subprocess
import time
import urllib.request
import urllib.error
from typing import Any, Dict, Optional, Tuple

# =========================
# 1. 설정값
# =========================

API_URL = "http://ssafy-agv-management.kro.kr:8080/api/simulation/temp"

# 서버 agv_id와 Gazebo 모델 이름 매핑
# gz model --list 로 실제 모델명을 확인한 뒤 다르면 수정하세요.
AGV_MODEL_NAMES = {
    1: "AGV01",
    2: "AGV02",
}

# AGV 모델의 z 높이입니다. AGV가 바닥에 파묻히거나 공중에 뜨면 이 값을 조정하세요.
MODEL_Z = 0.16

# Gazebo 모델의 앞 방향이 실제 이동 방향과 다르면 이 값을 조정하세요.
# 예: 90도 보정이 필요하면 math.pi / 2
YAW_OFFSET = 0.0

# Gazebo에서 보여줄 이동 속도입니다. 숫자가 클수록 빠릅니다.
SIM_SPEED = 1.0  # Gazebo 좌표 단위 / 초

# 이동 중 pose 업데이트 간격입니다. 작을수록 부드럽지만 VirtualBox에서는 무거울 수 있습니다.
STEP_INTERVAL = 0.08

# 서버 API 조회 주기입니다.
POLL_INTERVAL = 1.0

# True로 바꾸면 API를 한 번만 읽고 종료합니다.
RUN_ONCE = False

# debug가 OK가 아니어도 current_position만 있으면 현재 위치에 배치할지 여부
PLACE_CURRENT_POSITION_EVEN_IF_DEBUG_NOT_OK = True


# =========================
# 2. 유틸 함수
# =========================

def fetch_agv_status() -> Optional[Dict[str, Any]]:
    """서버 API에서 AGV 상태 JSON을 가져옵니다."""
    try:
        with urllib.request.urlopen(API_URL, timeout=3) as response:
            body = response.read().decode("utf-8")
            return json.loads(body)
    except urllib.error.URLError as e:
        print(f"[ERROR] API 연결 실패: {e}")
    except json.JSONDecodeError as e:
        print(f"[ERROR] JSON 파싱 실패: {e}")
    except Exception as e:
        print(f"[ERROR] 알 수 없는 오류: {e}")
    return None


def get_xy(position: Optional[Dict[str, Any]]) -> Optional[Tuple[float, float]]:
    """{'x': 1.0, 'y': 2.0} 형태를 (1.0, 2.0)으로 변환합니다."""
    if position is None:
        return None
    try:
        return float(position["x"]), float(position["y"])
    except (KeyError, TypeError, ValueError):
        return None


def calc_yaw(start: Tuple[float, float], end: Tuple[float, float]) -> float:
    """start에서 end 방향을 바라보는 yaw 값을 계산합니다."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return YAW_OFFSET
    return math.atan2(dy, dx) + YAW_OFFSET


def set_model_pose(model_name: str, x: float, y: float, z: float, yaw: float) -> bool:
    """gz model 명령으로 Gazebo 모델 위치를 변경합니다."""
    cmd = [
        "gz", "model",
        "-m", model_name,
        "-x", f"{x:.4f}",
        "-y", f"{y:.4f}",
        "-z", f"{z:.4f}",
        "-R", "0",
        "-P", "0",
        "-Y", f"{yaw:.4f}",
    ]

    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    if result.returncode != 0:
        print(f"[ERROR] Gazebo 모델 pose 변경 실패: model={model_name}")
        if result.stderr.strip():
            print(f"        stderr: {result.stderr.strip()}")
        print("        모델명이 맞는지 gz model --list 로 확인하세요.")
        return False

    return True


def move_smoothly(model_name: str, start: Tuple[float, float], end: Tuple[float, float]) -> None:
    """start에서 end까지 AGV를 부드럽게 이동시킵니다."""
    sx, sy = start
    ex, ey = end
    distance = math.hypot(ex - sx, ey - sy)

    if distance < 1e-6:
        yaw = calc_yaw(start, end)
        set_model_pose(model_name, sx, sy, MODEL_Z, yaw)
        return

    duration = max(distance / SIM_SPEED, STEP_INTERVAL)
    steps = max(int(duration / STEP_INTERVAL), 1)
    yaw = calc_yaw(start, end)

    print(f"[MOVE] {model_name}: ({sx:.2f}, {sy:.2f}) -> ({ex:.2f}, {ey:.2f}), steps={steps}")

    for i in range(steps + 1):
        t = i / steps
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        ok = set_model_pose(model_name, x, y, MODEL_Z, yaw)
        if not ok:
            break
        time.sleep(STEP_INTERVAL)


def make_segment_key(agv: Dict[str, Any]) -> Tuple[Any, ...]:
    """같은 이동 명령을 반복 실행하지 않기 위한 키입니다."""
    cur = get_xy(agv.get("current_position"))
    nxt = get_xy(agv.get("next_position"))
    return (
        agv.get("agv_id"),
        agv.get("status"),
        agv.get("current_marker"),
        agv.get("next_marker"),
        cur,
        nxt,
        agv.get("payload"),
        agv.get("debug"),
    )


# =========================
# 3. AGV 처리 로직
# =========================

def handle_agv(agv: Dict[str, Any], last_keys: Dict[int, Tuple[Any, ...]]) -> None:
    agv_id = agv.get("agv_id")
    model_name = AGV_MODEL_NAMES.get(agv_id)

    if model_name is None:
        print(f"[SKIP] 알 수 없는 agv_id={agv_id}")
        return

    status = agv.get("status")
    debug = agv.get("debug")
    payload = agv.get("payload")
    current_marker = agv.get("current_marker")
    next_marker = agv.get("next_marker")
    current_xy = get_xy(agv.get("current_position"))
    next_xy = get_xy(agv.get("next_position"))

    key = make_segment_key(agv)
    if last_keys.get(agv_id) == key:
        # 같은 데이터가 계속 내려오면 같은 이동을 반복하지 않습니다.
        return
    last_keys[agv_id] = key

    print(
        f"[API] {model_name}: status={status}, debug={debug}, "
        f"marker={current_marker}->{next_marker}, payload={payload}, "
        f"current={current_xy}, next={next_xy}"
    )

    if status == "OFFLINE":
        print(f"[STOP] {model_name}: OFFLINE 상태라 이동하지 않습니다.")
        return

    if debug != "OK":
        print(f"[WARN] {model_name}: debug={debug}")
        if PLACE_CURRENT_POSITION_EVEN_IF_DEBUG_NOT_OK and current_xy is not None:
            # 위치는 아는데 미션이 없거나 목적지가 없는 경우, 현재 위치에만 배치합니다.
            set_model_pose(model_name, current_xy[0], current_xy[1], MODEL_Z, YAW_OFFSET)
            print(f"[PLACE] {model_name}: 현재 위치에만 배치했습니다.")
        return

    if current_xy is None:
        print(f"[WARN] {model_name}: current_position이 null이라 이동할 수 없습니다.")
        return

    if status == "MOVING":
        if next_xy is None:
            print(f"[WARN] {model_name}: next_position이 null이라 현재 위치에만 배치합니다.")
            set_model_pose(model_name, current_xy[0], current_xy[1], MODEL_Z, YAW_OFFSET)
            return
        move_smoothly(model_name, current_xy, next_xy)
        return

    if status in {"IDLE", "WAITING", "ARRIVED", "LOADING", "UNLOADING", "DONE", "ASSIGNED"}:
        # 이동하지 않는 상태는 현재 위치에 고정합니다.
        set_model_pose(model_name, current_xy[0], current_xy[1], MODEL_Z, YAW_OFFSET)
        print(f"[PLACE] {model_name}: status={status}, 현재 위치에 고정했습니다.")
        return

    if status == "ERROR":
        print(f"[ERROR] {model_name}: ERROR 상태입니다. 이동하지 않습니다.")
        return

    print(f"[WARN] {model_name}: 처리하지 않은 status={status}")


# =========================
# 4. 메인 루프
# =========================

def main() -> None:
    print("====================================")
    print(" Gazebo AGV API 연동 이동 코드 시작")
    print("====================================")
    print(f"API_URL = {API_URL}")
    print(f"AGV_MODEL_NAMES = {AGV_MODEL_NAMES}")
    print("중지하려면 Ctrl + C 를 누르세요.")

    last_keys: Dict[int, Tuple[Any, ...]] = {}

    while True:
        data = fetch_agv_status()
        if data is None:
            time.sleep(POLL_INTERVAL)
            continue

        agvs = data.get("agvs", [])
        if not agvs:
            print("[WARN] agvs 배열이 비어 있습니다.")
        else:
            for agv in agvs:
                handle_agv(agv, last_keys)

        if RUN_ONCE:
            print("[DONE] RUN_ONCE=True 이므로 종료합니다.")
            break

        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[EXIT] 사용자가 중지했습니다.")
