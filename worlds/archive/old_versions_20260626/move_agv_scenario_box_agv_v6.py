#!/usr/bin/env python3
import math
import subprocess
import threading
import time

AGV_Z = 0.16
SIM_SPEED = 10.0
STEP_DT = 0.01
ACTION_SCALE = 0.08
GZ_LOCK = threading.Lock()  # 두 AGV가 동시에 gz model 명령을 보낼 때 충돌하지 않도록 순차 처리

# --------------------------------------------------
# Gazebo pose 제어
# --------------------------------------------------
def run_gz_model(model_name, x, y, z, yaw=0.0):
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
    with GZ_LOCK:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def set_agv_pose(agv_name, x, y, yaw=0.0):
    run_gz_model(agv_name, x, y, AGV_Z, yaw)


def move_path(agv_name, route, speed=SIM_SPEED, step_dt=STEP_DT):
    for i in range(len(route) - 1):
        x1, y1 = route[i]
        x2, y2 = route[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        distance = math.sqrt(dx * dx + dy * dy)
        if distance == 0:
            continue

        yaw = math.atan2(dy, dx)
        steps = max(2, int(distance / (speed * step_dt)))

        for step in range(steps + 1):
            t = step / steps
            x = x1 + dx * t
            y = y1 + dy * t
            set_agv_pose(agv_name, x, y, yaw)
            time.sleep(step_dt)


def action(agv_name, message, sec=1.2):
    print(f"[{agv_name}] {message}")
    time.sleep(sec * ACTION_SCALE)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def rotate_in_place(agv_name, point, start_yaw, end_yaw, duration=0.35, steps=10):
    """제자리 회전이 눈에 보이도록 yaw를 여러 단계로 바꿉니다."""
    x, y = point
    diff = normalize_angle(end_yaw - start_yaw)
    for i in range(steps + 1):
        t = i / steps
        yaw = normalize_angle(start_yaw + diff * t)
        set_agv_pose(agv_name, x, y, yaw)
        time.sleep((duration * ACTION_SCALE) / max(1, steps))


def pickup_motion(agv_name, base_point, approach_axis="y", direction=1, yaw=0.0, distance=0.22, repeat=1, start_yaw=None):
    """
    픽업/하역이 화면에서 보이도록 짧게 접근했다가 라인 위 정지점으로 복귀하는 보조 동작입니다.
    v6: 픽업 전 제자리 회전은 제거하고, 픽업 후 복귀 때처럼 yaw를 즉시 바꿉니다.
    """
    x, y = base_point

    # 기존 v5에서는 rotate_in_place()로 천천히 나눠 회전했습니다.
    # v6에서는 끊김을 줄이기 위해 방향만 즉시 바꾼 뒤 바로 픽업 접근 동작을 합니다.
    set_agv_pose(agv_name, x, y, yaw)
    time.sleep(0.05 * ACTION_SCALE)

    for _ in range(repeat):
        set_agv_pose(agv_name, x, y, yaw)
        time.sleep(0.10 * ACTION_SCALE)
        if approach_axis == "x":
            set_agv_pose(agv_name, x + direction * distance, y, yaw)
        else:
            set_agv_pose(agv_name, x, y + direction * distance, yaw)
        time.sleep(0.22 * ACTION_SCALE)
        set_agv_pose(agv_name, x, y, yaw)
        time.sleep(0.10 * ACTION_SCALE)


def log_system(message):
    print(f"[SYSTEM] {message}")

# --------------------------------------------------
# 이동 코드용 WAYPOINTS 좌표표
# 박스 중심이 아니라, AGV가 실제로 멈출 검정 라인 위 좌표입니다.
# 나중에 실제 마커를 정하면 이 이름을 ARUCO/QR ID와 매핑하면 됩니다.
# --------------------------------------------------
WAYPOINTS = {
    # AGV 시작 위치
    "AGV01_START": (2.20, -2.55),
    "AGV02_START": (3.05, 3.10),

    # 자재 상자 보관 구역 근처 정지 위치: 하단 검정 라인 위
    "MATERIAL_PICKUP_RED": (1.10, -2.55),
    "MATERIAL_PICKUP_YELLOW": (0.00, -2.55),
    "MATERIAL_PICKUP_BLUE": (-1.10, -2.55),
    "MATERIAL_ZONE": (1.10, -2.55),

    # 컨베이어 하역/분류 정지 위치: 컨베이어 중심이 아니라 왼쪽 검정 라인 위
    "CONVEYOR_DROPOFF": (-3.35, -1.85),
    "CONVEYOR_PICKUP": (-3.35, 1.25),
    "RAMP_FRONT": (-3.35, 1.25),
    "CONVEYOR_ZONE": (-3.35, -1.85),

    # 완성품 상자 보관 구역 근처 정지 위치: 상단 검정 라인 위
    "FINISHED_PICKUP_BLUE": (-1.55, 2.25),
    "FINISHED_PICKUP_YELLOW": (-0.35, 2.25),
    "FINISHED_PICKUP_RED": (0.85, 2.25),
    "FINISHED_ZONE": (-0.35, 2.25),

    # 입출고구역 및 교차/버퍼 정지 위치
    "INOUT_ZONE": (3.05, 3.10),
    "BUFFER_ZONE": (3.05, -0.30),
}

AGV01_START = WAYPOINTS["AGV01_START"]
MATERIAL_ZONE = WAYPOINTS["MATERIAL_ZONE"]
CONVEYOR_ZONE = WAYPOINTS["CONVEYOR_ZONE"]
BUFFER_ZONE = WAYPOINTS["BUFFER_ZONE"]

AGV02_START = WAYPOINTS["AGV02_START"]
INOUT_ZONE = WAYPOINTS["INOUT_ZONE"]
FINISHED_ZONE = WAYPOINTS["FINISHED_ZONE"]

# --------------------------------------------------
# 라인 기반 경로 좌표
# --------------------------------------------------
BOTTOM_RIGHT_TO_LEFT = [
    (2.20, -2.55),
    (1.55, -2.55),
    (0.80, -2.55),
    (0.00, -2.55),
    (-0.80, -2.55),
    (-1.55, -2.55),
    (-2.20, -2.55),
    (-3.20, -2.05),
    (-3.35, -1.45),
    (-3.35, -1.25),
]

LEFT_TO_TOP = [
    (-3.35, -0.45),
    (-3.35, 0.05),
    (-3.35, 0.55),
    (-3.35, 1.05),
    (-3.35, 1.45),
    (-2.65, 2.25),
    (-1.55, 2.25),
    (-0.35, 2.25),
]

# 입출고구역 ↔ 완성품 보관 구역 경로
# v4: 90도 직선 꺾임을 줄이기 위해 오른쪽 S자 곡선 쪽 좌표를 더 촘촘히 사용
INOUT_TO_TOP = [
    WAYPOINTS["INOUT_ZONE"],
    (3.05, 2.50),
    (3.05, 1.95),
    (3.05, 1.62),
    (2.94, 1.72),
    (2.84, 1.88),
    (2.72, 2.03),
    (2.58, 2.15),
    (2.42, 2.22),
    (2.05, 2.25),
    (1.25, 2.25),
    (0.45, 2.25),
    WAYPOINTS["FINISHED_ZONE"],
]

TOP_TO_INOUT = list(reversed(INOUT_TO_TOP))

INOUT_TO_BUFFER = [
    WAYPOINTS["INOUT_ZONE"],
    (3.05, 2.50),
    (3.05, 1.95),
    (3.05, 1.40),
    (3.05, 0.85),
    (3.05, 0.30),
    WAYPOINTS["BUFFER_ZONE"],
]

BUFFER_TO_INOUT = list(reversed(INOUT_TO_BUFFER))

MATERIAL_TO_CONVEYOR = [
    WAYPOINTS["MATERIAL_ZONE"],
    (0.80, -2.55),
    (0.00, -2.55),
    (-0.80, -2.55),
    (-1.55, -2.55),
    (-2.20, -2.55),
    (-3.20, -2.05),
    WAYPOINTS["CONVEYOR_DROPOFF"],
]

CONVEYOR_TO_MATERIAL = list(reversed(MATERIAL_TO_CONVEYOR))

MATERIAL_TO_BUFFER = [
    WAYPOINTS["MATERIAL_ZONE"],
    (1.55, -2.55),
    (2.20, -2.55),
    (2.68, -2.50),
    (3.05, -2.40),
    (3.05, -1.90),
    (3.05, -1.35),
    (3.05, -0.80),
    WAYPOINTS["BUFFER_ZONE"],
]

BUFFER_TO_MATERIAL = list(reversed(MATERIAL_TO_BUFFER))

FINISHED_TO_RAMP_FRONT = [
    WAYPOINTS["FINISHED_ZONE"],
    (-1.55, 2.25),
    (-2.65, 2.25),
    (-3.35, 1.45),
    WAYPOINTS["RAMP_FRONT"],
]

RAMP_FRONT_TO_FINISHED = list(reversed(FINISHED_TO_RAMP_FRONT))

# 기존 이름 호환용: 실제로는 컨베이어까지 가지 않고 경사로 앞까지만 이동
FINISHED_TO_CONVEYOR = FINISHED_TO_RAMP_FRONT

CONVEYOR_TO_FINISHED = RAMP_FRONT_TO_FINISHED

# --------------------------------------------------
# AGV02가 교차구역에 자재를 먼저 놓고, AGV01이 나중에 가져가는 동기화 이벤트
# --------------------------------------------------
supply_box_ready = threading.Event()
empty_box_ready = threading.Event()


def agv01_scenario():
    set_agv_pose("AGV01", AGV01_START[0], AGV01_START[1], 0.0)
    action("AGV01", "자재 보관 구역에서 시작", 0.8)

    # 기본 자재 운반 1회
    move_path("AGV01", [AGV01_START, MATERIAL_ZONE])
    action("AGV01", "자재 픽업 위치 도착", 0.4)
    pickup_motion("AGV01", MATERIAL_ZONE, approach_axis="y", direction=1, yaw=math.pi / 2, distance=0.25, start_yaw=math.pi)
    action("AGV01", "자재 픽업 완료", 0.6)
    move_path("AGV01", MATERIAL_TO_CONVEYOR)
    action("AGV01", "컨베이어 벨트에 자재 하역", 1.0)
    move_path("AGV01", CONVEYOR_TO_MATERIAL)
    action("AGV01", "자재 보관 구역으로 복귀", 0.8)

    # AGV02가 교차구역에 보급 자재를 먼저 놓을 때까지 AGV01은 기본 업무를 한 번 더 수행
    action("AGV01", "다음 자재 픽업", 0.5)
    pickup_motion("AGV01", MATERIAL_ZONE, approach_axis="y", direction=1, yaw=math.pi / 2, distance=0.25, start_yaw=math.pi)
    move_path("AGV01", MATERIAL_TO_CONVEYOR)
    action("AGV01", "컨베이어 벨트에 추가 자재 하역", 0.8)
    move_path("AGV01", CONVEYOR_TO_MATERIAL)
    action("AGV01", "빈 상자 적재", 0.8)

    if not supply_box_ready.is_set():
        action("AGV01", "교차구역 보급 상자 준비 대기", 0.8)
        supply_box_ready.wait()

    move_path("AGV01", MATERIAL_TO_BUFFER)
    action("AGV01", "교차구역에서 빈 상자 하역", 0.9)
    empty_box_ready.set()
    action("AGV01", "교차구역에서 보급 자재 상자 적재", 0.9)
    move_path("AGV01", BUFFER_TO_MATERIAL)
    action("AGV01", "보급 자재 상자를 자재 보관 구역으로 운반 완료", 1.0)

    print("[AGV01] scenario done")


def agv02_scenario():
    set_agv_pose("AGV02", AGV02_START[0], AGV02_START[1], -math.pi / 2)
    action("AGV02", "입출고구역에서 시작", 0.8)

    # 1. 입출고구역에서 빈 상자 가져와 완성품 상자 구역에 놓기
    action("AGV02", "입출고구역에서 빈 상자 픽업", 0.5)
    pickup_motion("AGV02", INOUT_ZONE, approach_axis="x", direction=1, yaw=0.0, distance=0.18, start_yaw=-math.pi / 2)
    move_path("AGV02", INOUT_TO_TOP)
    action("AGV02", "완성품 상자 보관 구역 도착", 0.4)
    pickup_motion("AGV02", FINISHED_ZONE, approach_axis="y", direction=-1, yaw=-math.pi / 2, distance=0.22, start_yaw=math.pi)
    action("AGV02", "완성품 상자 보관 구역에 빈 상자 배치 완료", 0.6)

    # 2. 컨베이어에서 나온 상자 분류
    move_path("AGV02", FINISHED_TO_CONVEYOR)
    action("AGV02", "경사로 앞에서 컨베이어 배출 상자 확인 및 분류", 0.8)
    move_path("AGV02", CONVEYOR_TO_FINISHED)
    action("AGV02", "완성품 상자 보관 구역에 분류 완료", 0.9)

    # 3. 완성품 상자 입출고구역에 가져다놓기
    action("AGV02", "완성품 상자 적재", 0.5)
    pickup_motion("AGV02", FINISHED_ZONE, approach_axis="y", direction=-1, yaw=-math.pi / 2, distance=0.22, start_yaw=0.0)
    move_path("AGV02", TOP_TO_INOUT)
    action("AGV02", "입출고구역에 완성품 상자 하역", 0.9)

    # 4. 입출고구역 -> 완성품 상자 구역으로 빈 상자 가져다놓기
    action("AGV02", "입출고구역에서 빈 상자 재픽업", 0.5)
    pickup_motion("AGV02", INOUT_ZONE, approach_axis="x", direction=1, yaw=0.0, distance=0.18, start_yaw=-math.pi / 2)
    move_path("AGV02", INOUT_TO_TOP)
    action("AGV02", "완성품 상자 보관 구역에 빈 상자 추가 배치", 0.5)
    pickup_motion("AGV02", FINISHED_ZONE, approach_axis="y", direction=-1, yaw=-math.pi / 2, distance=0.22, start_yaw=math.pi)
    move_path("AGV02", TOP_TO_INOUT)

    # 5. 자재 부족 신호 받고 입출고구역 -> 교차구역으로 자재 보충
    log_system("자재 부족 신호 발생 → AGV02에 보급 작업 배정")
    action("AGV02", "입출고구역에서 보급 자재 상자 적재", 0.8)
    move_path("AGV02", INOUT_TO_BUFFER)
    action("AGV02", "교차구역에 보급 자재 상자 하역", 1.0)
    supply_box_ready.set()
    log_system("교차구역 상태 변경: supply_box_ready")

    # AGV02는 AGV01을 기다리지 않고 입출고구역으로 빠졌다가, 빈 상자 준비 신호를 받고 다시 이동
    move_path("AGV02", BUFFER_TO_INOUT)
    action("AGV02", "입출고구역 복귀 후 다음 작업 대기", 0.8)

    if not empty_box_ready.is_set():
        action("AGV02", "AGV01의 빈 상자 하역 신호 대기", 0.8)
        empty_box_ready.wait()

    # 6. AGV01이 가져온 빈 상자 교차구역 -> 입출고구역 이동
    move_path("AGV02", INOUT_TO_BUFFER)
    action("AGV02", "교차구역에서 빈 상자 적재", 0.9)
    move_path("AGV02", BUFFER_TO_INOUT)
    action("AGV02", "입출고구역에 빈 상자 반납", 1.0)

    print("[AGV02] scenario done")


def main():
    print("Gazebo AGV 협업 시나리오 데모")
    print("전제: Gazebo에서 agv_factory_final_box_agv_v2.world가 먼저 실행되어 있어야 합니다.")
    print("중지: Ctrl + C")

    set_agv_pose("AGV01", AGV01_START[0], AGV01_START[1], 0.0)
    set_agv_pose("AGV02", AGV02_START[0], AGV02_START[1], -math.pi / 2)
    time.sleep(1)

    t1 = threading.Thread(target=agv01_scenario)
    t2 = threading.Thread(target=agv02_scenario)

    # 두 AGV는 항상 동시에 작업 시작
    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Demo finished")


if __name__ == "__main__":
    main()
