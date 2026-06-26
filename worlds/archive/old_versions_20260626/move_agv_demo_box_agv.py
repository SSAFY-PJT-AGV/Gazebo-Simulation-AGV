#!/usr/bin/env python3
import math
import subprocess
import threading
import time

# Gazebo Classic에서 AGV 모델의 pose를 직접 바꿔서 움직이는 데모 스크립트입니다.
# 사용 전제:
# 1) Gazebo가 먼저 실행되어 있어야 합니다.
# 2) world 안에 AGV01, AGV02 모델이 있어야 합니다.

AGV_Z = 0.16
# 숫자가 클수록 빠르게 움직입니다.
SIM_SPEED = 0.75

# 숫자가 작을수록 더 부드럽지만 컴퓨터가 느리면 버벅일 수 있습니다.
STEP_DT = 0.08


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

    subprocess.run(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def set_agv_pose(agv_name, x, y, yaw):
    run_gz_model(agv_name, x, y, AGV_Z, yaw)


def move_route(agv_name, route, speed=SIM_SPEED, step_dt=STEP_DT):
    print(f"[{agv_name}] move start")

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

    print(f"[{agv_name}] move done")


# AGV01: 오른쪽 하단 시작 → 하단 라인 → 왼쪽 컨베이어 쪽 → 상단 → 오른쪽 라인 → 시작점 복귀
AGV01_ROUTE = [
    (2.20, -2.55),
    (1.55, -2.55),
    (0.80, -2.55),
    (0.00, -2.55),
    (-0.80, -2.55),
    (-1.55, -2.55),
    (-2.20, -2.55),
    (-3.20, -2.05),
    (-3.35, -1.45),
    (-3.35, -0.45),
    (-3.35, 0.55),
    (-3.35, 1.45),
    (-2.65, 2.25),
    (-1.55, 2.25),
    (-0.35, 2.25),
    (0.85, 2.25),
    (2.05, 2.25),
    (2.58, 2.15),
    (2.84, 1.88),
    (3.05, 1.62),
    (2.76, 1.40),
    (2.58, 0.98),
    (2.72, 0.54),
    (3.05, -0.55),
    (3.05, -1.90),
    (2.68, -2.50),
    (2.20, -2.55),
]

# AGV02: 상단 시작 → 오른쪽 입출고구역 방향 → 교차구역 근처 → 상단 시작점 복귀
AGV02_ROUTE = [
    (-1.35, 2.25),
    (-0.35, 2.25),
    (0.85, 2.25),
    (2.05, 2.25),
    (3.05, 2.25),
    (3.05, 2.75),
    (3.05, 3.10),
    (3.05, 2.25),
    (3.05, 1.40),
    (3.05, 0.30),
    (3.05, -0.80),
    (3.05, -1.90),
    (2.68, -2.50),
    (2.20, -2.55),
    (1.10, -2.55),
    (0.00, -2.55),
    (-1.10, -2.55),
    (-2.20, -2.55),
    (-3.20, -2.05),
    (-3.35, -1.45),
    (-3.35, -0.45),
    (-3.35, 0.55),
    (-3.35, 1.45),
    (-2.65, 2.25),
    (-1.35, 2.25),
]


def main():
    print("Gazebo AGV move demo")
    print("Gazebo가 먼저 켜져 있어야 합니다.")
    print("중지하려면 Ctrl + C를 누르세요.")

    # 시작 위치 정렬
    set_agv_pose("AGV01", AGV01_ROUTE[0][0], AGV01_ROUTE[0][1], 0.0)
    set_agv_pose("AGV02", AGV02_ROUTE[0][0], AGV02_ROUTE[0][1], 0.0)
    time.sleep(1)

    # 두 AGV를 동시에 이동
    t1 = threading.Thread(target=move_route, args=("AGV01", AGV01_ROUTE))
    t2 = threading.Thread(target=move_route, args=("AGV02", AGV02_ROUTE))

    t1.start()
    time.sleep(1.5)  # 두 대가 완전히 겹쳐 출발하지 않도록 약간 지연
    t2.start()

    t1.join()
    t2.join()

    print("Demo finished")


if __name__ == "__main__":
    main()
