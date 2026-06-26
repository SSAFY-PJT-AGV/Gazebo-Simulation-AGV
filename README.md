# Gazebo AGV Simulation

본 폴더는 SSAFY AGV 프로젝트의 **시뮬레이션 파트**입니다.

본 시뮬레이션은 
**서버가 관리하는 AGV 상태와 위치 정보를 Gazebo에 반영하여 제조물류 흐름을 시각화하는 상태 기반 시뮬레이션**입니다.

---

## 1. 시뮬레이션 목적

- 삼성전자 싸피공장 내 제조물류 흐름 시각화
- AGV01 / AGV02 협업 시나리오 검증
- 자재 투입, 완제품 회수, 부족 자재 보급, 빈 상자 회수 흐름 표현
- 서버 API 기반 AGV 상태·위치 반영
- 관제 웹에서 FACTORY MAP 클릭 시 Gazebo 실행 연동
- Gazebo 카메라 기반 ArUco 마커 인식 검증

---

## 2. 전체 구조

```text
실제 AGV
    ↓ 상태 / 마커 / 적재 정보 전송

Backend Server
    ↓ AGV 상태 API 제공

Gazebo Simulation
    ↓ 서버 상태 기반 AGV 위치 반영

관제 웹
    ↓ FACTORY MAP 클릭

Gazebo Launcher
    ↓

Gazebo 실행
```

시뮬레이션은 크게 두 가지 방식으로 실행할 수 있습니다.

| 실행 방식 | 설명 |
|---|---|
| 로컬 시나리오 기반 실행 | 서버 없이 JSON 시나리오 파일을 읽어 AGV01/AGV02 이동 |
| 서버 API 기반 실행 | 백엔드 서버에서 AGV 상태·위치 데이터를 받아 Gazebo에 반영 |

---

## 3. 디렉토리 구조

agv_sim/
├── config/
│   ├── marker_map.json
│   ├── line_path_map_final_v6_cross_7_9_names.json
│   ├── zone_route_map_final_cross_7_9_v2_empty_box_return.json
│   └── scenario_plan_cross_7_9_v3_empty_box_return.json
│
├── scripts/
│   ├── move_agv_from_scenario_v2_empty_box_return.py
│   ├── move_agv_from_api.py
│   ├── move_agv_follow_line_path.py
│   ├── gazebo_launcher.py
│   ├── detect_aruco_demo.py
│   ├── detect_aruco_demo_agv02.py
│   └── save_camera_frame.py
│
├── worlds/
│   ├── final/
│   │   ├── make_agv_world_final_box_agv_v2_revised_v7_red_cube_moved.py
│   │   └── agv_factory_final_box_agv_v2.world
│   │
│   ├── aruco_markers/
│   │   ├── generate_aruco_markers.py
│   │   └── make_aruco_marker_models.py
│   │
│   └── archive/
│
├── resource/
├── test/
└── README.md

<img width="1045" height="1099" alt="image" src="https://github.com/user-attachments/assets/143304f0-1d9d-49f4-93a2-01df9af11918" />

```

---

## 4. 폴더별 역할

| 폴더 | 역할 |
|---|---|
| `config/` | 마커 좌표, AGV 이동 경로, 구역 간 route, 전체 작업 시나리오 JSON 저장 |
| `scripts/` | AGV 이동 실행, 서버 API 연동, Gazebo 실행 연동, ArUco 인식 코드 저장 |
| `worlds/final/` | 최종 Gazebo world 생성 파일 및 최종 world 파일 저장 |
| `worlds/aruco_markers/` | ArUco 마커 이미지 및 Gazebo 마커 모델 생성 코드 저장 |
| `worlds/archive/` | 이전 버전 world 파일 백업 |
| `test/` | ROS2 패키지 기본 테스트 파일 |

---

## 5. 핵심 파일 설명

| 구분 | 파일명 | 역할 |
|---|---|---|
| World 생성 | `make_agv_world_final_box_agv_v2_revised_v7_red_cube_moved.py` | Gazebo 공장 MAP, AGV01/AGV02, 컨베이어, 트레이, 보관 구역, 카메라 등을 생성하는 Python 파일 |
| World 실행 파일 | `agv_factory_final_box_agv_v2.world` | Gazebo에서 실제로 실행되는 최종 world 파일 |
| 마커 기준표 | `marker_map.json` | ArUco marker_id와 위치 의미를 연결하는 기준표 |
| 라인 경로 | `line_path_map_final_v6_cross_7_9_names.json` | AGV가 검정색 라인 중심을 따라 이동할 수 있도록 waypoint별 좌표를 정의한 파일 |
| 구역 경로 | `zone_route_map_final_cross_7_9_v2_empty_box_return.json` | `MATERIAL_BOX_STORAGE → CONVEYOR_START` 같은 구역 이동을 waypoint 경로로 변환하는 파일 |
| 전체 시나리오 | `scenario_plan_cross_7_9_v3_empty_box_return.json` | AGV01/AGV02의 전체 작업 순서, 상태, 명령, 적재물을 정의한 파일 |
| 시나리오 실행 | `move_agv_from_scenario_v2_empty_box_return.py` | 로컬 JSON 시나리오를 읽어 Gazebo에서 AGV01/AGV02를 단계별로 이동시키는 코드 |
| 서버 API 연동 | `move_agv_from_api.py` | 서버 API에서 AGV 상태와 위치를 받아 Gazebo AGV 위치에 반영하는 코드 |
| 경로 테스트 | `move_agv_follow_line_path.py` | 특정 waypoint 경로만 테스트하기 위한 이동 코드 |
| 웹 실행 연동 | `gazebo_launcher.py` | 관제 웹에서 FACTORY MAP 클릭 시 Gazebo와 API 이동 코드를 실행하는 로컬 Flask 서버 |
| ArUco 인식 | `detect_aruco_demo.py` | AGV01 카메라 영상에서 ArUco marker_id를 인식하는 코드 |
| ArUco 인식 | `detect_aruco_demo_agv02.py` | AGV02 카메라 영상에서 ArUco marker_id를 인식하는 코드 |
| 카메라 저장 | `save_camera_frame.py` | Gazebo 카메라 화면을 이미지로 저장하는 보조 코드 |
| 마커 생성 | `generate_aruco_markers.py` | ArUco 마커 이미지를 생성하는 코드 |
| 마커 모델 생성 | `make_aruco_marker_models.py` | 생성한 ArUco 이미지를 Gazebo 모델로 변환하는 코드 |

---

## 6. 시뮬레이션 구성 요소

Gazebo world 안에는 다음 요소들이 구현되어 있습니다.

| 구성 요소 | 설명 |
|---|---|
| 공장 MAP | 검정색 라인 기반 제조물류 이동 경로 |
| AGV01 | 자재 보관 구역에서 자재를 픽업하여 컨베이어 투입 구역으로 운반 |
| AGV02 | 완제품 회수, 부족 자재 보급, 빈 상자 회수 담당 |
| 자재 보관 구역 | CHIP, SENSOR 등 자재를 보관하는 트레이 영역 |
| 컨베이어 구역 | AGV01이 자재를 투입하고, AGV02가 완제품을 회수하는 구역 |
| 완제품 보관 구역 | 완제품 상자를 보관하는 구역 |
| 입출고 구역 | 입고 자재 및 출고 완제품이 위치하는 구역 |
| 교차 구역 | AGV01/AGV02가 보급 상자와 빈 상자를 주고받는 구역 |
| Gazebo 카메라 | AGV01/AGV02에 부착된 RGB 카메라 |
| ArUco 마커 | 카메라 기반 marker_id 인식 검증용 마커 |

---

## 7. AGV 역할

| AGV | 주요 역할 |
|---|---|
| AGV01 | 자재 보관 구역에서 CHIP, SENSOR를 픽업하여 컨베이어 투입 구역으로 운반 |
| AGV02 | 완제품 회수, 부족 자재 보급, 빈 상자 회수 담당 |

### AGV01 흐름

```text
자재 보관 구역
    ↓
자재 픽업
    ↓
컨베이어 투입 구역 이동
    ↓
자재 하역
    ↓
필요 시 교차구역에서 보급 자재 회수
```

### AGV02 흐름

```text
입출고 구역
    ↓
빈 상자 또는 보급 상자 픽업
    ↓
완제품 회수 / 부족 자재 보급
    ↓
교차구역 빈 상자 회수
    ↓
입출고 구역 반납
```

---

## 8. AGV 이동 구조

AGV는 실제 바퀴 물리 제어를 사용하지 않고,  
**Gazebo 모델의 pose를 직접 변경하는 방식**으로 이동합니다.

```text
scenario_plan
    ↓
zone_route_map
    ↓
line_path_map
    ↓
gz model 명령으로 AGV pose 변경
```

예시:

```text
MATERIAL_BOX_STORAGE_TO_CONVEYOR_START
    ↓
[4, 3, 2, 1]
    ↓
4_to_3 좌표 목록
3_to_2 좌표 목록
2_to_1 좌표 목록
    ↓
AGV01 이동
```

---

## 9. JSON 파일 역할

### 9-1. `scenario_plan`

AGV01과 AGV02의 전체 작업 순서를 정의합니다.

주요 정보:

| 항목 | 의미 |
|---|---|
| `step` | 시나리오 단계 번호 |
| `command` | AGV가 수행할 명령 |
| `status` | AGV 현재 상태 |
| `from` | 출발 구역 |
| `to` | 도착 구역 |
| `route` | 사용할 이동 경로 이름 |
| `payload` | AGV가 싣고 있는 물건 |

주요 status:

| status | 의미 |
|---|---|
| `MOVING` | 경로를 따라 이동 |
| `LOADING` | 적재 |
| `UNLOADING` | 하역 |
| `WAITING` | 대기 |
| `IDLE` | 정지 |
| `DONE` | 작업 완료 |

---

### 9-2. `zone_route_map`

구역명을 waypoint 경로로 변환합니다.

예시:

```text
MATERIAL_BOX_STORAGE_TO_CONVEYOR_START = [4, 3, 2, 1]
```

즉, 사람이 이해하기 쉬운 구역명을 AGV가 이동할 waypoint 목록으로 바꿔줍니다.

---

### 9-3. `line_path_map`

각 waypoint 사이의 실제 라인 중심 좌표를 정의합니다.

예시:

```text
4_to_5 = [
  [0.0, -2.55, 0.16],
  [0.688, -2.55, 0.16],
  [1.333, -2.55, 0.16],
  [1.892, -2.55, 0.16]
]
```

`line_path_map`의 waypoint는 실제 ArUco 마커가 아니라,  
**Gazebo 시뮬레이션에서 AGV가 따라갈 가상 경로점**입니다.

---

### 9-4. `marker_map`

ArUco marker_id와 위치 의미를 연결하는 기준표입니다.

예시:

```text
marker_id
    ↓
위치 이름
    ↓
Gazebo 좌표
    ↓
설명
```

`marker_map`은 위치 인식/설명용 기준표이고,  
AGV의 실제 이동은 `line_path_map`을 기준으로 수행합니다.

---

## 10. 시뮬레이션 실행 방법

### 10-1. Gazebo world 실행

```bash
source /opt/ros/humble/setup.bash
gazebo --verbose ~/gazebo_agv_ws/src/agv_sim/worlds/final/agv_factory_final_box_agv_v2.world
```

---

### 10-2. 로컬 시나리오 기반 실행

서버 없이 전체 시나리오를 확인할 때 사용합니다.

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts

python3 move_agv_from_scenario_v2_empty_box_return.py \
  --scenario-plan ../config/scenario_plan_cross_7_9_v3_empty_box_return.json \
  --zone-route-map ../config/zone_route_map_final_cross_7_9_v2_empty_box_return.json \
  --line-path-map ../config/line_path_map_final_v6_cross_7_9_names.json \
  --speed 4.0
```

---

### 10-3. 특정 구간 이동 테스트

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts

python3 move_agv_follow_line_path.py \
  --agv AGV01 \
  --line-path-map ../config/line_path_map_final_v6_cross_7_9_names.json \
  --route 5,6,7,8,9 \
  --speed 4.0
```

예시:

| route | 의미 |
|---|---|
| `5,6,7,8,9` | AGV01 시작점 근처에서 오른쪽 세로 라인까지 이동 테스트 |
| `1,2,3,4,5` | 하단 라인 이동 테스트 |
| `13,12,11,10,9` | AGV02 시작점에서 오른쪽 방향 이동 테스트 |

---

### 10-4. 서버 API 기반 실행

서버 API에서 AGV 상태를 받아 Gazebo에 반영할 때 사용합니다.

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts
python3 move_agv_from_api.py
```

서버 API 응답에는 다음 정보가 필요합니다.

| 필드 | 의미 |
|---|---|
| `agv_id` | AGV01 / AGV02 구분 |
| `status` | `MOVING`, `IDLE`, `LOADING`, `UNLOADING` 등 상태 |
| `current_position` | 현재 Gazebo 좌표 |
| `next_position` | 다음 Gazebo 좌표 |
| `payload` | 적재물 정보 |
| `current_marker` | 현재 마커 |
| `next_marker` | 다음 마커 |

---

### 10-5. 관제 웹에서 Gazebo 실행

관제 웹에서 FACTORY MAP을 클릭하면 로컬 launcher 서버가 Gazebo를 실행합니다.

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts
python3 gazebo_launcher.py
```

상태 확인:

```bash
curl http://127.0.0.1:5001/health
```

Gazebo 실행 요청:

```bash
curl -X POST http://127.0.0.1:5001/open-gazebo
```

정상 흐름:

```text
관제 웹 FACTORY MAP 클릭
    ↓
http://127.0.0.1:5001/open-gazebo 호출
    ↓
gazebo_launcher.py 요청 수신
    ↓
Gazebo world 실행
    ↓
move_agv_from_api.py 실행
    ↓
서버 상태 기반 AGV 이동
```

---

## 11. ArUco 인식 구조

Gazebo 안의 AGV 모델에는 RGB 카메라가 포함되어 있습니다.

| AGV | ROS2 image topic |
|---|---|
| AGV01 | `/AGV01/agv01_camera/image_raw` |
| AGV02 | `/AGV02/agv02_camera/image_raw` |

ArUco 인식 흐름:

```text
Gazebo 카메라
    ↓
ROS2 image_raw topic
    ↓
OpenCV ArUco
    ↓
marker_id 검출
```

AGV01 ArUco 실행:

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts
python3 detect_aruco_demo.py
```

AGV02 ArUco 실행:

```bash
cd ~/gazebo_agv_ws/src/agv_sim/scripts
python3 detect_aruco_demo_agv02.py
```

ArUco 인식은 AGV 이동 제어의 핵심 로직이 아니라,  
실제 AGV의 비전 기반 위치 인식 구조를 Gazebo에서도 검증하기 위한 보조 기능입니다.

---

## 12. 서버 API 기반 이동 구조

서버 API 기반 실행 시 구조는 다음과 같습니다.

```text
실제 AGV
    ↓ status / marker / payload 전송

Backend Server
    ↓ AGV 상태 저장 및 API 제공

move_agv_from_api.py
    ↓ current_position / next_position 읽기

Gazebo
    ↓ AGV01 / AGV02 위치 반영
```

`move_agv_from_api.py`는 서버에서 받은 `status`에 따라 다음과 같이 처리합니다.

| status | Gazebo 처리 |
|---|---|
| `MOVING` | `current_position → next_position`으로 AGV 이동 |
| `IDLE` | 현재 위치에 정지 |
| `LOADING` | 현재 위치에 정지, 적재 상태로 처리 |
| `UNLOADING` | 현재 위치에 정지, 하역 상태로 처리 |
| `WAITING` | 현재 위치에 대기 |
| `ARRIVED` | 목적지 도착 상태 |
| `ERROR` | 오류 상태로 이동하지 않음 |
| `OFFLINE` | 통신 불가 상태로 이동하지 않음 |

---

## 13. 기술 스택

| 구분 | 기술 |
|---|---|
| 시뮬레이션 | Gazebo |
| 로봇 미들웨어 | ROS2 Humble |
| 개발 언어 | Python |
| 비전 인식 | OpenCV ArUco |
| 데이터 연동 | REST API, JSON |
| 웹-Gazebo 실행 연동 | Flask |

---

## 14. 주의사항

- Gazebo 모델명은 `AGV01`, `AGV02`를 유지해야 합니다.
- `line_path_map`의 waypoint는 실제 ArUco 마커가 아니라 시뮬레이션 이동용 가상 경로점입니다.
- 서버 API 기반 실행 시 `current_position`, `next_position`은 Gazebo 좌표 기준이어야 합니다.
- 서버 연동이 불안정할 경우 `move_agv_from_scenario_v2_empty_box_return.py`로 로컬 시나리오 실행이 가능합니다.
- ArUco 인식은 AGV 이동 제어용이 아니라 비전 기반 위치 인식 구조 검증용입니다.
- `worlds/final/`에는 최종 world 파일만 두고, 이전 버전은 `worlds/archive/`에 보관하는 것을 권장합니다.

---

## 15. 핵심 요약

```text
Gazebo는 공장 MAP과 AGV 작업 흐름을 시각화합니다.
AGV 이동은 실제 바퀴 제어가 아니라 Gazebo 모델 pose 변경 방식입니다.
로컬 시나리오는 scenario_plan, zone_route_map, line_path_map을 기반으로 실행됩니다.
서버 연동 시에는 move_agv_from_api.py가 서버 상태를 읽어 AGV 위치를 반영합니다.
ArUco 인식은 Gazebo 카메라 기반 위치 인식 검증용으로 사용됩니다.
```
