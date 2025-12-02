# py_bt_ros

Behavior Tree framework for ROS 2 with Python implementation.

## 📁 프로젝트 구조

- `scenarios/rescue_mission/` - 구조 로봇 시나리오 (팀원 3 구현)
- `modules/` - BT 프레임워크 핵심 모듈
- `config.yaml` - 시나리오 설정 파일

---

## 🚁 구조 로봇 시나리오 (Rescue Mission)

### 개요
LIMO 로봇을 사용한 조난자 구조 미션. GridMap 기반 지형 분석, A* 경로 계획, 구조대 동반 안내 기능 포함.

### 구현 노드 (11개 - 팀원 3)

**경로 생성 및 탐색**
- `GenerateSpiralWaypoints` - 나선형 탐색 경로 생성
- `GetNextWaypoint` - 경로점 큐 관리
- `GenerateRescuePaths` - A* 기반 3가지 경로 (안전/균형/최단)
- `VisualizeResults` - RViz 시각화 (MarkerArray)

**미션 관리**
- `PublishMissionSuccess` - 미션 성공 알림
- `ReturnToBase` - 초기 위치 귀환

**구조대 동반 안내**
- `WaitForRescueTeam` - 구조대 준비 대기
- `EscortToVictim` - 구조대와 함께 이동
- `CheckTeamFollowing` - 구조대 추종 확인
- `WarnDangerZone` - 위험 구간 경고 (5m 전)
- `AnnounceArrival` - 도착 알림

### 주요 모듈

**`path_planner.py`**
- `DualPathPlanner` - 로봇/사람 이중 경로 계획
- A* 알고리즘 구현 (8방향 이동)
- 사람 기준 Costmap 생성 (경사 < 30도, 거칠기 < 0.1)
- 3가지 전략: safest, balanced, shortest

**`escort_mode.py`**
- `EscortMode` - 구조대 동반 안내 (보행속도 1.0 m/s)
- `DangerZoneAnalyzer` - 경로상 위험 구간 분석
- 선행 거리 2m 유지, 위험 구간 5m 전 경고

**`bt_nodes.py`**
- GridMap 연동 Condition/Action 노드
- Nav2 통합 (NavigateToPose)
- Odometry 기반 위치 추적

### 실행 방법

#### 1. 의존성 설치
```bash
cd ~/limo_ws/py_bt_ros
pip3 install -r requirements.txt
```

#### 2. Terrain Analysis 실행
```bash
# team_ws에서
cd ~/team_ws
source install/setup.bash
ros2 run terrain_analysis terrain_analysis_node_limo
```

#### 3. GridMap → Costmap 변환
```bash
ros2 run remo_navigation grid_map_costmap_converter
```

#### 4. Nav2 실행
```bash
ros2 launch nav2_bringup navigation_launch.py \
  namespace:=limo \
  use_sim_time:=False
```

#### 5. Behavior Tree 실행
```bash
cd ~/limo_ws/py_bt_ros
python3 main.py
```

#### 6. RViz 시각화 (선택)
```bash
rviz2
# Add → MarkerArray → Topic: /visualization_marker_array
```

### 설정 파일

**`config.yaml`**
```yaml
scenario: scenarios.rescue_mission
agent:
  namespaces: "limo"  # 로봇 namespace
  behavior_tree_xml: "default_bt.xml"
bt_runner:
  bt_tick_rate: 10.0  # BT 실행 주기 (Hz)
```

### BT 구조 (default_bt.xml)

```
MainTree:
  1. HasGridMapData (GridMap 확인)
  2. GenerateSpiralWaypoints (나선형 경로 생성)
  3. GetNextWaypoint (경로점 추출)
  4. GenerateRescuePaths (구조 경로 생성)
  5. VisualizeResults (시각화)
  6. PublishMissionSuccess (성공 알림)
  7. ReturnToBase (귀환)
  8. WaitForRescueTeam (구조대 대기)
  9. WarnDangerZone (위험 경고)
  10. CheckTeamFollowing (추종 확인)
  11. EscortToVictim (동반 안내)
  12. AnnounceArrival (도착 알림)
```

### 토픽 구조

**Subscribe (입력)**
- `/grid_map` - 지형 분석 데이터
- `/limo/odom` - 로봇 위치
- `/yolo/detections` - 객체 감지 (조난자)

**Publish (출력)**
- `/limo/goal_pose` - 목표 위치
- `/visualization_marker_array` - 경로 시각화
- `/mission_status` - 미션 상태

**Action (양방향)**
- `/limo/navigate_to_pose` - Nav2 경로 추종

### Groot 시각화

```bash
groot2
# File → Load → scenarios/rescue_mission/default_bt.xml
```

---

## 🐢 Simple Example (Turtlesim Navigation)

Launch Turtlesim
```
ros2 run turtlesim turtlesim_node
```

Spawn a Target Turtle
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 5.5, theta: 0.0, name: 'turtle_target'}"
```

Teleoperate the Target Turtle
```
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle_target/cmd_vel
```

Move the spawned turtle using keyboard input
(The original turtle /turtle1 will be controlled by Behaviour Tree)


Run Turtle1 Action Server
```
python3 scenarios/example_turtlesim/turtle_nav_action_server.py --ns /turtle1
```

Run Behaviour Tree Controller
```
python3 main.py
```


