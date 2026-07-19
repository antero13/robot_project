# rl_model_policy

학습된 Isaac Lab/skrl 회피·탐색 정책을 실제 ROS 2 로봇에서 실행하는 노드이다.

## 모델 체크포인트 계약

이 패키지를 수정하기 전에 다음 문서를 먼저 확인한다.

```text
rl_model_policy/MODEL_CHECKPOINT_README.md
```

현재 포함된 `mission_manager/models/rl_avoid_search_best.pt` 모델은 YOLO에서
만든 값 10개만 입력으로 사용하는 체크포인트이다.

```text
policy.net_container.0.weight: (128, 10)
state_preprocessor.running_mean: (10,)
state_preprocessor.running_variance: (10,)
```

새 학습도 처음 10개의 YOLO 기반 관측값만 사용한다. 실행기는 체크포인트의
입력 폭을 자동으로 확인하고 다음 두 형식을 모두 지원한다.

```text
10개 입력: YOLO 목표·회피 값만 사용(새 학습 형식)
18개 입력: 같은 10개 값 + 예전 pose/IMU 값 8개
```

10개 관측값을 사용하는 Isaac Lab 환경에서 18개 입력 체크포인트의 학습을
재개하면 안 된다. 포함된 모델과 새 학습 환경은 모두 10개 입력 형식을
사용한다. 다만 ROS 실행기는 `model_path`로 명시한 예전 18개 입력
체크포인트도 계속 실행할 수 있다.

## 입력

- `/target_object` (`geometry_msgs/PointStamped`)
  - `point.x`: `[-1, 1]`로 정규화한 목표 물체의 x 위치
  - `point.y`: `[0, 1]`로 정규화한 bounding box 아래쪽 y 위치
  - CSV 지도 보정은 `/yolo/detections`의 bounding box 중심을 별도로 사용
- `/avoid_objects` (`std_msgs/String`)
  - `ros2_yolo_detector`가 보내는 JSON
- `/odom` (`nav_msgs/Odometry`)
  - 목표가 없을 때 coverage controller가 사용하는 경기장 중심 기준 pose
  - 포함된 10개 입력 모델의 관측값에는 절대 들어가지 않음
  - 예전 18개 입력 모델에서는 `pose_observation_enabled:=true`일 때만 사용
- `/wall/distance_angle` (`geometry_msgs/Vector3Stamped`)
  - `vector.x`: VL53L1X 두 개로 계산한 벽까지의 수직 거리
  - `vector.y`: 두 센서 거리 차로 계산한 벽 각도(rad), `0`이면 벽을 정면으로 봄
  - 레인 x 보정과 보관소 x/y 보정에 사용

새 10개 입력 체크포인트에는 pose 데이터가 네트워크로 전달되지 않는다.
예전 18개 입력 체크포인트에서 pose 관측을 끄면 마지막 8개 입력에 0을
넣는다. 보정된 예전 pose 모델을 의도적으로 시험할 때만
`pose_observation_enabled:=true`를 사용한다.

## 출력

- `/cmd_vel` (`geometry_msgs/Twist`)
- `/ros_robot_controller/bus_servo/set_state`: 기본 bus-servo 집게 명령
- `/rl_estimated_objects` (`std_msgs/String`): GUI 지도 물체 마커
- `/robot_pose/correct_x` (`std_msgs/Float64`): 레인 또는 보관소 x 보정
- `/robot_pose/correct_y` (`std_msgs/Float64`): 보관소 y 보정
- `/robot_pose/correct_yaw` (`std_msgs/Float64`): 남쪽 주도로 ToF 보정 후 yaw `-90도` 재설정(rad)

## 기본 실행

빌드하고 환경을 불러온다.

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
colcon build --packages-select rl_model_policy
.\install\setup.ps1
```

정책 노드를 실행한다.

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py
```

주행을 시작한다.

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

주행을 정지한다.

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

상태를 확인한다.

```powershell
ros2 topic echo /rl_model_policy_state
ros2 topic echo /cmd_vel
```

다른 체크포인트를 사용하려면 다음과 같이 실행한다.

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py model_path:="C:\path\to\best_agent.pt"
```

속도 명령 비율을 낮추려면 다음 값을 사용한다.

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py speed_scale:=0.25
```

ROS 2가 사용하는 Python 환경에는 PyTorch가 설치되어 있어야 한다.

## Jetson 통합 실행

저장소를 받은 뒤 통합 실행 패키지를 한 번 빌드한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to mission_manager wall_distance_sensor robot_pose_tracker rl_model_policy --symlink-install
source ~/ros2_ws/install/setup.bash
```

아래 launch는 controller, camera/YOLO, pose tracker, motor converter, RL
정책을 한 터미널에서 실행한다. Pose tracker는 coverage waypoint에는
사용되지만, 포함된 10개 입력 정책의 관측값에는 들어가지 않는다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  launch_pose_tracker:=true \
  pose_observation_enabled:=false
```

기본 설정은 별도의 시작 명령이 올 때까지 기다린다.

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

launch 한 번으로 주행까지 시작하려면 `auto_start:=true`를 추가한다. Camera와
YOLO 모델이 준비될 시간을 확보하기 위해 실제 이동은 8초 뒤에 시작한다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  speed_scale:=0.25 \
  auto_start:=true
```

launch 터미널에서 `Ctrl+C`를 누르면 이 launch가 시작한 모든 프로세스를
정지한다. 동시에 `mission_manager`, 키보드 teleoperation 또는 다른
`/cmd_vel` publisher를 실행하면 안 된다.

`camera_horizontal_fov_deg` 기본값은 학습 환경과 같은 `80.0`이다.
`target_bearing_prediction_enabled:=true`이면 짧은 검출 공백 동안 odometry
yaw로 마지막 목표 위치를 image x에 다시 투영한다. 이 기능은 현재 10개
입력 정책과 호환되며 pose 값을 관측 벡터에 추가하지 않는다.

`target_timeout_s` 기본값은 `1.0`초이다. 짧은 YOLO 검출 공백에서는 바로
탐색 모드로 바꾸지 않고 마지막 목표 x/y를 유지한다. 실제 수거 시작에는
`grab_detection_timeout_s` 기본값 `0.25`초보다 최신인 원시 YOLO 검출이
필요하다.

## RL 주기와 목표 정렬 PD 제어

RL 정책 추론과 `/cmd_vel` 발행 주기는 기본 `10.0 Hz`이다. 목표가 확인되면
RL 출력 중 선속도는 그대로 사용하고, 각속도는 화면 중심에 대한 목표의
정규화된 x 오차로 계산한 PD 출력으로 교체한다. 양의 x 오차는 화면 오른쪽을
뜻하므로 음의 `angular.z`를 내보내고, 음의 x 오차는 반대로 회전한다.

기본 PD 값은 다음과 같다.

```text
Kp: 0.8
Kd: 0.12
미분 제한: 0.25
중앙 deadband: 0.06
최대 각속도: 0.45 rad/s
```

통합 launch에서 값을 바꿀 수 있다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  timer_rate_hz:=10.0 \
  target_pd_enabled:=false \
  target_pd_proportional_gain:=0.8 \
  target_pd_derivative_gain:=0.12 \
  target_pd_derivative_limit:=0.25 \
  target_pd_center_deadband:=0.06 \
  target_pd_max_angular_z:=0.45
```

기본값인 `target_pd_enabled:=false`에서는 목표 추적의 선속도와 각속도를 모두 RL 출력으로 제어한다.
PD 정렬을 다시 시험할 때만 `target_pd_enabled:=true`로 지정한다.
실행 중 `/rl_model_policy_state`의 `timer_rate_hz`와
`target_alignment_pd`에서 현재 오차, 미분값과 PD 각속도를 확인할 수 있다.

## 목표가 없을 때의 레인 탐색

실행기는 RL과 규칙 기반 제어를 함께 사용한다. 목표가 보이면 학습 정책이
처리하고, 목표가 없으면 deterministic coverage controller가 처리한다.

```text
TRACK_TARGET -> LOCAL_REACQUIRE(양방향 재탐색) -> COVERAGE_SEARCH
                                      |
목표 감지 ----------------------------+--> TRACK_TARGET
```

Local reacquisition은 전진을 멈추고 1.5초 동안 목표를 다시 찾는다. 처음
0.75초는 마지막으로 목표가 보인 방향으로 회전하고, 다음 0.75초는 반대
방향으로 회전한다. 두 방향에서 모두 실패한 뒤에만 coverage search를
시작한다.

물체를 놓을 수 있는 7개 열의 간격은 0.5 m이다. Coverage는 차체 중심 기준
x 좌표 `1.25`, `0.25`, `-0.75`, `-1.25 m`의 4개 레인을 사용한다. 각
레인을 북쪽으로 올라가고, 위쪽에서 제자리로 180도 선회한 뒤, 카메라가
이동 방향을 보도록 남쪽으로 내려온다. 다음 레인 이동은 아래쪽 주도로에서만
수행한다. 최초 순서는 1→2→3→4번이며, 보관소에 다녀온 뒤에는 controller를
새로 만들어 4→3→2→1번 역순으로 탐색한다.

작은 heading 오차는 움직이면서 보정한다. 레인 탐색 중 목표가 아닌 물체가
너무 가까우면 전진 속도의 70%를 유지하면서 더 비어 있는 방향으로 조향한다.
레인 끝의 180도 회전처럼 방향 차이가 큰 경우에만 제자리 선회를 사용한다.

`/odom`이 없거나 오래되면 `WAITING_FOR_POSE`로 바뀌고 정지 명령을 낸다.
기본 레인 설정은 다음과 같다.

```text
x 레인 중심: 1.25, 0.25, -0.75, -1.25 m (1, 2, 3, 4번 레인)
주도로 y: -1.3343 m
탐색 끝 y: 1.0 m
상행 탐색 속도: 0.24 m/s
하행 탐색 속도: 0.24 m/s
레인 이동 속도: 0.30 m/s
coverage 각속도 제한: 1.00 rad/s
waypoint 허용 오차: 0.10 m
곡선 회피 전진 비율: 0.70
```

통합 launch는 기본적으로 `wall_distance_sensor`를 시작한다. 레인에서
남쪽으로 내려온 뒤 `SHIFT_TO_NEXT_LANE`은 먼저 기존 odometry waypoint로
다음 레인 중심까지 이동한다. 이 구간에는 ToF가 필요하지 않다. Odometry가
waypoint 허용 오차 안에 들어온 뒤에만 선택된 벽을 향해 회전하고, 신선한
VL53L1X 값으로 남은 x 오차를 기본 3 cm 이내로 보정한다.

벽 선택은 이동 방향이 아니라 **도착할 레인 번호**를 기준으로 한다. 1·2번
레인은 동쪽 벽 `x=2.0`, 3·4번 레인은 서쪽 벽 `x=-2.0`을 사용한다.
따라서 1→2는 동쪽을 보고 후진, 2→3은 서쪽을 보고 전진한다. 보관소 이후
역순에서는 4→3은 서쪽을 보고 후진, 3→2는 동쪽을 보고 전진한다. 여기서
left/right wall은 경기장의 서쪽/동쪽 벽 좌표이며, 차체에 달린 두 센서의
좌우를 뜻하지 않는다.
버스 1번과 7번의 두 ToF 센서는 같은 벽의 거리와 각도를 함께 계산한다.
보관소 진입 전과 후진 완료 후의 x 보정에서는 서쪽 벽만 사용한다.

Coverage와 보관소 waypoint, `robot_x/y`, pose correction 토픽은 모두 차체의
기하학적 중심을 좌표 원점으로 사용한다. 동·서쪽 벽 보정은 `vector.y` 벽
각도를 기본 `0.05 rad` 이내로 맞춘 뒤 목표 차체 중심 x만
`/robot_pose/correct_x`로 발행하며 yaw는 변경하지 않는다.

각 레인에서 남쪽 주도로로 내려온 직후에는 남쪽 벽 ToF 거리로 y를 먼저
3 cm 이내로 보정한다. 거리 보정이 끝난 뒤 벽 각도가 10도 이상일 때만 각도
보정을 시작하고, 시작된 보정은 5도 이하까지 계속한다. 완료 시
`/robot_pose/correct_y`, `/robot_pose/correct_yaw`로 주도로 y와 yaw `-90도`를
설정한다. 이후 IMU gyro 적분은 이 기준에서 계속된다. ToF가 오래되면 정지해서
새 측정값을 기다린다.

벽 좌표와 센서 오프셋은 다음과 같이 바꿀 수 있다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  lane_tof_left_wall_x_m:=-2.0 \
  lane_tof_right_wall_x_m:=2.0 \
  lane_tof_sensor_forward_offset_m:=0.09
```

남쪽 주도로 보정의 기본 시작/종료 각도는 각각 10도와 5도이며 rad 단위 launch
인자로 조정할 수 있다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  main_road_tof_angle_trigger_rad:=0.1745329252 \
  main_road_tof_angle_release_rad:=0.0872664626
```

`lane_tof_sensor_forward_offset_m`은 차체 중심에서 ToF 센서 평면까지의
거리이다. 차체 길이가 40 cm이고 차체 중심이 앞에서 20 cm, 센서가 앞면에서
11 cm 뒤에 있으므로 센서는 앞에서 29 cm 위치이며 오프셋은 `29-20=9 cm`이다.
회전 중심의 y가 16 cm인 점은 실제 선회 궤적에는 영향을 주지만 지도 좌표의
원점으로 사용하지 않는다.

레인 ToF 보정만 끄려면 `lane_tof_correction_enabled:=false`, 센서 하드웨어
노드 실행까지 끄려면 `launch_wall_distance_sensor:=false`를 사용한다.

통합 센서 launch는 VL53L1X mode `3`(long range)을 기본으로 사용한다.
필요한 경우에만 `wall_ranging_mode:=1` 또는 `2`로 바꾼다.

현재 mode, waypoint, pose, route leg는 다음 명령으로 확인한다.

```bash
ros2 topic echo /rl_model_policy_state
```

`coverage_enabled:=false`는 목표가 없을 때에도 RL 정책에 관측값을 보내던
예전 동작을 재현할 때만 사용한다.

실제 로봇 기본 설정은 다음과 같다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  pose_observation_enabled:=false \
  target_timeout_s:=1.0 \
  target_bearing_prediction_enabled:=true \
  coverage_reacquire_duration_s:=1.5 \
  coverage_min_x:=-1.25 coverage_max_x:=1.25 \
  coverage_avoid_linear_scale:=0.70 \
  coverage_rejoin_speed:=0.20 \
  coverage_turn_in_place_threshold:=0.65 \
  target_activation_center_y_min:=0.30 \
  camera_horizontal_fov_deg:=80.0
```

새 목표는 정규화된 bounding box 중심 y가
`target_activation_center_y_min`에 도달해야 coverage를 중단할 수 있다.
기본값 `0.30`은 측정한 camera calibration에서 약 0.6 m 거리이다. RL
tracking이 시작된 뒤에는 이 gate를 다시 적용하지 않으므로 검출 흔들림으로
coverage와 tracking 사이를 반복하지 않는다.

수거가 끝나면 scan leg는 먼저 `ALIGN_CURVED_REJOIN`에서 진행 중이던 레인을
45도로 바라본다. 이후 `CURVE_REJOIN_LANE`에서 전진을 유지하면서 횡오차가
줄어들수록 진행 방향을 레인 방향으로 연속 보정한다. 레인 중심 x에 도착하면
즉시 중단했던 상행/하행 탐색을 재개한다.

## 자동 수거

주행을 시작할 때 집게는 닫힌 상태이다. 목표 탐색, 정렬, 영상 기반 접근은
RL이 담당한다. 목표가 중앙에 있고 bounding box 아래쪽 y가
`grab_area_ratio`에 도달하면 runtime이 제어권을 받아 다음 순서로 수거한다.

```text
TRACKING -> OPENING -> FINAL_FORWARD -> CLOSING -> GRABBED
```

기본 bus-servo와 수거 설정은 다음과 같다.

```text
서보 ID: 1
열림 위치: 1000
닫힘 위치: 300
수거 중앙 허용 오차: 0.18
수거 영역 비율: 0.70
최종 전진: 0.20 m/s로 1.2초
수거 뒤 정지: false
```

필요하면 통합 launch에서 값을 바꾼다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  gripper_open_position:=1000 \
  gripper_closed_position:=300 \
  grab_area_ratio:=0.70 \
  final_forward_linear_x:=0.20 \
  final_forward_duration_s:=1.2
```

수거 상태는 다음 명령으로 확인한다.

```bash
ros2 topic echo /rl_model_policy_state
```

수동 집게 명령도 같은 control topic을 사용한다.

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: open}"
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: close}"
```

## 상태 GUI와 주행 일시정지

`launch_status_gui:=true`를 사용하면 PyQt 상태 창을 연다. Autonomous launch는
`rl_object_world_mapper`도 실행하며, 검출한 물체의 연속 경기장 좌표를
계산한다. `config/distance_normalized_points.csv`의 bounding box 중심 보정을
보간하며, 물체 마커를 42개 합법 배치점에 강제로 맞추지 않는다.

GUI는 `/yolo/detections`도 직접 구독한다. 독립적인
`/rl_estimated_objects` mapper stream이 2초 넘게 없으면 GUI 표시용 fallback으로
같은 CSV 보간을 수행한다. 이 fallback 좌표는 정책 관측값이나 `/cmd_vel`에
영향을 주지 않는다. 따라서 `ROS data 3/3`은 odometry, policy state, 그리고
mapper stream 또는 원시 YOLO detection이 모두 최신이라는 뜻이다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  target_classes:=0 \
  launch_status_gui:=true
```

Jetson에서 GUI 입력 세 가지를 확인한다.

```bash
ros2 topic hz /yolo/detections
ros2 topic hz /odom
ros2 topic hz /rl_estimated_objects
```

`/rl_estimated_objects`가 없어도 앞의 두 토픽이 살아 있으면 GUI는
`gui_detection_fallback`으로 전환했다는 로그를 남기고 보정된 물체 마커를
계속 그린다.

GUI pause 버튼은 `pause_motion`을 발행한다. 인식, 정책 계산, 상태 토픽은
계속 동작하지만 실제 base velocity 출력은 0으로 유지된다.

## 전체 대회 미션

RL 목표 controller 위에 deterministic mission coordinator가 다음 순서로
동작한다.

```text
COLLECTING
  -> REJOIN_STORAGE_LANE
  -> RETURN_MAIN_ROAD
  -> RETURN_STAGING
  -> CORRECT_STORAGE_STAGING_X
  -> OPEN_STORAGE_ENTRY
  -> ENTER_STORAGE
  -> EXIT_STORAGE
  -> ALIGN_STORAGE_EXIT_WEST
  -> CLOSE_STORAGE_EXIT
  -> CORRECT_STORAGE_EXIT_X
  -> RETURN_FROM_STORAGE
  -> COLLECTING (4 -> 3 -> 2 -> 1번 레인) 또는 COMPLETE
```

### 현재 `TOF_correction` 동작 순서

모든 waypoint와 `robot_x/y`는 차체의 기하학적 중심 좌표를 사용한다. ToF
센서는 차체 중심보다 전방 9 cm에 있는 것으로 계산하며,
`/wall/distance_angle`에서 두 센서로 계산한 벽 거리로 차체 중심 좌표를
다시 구한다.

1. **레인 탐색**
   - 최초 탐색은 1→2→3→4번 레인 순서이며 중심 x는 각각 `1.25`,
     `0.25`, `-0.75`, `-1.25 m`이다.
   - 각 레인을 북쪽으로 탐색한 뒤 제자리로 180도 선회하고 남쪽으로
     내려오면서 같은 레인을 다시 탐색한다.
   - 주도로 도착 직후 남쪽 벽을 본 상태에서 ToF 거리로 `y=-1.3343 m`를
     먼저 보정한다. 거리 보정 후 벽 각도가 10도 이상이면 회전을 시작하고
     5도 이하가 되면 멈춘 뒤 pose yaw를 `-90도`로 재설정한다.
   - 다음 레인 이동은 도착 레인이 1·2번이면 동쪽 벽, 3·4번이면 서쪽 벽을
     바라본 상태로 odometry waypoint까지 먼저 수행한다. 10 cm 허용 오차에
     들어온 뒤 같은 벽의 ToF 각도를 0도 근처로 맞추고 남은 x 오차를
     3 cm 이내로 보정한다. 이 동·서쪽 벽 단계에서는 pose yaw를 변경하지 않는다.
   - ToF 보정이 끝나면 1→2에서는 좌측 회전, 서쪽을 보던 이동에서는 우측
     회전으로 북쪽을 향한 뒤 다음 레인 탐색을 시작한다.

2. **물체 수거 후 레인 복귀**
   - 수거 뒤 `CURVE_REJOIN_LANE`으로 현재 레인의 중심 x에 즉시 복귀한다.
     처음에는 레인을 45도로 바라보고 전진하면서 레인 진행 방향으로 합류한다.
   - 4개 적재, 일곱 번째 물체 수거, 종료 30초 전 또는 수동 복귀도 먼저
     `REJOIN_STORAGE_LANE`을 끝낸 뒤 보관소 이동을 시작한다.

3. **주도로 복귀와 보관소 입구 x 보정**
   - 현재 레인 x를 유지한 채 남쪽으로 내려가 `y=-1.3343 m` 주도로에
     복귀한다.
   - 남쪽 벽 ToF 거리로 주도로 y를 먼저 보정한다. 그다음 벽 각도가 10도
     이상이면 5도 이하까지 정렬하고 yaw를 `-90도`로 재설정한다.
   - 주도로에서 서쪽을 바라보고 `(-1.25, -1.3343) m`까지 이동한다.
   - 도착 뒤 서쪽 벽 ToF 각도를 0도 근처로 정렬하고 `x=-1.25 m`를
     3 cm 이내로 보정한다. yaw는 변경하지 않는다. 예상 ToF 거리는 66 cm이며,
     값이 없거나 오래되면 정지해서 기다린다.

4. **서보 개방과 보관소 진입**
   - 입구 x 보정이 끝난 뒤 서보를 열고 기본 0.5초 기다린다.
   - 입구 `(-1.25, -1.3343) m`에서 보관소 중심 `(-1.75, -1.75) m`까지
     향하는 고정 yaw로 먼저 정렬한 뒤 `0.40 m/s`로 2.50초 연속 진입한다.
     이 구간에서는 pose x/y로 방향을 다시 계산하거나 ToF로 중단하지 않는다.
   - 진입 정지 후 0.20초 안정화하고 pose를 `storage_center_x/y`로 보정한다.

5. **같은 경로 후진과 출구 x 재보정**
   - pose 보정 반영을 확인한 뒤 서보를 연 상태로 같은 IMU yaw를 유지하며
     `-0.40 m/s`로 기본 1.50초 후진한다.
   - 후진이 끝나면 서보를 연 상태로 odometry yaw를 사용해 서쪽 180도로
     제자리 회전한다. 서쪽 정렬이 끝난 뒤 서보를 닫고 기본 0.5초 기다린다.
   - 서쪽 벽 ToF 거리로 `x=-1.25 m`를 먼저 보정한다. 거리 완료 후 벽 각도가
     10도 이상이면 정렬을 시작하고 5도 이하에서 멈춘 뒤 yaw를 180도로
     재설정한다. 신선한 ToF 값이 연속 1초 동안 없으면 x를 `-1.25 m`로
     간주하되 yaw는 변경하지 않는 fallback을 사용한다.

6. **역순 탐색 재개**
   - 이미 주도로에 있으므로 북쪽으로 회전한 뒤 바로 역순 탐색을 시작한다.
   - Coverage를 4→3→2→1번 역순으로 다시 시작한다. 4→3은 서쪽을 보고
     후진하며, 3→2와 2→1은 동쪽을 보고 전진한다. 각 이동은 waypoint 도착
     뒤 현재 바라보는 벽의 ToF로 벽 각도를 0도 근처로 정렬하고 x만 보정한다.

7. **pose와 GUI 갱신**
   - 동·서쪽 벽 보정 완료 시 `/robot_pose/correct_x`만 발행한다.
   - 남쪽 주도로 보정 완료 시 `/robot_pose/correct_y`와 yaw `-90도`를
     `/robot_pose/correct_yaw`로 발행한다.
   - Pose tracker는 해당 위치 축과 yaw 기준값만 바꾸고 누적 이동량을 유지한다.
   - GUI의 `mission.waypoint`는 노란색 `W` 목표 마커이고, 로봇 마커는
     보정된 pose를 사용한다. 경기장 표시에는 centered-frame x/y에
     `+2.0 m`를 더한다.

기본 보관 용량은 4개이고 미션 목표는 총 7개이다. 4개를 적재했거나 일곱
번째 물체를 수거하면 보관소로 복귀한다. 남은 시간이 30초 이하이고 적재한
물체가 하나 이상이어도 복귀한다. 적재물이 없으면 물체를 잡거나 180초
경기가 끝날 때까지 탐색한다.

최초 coverage는 1, 2, 3, 4번 레인을 포함한다. 보관소 방문 뒤에는 4번
레인의 `x=-1.25 m`에서 시작해 `-0.75`, `0.25`, `1.25 m` 순으로 역순
탐색한다. 기본 차체 중심 waypoint는 다음과 같다.

```text
주도로 y:          -1.3343 m
보관소 입구:       (-1.25, -1.3343) m
보관소 중심:       (-1.75, -1.75) m
출구 ToF yaw:      180도(서쪽)
고속 진입:         입구 -> 보관소 중심, 0.40 m/s, 2.50초
후진:              보관소 중심 -> 입구, -0.40 m/s, 1.50초
역순 탐색 yaw:     90도(북쪽)
```

실제 차체 중심이 40 cm 보관소 영역 안에 들어오지 않으면 통합 launch에서
다음 값을 조정한다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  storage_main_road_y:=-1.3343 storage_staging_x:=-1.25 \
  storage_center_x:=-1.75 storage_center_y:=-1.75 \
  storage_exit_x:=-1.25 \
  storage_x_entry_speed:=0.40 \
  storage_entry_dash_duration_s:=2.50 \
  storage_exit_reverse_speed:=0.40 \
  storage_exit_dash_duration_s:=1.50 \
  storage_contact_settle_duration_s:=0.20 \
  storage_tof_left_wall_x_m:=-2.0 \
  storage_tof_sensor_forward_offset_m:=0.09 \
  storage_exit_tof_fallback_timeout_s:=1.0 \
  storage_exit_tof_angle_trigger_rad:=0.1745329252 \
  storage_exit_tof_angle_release_rad:=0.0872664626
```

보관소 진입 방향은 입구 `(-1.25, -1.3343)`에서 접촉 기준점
`(-1.75, -1.75)`로 향하는 각도(기본 약 `-140.26 deg`)로 한 번 정렬한다.
그 뒤에는 pose x/y로 목표 방향을 다시 계산하지 않고 IMU yaw만 유지하면서
`storage_entry_dash_duration_s` 동안 연속 전진한다. 정지 및 접촉 안정화 후
`storage_center_x/y`를 `/robot_pose/correct_x`, `/robot_pose/correct_y`로 동시에
발행하며, 보정 반영을 확인한 다음 같은 yaw로
`storage_exit_dash_duration_s` 동안 후진한다. 실제 로봇 속도에 따라 두 시간은
현장에서 조정한다.

후진이 끝나면 먼저 odometry yaw로 서쪽 180도를 바라보도록 제자리 회전하고,
회전 완료 후 서보를 닫는다. 그다음 서쪽 벽 ToF는 x 거리부터 보정한다. 거리 완료 후
각도가 10도 이상일 때만 회전을 시작하고, 시작된 회전은 5도 이하까지 계속한
다음 pose yaw를 180도로 재설정한다.

`storage_tof_correction_enabled:=false`를 사용하면 진입 전·후의 입구 x ToF
보정만 생략한다. 고정 yaw 시간 기반 진입·후진과 보관소 접촉 pose x/y 보정은
그대로 수행한다. `storage_tof_xy_tolerance_m` 기본값은 `0.03 m`이다.
