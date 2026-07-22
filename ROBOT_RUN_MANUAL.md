# Jetson Robot Run Manual

기준 코드: GitHub `main` 18-observation 호환 버전
대상 환경: Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble

## 0. 현재 실행 범위와 중요 경고

현재 자동 RL 경로는 다음을 수행한다.

```text
USB camera
  -> v4l2_camera
  -> image correction + per-frame YOLO prediction
  -> /target_object, /avoid_objects
robot_pose_tracker -> /odom
  -> rl_model_policy
  -> /cmd_vel
  -> cmd_vel_to_motor
  -> ros_robot_controller
  -> DC motors
```

현재 구현은 경기장을 탐색하면서 최대 4개를 수납하고, odometry 경로로 Storage
Zone에 복귀해 배출한 뒤 다시 탐색한다. 총 7개 배출 시 완료하며, 남은 시간이
30초 이하일 때 내부에 물체가 있으면 즉시 복귀한다. 태극기 인식은 사용하지 않는다.

### 18개 observation

현재 런타임은 최신 모델과 같은 18개 observation을 생성한다. 마지막 8개는
`robot_pose_tracker`가 발행하는 `/odom`의 위치, 방향, yaw rate와 마지막 목표
bearing이다. `/odom`이 오래됐거나 없으면 `pose_valid=0`이고 10~17번 값은 모두
0이 된다.

## 1. 안전 준비

첫 시험은 바퀴를 바닥에서 띄우고 수행한다.

- 로봇 주변에서 사람과 물체를 치운다.
- 배터리와 USB/모터/서보 연결을 확인한다.
- `speed_scale:=0.25`, `auto_start:=false`로 시작한다.
- 정지 명령을 입력할 별도 터미널을 미리 열어둔다.
- `mission_manager`, `keyboard_teleop`, `timed_motion`을 동시에 실행하지 않는다.
- 통합 RL launch를 사용할 때 `ros_robot_controller`와 `cmd_vel_to_motor`를
  별도로 실행하지 않는다.

긴급 정지 명령:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

launch 전체 종료는 launch 터미널에서 `Ctrl+C`를 누른다.

## 2. 최신 코드 받기

```bash
cd ~/ros2_ws/src/robot_project
git status --short
git pull --ff-only origin main
git log --oneline -1
```

`git pull`이 로컬 수정 때문에 중단되면 `git pull -f` 또는 `git reset --hard`를
사용하지 않는다. 먼저 다음 결과를 확인해 로컬 변경을 보존한다.

```bash
git status --short
git diff
```

## 3. 최신 RL 모델 확인

저장소에 포함된 기본 모델의 입력 차원을 확인한다.

```bash
python3 -c "import torch; p=torch.load('/home/airobot/ros2_ws/src/robot_project/mission_manager/models/rl_avoid_search_best.pt', map_location='cpu'); print(p['policy']['net_container.0.weight'].shape); print(p['state_preprocessor']['running_mean'].shape)"
```

정상 출력:

```text
torch.Size([128, 18])
torch.Size([18])
```

## 4. ROS 패키지 빌드

새 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

colcon build --symlink-install \
  --packages-up-to ros2_yolo_detector cmd_vel_to_motor mission_manager \
  rl_model_policy robot_pose_tracker

source ~/ros2_ws/install/setup.bash
```

`ros_robot_controller`와 `ros_robot_controller_msgs`가 워크스페이스에 이미 있어야
한다. 패키지를 찾을 수 없다면 먼저 확인한다.

```bash
colcon list | grep -E 'ros_robot_controller|ros_robot_controller_msgs'
```

설치된 실행 파일 확인:

```bash
ros2 pkg executables rl_model_policy
ros2 pkg executables ros2_yolo_detector
ros2 pkg executables cmd_vel_to_motor
```

## 5. Python, CUDA, TensorRT 확인

기존에 GPU inference가 정상인 환경에서는 PyTorch를 다시 설치하지 않는다.
일반 PyPI PyTorch 설치는 JetPack CUDA/cuDNN과 충돌할 수 있다.

```bash
python3 -c "import torch; print('torch', torch.__version__); print('cuda', torch.version.cuda); print('available', torch.cuda.is_available()); print('cudnn', torch.backends.cudnn.version())"
```

CUDA convolution smoke test:

```bash
python3 -c "import torch; m=torch.nn.Conv2d(3,16,3).cuda(); x=torch.randn(1,3,64,64,device='cuda'); print(m(x).shape)"
```

YOLO와 OpenCV 확인:

```bash
python3 -c "import cv2, numpy, ultralytics; print('cv2', cv2.__version__); print('numpy', numpy.__version__); print('ultralytics', ultralytics.__version__)"
python3 -m pip check
```

`numpy`는 현재 프로젝트 요구사항상 2 미만이어야 한다.

## 6. 하드웨어 확인

### 6.1 모터 컨트롤러

```bash
ls -l /dev/ttyACM0
groups
```

사용자가 `dialout` 그룹에 없으면 영구 권한을 설정한 뒤 로그아웃/로그인한다.

```bash
sudo usermod -aG dialout "$USER"
```

당일 임시 확인만 필요하면 다음을 사용할 수 있지만 재부팅 후 유지되지 않는다.

```bash
sudo chmod a+rw /dev/ttyACM0
```

### 6.2 camera1 안정 경로

```bash
ls -l /dev/v4l/by-path/
readlink -f /dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0
```

`video-index0`를 사용한다. `video-index1`은 일반적으로 실제 capture interface가
아니다.

### 6.3 YOLO TensorRT engine

```bash
ls -lh /home/airobot/ros2_ws/best.engine
```

TensorRT engine은 가능하면 실제 경기에 사용할 같은 Jetson에서 생성한다.

## 7. 기존 ROS 프로세스 정리

기존 launch 터미널에서 먼저 `Ctrl+C`를 누른다. 다음 노드가 동시에 남아 있지
않아야 한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list
```

특히 다음 노드는 중복 실행하지 않는다.

```text
/ros_robot_controller
/v4l2_camera_node
/yolo_camera_node
/detections_to_target_node
/cmd_vel_to_motor
/rl_model_policy
/mission_manager
/keyboard_teleop
/timed_motion
```

`ros2 node list`에서 같은 이름 경고가 나오면 실제 프로세스를 종료한 뒤 ROS
그래프를 갱신한다.

```bash
ros2 daemon stop
ros2 daemon start
```

daemon 재시작은 살아 있는 프로세스를 종료하지 않으므로 먼저 launch 터미널을
종료해야 한다.

## 8. 1단계: dry-run 시험

통합 launch는 컨트롤러, camera1, YOLO, target converter, motor converter,
pose tracker, RL 정책을 함께 실행한다. 별도 `ros_robot_controller`,
`cmd_vel_to_motor`, `robot_pose_tracker`를
실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0 \
  target_classes:=0 \
  initial_x:=1.8 \
  initial_y:=-1.8 \
  initial_yaw_deg:=90.0 \
  speed_scale:=0.25 \
  dry_run:=true \
  gripper_enabled:=true \
  gripper_type:=bus \
  gripper_servo_id:=1 \
  gripper_open_position:=1000 \
  gripper_closed_position:=300 \
  publish_annotated:=true \
  stop_after_grab:=false \
  auto_start:=false
```

각 줄의 `\` 뒤에는 공백을 넣지 않는다. `\`는 반드시 줄의 마지막 문자여야
한다.

정상 로그:

```text
RL model policy ready
Send start/stop on /rl_model_policy_control
YOLO model loaded
Listening on /cmd_vel
```

dry-run에서는 RL state/action은 계산하지만 `/cmd_vel`과 그리퍼 명령을 발행하지
않는다.

## 9. 시작 전 상태 확인

별도 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 node list | grep -E 'ros_robot_controller|v4l2|yolo|detections|cmd_vel|rl_model'
ros2 topic info /rl_model_policy_control
ros2 topic hz /image_raw
ros2 topic hz /yolo/detections
```

`/rl_model_policy_control`은 다음과 같아야 한다.

```text
Subscription count: 1
```

dry-run 정책을 시작한다. 이 모드에서는 모터와 그리퍼 명령이 실제로 발행되지
않는다.

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

RL 상태 확인:

```bash
ros2 topic echo /rl_model_policy_state
```

주요 필드:

- `model_loaded: true`
- `dry_run: true`
- `active`: start 전 false, start 후 true
- `obs`: 길이 18
- `obs[10]`: `/odom`이 정상일 때 `1.0`
- `linear_x`, `angular_z`: 정책이 계산한 명령

확인이 끝나면 dry-run 정책을 정지한다.

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

NoMachine 화면에서 YOLO 결과를 확인하려면:

```bash
ros2 run rqt_image_view rqt_image_view /yolo/annotated_image
```

## 10. 2단계: 실제 저속 주행

dry-run launch를 `Ctrl+C`로 종료한 뒤 동일 명령에서 `dry_run:=false`로 바꿔
실행한다. 첫 시험은 바퀴를 띄운 상태에서 한다.

```bash
ros2 launch rl_model_policy rl_autonomous_drive.launch.py \
  yolo_model_path:=/home/airobot/ros2_ws/best.engine \
  video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0 \
  target_classes:=0 \
  initial_x:=1.8 \
  initial_y:=-1.8 \
  initial_yaw_deg:=90.0 \
  speed_scale:=0.25 \
  dry_run:=false \
  gripper_enabled:=true \
  gripper_type:=bus \
  gripper_servo_id:=1 \
  gripper_open_position:=1000 \
  gripper_closed_position:=300 \
  publish_annotated:=true \
  stop_after_grab:=false \
  auto_start:=false
```

상태 확인 후 수동 시작:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

정지:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

첫 시험에서 `auto_start:=true`를 사용하지 않는다. 안정성이 확인된 뒤에만 8초
자동 시작을 사용한다.

## 11. 속도 조정

`speed_scale`은 RL이 계산한 전진·회전 명령에 함께 적용된다.

```text
0.25: 첫 실차 시험 권장
0.40: 저속 시험 후 단계적으로 사용
0.50: 충분한 공간과 정지 준비 후 사용
1.00: 기본 0.25의 4배이므로 초기 시험에 사용하지 않음
```

실행 중에도 변경할 수 있다.

```bash
ros2 param set /rl_model_policy speed_scale 0.40
```

모터 변환기의 `max_rps=2.0`은 현재 RL 기본 최대 전진 속도 범위에서 먼저
조정할 항목이 아니다.

## 12. 목표 클래스 설정

단일 클래스 ID:

```bash
target_classes:=0
```

여러 클래스는 쉼표로 구분한다.

```bash
target_classes:=0,1,2
```

TensorRT engine에서 이름이 `class0`처럼 보이는 경우에도 ID `0`으로 선택할 수
있다. 경기 전 `/yolo/annotated_image`와 `/yolo/detections`에서 실제 ID와 이름을
반드시 확인한다.

```bash
ros2 topic echo /yolo/detections
```

## 13. 그리퍼 동작

현재 bus servo 설정:

```text
servo ID: 1
open: 1000
closed: 300
move duration: 0.5 s
```

RL은 목표가 중앙에 있고 가까워지면 다음 순서로 동작한다.

```text
TRACKING -> OPENING -> FINAL_FORWARD -> CLOSING -> GRABBED
```

기본값 `stop_after_grab:=false`, `full_mission_enabled:=true`에서는 상위 미션
상태기가 수납 개수와 남은 시간을 판단한다. 네 번째 수납 또는 일곱 번째 수집,
내부 물체가 있는 상태에서 남은 시간 30초 조건이 되면 보관함 복귀가 시작된다.

기본 보관함 경로는 경기장 중심 좌표계에서 다음과 같다.

```text
최초 1번 레인 진입: y=-1.3343
주 경로: y=-1.40
진입 대기점: (-1.75, -1.25)
보관함 내부: (-1.75, -1.75)
진입 방향: -90도
```

실제 시험에서는 로봇 중심이 보관함 안에 들어오는지 저속으로 확인한 뒤
`storage_staging_*`, `storage_center_*` launch 파라미터를 조정한다.

수동 그리퍼 시험:

```bash
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: open}"
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: close}"
```

## 14. 선택 사항: 위치와 방향 RViz 표시

RL launch는 이미 `ros_robot_controller`를 실행한다. 별도 터미널에서 다음
시각화 launch만 추가할 수 있다.

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch robot_pose_tracker robot_pose_visualization.launch.py \
  initial_x:=1.8 \
  initial_y:=-1.8 \
  initial_yaw_deg:=90.0
```

위치 추정 노드의 자이로 보정이 끝날 때까지 약 2초간 로봇을 정지시킨다.

주의: 이 위치는 `/cmd_vel`과 IMU를 적분한 추정값이며 실제 바퀴 이동을 측정한
값이 아니다. 보관함 자동 복귀의 단독 기준으로 사용하지 않는다.

## 15. 문제 해결

### `/rl_model_policy_control`이 Unknown topic

RL 노드가 실행되지 않았거나 모델 로딩 중 죽은 상태다.

```bash
ros2 node list | grep rl_model
ros2 topic info /rl_model_policy_control
```

launch 터미널의 `[rl_model_policy-*] Traceback`을 확인한다.

### `[128, 18]` 대 `[128, 10]` size mismatch

이전 10-input 실행 코드가 install 공간에 남은 상태다. 최신 코드를 pull한 뒤
`rl_model_policy`를 다시 빌드하고 새 터미널에서 setup을 source한다.

### `Waiting for at least 1 matching subscription(s)`

RL control subscriber가 없다. publisher를 `Ctrl+C`로 중단하고 RL 노드가 살아
있는지 확인한다. `source` 명령은 환경만 설정하며 노드를 실행하지 않는다.

### 같은 이름의 노드 경고

이전 launch가 남아 있다. 모든 launch 터미널에서 `Ctrl+C`를 누르고 실제
프로세스를 종료한다. 동일한 통합 launch를 두 번 실행하지 않는다.

### 카메라 topic이 없음

```bash
ls -l /dev/v4l/by-path/
ros2 node list | grep v4l2
ros2 topic list | grep image_raw
ros2 topic hz /image_raw
```

다른 녹화 프로그램이나 camera launch가 같은 장치를 점유하고 있지 않은지
확인한다.

### YOLO는 실행되지만 detection이 없음

```bash
ros2 topic hz /image_raw
ros2 topic hz /yolo/detections
ros2 topic echo /yolo/detections
ros2 run rqt_image_view rqt_image_view /yolo/annotated_image
```

`target_classes`, confidence, 조명, 카메라 초점, 프레임 보정 설정을 확인한다.
최신 camera1 pipeline은 기본적으로 gamma/CLAHE/chroma 보정을 사용하고
`imgsz=800`으로 추론한다.

### TensorRT engine 경고 또는 오류

다른 장치에서 만든 engine은 같은 모델이어도 호환이나 성능 문제가 생길 수 있다.
실제 Jetson에서 ONNX를 engine으로 다시 변환한다.

### 로봇이 직진 명령에서 회전함

모터 포트와 부호를 확인한다.

```text
left motor IDs: 4, 3
right motor IDs: 2, 1
left signs: +1, +1
right signs: -1, -1
wheel radius: 0.05 m
wheel separation: 0.32 m
```

### 녹화가 필요함

자동 mission launch를 먼저 종료한다. mission과 녹화 도구가 같은 카메라를
동시에 열 수 없다.

```bash
bash ~/recording_tools/record_once.sh camera1
bash ~/recording_tools/record_once.sh camera2
```

## 16. 경기 전 체크리스트

- [ ] Git commit 확인
- [ ] 최신 RL 모델 shape `(128, 18)` 확인
- [ ] `/odom` 발행 및 `obs[10] == 1.0` 확인
- [ ] YOLO engine 존재 및 같은 Jetson에서 생성
- [ ] `/dev/ttyACM0` 권한 확인
- [ ] camera1 by-path 확인
- [ ] 중복 ROS 노드 없음
- [ ] `/rl_model_policy_control` subscriber 1개
- [ ] `/image_raw`와 `/yolo/detections` Hz 정상
- [ ] annotated image에서 목표 클래스 확인
- [ ] 그리퍼 open/closed 위치 확인
- [ ] `speed_scale=0.25`로 첫 시험
- [ ] 정지 터미널 준비
- [ ] RViz 사용 시 자이로 보정 2초 대기
- [ ] 현재 버전은 한 객체 집기까지만 가능함을 팀원이 공유
