# RL 모델 정책 실행 가이드

이 문서는 `mission_manager/models/rl_avoid_search_best.pt` 학습 모델을 사용해서 로봇을 움직이는 실행 순서를 정리한 문서입니다.

## 전체 구조

```text
카메라 / YOLO
  -> /target_object
  -> /avoid_objects
  -> rl_model_policy
  -> /cmd_vel
  -> cmd_vel_to_motor
  -> 모터
```

`rl_model_policy`는 YOLO 화면 좌표를 학습 때와 같은 10개 observation으로 만들고, 학습 모델을 통해 action을 추론한 뒤 `/cmd_vel`을 publish합니다.

## 주의사항

- `mission_manager`와 `rl_model_policy`를 동시에 켜지 마세요.
- 둘 다 `/cmd_vel`을 publish할 수 있어서 명령이 충돌할 수 있습니다.
- 처음 실행은 `speed_scale:=0.25`처럼 낮은 속도로 시작하세요.
- 정지 명령은 항상 별도 터미널에 준비해두는 것을 권장합니다.
- ROS가 사용하는 Python 환경에 `torch`가 설치되어 있어야 합니다.

## 1. 최신 코드 받기

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
git pull
```

모델 파일이 있는지 확인합니다.

```powershell
Test-Path .\mission_manager\models\rl_avoid_search_best.pt
```

`True`가 나오면 됩니다.

## 2. PyTorch 확인

ROS 실행에 쓰는 같은 터미널에서 확인합니다.

```powershell
python -c "import torch; print(torch.__version__)"
```

오류가 나면 그 환경에는 `torch`가 없는 상태입니다. 먼저 PyTorch를 설치해야 `rl_model_policy`가 실행됩니다.

## 3. 빌드

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
colcon build --packages-select rl_model_policy
.\install\setup.ps1
```

필요하면 전체 빌드도 가능합니다.

```powershell
colcon build
.\install\setup.ps1
```

## 4. 터미널별 실행

### 터미널 1: YOLO 실행

사용 중인 카메라 launch를 실행합니다.

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
.\install\setup.ps1
ros2 launch ros2_yolo_detector v4l2_yolo_camera.launch.py
```

다른 launch를 쓰는 팀원은 기존에 쓰던 YOLO launch를 그대로 사용하면 됩니다.

### 터미널 2: 모터 변환 노드 실행

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
.\install\setup.ps1
ros2 launch cmd_vel_to_motor cmd_vel_to_motor.launch.py
```

### 터미널 3: RL 모델 정책 실행

처음에는 낮은 속도로 실행합니다.

```powershell
cd "C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS"
.\install\setup.ps1
ros2 launch rl_model_policy rl_model_policy.launch.py speed_scale:=0.25
```

동작이 안정적이면 나중에 조금 올릴 수 있습니다.

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py speed_scale:=0.50
```

명령을 실제로 내보내지 않고 상태만 보고 싶으면:

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py dry_run:=true
```

### 터미널 4: 상태 확인 및 시작/정지

상태 확인:

```powershell
ros2 topic echo /rl_model_policy_state
```

출력 명령 확인:

```powershell
ros2 topic echo /cmd_vel
```

로봇 시작:

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: start}"
```

로봇 정지:

```powershell
ros2 topic pub --once /rl_model_policy_control std_msgs/msg/String "{data: stop}"
```

## 5. 확인해야 하는 값

`/rl_model_policy_state`에서 주로 볼 값:

```text
active
model_loaded
obs
raw_action
filtered_action
linear_x
angular_z
avoid_left
avoid_center
avoid_right
nearest_avoid_x
nearest_avoid_y
```

정상 예:

```text
model_loaded: true
active: true
linear_x, angular_z 값이 상황에 따라 변함
```

## 6. 자주 생기는 문제

### `model_loaded`가 false인 경우

모델 경로를 직접 지정합니다.

```powershell
ros2 launch rl_model_policy rl_model_policy.launch.py model_path:="C:\Users\user\Desktop\박준현\2026-1\로봇 대회\ROS\mission_manager\models\rl_avoid_search_best.pt"
```

### `ModuleNotFoundError: No module named 'torch'`

ROS가 사용하는 Python 환경에 PyTorch가 없습니다. 해당 환경에 `torch`를 설치해야 합니다.

### 로봇이 움직이지 않는 경우

확인 순서:

```powershell
ros2 topic echo /target_object
ros2 topic echo /avoid_objects
ros2 topic echo /rl_model_policy_state
ros2 topic echo /cmd_vel
```

`/target_object`나 `/avoid_objects`가 안 나오면 YOLO 쪽 문제입니다.

`/rl_model_policy_state`는 나오는데 `/cmd_vel`이 안 나오면 `dry_run:=true`로 실행했는지 확인합니다.

### 기존 mission_manager와 충돌하는 경우

`mission_manager`를 끄고 `rl_model_policy`만 `/cmd_vel`을 publish하게 해야 합니다.

```powershell
ros2 node list
```

`mission_manager`와 `rl_model_policy`가 동시에 떠 있으면 하나를 종료하세요.

## 7. 추천 첫 테스트 순서

1. `dry_run:=true`로 실행해서 `/rl_model_policy_state`가 나오는지 확인
2. `speed_scale:=0.25`로 실행
3. `/cmd_vel` 값 확인
4. `start` 명령 전송
5. 이상하면 즉시 `stop`
6. 동작이 맞으면 `speed_scale`을 조금씩 증가
