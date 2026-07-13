# Robot Project Code Review

최초 검토 기준 커밋: `21449fd` (`origin/main`, 2026-07-13 확인)

검토 범위:

- `ros2_yolo_detector`
- `rl_model_policy`
- `cmd_vel_to_motor`
- `mission_manager`
- `robot_pose_tracker`
- `keyboard_teleop`, `timed_motion`
- `wall_distance_sensor`
- `recording_tools`

외부 워크스페이스 패키지인 `ros_robot_controller`와
`ros_robot_controller_msgs`는 이 저장소에 포함되지 않아 인터페이스 사용 부분만
검토했다.

## 결론

코드는 카메라 입력부터 YOLO, 목표 변환, RL 정책, `/cmd_vel`, 모터 명령까지
모듈 경계가 비교적 명확하다. 카메라의 안정적인 USB 경로, 모터 명령 타임아웃,
프레임별 YOLO 결과 발행, 그리퍼 상태 머신, 선택적인 RViz 시각화도 실전 운용에
도움이 된다.

최신 수정으로 기본 RL 체크포인트와 런타임 코드의 observation 차원 문제는
해결됐다. 다만 현재 자동화 범위는 목표 객체
하나를 찾아 집는 데까지이며, 대회 전체 요구사항인 다중 수집, 보관함 복귀,
태극기 정렬, 배출은 아직 구현되지 않았다.

## 심각도 정의

- **P0**: 기본 실행 경로가 시작되지 않거나 로봇을 통제할 수 없는 결함
- **P1**: 경기 실패, 충돌, 잘못된 주행으로 이어질 가능성이 큰 결함
- **P2**: 성능·재현성·유지보수에 의미 있는 위험
- **P3**: 문서, 테스트, 개발 편의성 개선 사항

## Findings

### [해결됨] 최신 RL 체크포인트와 런타임 observation 차원이 달랐다

기존 런타임 네트워크의 첫 레이어는 입력 10개로 고정돼 있었다.

- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:34`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:345`

현재 `main`의 `mission_manager/models/rl_avoid_search_best.pt`를 검사하면 다음과
같다.

```text
policy net_container.0.weight: (128, 18)
state_preprocessor running_mean: (18,)
state_preprocessor running_variance: (18,)
```

노드는 체크포인트를 strict 모드로 로드한다.

- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:192`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:194`

이전에는 `size mismatch ... [128, 18] ... [128, 10]` 예외로 RL 프로세스가
종료됐다. 현재는 README에 공개된 18개 observation 순서대로 `/odom`의 위치,
yaw, yaw rate와 마지막 목표 bearing을 포함해 생성하며, 체크포인트와 전처리기
shape도 로딩 시 검증한다.

권장 수정:

1. 학습 환경에서 사용한 18개 observation의 정확한 이름, 순서, 정규화 범위를
   체크포인트와 함께 버전 관리한다.
2. `PolicyNetwork` 입력 차원과 `make_observation()`을 같은 스키마로 변경한다.
3. 체크포인트에 `observation_schema_version`과 `observation_names`를 저장한다.
4. 빌드/CI에서 저장된 모델을 실제로 로드하고 observation 길이를 검증한다.

18개 입력의 의미가 없는 상태에서 8개를 임의로 0으로 채우는 것은 모델을
실행만 가능하게 할 뿐 학습 동작을 재현하지 못하므로 권장하지 않는다.

따라서 이전 10-input 모델을 꺼내 쓰는 임시 우회는 더 이상 사용하지 않는다.
남은 보정 항목은 실제 카메라 수평 화각과 `robot_pose_tracker` 위치 오차다.

### [P1] 핵심 RL 프로세스가 죽어도 통합 launch의 나머지 프로세스는 계속 실행된다

`rl_autonomous_drive.launch.py`는 컨트롤러, YOLO, 모터 변환기, RL 정책을
독립적으로 include한다.

- `rl_model_policy/launch/rl_autonomous_drive.launch.py:41`
- `rl_model_policy/launch/rl_autonomous_drive.launch.py:49`
- `rl_model_policy/launch/rl_autonomous_drive.launch.py:65`
- `rl_model_policy/launch/rl_autonomous_drive.launch.py:73`

RL 프로세스가 모델 로딩 중 종료돼도 카메라와 모터 컨트롤러는 계속 실행된다.
사용자는 YOLO 로그가 계속 출력되는 것을 보고 전체 launch가 정상이라고 오해할
수 있다. `auto_start:=true`이면 8초 뒤 실행되는 `ros2 topic pub --once`가 없는
subscriber를 기다릴 수도 있다.

권장 수정:

- RL 프로세스 종료 시 전체 launch를 종료하는 `OnProcessExit`/`Shutdown` 처리
- 시작 전 모델 호환성을 확인하는 preflight 노드 또는 스크립트
- `/system/ready` 같은 명시적인 준비 상태와 시작 조건

### [P1] 현재 코드는 대회 전체 미션을 완료하지 않는다

RL 그리퍼 상태 머신은 목표가 가까워지면 열기, 최종 전진, 닫기를 수행한다.
기본값 `stop_after_grab=true`로 객체 하나를 집은 뒤 RL 주행을 중지한다.

- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:106`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:389`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:422`
- `rl_model_policy/launch/rl_autonomous_drive.launch.py:162`

`stop_after_grab=false`로 계속 주행할 수는 있지만 다음 기능은 없다.

- 수집된 목표 개수와 클래스 추적
- 이미 수집한 객체 중복 방지
- 모든 목표 수집 완료 판단
- Storage Zone 복귀 경로
- 태극기 탐지와 보관함 정렬
- 객체 배출 동작

`mission_manager`도 `GRAB_OBJECT -> DONE` 흐름이므로 대회 전체 미션을
대체하지 않는다.

### [P1] `/cmd_vel`에 대한 단일 소유권 또는 arbitration이 없다

다음 노드가 모두 `/cmd_vel` publisher가 될 수 있다.

- `rl_model_policy`
- `mission_manager`
- `keyboard_teleop`
- `timed_motion`

특히 RL 노드는 비활성 상태에서도 기본적으로 20Hz의 정지 명령을 발행한다.

- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:92`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:317`

다른 제어 노드와 동시에 실행하면 실제 모터 명령이 publisher 실행 순서에 따라
바뀔 수 있다. 문서 경고만으로는 안전을 보장하기 어렵다.

권장 수정:

- `twist_mux` 또는 명시적인 command arbiter 도입
- 제어 모드별 topic 분리: `/cmd_vel/rl`, `/cmd_vel/teleop`, `/cmd_vel/safety`
- 안전 정지 입력에 가장 높은 우선순위 부여

### [P1] 위치 추정은 실제 odometry가 아니라 command dead reckoning이다

`robot_pose_tracker`는 `/cmd_vel.linear.x`를 이동 속도로 사용하고 IMU z축
각속도를 방향 변화에 사용한다.

- `robot_pose_tracker/robot_pose_tracker/robot_pose_tracker_node.py:138`
- `robot_pose_tracker/robot_pose_tracker/robot_pose_tracker_node.py:180`
- `robot_pose_tracker/robot_pose_tracker/robot_pose_tracker_node.py:193`

바퀴가 미끄러지거나 객체·벽에 막혀도 명령이 존재하면 이동한 것으로 계산한다.
공분산을 거리와 함께 증가시키는 처리는 있지만 실제 오차를 보정하지는 않는다.

- `robot_pose_tracker/robot_pose_tracker/robot_pose_tracker_node.py:246`

따라서 이 pose만으로 보관함에 복귀하는 것은 위험하다. 엔코더 피드백, 하향
카메라 optical flow, VIO 또는 경기장 landmark 보정 중 하나가 추가돼야 한다.

### [P2] 모델-런타임 계약을 검증하는 테스트가 없다

현재 테스트는 프레임 보정 4개와 pose 적분 5개가 중심이다. 다음 테스트가 없다.

- 저장된 RL 체크포인트 로딩
- observation 길이·순서·정규화 검증
- RL action에서 `/cmd_vel` 변환 검증
- 그리퍼 상태 전이 검증
- 통합 launch smoke test

이번 P0 결함은 모델 파일만 교체되고 런타임 계약 테스트가 없어서 발생했다.

### [P2] Jetson Python/CUDA 의존성이 재현 가능하게 고정돼 있지 않다

`rl_model_policy`는 `torch`를 직접 import하지만 ROS package dependency나
버전 매트릭스가 없다. `ros2_yolo_detector/requirements.txt`는
`ultralytics` 버전을 고정하지 않고, `setup.py`는 `opencv-python`을
install requirement로 둔다.

- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:15`
- `rl_model_policy/package.xml`
- `ros2_yolo_detector/requirements.txt`
- `ros2_yolo_detector/setup.py`

Jetson에서는 일반 PyPI PyTorch/OpenCV가 JetPack CUDA, cuDNN, apt의
`cv_bridge`와 충돌할 수 있다. 실제로 이 프로젝트는 CUDA driver, cuDNN,
`numpy<2`, `setuptools<80` 호환성에 영향을 받는다.

권장 수정:

- JetPack release별 검증된 버전을 문서와 lock 파일에 기록
- 시스템 OpenCV와 pip OpenCV를 혼용하지 않도록 설치 전략 통일
- `python3 -m pip check`와 CUDA convolution smoke test를 배포 전 수행

### [P2] 실시간 프레임 보정의 지연시간 예산이 측정되지 않았다

최신 코드에서는 camera1 inference 전에 gamma LUT, CLAHE, LAB 색상 변환과
chroma gain을 적용하며 기본값으로 활성화한다.

- `ros2_yolo_detector/ros2_yolo_detector/frame_correction.py`
- `ros2_yolo_detector/ros2_yolo_detector/yolo_camera_node.py:267`
- `ros2_yolo_detector/launch/v4l2_yolo_camera.launch.py:71`

이미지 속성 단위 테스트는 있지만 Jetson에서의 FPS와 end-to-end latency 테스트는
없다. 보정이 학습 데이터와 일치한다는 장점이 있으나, 1280x720 입력과
`imgsz=800`에서 CPU 전처리가 TensorRT 처리량을 제한할 수 있다.

권장 수정:

- 보정 on/off 상태에서 `/yolo/detections` Hz 및 callback latency 측정
- 프레임 드롭 정책과 목표 최대 지연시간 정의
- 필요하면 보정 해상도 축소 또는 CUDA 전처리 검토

### [P2] 두 번째 카메라와 wall sensor가 RL 정책에 통합되지 않았다

`mission_manager/object_pickup_mission.launch.py`는 두 번째 카메라 pipeline을
선택적으로 만들지만 `rl_autonomous_drive.launch.py`는 camera1만 include한다.
또한 `wall_distance_sensor`는 거리와 벽 각도를 publish하지만 RL observation이나
안전 정지 로직에서 구독하지 않는다.

현재 RL 정책은 전방 YOLO의 target/avoid 데이터만 사용한다. 두 번째 카메라 또는
ToF 센서가 켜져 있어도 벽 회피와 보관함 탐색에 자동으로 반영되지 않는다.

### [P2] detection 인터페이스가 JSON 문자열이다

YOLO detection과 다중 회피 객체는 `std_msgs/String` JSON으로 전달된다.

- `ros2_yolo_detector/ros2_yolo_detector/yolo_camera_node.py:130`
- `ros2_yolo_detector/ros2_yolo_detector/detections_to_target_node.py:75`
- `rl_model_policy/rl_model_policy/rl_model_policy_node.py:239`

필드 오타와 스키마 변경이 빌드 시점에 검출되지 않으며 각 노드가 별도로
파싱해야 한다. 장기적으로 custom message 또는 표준 detection message를
사용하는 편이 안전하다.

### [해결됨] 사용자 문서가 최신 코드 상태와 맞지 않았다

루트 `RL_MODEL_POLICY_RUN_GUIDE.md`의 10개 observation 설명과 Windows 중심
명령을 제거하고, 18개 observation 및 Jetson 통합 launch 기준으로 교체했다.

루트 `README.md`도 클래스 이름 한 줄만 있어 설치, 빌드, 안전 정지, 현재 제한을
안내하지 않는다. 이 리뷰와 함께 추가한 `ROBOT_RUN_MANUAL.md`를 현재 Jetson
운용 기준 문서로 사용해야 한다.

### [P3] 테스트 범위가 좁고 실행 위치에 민감하다

검토 환경에서 다음 테스트는 통과했다.

```text
ros2_yolo_detector frame correction: 4 passed
robot_pose_tracker pose estimator: 5 passed
Python compileall: passed
```

`robot_pose_tracker` 테스트는 패키지 디렉터리에서는 통과하지만 저장소 루트에서
직접 `unittest discover`를 실행하면 source path가 추가되지 않아 import에
실패한다. colcon 환경과 독립 실행 양쪽을 지원하도록 테스트 path 설정을
통일하는 것이 좋다.

검토 호스트는 Windows이며 ROS 2 Humble 런타임이 없어서 `colcon build`, 실제
launch, USB camera, serial motor controller, TensorRT inference는 실행하지 못했다.
이 부분은 Jetson에서 `ROBOT_RUN_MANUAL.md`의 preflight와 dry-run으로 확인해야 한다.

## 좋은 점

- `cmd_vel_to_motor`가 0.5초 command timeout 후 모터 정지 명령을 계속 발행한다.
- 바퀴 반지름, 차폭, 모터 ID와 방향이 parameter로 분리돼 있다.
- 카메라를 `/dev/v4l/by-path`로 고정해 `/dev/video*` 번호 변화를 피한다.
- YOLO 결과가 이전 프레임 상태 없이 현재 프레임의 bbox와 confidence를 직접 제공한다.
- 목표와 회피 객체가 겹칠 때 중복 회피를 줄이는 필터가 있다.
- RL 그리퍼 동작이 `TRACKING -> OPENING -> FINAL_FORWARD -> CLOSING`으로
  명확히 분리돼 있다.
- `dry_run`, 수동 start/stop, `speed_scale`, motor timeout 등 단계적 시험 수단이
  있다.
- 카메라 프레임 보정과 pose 적분에는 순수 Python 단위 테스트가 있다.
- 녹화 도구는 카메라별 node/topic을 분리해 프레임 혼합을 방지한다.

## 권장 수정 순서

1. **RL 프로세스 실패 시 전체 launch 종료 및 readiness 추가**
2. **`/cmd_vel` mux와 안전 정지 우선순위 도입**
3. **다중 수집, 완료 조건, 보관함 복귀, 태극기 정렬, 배출 구현**
4. **엔코더/optical flow/landmark 기반 위치 보정 추가**
5. **JetPack별 Python/CUDA 의존성 고정**
6. **통합 launch와 실제 체크포인트를 포함한 CI/smoke test 추가**
