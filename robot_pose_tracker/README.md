# robot_pose_tracker

`/cmd_vel`의 선속도와 모터 드라이버 IMU의 z축 각속도를 적분해 평면상의
`x`, `y`, `yaw`를 추정하는 ROS 2 노드입니다. 엔코더가 없는 현재 로봇에서
보관함 근처까지 돌아가기 위한 대략적인 위치를 제공합니다.

이 노드는 실제 이동 거리를 측정하지 않습니다. 바퀴 미끄러짐, 물체 충돌,
모터 속도 오차가 누적되므로 태극기가 보이기 시작한 뒤에는 카메라 기준
정렬로 전환해야 합니다.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_pose_tracker
source install/setup.bash
```

## Run

먼저 `ros_robot_controller`를 실행하고 로봇을 움직이지 않은 상태에서 실행합니다.
기본 자이로 보정 시간은 2초입니다.

경기장 왼쪽 아래를 `(0, 0)`으로 두고, Start Zone 중심에서 경기장 위쪽을
바라보고 시작한다면 다음과 같이 실행합니다.

```bash
ros2 launch robot_pose_tracker robot_pose_tracker.launch.py \
  initial_x:=3.8 \
  initial_y:=0.2 \
  initial_yaw_deg:=90.0
```

ROS 좌표 규칙에서 `0도`는 경기장 오른쪽, `90도`는 위쪽, `180도`는 왼쪽입니다.
실제 출발 방향과 `initial_yaw_deg`가 반드시 일치해야 합니다.

## Check

```bash
ros2 topic echo /robot_pose
ros2 topic echo /robot_pose/status
ros2 topic echo /odom
```

정상적으로 IMU를 사용하면 상태가 다음과 같이 바뀝니다.

```text
CALIBRATING_GYRO_CMD_FALLBACK
TRACKING_WITH_IMU
```

출발 자세로 위치 추정값을 초기화하려면 다음 서비스를 호출합니다.

```bash
ros2 service call /robot_pose/reset std_srvs/srv/Trigger "{}"
```

로봇을 멈춘 상태에서 자이로 영점을 다시 측정하려면 다음을 사용합니다.

```bash
ros2 service call /robot_pose/recalibrate_gyro std_srvs/srv/Trigger "{}"
```

## Calibration

로봇을 실제로 1m 직진시킨 뒤 `/robot_pose`의 이동량이 다르면
`linear_scale`을 조정합니다. 예를 들어 실제로 1m 이동했는데 추정값이
0.8m라면 `1.0 / 0.8 = 1.25`를 사용합니다.

```bash
ros2 launch robot_pose_tracker robot_pose_tracker.launch.py linear_scale:=1.25
```

로봇을 반시계 방향으로 돌렸을 때 추정 yaw가 감소한다면 IMU 방향이 반대입니다.

```bash
ros2 launch robot_pose_tracker robot_pose_tracker.launch.py imu_yaw_sign:=-1.0
```

## RViz visualization

NoMachine으로 Jetson 화면에 접속한 뒤 터미널에서 다음 명령을 실행하면 위치
추정 노드, 경기장 시각화 노드, RViz가 한 번에 실행됩니다.

```bash
ros2 launch robot_pose_tracker robot_pose_visualization.launch.py \
  initial_x:=3.8 \
  initial_y:=0.2 \
  initial_yaw_deg:=90.0
```

RViz에는 다음 항목이 표시됩니다.

- 4m x 4m 경기장 외곽
- Start Zone과 Storage Zone
- 50cm 간격의 객체 후보점 42개
- 청록색 로봇 본체와 빨간색 진행 방향 화살표
- 로봇이 지나온 빨간 경로

이미 `robot_pose_tracker`를 따로 실행 중이라면 중복 실행하지 않도록 다음과
같이 시각화만 시작합니다.

```bash
ros2 launch robot_pose_tracker robot_pose_visualization.launch.py \
  start_tracker:=false
```

SSH 터미널만 사용하고 화면 전달을 설정하지 않았다면 RViz 창을 열 수 없습니다.
이 경우 `start_rviz:=false`로 ROS 시각화 토픽만 실행할 수 있습니다.
