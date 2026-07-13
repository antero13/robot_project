# 전체 대회 미션 구현 계획

## 목표

3분 동안 목표 객체를 반복해서 수집하고, 종료 조건이 되면 좌측 하단 Storage
Zone으로 이동해 태극기로 최종 위치를 보정한 뒤 객체를 내려놓는다.

현재 RL 정책은 카메라 기반 탐색, 회피, 목표 접근과 한 번의 집기를 담당한다.
전체 경기 흐름은 RL 내부에 추가하지 않고 상위 mission coordinator가 관리한다.

## 제안 구조

```text
competition_mission_manager
  -> rl_model_policy_control       search/pickup start, stop, reset
  -> return_to_storage_controller  storage return and final approach
  -> gripper_controller            deposit

camera/YOLO -> targets/obstacles -> RL policy
robot_pose_tracker -> odom -------^-----------> return controller
flag_detector -> flag target -----------------> return controller

RL /cmd_vel -----------+
return /cmd_vel -------+-> cmd_vel_mux -> cmd_vel_to_motor
teleop /cmd_vel -------+
emergency stop --------+  highest priority
```

`/cmd_vel`을 여러 노드가 직접 publish하지 않도록 mux를 먼저 도입한다. 긴급 정지,
수동 조종, 자동 복귀, RL 순서로 우선순위를 명시한다.

## 상태 머신

```text
INIT
  -> LEAVE_START
  -> COLLECT_SEARCH
  -> COLLECT_APPROACH
  -> GRAB
  -> VERIFY_GRAB
  -> COLLECT_SEARCH (반복)
  -> RETURN_TO_STAGING
  -> SEARCH_FLAG
  -> ALIGN_STORAGE
  -> APPROACH_STORAGE
  -> DEPOSIT
  -> DONE
```

모든 상태에서 `STOPPED`와 `RECOVERY`로 이동할 수 있어야 한다.

수집 종료 조건은 다음을 parameter로 둔다.

- `target_count`: 기본 7개
- `return_time_remaining_s`: 복귀에 남겨둘 시간
- `max_consecutive_search_failures`
- 운영자 강제 복귀 명령

## 수집 단계

RL의 국소 탐색만으로 4m x 4m 전체를 확실히 덮기는 어렵다. 상위 노드는 경기장
중심 좌표계의 coverage waypoint를 순서대로 선택하고, 각 구간의 장애물 회피와
목표 추적은 RL에 맡긴다.

1. Start Zone을 빠져나온다.
2. 지그재그 또는 ㄹ자 coverage waypoint로 미탐색 영역을 줄인다.
3. 목표가 감지되면 waypoint 이동을 중지하고 RL pickup을 시작한다.
4. 그리퍼 닫힘과 접근 완료를 확인해 수집 수를 증가시킨다.
5. 실패하면 동일 물체에 무한 재시도하지 않도록 위치와 시간을 기록한다.
6. 목표 수 또는 복귀 시간이 되면 Storage Zone 복귀로 전환한다.

실제 객체 보유 여부 센서가 없으므로 초기 `VERIFY_GRAB`은 그리퍼 명령 성공,
목표가 카메라 하단에서 사라짐, 짧은 재확인 시간을 조합한다. 이후 리미트 스위치나
전류 감지를 추가하면 이를 우선 사용한다.

## 보관함 복귀

1. odometry로 좌측 하단의 staging point까지 이동한다.
2. staging point에서 천천히 회전하며 태극기를 탐색한다.
3. 태극기 bounding box를 화면 중앙에 맞춘다.
4. 태극기 크기 또는 하단 y값으로 거리를 줄인다.
5. Storage Zone 진입 자세가 되면 정지하고 그리퍼를 열어 배출한다.

태극기가 멀리서 보이지 않으므로 odometry는 최종 정렬 수단이 아니라 태극기가
보이는 근처까지 가기 위한 수단으로 사용한다. `/cmd_vel` 적분 기반 위치는 미끄럼
오차가 누적되므로 optical flow, encoder feedback, 벽/랜드마크 보정 중 하나를
추가하는 것이 전체 미션 성공 조건이다.

## ROS 인터페이스

새 토픽과 서비스의 권장 형태:

```text
/competition/control        std_msgs/String: start, stop, reset, return
/competition/state          std_msgs/String 또는 custom MissionState
/competition/progress       collected, failures, remaining_time
/flag_target                geometry_msgs/PointStamped
/rl_model_policy_control    start, stop, reset
/cmd_vel/rl                 geometry_msgs/Twist
/cmd_vel/return             geometry_msgs/Twist
/cmd_vel/teleop             geometry_msgs/Twist
/cmd_vel                    mux 최종 출력만 사용
```

장기적으로 detection JSON은 custom message로 변경해 class, bbox, confidence,
track ID를 타입 안전하게 전달한다.

## 구현 순서와 완료 기준

### M0: 18-input 정책 안정화

- 최신 체크포인트 strict load
- `/rl_model_policy_state.obs` 길이 18
- `/odom` 정상 시 `obs[10] == 1`
- dry-run 10분 동안 예외 없음

### M1: 한 객체 수집

- RL start부터 그리퍼 닫힘까지 자동 수행
- 실패 시 정지 또는 재탐색
- `/cmd_vel` publisher 충돌 없음

### M2: 반복 수집

- `stop_after_grab:=false` 기반 반복 또는 coordinator 재시작
- 수집 횟수와 실패 횟수 기록
- 이미 집은 위치로 즉시 되돌아가지 않음

### M3: 경기장 coverage

- 목표가 없어도 4m x 4m의 주요 영역을 제한 시간 안에 훑음
- 벽 근처 waypoint와 안전 여유 거리 검증

### M4: 보관함 복귀와 태극기 정렬

- 경기장 여러 위치에서 staging point 도달
- 태극기 감지 후 중앙 정렬
- 태극기 일시 소실 시 마지막 방향으로 제한 시간 재탐색

### M5: 전체 3분 시험

- 수집, 시간 기반 복귀, 배출, 종료를 한 launch로 수행
- 긴급 정지와 수동 takeover 검증
- 최소 20회 반복 시험의 성공률과 실패 원인 기록

## 우선 수정 파일

```text
새 패키지: competition_mission_manager
새 패키지 또는 노드: cmd_vel_mux
새 노드: flag_detector
새 노드: return_to_storage_controller
수정: rl_model_policy (상태/결과 인터페이스)
수정: robot_pose_tracker (외부 위치 보정 입력)
수정: rl_autonomous_drive.launch.py (전체 launch 및 실패 전파)
```

첫 구현 작업은 `cmd_vel_mux`와 `competition_mission_manager`의 상태 뼈대를 만든
뒤, 기존 RL 한 번 집기 결과를 명시적인 성공/실패 이벤트로 바꾸는 것이다.
