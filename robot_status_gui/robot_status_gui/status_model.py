import json
import math


MODE_LABELS = {
    "IDLE": "정지",
    "TRACK_TARGET": "목표 물체 접근 중",
    "LOCAL_REACQUIRE": "목표 물체 재탐색 중",
    "COVERAGE_SEARCH": "경기장 탐색 중",
    "WAITING_FOR_POSE": "위치 정보 대기 중",
    "GRAB_SEQUENCE": "물체 수납 동작 중",
    "RETURN_TO_STORAGE": "보관함으로 복귀 중",
    "ENTER_STORAGE": "보관함 진입 중",
    "DEPOSIT": "물체 배출 중",
    "EXIT_STORAGE": "보관함에서 후진 중",
    "MISSION_COMPLETE": "미션 완료",
    "MISSION_TIMEOUT": "경기 시간 종료",
}

MISSION_LABELS = {
    "IDLE": "미션 대기",
    "COLLECTING": "물체 탐색 및 수집",
    "RETURN_MAIN_ROAD": "보관함 복귀 · 주 경로 이동",
    "RETURN_STAGING": "보관함 복귀 · 진입 위치 이동",
    "ENTER_STORAGE": "보관함 진입",
    "DEPOSIT": "수납 물체 배출",
    "EXIT_STORAGE": "보관함에서 후진",
    "CLOSE_AFTER_DEPOSIT": "수납구 닫기",
    "COMPLETE": "목표 7개 운반 완료",
    "TIMEOUT": "3분 경기 종료",
    "STOPPED": "미션 정지",
}

RETURN_REASON_LABELS = {
    "CAPACITY": "내부 수납 한도 4개",
    "TARGET_COUNT": "목표 7개 수집",
    "TIME_LIMIT": "남은 시간 30초",
    "MANUAL": "운영자 복귀 명령",
}

GRAB_LABELS = {
    "TRACKING": "대기",
    "OPENING": "수납구 열기",
    "FINAL_FORWARD": "물체 안쪽으로 이동",
    "CLOSING": "수납구 닫기",
    "GRABBED": "수납 완료",
}

MAPPER_STATUS_LABELS = {
    "ready": "보정 준비",
    "waiting_for_odometry": "위치 정보 대기",
    "detections_outside_calibration": "보정 범위 밖",
}


def parse_json_message(raw, fallback=None):
    try:
        parsed = json.loads(raw)
    except (TypeError, json.JSONDecodeError):
        return {} if fallback is None else fallback
    if not isinstance(parsed, dict):
        return {} if fallback is None else fallback
    return parsed


def mode_label(state):
    if state.get("motion_paused"):
        return "주행 일시정지"
    mission = state.get("mission", {})
    phase = mission.get("phase") if isinstance(mission, dict) else None
    if phase not in (None, "IDLE", "COLLECTING"):
        return MISSION_LABELS.get(str(phase), str(phase))
    mode = str(state.get("control_mode", "IDLE"))
    if mode == "GRAB_SEQUENCE":
        grab = GRAB_LABELS.get(str(state.get("grab_state", "")), "수납 동작")
        return f"물체 수납 중 · {grab}"
    return MODE_LABELS.get(mode, mode)


def stored_object_label(state):
    stored = state.get("stored_objects", [])
    if not isinstance(stored, list) or not stored:
        pickup = state.get("pickup_label")
        return f"수납 중: {pickup}" if pickup else "없음"
    counts = {}
    for label in stored:
        name = str(label)
        counts[name] = counts.get(name, 0) + 1
    return ", ".join(
        name if count == 1 else f"{name} × {count}"
        for name, count in counts.items()
    )


def mission_progress_label(state):
    mission = state.get("mission", {})
    if not isinstance(mission, dict):
        return "-"
    onboard = int(mission.get("onboard_count", 0))
    capacity = int(mission.get("storage_capacity", 4))
    delivered = int(mission.get("delivered_count", 0))
    total = int(mission.get("total_collected_count", onboard + delivered))
    target = int(mission.get("target_object_count", 7))
    return (
        f"수집 {total}/{target} · 내부 {onboard}/{capacity} · "
        f"배출 {delivered}/{target}"
    )


def mission_time_label(state):
    mission = state.get("mission", {})
    if not isinstance(mission, dict):
        return "-"
    remaining = max(0, int(math.ceil(float(mission.get("remaining_s", 0.0)))))
    minutes, seconds = divmod(remaining, 60)
    return f"{minutes:02d}:{seconds:02d}"


def return_reason_label(state):
    mission = state.get("mission", {})
    if not isinstance(mission, dict):
        return "-"
    reason = mission.get("return_reason")
    return RETURN_REASON_LABELS.get(str(reason), "-") if reason else "-"


def mapper_diagnostics_label(state):
    if not isinstance(state, dict) or not state:
        return "객체 mapper 데이터 대기"
    status = str(state.get("mapper_status", "unknown"))
    status_label = MAPPER_STATUS_LABELS.get(status, status)
    detected = int(state.get("detection_count", 0))
    mapped = int(state.get("mapped_count", 0))
    calibration = "CSV" if state.get("calibration_loaded") else "보정 없음"
    return f"{status_label} · {calibration} · 감지 {detected} / 변환 {mapped}"


def quaternion_to_yaw(x, y, z, w):
    sin_yaw = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cos_yaw = 1.0 - 2.0 * (float(y) ** 2 + float(z) ** 2)
    return math.atan2(sin_yaw, cos_yaw)


def centered_pose_to_map(x, y, offset_x=2.0, offset_y=2.0):
    return float(x) + float(offset_x), float(y) + float(offset_y)
