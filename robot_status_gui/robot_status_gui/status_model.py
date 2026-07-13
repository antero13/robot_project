import json
import math


MODE_LABELS = {
    "IDLE": "정지",
    "TRACK_TARGET": "목표 물체 접근 중",
    "LOCAL_REACQUIRE": "목표 물체 재탐색 중",
    "COVERAGE_SEARCH": "경기장 탐색 중",
    "WAITING_FOR_POSE": "위치 정보 대기 중",
    "GRAB_SEQUENCE": "물체 수납 동작 중",
}

GRAB_LABELS = {
    "TRACKING": "대기",
    "OPENING": "수납구 열기",
    "FINAL_FORWARD": "물체 안쪽으로 이동",
    "CLOSING": "수납구 닫기",
    "GRABBED": "수납 완료",
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


def quaternion_to_yaw(x, y, z, w):
    sin_yaw = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cos_yaw = 1.0 - 2.0 * (float(y) ** 2 + float(z) ** 2)
    return math.atan2(sin_yaw, cos_yaw)


def centered_pose_to_map(x, y, offset_x=2.0, offset_y=2.0):
    return float(x) + float(offset_x), float(y) + float(offset_y)
