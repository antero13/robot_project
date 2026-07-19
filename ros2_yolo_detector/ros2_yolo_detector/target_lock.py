from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetLock:
    bbox_xyxy: tuple[float, float, float, float]
    normalized_x: float
    normalized_center_y: float


def bbox_iou(first, second):
    first_x1, first_y1, first_x2, first_y2 = first
    second_x1, second_y1, second_x2, second_y2 = second
    intersection_width = max(0.0, min(first_x2, second_x2) - max(first_x1, second_x1))
    intersection_height = max(0.0, min(first_y2, second_y2) - max(first_y1, second_y1))
    intersection_area = intersection_width * intersection_height
    first_area = max(0.0, first_x2 - first_x1) * max(0.0, first_y2 - first_y1)
    second_area = max(0.0, second_x2 - second_x1) * max(0.0, second_y2 - second_y1)
    union_area = first_area + second_area - intersection_area
    return 0.0 if union_area <= 0.0 else intersection_area / union_area


def lock_from_candidate(candidate):
    _, _, point_msg, bbox_xyxy, center_y = candidate
    return TargetLock(
        bbox_xyxy=tuple(float(value) for value in bbox_xyxy),
        normalized_x=float(point_msg.point.x),
        normalized_center_y=float(center_y),
    )


def candidate_matches_lock(
    candidate,
    target_lock,
    *,
    iou_threshold,
    center_distance_threshold,
):
    candidate_lock = lock_from_candidate(candidate)
    iou = bbox_iou(candidate_lock.bbox_xyxy, target_lock.bbox_xyxy)
    center_distance = math.hypot(
        candidate_lock.normalized_x - target_lock.normalized_x,
        candidate_lock.normalized_center_y - target_lock.normalized_center_y,
    )
    return iou >= iou_threshold or center_distance <= center_distance_threshold


def select_locked_candidate(
    candidates,
    target_lock,
    *,
    score,
    iou_threshold,
    center_distance_threshold,
):
    if not candidates:
        return None
    if target_lock is None:
        return max(candidates, key=score)

    matching = [
        candidate
        for candidate in candidates
        if candidate_matches_lock(
            candidate,
            target_lock,
            iou_threshold=iou_threshold,
            center_distance_threshold=center_distance_threshold,
        )
    ]
    if not matching:
        return None
    return max(
        matching,
        key=lambda candidate: (
            bbox_iou(candidate[3], target_lock.bbox_xyxy),
            score(candidate),
        ),
    )
