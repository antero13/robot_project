from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class ClassObservation:
    class_id: int
    class_name: str
    confidence: float


@dataclass
class TrackState:
    history: deque[ClassObservation]
    last_seen_s: float
    locked_class_id: int | None = None
    locked_class_name: str | None = None
    last_detection: dict[str, Any] | None = None


@dataclass
class ClassSummary:
    count: int = 0
    confidence_sum: float = 0.0
    class_name: str = ""
    latest_index: int = -1

    @property
    def mean_confidence(self) -> float:
        if self.count <= 0:
            return 0.0
        return self.confidence_sum / self.count


@dataclass
class TemporalDetectionStabilizer:
    enabled: bool = True
    history_size: int = 5
    lock_min_votes: int = 3
    lock_min_confidence: float = 0.4
    switch_min_votes: int = 4
    switch_min_confidence: float = 0.6
    detection_hold_s: float = 0.35
    track_retention_s: float = 1.0
    tracks: dict[tuple[str, int], TrackState] = field(default_factory=dict, init=False)

    def __post_init__(self) -> None:
        if self.history_size <= 0:
            raise ValueError("history_size must be greater than 0")
        if not 1 <= self.lock_min_votes <= self.history_size:
            raise ValueError("lock_min_votes must be within the class history size")
        if not 1 <= self.switch_min_votes <= self.history_size:
            raise ValueError("switch_min_votes must be within the class history size")
        if not 0.0 <= self.lock_min_confidence <= 1.0:
            raise ValueError("lock_min_confidence must be within 0..1")
        if not 0.0 <= self.switch_min_confidence <= 1.0:
            raise ValueError("switch_min_confidence must be within 0..1")
        if self.detection_hold_s < 0.0:
            raise ValueError("detection_hold_s must be 0 or greater")
        if self.track_retention_s <= 0.0:
            raise ValueError("track_retention_s must be greater than 0")

    def update(self, detections: list[dict[str, Any]], now_s: float) -> list[dict[str, Any]]:
        if not self.enabled:
            return deepcopy(detections)

        self._prune(now_s)
        current_keys = set()
        current_detections = []

        for source_detection in detections:
            detection = deepcopy(source_detection)
            detection["temporally_held"] = False
            detection["hold_age_s"] = 0.0
            track_key = self._track_key(detection)
            if track_key is None:
                detection["class_locked"] = False
                current_detections.append(detection)
                continue

            current_keys.add(track_key)
            state = self.tracks.get(track_key)
            if state is None:
                state = TrackState(
                    history=deque(maxlen=self.history_size),
                    last_seen_s=now_s,
                )
                self.tracks[track_key] = state

            state.last_seen_s = now_s
            observation = self._observation(detection)
            if observation is not None:
                state.history.append(observation)
                self._update_class_lock(state)
                self._apply_locked_class(detection, observation, state)
            else:
                detection["class_locked"] = state.locked_class_id is not None

            state.last_detection = deepcopy(detection)
            current_detections.append(detection)

        held_detections = []
        if self.detection_hold_s > 0.0:
            for track_key, state in self.tracks.items():
                if track_key in current_keys or state.last_detection is None:
                    continue
                hold_age_s = max(0.0, now_s - state.last_seen_s)
                if hold_age_s > self.detection_hold_s:
                    continue

                held = deepcopy(state.last_detection)
                if self._overlaps_current_detection(held, current_detections):
                    continue
                held["temporally_held"] = True
                held["hold_age_s"] = round(hold_age_s, 4)
                held_detections.append(held)

        return current_detections + held_detections

    @staticmethod
    def _track_key(detection: dict[str, Any]) -> tuple[str, int] | None:
        for field_name in ("stable_track_id", "track_id"):
            value = detection.get(field_name)
            if value is None:
                continue
            try:
                return field_name, int(value)
            except (TypeError, ValueError):
                continue
        return None

    @staticmethod
    def _observation(detection: dict[str, Any]) -> ClassObservation | None:
        try:
            class_id = int(detection["class_id"])
            class_name = str(detection.get("class_name", class_id))
            confidence = float(detection.get("confidence", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        confidence = max(0.0, min(1.0, confidence))
        return ClassObservation(class_id, class_name, confidence)

    def _update_class_lock(self, state: TrackState) -> None:
        summaries = self._class_summaries(state.history)
        if not summaries:
            return

        winner_id, winner = max(
            summaries.items(),
            key=lambda item: (
                item[1].count,
                item[1].confidence_sum,
                item[1].latest_index,
            ),
        )
        if state.locked_class_id is None:
            if (
                winner.count >= self.lock_min_votes
                and winner.mean_confidence >= self.lock_min_confidence
            ):
                state.locked_class_id = winner_id
                state.locked_class_name = winner.class_name
            return

        if winner_id == state.locked_class_id:
            state.locked_class_name = winner.class_name
            return

        if (
            winner.count >= self.switch_min_votes
            and winner.mean_confidence >= self.switch_min_confidence
        ):
            state.locked_class_id = winner_id
            state.locked_class_name = winner.class_name

    @staticmethod
    def _class_summaries(history: deque[ClassObservation]) -> dict[int, ClassSummary]:
        summaries: dict[int, ClassSummary] = {}
        for index, observation in enumerate(history):
            summary = summaries.setdefault(observation.class_id, ClassSummary())
            summary.count += 1
            summary.confidence_sum += observation.confidence
            summary.class_name = observation.class_name
            summary.latest_index = index
        return summaries

    def _apply_locked_class(
        self,
        detection: dict[str, Any],
        observation: ClassObservation,
        state: TrackState,
    ) -> None:
        detection["observed_class_id"] = observation.class_id
        detection["observed_class_name"] = observation.class_name
        if state.locked_class_id is None:
            detection["class_locked"] = False
            detection["class_stabilized"] = False
            return

        summaries = self._class_summaries(state.history)
        locked_summary = summaries.get(state.locked_class_id)
        detection["class_id"] = state.locked_class_id
        detection["class_name"] = state.locked_class_name or str(state.locked_class_id)
        detection["class_locked"] = True
        detection["class_stabilized"] = observation.class_id != state.locked_class_id
        detection["temporal_class_confidence"] = round(
            locked_summary.mean_confidence if locked_summary is not None else 0.0,
            6,
        )

    def _prune(self, now_s: float) -> None:
        retention_s = max(self.track_retention_s, self.detection_hold_s)
        stale_keys = [
            track_key
            for track_key, state in self.tracks.items()
            if now_s - state.last_seen_s > retention_s
        ]
        for track_key in stale_keys:
            del self.tracks[track_key]

    @classmethod
    def _overlaps_current_detection(
        cls,
        held: dict[str, Any],
        current_detections: list[dict[str, Any]],
    ) -> bool:
        held_bbox = cls._bbox_tuple(held)
        if held_bbox is None:
            return False
        return any(
            cls._bbox_iou(held_bbox, current_bbox) >= 0.5
            for current_bbox in (cls._bbox_tuple(item) for item in current_detections)
            if current_bbox is not None
        )

    @staticmethod
    def _bbox_tuple(
        detection: dict[str, Any],
    ) -> tuple[float, float, float, float] | None:
        bbox = detection.get("bbox_xyxy", {})
        try:
            return (
                float(bbox["x1"]),
                float(bbox["y1"]),
                float(bbox["x2"]),
                float(bbox["y2"]),
            )
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _bbox_iou(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        first_x1, first_y1, first_x2, first_y2 = first
        second_x1, second_y1, second_x2, second_y2 = second
        intersection_width = max(0.0, min(first_x2, second_x2) - max(first_x1, second_x1))
        intersection_height = max(0.0, min(first_y2, second_y2) - max(first_y1, second_y1))
        intersection_area = intersection_width * intersection_height
        first_area = max(0.0, first_x2 - first_x1) * max(0.0, first_y2 - first_y1)
        second_area = max(0.0, second_x2 - second_x1) * max(0.0, second_y2 - second_y1)
        union_area = first_area + second_area - intersection_area
        if union_area <= 0.0:
            return 0.0
        return intersection_area / union_area
