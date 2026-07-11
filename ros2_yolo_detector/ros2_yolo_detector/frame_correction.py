from __future__ import annotations

import cv2
import numpy as np


class FrameCorrector:
    def __init__(
        self,
        enabled: bool = True,
        gamma: float = 0.65,
        clahe_clip_limit: float = 1.2,
        clahe_tile_grid: int = 8,
        chroma_gain: float = 1.3,
    ) -> None:
        if gamma <= 0:
            raise ValueError("gamma must be greater than 0")
        if clahe_clip_limit < 0:
            raise ValueError("clahe_clip_limit must be 0 or greater")
        if clahe_tile_grid <= 0:
            raise ValueError("clahe_tile_grid must be greater than 0")
        if chroma_gain <= 0:
            raise ValueError("chroma_gain must be greater than 0")

        self.enabled = enabled
        self.chroma_gain = chroma_gain
        self.gamma_lut = None
        self.clahe = None
        if not enabled:
            return

        values = np.arange(256, dtype=np.float32) / 255.0
        self.gamma_lut = np.clip(
            np.power(values, gamma) * 255.0 + 0.5,
            0,
            255,
        ).astype(np.uint8)
        if clahe_clip_limit > 0:
            self.clahe = cv2.createCLAHE(
                clipLimit=clahe_clip_limit,
                tileGridSize=(clahe_tile_grid, clahe_tile_grid),
            )

    def apply(self, frame: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return frame

        corrected = cv2.LUT(frame, self.gamma_lut)
        if self.clahe is None and self.chroma_gain == 1.0:
            return corrected

        lab = cv2.cvtColor(corrected, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        if self.clahe is not None:
            l_channel = self.clahe.apply(l_channel)
        if self.chroma_gain != 1.0:
            a_channel = self._boost_chroma(a_channel)
            b_channel = self._boost_chroma(b_channel)
        return cv2.cvtColor(
            cv2.merge((l_channel, a_channel, b_channel)),
            cv2.COLOR_LAB2BGR,
        )

    def _boost_chroma(self, channel: np.ndarray) -> np.ndarray:
        centered = channel.astype(np.float32) - 128.0
        return np.clip(
            centered * self.chroma_gain + 128.0,
            0,
            255,
        ).astype(np.uint8)
