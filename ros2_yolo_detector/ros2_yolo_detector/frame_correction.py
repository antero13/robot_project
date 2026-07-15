from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


def _validate_settings(
    gamma: float,
    clahe_clip_limit: float,
    clahe_tile_grid: int,
    chroma_gain: float,
) -> None:
    if gamma <= 0:
        raise ValueError("gamma must be greater than 0")
    if clahe_clip_limit < 0:
        raise ValueError("clahe_clip_limit must be 0 or greater")
    if clahe_tile_grid <= 0:
        raise ValueError("clahe_tile_grid must be greater than 0")
    if chroma_gain <= 0:
        raise ValueError("chroma_gain must be greater than 0")


@dataclass(frozen=True)
class LetterboxTransform:
    original_width: int
    original_height: int
    input_width: int
    input_height: int
    resized_width: int
    resized_height: int
    pad_left: int
    pad_top: int

    @classmethod
    def for_square(
        cls,
        original_width: int,
        original_height: int,
        image_size: int,
    ) -> "LetterboxTransform":
        if original_width <= 0 or original_height <= 0 or image_size <= 0:
            raise ValueError("image dimensions must be greater than 0")

        scale = min(image_size / original_width, image_size / original_height)
        resized_width = max(1, min(image_size, round(original_width * scale)))
        resized_height = max(1, min(image_size, round(original_height * scale)))
        return cls(
            original_width=original_width,
            original_height=original_height,
            input_width=image_size,
            input_height=image_size,
            resized_width=resized_width,
            resized_height=resized_height,
            pad_left=(image_size - resized_width) // 2,
            pad_top=(image_size - resized_height) // 2,
        )

    def to_original_bbox(self, xyxy: list[float]) -> list[float]:
        if len(xyxy) != 4:
            raise ValueError("xyxy must contain exactly four values")

        scale_x = self.resized_width / self.original_width
        scale_y = self.resized_height / self.original_height
        x1 = (float(xyxy[0]) - self.pad_left) / scale_x
        y1 = (float(xyxy[1]) - self.pad_top) / scale_y
        x2 = (float(xyxy[2]) - self.pad_left) / scale_x
        y2 = (float(xyxy[3]) - self.pad_top) / scale_y
        return [
            min(max(x1, 0.0), float(self.original_width)),
            min(max(y1, 0.0), float(self.original_height)),
            min(max(x2, 0.0), float(self.original_width)),
            min(max(y2, 0.0), float(self.original_height)),
        ]


@dataclass(frozen=True)
class CudaCorrectedFrame:
    tensor: object
    transform: LetterboxTransform


class FrameCorrector:
    def __init__(
        self,
        enabled: bool = True,
        gamma: float = 0.65,
        clahe_clip_limit: float = 1.2,
        clahe_tile_grid: int = 8,
        chroma_gain: float = 1.3,
    ) -> None:
        _validate_settings(
            gamma,
            clahe_clip_limit,
            clahe_tile_grid,
            chroma_gain,
        )

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


class CudaFrameCorrector:
    """Correct and letterbox a BGR frame as a CUDA RGB tensor."""

    def __init__(
        self,
        image_size: int,
        device: str = "cuda:0",
        enabled: bool = True,
        gamma: float = 0.65,
        clahe_clip_limit: float = 1.2,
        clahe_tile_grid: int = 8,
        chroma_gain: float = 1.3,
    ) -> None:
        _validate_settings(
            gamma,
            clahe_clip_limit,
            clahe_tile_grid,
            chroma_gain,
        )
        if image_size <= 0:
            raise ValueError("image_size must be greater than 0")

        try:
            import torch
            import torch.nn.functional as functional
            from kornia.color import lab_to_rgb, rgb_to_lab
            from kornia.enhance import equalize_clahe
        except ImportError as exc:
            raise RuntimeError(
                "CUDA correction requires torch and kornia. "
                "Install ros2_yolo_detector/requirements.txt."
            ) from exc

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA correction requested but torch CUDA is unavailable")

        torch_device = torch.device(device)
        if torch_device.type != "cuda":
            raise ValueError("CUDA correction device must be a CUDA device")
        try:
            torch.empty(1, device=torch_device)
        except Exception as exc:
            raise RuntimeError(
                f"CUDA correction device is unavailable: {torch_device}"
            ) from exc

        self.torch = torch
        self.functional = functional
        self.rgb_to_lab = rgb_to_lab
        self.lab_to_rgb = lab_to_rgb
        self.equalize_clahe = equalize_clahe
        self.device = torch_device
        self.image_size = int(image_size)
        self.enabled = enabled
        self.gamma = float(gamma)
        self.clahe_clip_limit = float(clahe_clip_limit)
        self.clahe_tile_grid = int(clahe_tile_grid)
        self.chroma_gain = float(chroma_gain)

    def apply(self, frame: np.ndarray) -> CudaCorrectedFrame:
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("CUDA correction expects a BGR image with three channels")
        if frame.dtype != np.uint8:
            raise ValueError("CUDA correction expects a uint8 image")

        height, width = frame.shape[:2]
        transform = LetterboxTransform.for_square(width, height, self.image_size)
        contiguous = np.ascontiguousarray(frame)

        with self.torch.inference_mode():
            tensor = self.torch.from_numpy(contiguous).to(
                device=self.device,
                dtype=self.torch.float32,
                non_blocking=True,
            )
            tensor = tensor.permute(2, 0, 1).unsqueeze(0).div_(255.0)
            tensor = tensor[:, [2, 1, 0], :, :]

            if self.enabled:
                tensor = tensor.clamp_(0.0, 1.0).pow_(self.gamma)
                if self.clahe_clip_limit > 0.0 or self.chroma_gain != 1.0:
                    lab = self.rgb_to_lab(tensor)
                    l_channel = lab[:, 0:1]
                    if self.clahe_clip_limit > 0.0:
                        l_channel = self.equalize_clahe(
                            (l_channel / 100.0).clamp(0.0, 1.0),
                            clip_limit=self.clahe_clip_limit,
                            grid_size=(self.clahe_tile_grid, self.clahe_tile_grid),
                            slow_and_differentiable=False,
                        ) * 100.0
                    chroma = lab[:, 1:3]
                    if self.chroma_gain != 1.0:
                        chroma = (chroma * self.chroma_gain).clamp(-128.0, 127.0)
                    tensor = self.lab_to_rgb(
                        self.torch.cat((l_channel, chroma), dim=1),
                        clip=True,
                    )

            if tensor.shape[-2:] != (
                transform.resized_height,
                transform.resized_width,
            ):
                tensor = self.functional.interpolate(
                    tensor,
                    size=(transform.resized_height, transform.resized_width),
                    mode="bilinear",
                    align_corners=False,
                )

            pad_right = (
                transform.input_width - transform.resized_width - transform.pad_left
            )
            pad_bottom = (
                transform.input_height - transform.resized_height - transform.pad_top
            )
            if transform.pad_left or transform.pad_top or pad_right or pad_bottom:
                tensor = self.functional.pad(
                    tensor,
                    (transform.pad_left, pad_right, transform.pad_top, pad_bottom),
                    value=114.0 / 255.0,
                )

        return CudaCorrectedFrame(tensor.contiguous(), transform)
