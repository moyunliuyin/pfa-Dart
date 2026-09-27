#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""海康 SDK 取流画面录制：每帧保存为图片，每次运行在总文件夹下新建 game* 子目录。"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np

SUPPORTED_IMAGE_FORMATS = {"jpg", "jpeg", "png", "bmp"}


def _parse_wh(resolution: str | None) -> tuple[int, int] | None:
    """解析 ``宽x高`` 配置；格式不合法时返回 ``None``。"""
    if resolution is None:
        return None
    s = str(resolution).strip().lower().replace(" ", "")
    if not s or "x" not in s:
        return None
    a, b = s.split("x", 1)
    try:
        return int(a), int(b)
    except ValueError:
        return None


def _next_game_folder(output_folder: Path) -> Path:
    """在 output_folder 下找到下一个可用的 game* 文件夹并创建。"""
    index = 1
    while True:
        candidate = output_folder / f"game{index}"
        if not candidate.exists():
            candidate.mkdir(parents=True, exist_ok=True)
            return candidate
        index += 1


class HikFrameRecorder:
    """将相机帧逐帧保存为图片，存入当次运行专属的 game* 子目录。"""

    def __init__(
        self,
        output_folder: str | os.PathLike | None = None,
        format: str = "jpg",
        frame_rate: float = 15.0,
        resolution: str | None = None,
    ) -> None:
        root = Path(__file__).resolve().parent
        self.output_folder = Path(output_folder) if output_folder else root / "video"
        self.output_folder.mkdir(parents=True, exist_ok=True)

        fmt = str(format or "jpg").lower().lstrip(".")
        self.img_ext = fmt if fmt in SUPPORTED_IMAGE_FORMATS else "jpg"
        self.target_wh = _parse_wh(resolution)

        self._game_folder: Path | None = None
        self._frames_written = 0

    def _ensure_game_folder(self) -> None:
        if self._game_folder is None:
            self._game_folder = _next_game_folder(self.output_folder)
            print(f"海康相机录制已开始: {self._game_folder}")

    def write_frame_rgb(self, frame_rgb: np.ndarray) -> None:
        """把 RGB 帧转换为 BGR 后按序号写入本次运行的目录。"""
        if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            return

        if self.target_wh is not None:
            tw, th = self.target_wh
            out_rgb = cv2.resize(frame_rgb, (tw, th), interpolation=cv2.INTER_AREA)
        else:
            out_rgb = frame_rgb

        # OpenCV 的 imwrite 接收 BGR；外部检测链路统一使用 RGB。
        bgr = cv2.cvtColor(out_rgb, cv2.COLOR_RGB2BGR)

        self._ensure_game_folder()
        filename = self._game_folder / f"frame_{self._frames_written:06d}.{self.img_ext}"
        bgr = cv2.resize(bgr, (300, 300))
        cv2.imwrite(str(filename), bgr)
        self._frames_written += 1

    def close(self) -> None:
        frames = self._frames_written
        folder = self._game_folder
        self._game_folder = None
        self._frames_written = 0

        if folder is not None:
            print(f"海康相机录制已结束: {folder} 共 {frames} 帧")
        else:
            print("海康相机录制结束（未写入任何帧）。")


_recorder: HikFrameRecorder | None = None


def start_system_recording(
    format: str = "jpg",
    output_folder: str | os.PathLike | None = None,
    frame_rate: float | int = 15,
    resolution: str | None = None,
) -> bool:
    """创建录制会话，在总文件夹下自动分配新的 game* 子目录。"""
    global _recorder
    _recorder = HikFrameRecorder(
        output_folder=output_folder,
        format=format,
        resolution=resolution,
    )
    print("海康取流录制已启用，待首帧到达后开始写入图片。")
    return True


def record_hik_stream_frame(frame_rgb: np.ndarray) -> None:
    if _recorder is not None:
        _recorder.write_frame_rgb(frame_rgb)


def stop_system_recording() -> None:
    global _recorder
    if _recorder is not None:
        _recorder.close()
        _recorder = None


def sync_recording_from_config(recording: dict, project_root: Path) -> None:
    """与 config 同步：关闭录制或首次启用时创建会话。"""
    if not bool(recording.get("enabled", False)):
        stop_system_recording()
        return
    global _recorder
    if _recorder is not None:
        return
    start_system_recording(
        format=str(recording.get("format", "jpg")),
        output_folder=project_root / str(recording.get("output_dir", "video")),
        frame_rate=float(recording.get("frame_rate", 15)),
        resolution=recording.get("resolution"),
    )
