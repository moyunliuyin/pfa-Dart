"""DartSystem 的默认配置、配置加载与带注释 YAML 保存逻辑。"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent
IS_WINDOWS = sys.platform.startswith("win")
IS_LINUX = sys.platform.startswith("linux")


def mv_sdk_python_path() -> Path:
    """返回当前平台对应的海康 Python SDK 目录。"""
    return PROJECT_ROOT / ("MvImport" if IS_WINDOWS else "MvImport_Linux")


def platform_default_serial_port() -> str:
    """返回平台默认串口名；实际部署时仍应按设备情况修改配置。"""
    if IS_WINDOWS:
        return "COM1"
    return "/dev/ttyACM0"


DEFAULT_CONFIG = {
    "runtime": {
        "mode": "hik",
    },
    "ui": {
        "enable_display_window": True,
        "enable_tuning_window": True,
    },
    "test": {
        "source": "video",
        "path": "",
        "auto_start": False,
    },
    "camera": {
        "exposure": 16000.0,
        "gain": 15.9,
    },
    "calibration": {
        "fx": 15989.09,
        "fy": 15985.61,
        "cx": None,
        "cy": None,
        "image_width_px": 1440,
        "image_height_px": 1080,
    },
    "serial": {
        "enabled": False,
        "port": platform_default_serial_port(),
        "baudrate": 115200,
        "data_bits": 8,
        "parity": "none",
        "stop_bits": 1,
        "write_timeout_ms": 200,
        "reconnect_interval_ms": 5000,
        "min_send_interval_ms": 120,
        "send_every_n_frames": 2,
    },
    "detection": {
        "hsv": {
            "lower": [24, 31, 123],
            "upper": [83, 253, 255],
        },
        "brightness": {
            "min_v": 180,
            "max_std": 90,
        },
        "morphology": {
            "blur_kernel": 5,
            "open_kernel": 3,
            "close_kernel": 5,
            "erode_iterations": 0,
            "dilate_iterations": 1,
        },
        "contour": {
            "min_area": 60,
            "max_area": 20000,
            "far_min_area": 60,
            "near_min_area": 300,
            "min_circularity": 0.55,
            "min_aspect_ratio": 0.55,
            "min_fill_ratio": 0.55,
        },
        "circle": {
            "min_radius": 4,
            "max_radius": 120,
        },
        "scoring": {
            "brightness_weight": 1.4,
            "circularity_weight": 1.2,
            "fill_ratio_weight": 1.0,
            "center_weight": 0.35,
        },
        "angle": {
            "focal_length_mm": 50.0,
            "pixel_size_um": 3.45,
            "sensor_width_mm": 4.968,
            "native_width_px": 1440,
            "center_x_px": None,
            "invert_x": False,
        },
        "debug": {
            "console_log_interval_frames": 20,
            "draw_rejected": False,
        },
    },
    "recording": {
        "enabled": False,
        "format": "mp4",
        "output_dir": "video",
        "frame_rate": 15,
        "resolution": None,
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    """递归合并配置，保证用户只覆盖需要调整的字段。"""
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config(config_path: str | Path | None = None) -> dict:
    """读取 YAML 配置并与默认值合并，同时兼容旧版 ``can`` 配置。"""
    config_file = Path(config_path) if config_path else PROJECT_ROOT / "config.yaml"
    if not config_file.exists():
        return copy.deepcopy(DEFAULT_CONFIG)

    with config_file.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}

    if not isinstance(loaded, dict):
        raise ValueError(f"配置文件格式无效: {config_file}")

    merged = _deep_merge(DEFAULT_CONFIG, loaded)

    if "serial" not in loaded and isinstance(loaded.get("can"), dict):
        legacy_can = loaded["can"]
        merged["serial"] = _deep_merge(
            DEFAULT_CONFIG["serial"],
            {
                "enabled": bool(legacy_can.get("enabled", False)),
                "reconnect_interval_ms": int(legacy_can.get("reconnect_interval_ms", 5000)),
                "min_send_interval_ms": int(legacy_can.get("min_send_interval_ms", 120)),
                "send_every_n_frames": int(legacy_can.get("send_every_n_frames", 2)),
            },
        )

    return merged


def save_config(config: dict, config_path: str | Path | None = None) -> Path:
    """将配置写回 YAML，并保留面向调参人员的说明注释。"""
    config_file = Path(config_path) if config_path else PROJECT_ROOT / "config.yaml"
    content = render_config_with_comments(config)
    config_file.write_text(content, encoding="utf-8")
    return config_file


def _yaml_scalar(value) -> str:
    """把 Python 标量转换为本项目 YAML 模板使用的文本表示。"""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, str):
        return f"\"{value}\""
    return str(value)


def _coerce_int(value, default: int) -> int:
    """将普通整数或 ``0x`` 形式字符串转换为整数，失败时返回默认值。"""
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value, 0)
        except ValueError:
            return default
    return default


def render_config_with_comments(config: dict) -> str:
    """按固定章节生成配置文件，便于现场直接阅读和调参。"""
    runtime = config.get("runtime", {})
    ui = config.get("ui", {})
    test = config.get("test", {})
    camera = config.get("camera", {})
    calibration = config.get("calibration", {})
    serial = config.get("serial", {})
    detection = config.get("detection", {})
    hsv = detection.get("hsv", {})
    brightness = detection.get("brightness", {})
    morphology = detection.get("morphology", {})
    contour = detection.get("contour", {})
    circle = detection.get("circle", {})
    scoring = detection.get("scoring", {})
    angle = detection.get("angle", {})
    debug = detection.get("debug", {})
    recording = config.get("recording", {})

    lines = [
        "# 运行模式",
        "# mode: hik 使用海康相机实时检测",
        "# mode: test 使用测试文件，不接相机也能调试",
        "runtime:",
        f"  mode: \"{runtime.get('mode', 'hik')}\"",
        "",
        "# UI 窗口开关",
        "# enable_display_window: 是否显示主画面窗口（原图/掩膜/状态）",
        "# enable_tuning_window: 是否显示实时调参窗口",
        "ui:",
        f"  enable_display_window: {str(bool(ui.get('enable_display_window', True))).lower()}",
        f"  enable_tuning_window: {str(bool(ui.get('enable_tuning_window', True))).lower()}",
        "",
        "# test 模式下的测试源设置",
        "# source: video 或 image",
        "# path: 测试文件路径，支持相对路径和绝对路径",
        "# auto_start: true 时程序启动后自动加载测试文件",
        "test:",
        f"  source: \"{test.get('source', 'video')}\"",
        f"  path: \"{test.get('path', '')}\"",
        f"  auto_start: {str(bool(test.get('auto_start', False))).lower()}",
        "",
        "# 相机参数",
        "camera:",
        f"  exposure: {float(camera.get('exposure', 16000.0))}",
        f"  gain: {float(camera.get('gain', 15.9))}",
        "",
        "# Camera calibration. fx is used for x-axis angle conversion; fy is kept for traceability.",
        "# If cx/cy are null, the current frame center is used.",
        "calibration:",
        f"  fx: {float(calibration.get('fx', 15989.09))}",
        f"  fy: {float(calibration.get('fy', 15985.61))}",
        f"  cx: {_yaml_scalar(calibration.get('cx'))}",
        f"  cy: {_yaml_scalar(calibration.get('cy'))}",
        f"  image_width_px: {_yaml_scalar(calibration.get('image_width_px', 1440))}",
        f"  image_height_px: {_yaml_scalar(calibration.get('image_height_px', 1080))}",
        "",
        "# 串口通信参数",
        "# port: 串口设备名，Linux 常见 /dev/ttyACM0 或 /dev/ttyUSB0，Windows 常见 COM1",
        "# baudrate/data_bits/parity/stop_bits: 与下位机串口参数保持一致",
        "# write_timeout_ms: 单帧写入超时时间（毫秒），0 表示不启用写入超时",
        "# min_send_interval_ms: 发送最小间隔（毫秒），用于抑制串口拥塞",
        "serial:",
        f"  enabled: {str(bool(serial.get('enabled', False))).lower()}",
        f"  port: \"{serial.get('port', platform_default_serial_port())}\"",
        f"  baudrate: {int(serial.get('baudrate', 115200))}",
        f"  data_bits: {int(serial.get('data_bits', 8))}",
        f"  parity: \"{serial.get('parity', 'none')}\"",
        f"  stop_bits: {int(serial.get('stop_bits', 1))}",
        f"  write_timeout_ms: {int(serial.get('write_timeout_ms', 200))}",
        f"  reconnect_interval_ms: {int(serial.get('reconnect_interval_ms', 5000))}",
        f"  min_send_interval_ms: {int(serial.get('min_send_interval_ms', 120))}",
        f"  send_every_n_frames: {int(serial.get('send_every_n_frames', 2))}",
    ]

    lines.extend(
        [
            "",
            "# 绿光检测参数",
            "# hsv: 绿光颜色范围",
            "# brightness.min_v: 亮度下限，越高越偏向明亮目标",
            "# contour.min_area/max_area: 总面积范围",
            "# contour.far_min_area: 判定为远目标的最小面积阈值",
            "# contour.near_min_area: 判定为近目标的最小面积阈值",
            "# contour.min_circularity: 越高越要求接近圆",
            "# contour.min_fill_ratio: 越高越要求轮廓填充得更实",
            "# morphology: 掩膜去噪强度",
            "detection:",
            "  hsv:",
            f"    lower: {list(hsv.get('lower', [24, 31, 123]))}",
            f"    upper: {list(hsv.get('upper', [83, 253, 255]))}",
            "  brightness:",
            f"    min_v: {int(brightness.get('min_v', 180))}",
            f"    max_std: {int(brightness.get('max_std', 90))}",
            "  morphology:",
            f"    blur_kernel: {int(morphology.get('blur_kernel', 5))}",
            f"    open_kernel: {int(morphology.get('open_kernel', 3))}",
            f"    close_kernel: {int(morphology.get('close_kernel', 5))}",
            f"    erode_iterations: {int(morphology.get('erode_iterations', 0))}",
            f"    dilate_iterations: {int(morphology.get('dilate_iterations', 1))}",
            "  contour:",
            f"    min_area: {int(contour.get('min_area', 60))}",
            f"    max_area: {int(contour.get('max_area', 20000))}",
            f"    far_min_area: {int(contour.get('far_min_area', 60))}",
            f"    near_min_area: {int(contour.get('near_min_area', 300))}",
            f"    min_circularity: {float(contour.get('min_circularity', 0.55))}",
            f"    min_aspect_ratio: {float(contour.get('min_aspect_ratio', 0.55))}",
            f"    min_fill_ratio: {float(contour.get('min_fill_ratio', 0.55))}",
            "  circle:",
            f"    min_radius: {int(circle.get('min_radius', 4))}",
            f"    max_radius: {int(circle.get('max_radius', 120))}",
            "  scoring:",
            f"    brightness_weight: {float(scoring.get('brightness_weight', 1.4))}",
            f"    circularity_weight: {float(scoring.get('circularity_weight', 1.2))}",
            f"    fill_ratio_weight: {float(scoring.get('fill_ratio_weight', 1.0))}",
            f"    center_weight: {float(scoring.get('center_weight', 0.35))}",
            "  angle:",
            f"    focal_length_mm: {float(angle.get('focal_length_mm', 50.0))}",
            f"    pixel_size_um: {float(angle.get('pixel_size_um', 3.45))}",
            f"    sensor_width_mm: {float(angle.get('sensor_width_mm', 4.968))}",
            f"    native_width_px: {int(angle.get('native_width_px', 1440))}",
            f"    center_x_px: {_yaml_scalar(angle.get('center_x_px'))}",
            f"    invert_x: {str(bool(angle.get('invert_x', False))).lower()}",
            "  debug:",
            f"    console_log_interval_frames: {int(debug.get('console_log_interval_frames', 20))}",
            f"    draw_rejected: {str(bool(debug.get('draw_rejected', False))).lower()}",
            "",
            "# 海康相机取流录制（OpenCV VideoWriter），默认关闭；与主程序共用同一路 SDK 帧",
            "# resolution: null 为原始分辨率，也可写成 \"1280x720\"；format 推荐 mp4 或 avi",
            "recording:",
            f"  enabled: {str(bool(recording.get('enabled', False))).lower()}",
            f"  format: \"{recording.get('format', 'mp4')}\"",
            f"  output_dir: \"{recording.get('output_dir', 'video')}\"",
            f"  frame_rate: {int(recording.get('frame_rate', 15))}",
            f"  resolution: {_yaml_scalar(recording.get('resolution'))}",
            "",
        ]
    )
    return "\n".join(lines)

