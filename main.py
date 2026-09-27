"""DartSystem 主程序：相机取流、绿光检测、调参界面和串口发送。"""

# 标准库：配置、信号、串口文件描述符、协议打包和生命周期清理。
from __future__ import annotations

import atexit
import copy
import os
from pathlib import Path
import select
import signal
import struct
import sys
import time
from ctypes import POINTER, byref, c_ubyte, cast, memset, sizeof

try:
    import termios
except ImportError:  # pragma: no cover - termios is unavailable on Windows.
    termios = None

# 第三方库：OpenCV 负责图像处理，NumPy 负责帧缓冲，Qt 负责界面和定时器。
import cv2
import numpy as np
from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from app_config import PROJECT_ROOT, load_config, mv_sdk_python_path, save_config
from opencv_green_detection import detect_green_targets, get_debug_images, set_detection_config
from system_recorder import record_hik_stream_frame, stop_system_recording, sync_recording_from_config

SDK_PATH = mv_sdk_python_path()
if str(SDK_PATH) not in sys.path:
    sys.path.insert(0, str(SDK_PATH))

if sys.platform.startswith("win"):
    from MvImport.MvCameraControl_class import *  # noqa: F403
else:
    from MvImport_Linux.MvCameraControl_class import *  # noqa: F403


def get_camera_value(cam, param_type="float_value", node_name=""):
    """读取海康相机节点值；读取失败时返回 ``None``。"""
    if param_type == "float_value":
        st_param = MVCC_FLOATVALUE()  # noqa: F405
        memset(byref(st_param), 0, sizeof(MVCC_FLOATVALUE))  # noqa: F405
        ret = cam.MV_CC_GetFloatValue(node_name, st_param)
        return st_param.fCurValue if ret == 0 else None
    if param_type == "enum_value":
        st_param = MVCC_ENUMVALUE()  # noqa: F405
        memset(byref(st_param), 0, sizeof(MVCC_ENUMVALUE))  # noqa: F405
        ret = cam.MV_CC_GetEnumValue(node_name, st_param)
        return st_param.nCurValue if ret == 0 else None
    return None


def set_camera_value(cam, param_type="float_value", node_name="", node_value=0):
    """写入海康相机的浮点或枚举节点值。"""
    if param_type == "float_value":
        return cam.MV_CC_SetFloatValue(node_name, node_value)
    if param_type == "enum_value":
        return cam.MV_CC_SetEnumValue(node_name, node_value)
    return -1


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """计算发送帧使用的 CRC-16/CCITT 校验值。"""
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


PIXEL_TYPE_NAMES = {
    17301505: "Mono8",
    17301512: "BayerGR8",
    17301513: "BayerRG8",
    17301514: "BayerGB8",
    17301515: "BayerBG8",
    35127316: "RGB8 Packed",
    35127317: "BGR8 Packed",
    34603039: "YUV422 Packed",
}


class SerialCommunication(QObject):
    """基于 Linux termios 的非阻塞串口发送器。"""

    connection_status = pyqtSignal(bool)

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        data_bits: int = 8,
        parity: str = "none",
        stop_bits: int = 1,
        write_timeout_ms: int = 200,
    ):
        super().__init__()
        # 这些字段同时用于打开串口、界面状态显示和故障重连。
        self.driver_name = "serial"
        self.port = port
        self.connection_name = port
        self.baudrate = int(baudrate)
        self.data_bits = int(data_bits)
        self.parity = str(parity).lower()
        self.stop_bits = int(stop_bits)
        self.write_timeout_ms = max(int(write_timeout_ms), 0)
        self.fd: int | None = None
        self.last_error = ""
        self.connect_serial()

    def connect_serial(self) -> bool:
        """打开并配置串口；失败时保留错误信息供界面显示。"""
        if termios is None:
            self.last_error = "当前平台不支持 termios 串口配置。"
            self.connection_status.emit(False)
            print(self.last_error)
            return False

        fd: int | None = None
        try:
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self._configure_port(fd)
            self.fd = fd
            self.last_error = ""
            self.connection_status.emit(True)
            print(f"串口已连接: {self.port} baudrate={self.baudrate}")
            return True
        except OSError as exc:
            if fd is not None:
                os.close(fd)
            self.fd = None
            self.last_error = str(exc)
            self.connection_status.emit(False)
            print(f"串口连接失败 {self.port}: {exc}")
            return False

    def _configure_port(self, fd: int) -> None:
        """将串口设置为原始模式，并应用串口格式和超时策略。"""
        assert termios is not None
        attrs = termios.tcgetattr(fd)
        # 清空输入、输出和本地处理标志，按二进制原始字节流通信。
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = self._data_bits_flag() | self._parity_flag() | self._stop_bits_flag() | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        # 输入和输出使用同一波特率；VMIN/VTIME 为 0 表示非阻塞读写。
        attrs[4] = self._baudrate_flag()
        attrs[5] = self._baudrate_flag()
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

    def _baudrate_flag(self) -> int:
        """把配置中的整数波特率映射为 termios 常量。"""
        assert termios is not None
        baudrate_name = f"B{self.baudrate}"
        if not hasattr(termios, baudrate_name):
            raise OSError(f"不支持的串口波特率: {self.baudrate}")
        return getattr(termios, baudrate_name)

    def _data_bits_flag(self) -> int:
        """把 5/6/7/8 数据位映射为 termios 的字符长度标志。"""
        assert termios is not None
        data_bits_map = {
            5: termios.CS5,
            6: termios.CS6,
            7: termios.CS7,
            8: termios.CS8,
        }
        if self.data_bits not in data_bits_map:
            raise OSError(f"不支持的数据位: {self.data_bits}")
        return data_bits_map[self.data_bits]

    def _parity_flag(self) -> int:
        """把 none/even/odd 校验配置映射为 termios 标志。"""
        assert termios is not None
        if self.parity in {"none", "n", ""}:
            return 0
        if self.parity in {"even", "e"}:
            return termios.PARENB
        if self.parity in {"odd", "o"}:
            return termios.PARENB | termios.PARODD
        raise OSError(f"不支持的校验位: {self.parity}")

    def _stop_bits_flag(self) -> int:
        """把 1/2 停止位映射为 termios 标志。"""
        assert termios is not None
        if self.stop_bits == 1:
            return 0
        if self.stop_bits == 2:
            return termios.CSTOPB
        raise OSError(f"不支持的停止位: {self.stop_bits}")

    def is_connected(self) -> bool:
        return self.fd is not None

    def send_frame(self, payload: bytes) -> bool:
        """在超时时间内处理部分写入，确保整帧发送或明确失败。"""
        if self.fd is None:
            return False

        # os.write 可能只写出部分数据，因此使用 memoryview 记录未发送部分。
        deadline = None if self.write_timeout_ms <= 0 else time.monotonic() + (self.write_timeout_ms / 1000.0)
        pending = memoryview(payload)
        while pending:
            timeout = None if deadline is None else max(deadline - time.monotonic(), 0.0)
            try:
                _, writable, _ = select.select([], [self.fd], [], timeout)
                if not writable:
                    self.last_error = "串口写入超时"
                    print(f"串口发送失败: {self.last_error}")
                    return False
                written = os.write(self.fd, pending)
                pending = pending[written:]
            except BlockingIOError:
                if deadline is not None and time.monotonic() >= deadline:
                    self.last_error = "串口写入超时"
                    print(f"串口发送失败: {self.last_error}")
                    return False
            except OSError as exc:
                self.last_error = str(exc)
                print(f"串口发送失败: {exc}")
                self.close()
                self.connection_status.emit(False)
                return False
        return True

    def close(self) -> None:
        """关闭文件描述符；重复调用不会报错。"""
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
            print(f"串口已关闭: {self.port}")


class TuningSlider(QWidget):
    """把浮点调参范围映射到 Qt 整数滑块，并显示还原后的数值。"""

    valueChanged = pyqtSignal(object)

    def __init__(
        self,
        minimum: float,
        maximum: float,
        step: float = 1.0,
        decimals: int = 0,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        # Qt 的 QSlider 只能使用整数位置，这里把实际浮点范围离散化。
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.step = max(float(step), 1e-9)
        self.decimals = max(int(decimals), 0)
        self.position_count = max(int(round((self.maximum - self.minimum) / self.step)), 0)

        # 滑块负责输入，右侧标签负责显示还原后的实际值。
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, self.position_count)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(max(1, self.position_count // 10))
        self.slider.setTracking(True)

        self.value_label = QLabel()
        self.value_label.setMinimumWidth(72 if self.decimals else 64)
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.value_label)

        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self._update_value_label()

    def value(self):
        """把当前整数滑块位置还原为配置参数的实际数值。"""
        raw_value = self.minimum + self.slider.value() * self.step
        raw_value = min(max(raw_value, self.minimum), self.maximum)
        if self.decimals == 0:
            return int(round(raw_value))
        return round(raw_value, self.decimals)

    def setValue(self, value) -> None:
        """将实际数值转换为最近的滑块位置，并限制在合法范围内。"""
        position = int(((float(value) - self.minimum) / self.step) + 0.5)
        position = min(max(position, 0), self.position_count)
        if self.slider.value() == position:
            self._update_value_label()
            return
        self.slider.setValue(position)

    def _format_value(self) -> str:
        value = self.value()
        if self.decimals == 0:
            return str(value)
        return f"{value:.{self.decimals}f}"

    def _update_value_label(self) -> None:
        self.value_label.setText(self._format_value())

    def _on_slider_value_changed(self, _position: int) -> None:
        """刷新显示，并在未屏蔽信号时通知主窗口更新检测参数。"""
        self._update_value_label()
        if not self.signalsBlocked():
            self.valueChanged.emit(self.value())


class TuningWindow(QMainWindow):
    """集中承载 HSV、亮度、轮廓和形态学等实时调参控件。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setWindowTitle("DartSystem 实时调参")
        self.resize(620, 980)

        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout()

        self.live_tuning_checkbox = QCheckBox("启用实时调参")
        root_layout.addWidget(self.live_tuning_checkbox)

        self.save_tuning_button = QPushButton("保存当前实时参数到配置")
        root_layout.addWidget(self.save_tuning_button)

        self.tuning_group = QGroupBox("实时调参")
        tuning_form = QFormLayout()
        self.tuning_group.setLayout(tuning_form)
        root_layout.addWidget(self.tuning_group)

        self.tuning_controls = {
            "h_min": self.add_spin(tuning_form, "H Min", 0, 179),
            "h_max": self.add_spin(tuning_form, "H Max", 0, 179),
            "s_min": self.add_spin(tuning_form, "S Min", 0, 255),
            "s_max": self.add_spin(tuning_form, "S Max", 0, 255),
            "v_min": self.add_spin(tuning_form, "V Min", 0, 255),
            "v_max": self.add_spin(tuning_form, "V Max", 0, 255),
            "brightness_min_v": self.add_spin(tuning_form, "亮度下限", 0, 255),
            "brightness_max_std": self.add_spin(tuning_form, "亮度方差上限", 0, 255),
            "min_area": self.add_spin(tuning_form, "最小面积", 0, 100000),
            "max_area": self.add_spin(tuning_form, "最大面积", 1, 100000),
            "far_min_area": self.add_spin(tuning_form, "远目标最小面积", 0, 100000),
            "near_min_area": self.add_spin(tuning_form, "近目标最小面积", 0, 100000),
            "min_radius": self.add_spin(tuning_form, "最小半径", 0, 1000),
            "max_radius": self.add_spin(tuning_form, "最大半径", 1, 1000),
            "blur_kernel": self.add_spin(tuning_form, "模糊核", 1, 31, step=2),
            "open_kernel": self.add_spin(tuning_form, "开运算核", 1, 31, step=2),
            "close_kernel": self.add_spin(tuning_form, "闭运算核", 1, 31, step=2),
            "min_circularity": self.add_double_spin(tuning_form, "最小圆度", 0.0, 1.0, 0.01),
            "min_fill_ratio": self.add_double_spin(tuning_form, "最小填充率", 0.0, 1.0, 0.01),
            "min_aspect_ratio": self.add_double_spin(tuning_form, "最小长宽比", 0.0, 1.0, 0.01),
        }

        main_widget.setLayout(root_layout)

    @staticmethod
    def add_spin(layout, label, minimum, maximum, step=1):
        control = TuningSlider(minimum, maximum, step=step, decimals=0)
        layout.addRow(label, control)
        return control

    @staticmethod
    def add_double_spin(layout, label, minimum, maximum, step):
        control = TuningSlider(minimum, maximum, step=step, decimals=2)
        layout.addRow(label, control)
        return control


class MainWindow(QMainWindow):
    """主窗口及运行时状态机，统一承接相机和视频的检测链路。"""

    def __init__(self, config_path):
        super().__init__()
        # 先读取配置，再由 apply_runtime_config 将配置分发给各子系统。
        self.config_path = config_path
        self.config = load_config(self.config_path)
        self.active_detection_config = {}

        # 海康 SDK 取帧状态。p_data/st_frame_info 在相机初始化成功后才有效。
        self.camera = None
        self.data_size = 0
        self.p_data = None
        self.st_frame_info = None

        # 视频和串口是相机检测链路之外的两个可切换输入/输出端。
        self.serial_comm: SerialCommunication | None = None
        self.video_cap = None
        self.video_fps = 0.0
        self.video_total_frames = 0
        self.current_frame_idx = 0
        self.is_video_mode = False
        self.is_video_playing = False
        self.debug_mode = False
        self.prev_frame_time = 0.0
        self.processed_frame_count = 0
        self.last_serial_send_ts = 0.0
        self.last_pixel_type: int | None = None

        # 调参状态与关闭保护：关闭保护确保 Qt closeEvent 和 aboutToQuit 不会重复释放资源。
        self.live_tuning_enabled = False
        self.tuning_controls: dict[str, object] = {}
        self.tuning_window: TuningWindow | None = None
        self.ui_config: dict[str, object] = {}
        self._shutdown_done = False

        # 相机、视频、串口重连分别使用独立定时器，切换输入源时只启停对应定时器。
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.process_video_frame)
        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self.update_camera_frame)
        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.timeout.connect(self.refresh_connection)

        # 初始化顺序：界面控件 → 调参控件 → 配置 → 串口 → 相机帧定时器。
        self.init_ui()
        self.init_tuning_window()
        self.apply_runtime_config(self.config, reconnect_serial=False)
        self.setup_serial()
        self.frame_timer.start(30)

    def init_ui(self) -> None:
        """创建主窗口：左侧显示原图叠加层，右侧显示检测掩膜和运行状态。"""
        self.setWindowTitle("DartSystem Green Light Detector")
        self.resize(1600, 980)

        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout()

        # 两个 QLabel 只负责显示，检测算法本身不依赖 Qt 控件。
        image_layout = QHBoxLayout()
        self.camera_label = QLabel("等待图像")
        self.camera_label.setMinimumSize(720, 540)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setStyleSheet("border: 1px solid #666;")
        image_layout.addWidget(self.camera_label)

        self.mask_label = QLabel("等待掩膜")
        self.mask_label.setMinimumSize(720, 540)
        self.mask_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mask_label.setStyleSheet("border: 1px solid #666;")
        image_layout.addWidget(self.mask_label)
        root_layout.addLayout(image_layout)

        # 操作按钮均连接到 MainWindow 方法，避免把业务逻辑写进 Qt 回调匿名函数。
        button_layout = QHBoxLayout()
        self.open_video_button = QPushButton("打开视频")
        self.open_video_button.clicked.connect(self.open_video_file)
        button_layout.addWidget(self.open_video_button)

        self.video_play_button = QPushButton("播放视频")
        self.video_play_button.setEnabled(False)
        self.video_play_button.clicked.connect(self.toggle_video_playback)
        button_layout.addWidget(self.video_play_button)

        self.camera_mode_button = QPushButton("回到相机")
        self.camera_mode_button.clicked.connect(self.switch_to_camera_mode)
        button_layout.addWidget(self.camera_mode_button)

        self.refresh_serial_button = QPushButton("重连串口")
        self.refresh_serial_button.clicked.connect(self.refresh_connection)
        button_layout.addWidget(self.refresh_serial_button)

        self.reload_config_button = QPushButton("重载配置")
        self.reload_config_button.clicked.connect(self.reload_runtime_config)
        button_layout.addWidget(self.reload_config_button)
        root_layout.addLayout(button_layout)

        # 视频滑块只在视频模式可用，拖动时会暂停播放并跳转到指定帧。
        self.video_slider = QSlider(Qt.Orientation.Horizontal)
        self.video_slider.setEnabled(False)
        self.video_slider.valueChanged.connect(self.on_video_slider_changed)
        root_layout.addWidget(self.video_slider)

        # 状态标签集中放在窗口底部，便于现场查看连接、检测和参数状态。
        self.connection_status_label = QLabel("串口: 未初始化")
        self.camera_status_label = QLabel("相机: 未初始化")
        self.detect_status_label = QLabel("检测: 未开始")
        self.config_status_label = QLabel("配置: 未加载")
        self.runtime_hint_label = QLabel("提示: 修改 config.yaml 后点击“重载配置”即可生效。")
        self.param_summary_label = QLabel("")
        self.param_summary_label.setWordWrap(True)

        for label in [
            self.connection_status_label,
            self.camera_status_label,
            self.detect_status_label,
            self.config_status_label,
            self.runtime_hint_label,
            self.param_summary_label,
        ]:
            root_layout.addWidget(label)

        main_widget.setLayout(root_layout)

    def init_tuning_window(self) -> None:
        """创建独立调参窗口，并把控件事件接到运行时配置更新逻辑。"""
        self.tuning_window = TuningWindow(self)
        self.tuning_controls = self.tuning_window.tuning_controls
        self.live_tuning_checkbox = self.tuning_window.live_tuning_checkbox
        self.save_tuning_button = self.tuning_window.save_tuning_button

        # 勾选实时调参才允许控件修改检测器；保存按钮只写回用户主动确认的参数。
        self.live_tuning_checkbox.toggled.connect(self.on_live_tuning_toggled)
        self.save_tuning_button.clicked.connect(self.save_current_tuning_to_config)
        self.save_tuning_button.setEnabled(False)

        for control in self.tuning_controls.values():
            control.valueChanged.connect(self.on_tuning_value_changed)

        self.set_tuning_controls_enabled(False)

    def set_tuning_controls_enabled(self, enabled: bool) -> None:
        """同步启用/禁用调参分组及其中的所有滑块。"""
        if self.tuning_window is None:
            return
        self.tuning_window.tuning_group.setEnabled(enabled)
        for control in self.tuning_controls.values():
            control.setEnabled(enabled)

    def apply_runtime_config(self, config: dict, reconnect_serial: bool = True) -> None:
        """把配置拆分到各子系统，并同步检测器、串口、窗口和录制状态。"""
        self.config = config
        # 下面这些引用对应 config.yaml 的一级章节；检测配置会额外注入标定参数。
        self.camera_config = config.get("camera", {})
        self.calibration_config = config.get("calibration", {})
        self.serial_config = config.get("serial", {})
        self.detection_config = self.build_detection_runtime_config(config.get("detection", {}))
        self.recording_config = config.get("recording", {})
        self.runtime_config = config.get("runtime", {})
        self.test_config = config.get("test", {})
        self.ui_config = config.get("ui", {})
        self.debug_config = self.detection_config.get("debug", {})

        # 配置重载时先更新窗口和检测器，再决定是否重启串口/录制会话。
        self.apply_ui_window_visibility()

        self.sync_tuning_controls_from_config()
        if self.live_tuning_enabled:
            self.apply_tuning_controls_to_detection()
        else:
            # 非实时调参模式下，检测器始终使用配置文件的完整副本。
            self.active_detection_config = copy.deepcopy(self.detection_config)
            set_detection_config(self.active_detection_config)

        self.update_param_summary()
        self.config_status_label.setText(f"配置: 已加载 {self.config_path.name}")

        # 串口启用时持续尝试重连；禁用或平台不支持时停止重连定时器并关闭句柄。
        reconnect_interval = int(self.serial_config.get("reconnect_interval_ms", 5000))
        if self.serial_config.get("enabled", False) and self.serial_send_supported():
            self.reconnect_timer.start(reconnect_interval)
        else:
            self.reconnect_timer.stop()
            self.connection_status_label.setText(self.serial_disabled_status_text())
            self._close_serial()

        # 相机已运行时只更新曝光/增益，不重新创建设备句柄。
        if self.camera is not None:
            self.apply_camera_parameters()
        if reconnect_serial and self.serial_config.get("enabled", False) and self.serial_send_supported():
            self.setup_serial()

        sync_recording_from_config(self.recording_config, PROJECT_ROOT)

    def build_detection_runtime_config(self, detection_config: dict) -> dict:
        """将顶层标定参数注入检测配置，避免算法读取到过期副本。"""
        runtime_detection_config = copy.deepcopy(detection_config)
        runtime_detection_config.pop("calibration", None)
        runtime_detection_config["calibration"] = copy.deepcopy(self.calibration_config)
        return runtime_detection_config

    @staticmethod
    def serial_send_supported() -> bool:
        """判断当前平台是否具备 termios 串口配置能力。"""
        return termios is not None

    def serial_disabled_status_text(self) -> str:
        """生成串口状态文案，区分“配置禁用”和“平台不支持”。"""
        if self.serial_config.get("enabled", False) and not self.serial_send_supported():
            return "串口: 当前平台不支持"
        return "串口: 已禁用"

    def apply_ui_window_visibility(self) -> None:
        """应用窗口开关；两个窗口都被关闭时保留主窗口避免程序无界面运行。"""
        display_enabled = bool(self.ui_config.get("enable_display_window", True))
        tuning_enabled = bool(self.ui_config.get("enable_tuning_window", True))

        if not display_enabled and not tuning_enabled:
            display_enabled = True
            print("ui 配置中两个窗口都关闭，已自动保留主画面窗口。")

        if display_enabled:
            self.show()
        else:
            self.hide()

        if self.tuning_window is not None:
            if tuning_enabled:
                self.tuning_window.show()
            else:
                self.tuning_window.hide()
                if self.live_tuning_enabled:
                    self.live_tuning_checkbox.blockSignals(True)
                    self.live_tuning_checkbox.setChecked(False)
                    self.live_tuning_checkbox.blockSignals(False)
                    self.on_live_tuning_toggled(False)

    def startup_by_runtime_mode(self) -> None:
        """根据 ``runtime.mode`` 选择海康相机模式或测试文件模式。"""
        mode = str(self.runtime_config.get("mode", "hik")).lower()
        if mode == "test":
            self.enable_debug_mode("当前为 test 模式，不初始化相机。")
            if self.test_config.get("auto_start", False):
                self.load_test_source_from_config()
            return
        self.setup_camera()

    def sync_tuning_controls_from_config(self) -> None:
        """把配置值写入调参控件，且屏蔽信号避免初始化过程触发重复更新。"""
        hsv = self.detection_config.get("hsv", {})
        brightness = self.detection_config.get("brightness", {})
        contour = self.detection_config.get("contour", {})
        circle = self.detection_config.get("circle", {})
        morphology = self.detection_config.get("morphology", {})

        values = {
            "h_min": hsv.get("lower", [24, 31, 123])[0],
            "h_max": hsv.get("upper", [83, 253, 255])[0],
            "s_min": hsv.get("lower", [24, 31, 123])[1],
            "s_max": hsv.get("upper", [83, 253, 255])[1],
            "v_min": hsv.get("lower", [24, 31, 123])[2],
            "v_max": hsv.get("upper", [83, 253, 255])[2],
            "brightness_min_v": brightness.get("min_v", 180),
            "brightness_max_std": brightness.get("max_std", 90),
            "min_area": contour.get("min_area", 60),
            "max_area": contour.get("max_area", 20000),
            "far_min_area": contour.get("far_min_area", 60),
            "near_min_area": contour.get("near_min_area", 300),
            "min_radius": circle.get("min_radius", 4),
            "max_radius": circle.get("max_radius", 120),
            "blur_kernel": morphology.get("blur_kernel", 5),
            "open_kernel": morphology.get("open_kernel", 3),
            "close_kernel": morphology.get("close_kernel", 5),
            "min_circularity": contour.get("min_circularity", 0.55),
            "min_fill_ratio": contour.get("min_fill_ratio", 0.55),
            "min_aspect_ratio": contour.get("min_aspect_ratio", 0.55),
        }
        for name, value in values.items():
            control = self.tuning_controls[name]
            control.blockSignals(True)
            control.setValue(value)
            control.blockSignals(False)

    def on_live_tuning_toggled(self, checked: bool) -> None:
        """切换实时调参：开启时控件立即生效，关闭时恢复配置文件参数。"""
        self.live_tuning_enabled = checked
        self.set_tuning_controls_enabled(checked)
        self.save_tuning_button.setEnabled(checked)
        if checked:
            self.apply_tuning_controls_to_detection()
            self.runtime_hint_label.setText("提示: 实时调参已启用，界面控件修改会立即生效，但不会自动写回 config.yaml。")
        else:
            self.active_detection_config = copy.deepcopy(self.detection_config)
            set_detection_config(self.active_detection_config)
            self.update_param_summary()
            self.runtime_hint_label.setText("提示: 实时调参已关闭，检测恢复使用 config.yaml 当前值。")

    def on_tuning_value_changed(self, _value) -> None:
        """滑块变化回调；只有实时调参开启时才重建检测配置。"""
        if self.live_tuning_enabled:
            self.apply_tuning_controls_to_detection()

    def apply_tuning_controls_to_detection(self) -> None:
        """读取调参控件并生成检测器当前使用的运行时配置。"""
        h_min = self.tuning_controls["h_min"].value()
        h_max = self.tuning_controls["h_max"].value()
        s_min = self.tuning_controls["s_min"].value()
        s_max = self.tuning_controls["s_max"].value()
        v_min = self.tuning_controls["v_min"].value()
        v_max = self.tuning_controls["v_max"].value()
        min_area = self.tuning_controls["min_area"].value()
        max_area = self.tuning_controls["max_area"].value()
        far_min_area = self.tuning_controls["far_min_area"].value()
        near_min_area = self.tuning_controls["near_min_area"].value()
        min_radius = self.tuning_controls["min_radius"].value()
        max_radius = self.tuning_controls["max_radius"].value()

        blur_kernel = self.make_odd(self.tuning_controls["blur_kernel"].value())
        open_kernel = self.make_odd(self.tuning_controls["open_kernel"].value())
        close_kernel = self.make_odd(self.tuning_controls["close_kernel"].value())

        # 从配置副本开始覆盖控件对应字段，保留评分、角度和 debug 等未暴露参数。
        self.active_detection_config = copy.deepcopy(self.detection_config)
        self.active_detection_config["hsv"] = {
            # min/max 取值顺序在这里归一化，防止用户拖动滑块后上下限颠倒。
            "lower": [min(h_min, h_max), s_min, v_min],
            "upper": [max(h_min, h_max), s_max, v_max],
        }
        self.active_detection_config["brightness"] = {
            **self.active_detection_config.get("brightness", {}),
            "min_v": self.tuning_controls["brightness_min_v"].value(),
            "max_std": self.tuning_controls["brightness_max_std"].value(),
        }
        # 形态学核必须为奇数，make_odd 负责修正用户输入。
        self.active_detection_config["morphology"] = {
            **self.active_detection_config.get("morphology", {}),
            "blur_kernel": blur_kernel,
            "open_kernel": open_kernel,
            "close_kernel": close_kernel,
        }
        # 面积阈值同样归一化：近目标阈值不低于远目标阈值。
        self.active_detection_config["contour"] = {
            **self.active_detection_config.get("contour", {}),
            "min_area": min(min_area, max_area),
            "max_area": max(min_area, max_area),
            "far_min_area": min(far_min_area, near_min_area),
            "near_min_area": max(far_min_area, near_min_area),
            "min_circularity": self.tuning_controls["min_circularity"].value(),
            "min_fill_ratio": self.tuning_controls["min_fill_ratio"].value(),
            "min_aspect_ratio": self.tuning_controls["min_aspect_ratio"].value(),
        }
        self.active_detection_config["circle"] = {
            **self.active_detection_config.get("circle", {}),
            "min_radius": min(min_radius, max_radius),
            "max_radius": max(min_radius, max_radius),
        }
        set_detection_config(self.active_detection_config)
        self.update_param_summary()

    def save_current_tuning_to_config(self) -> None:
        """将当前实时调参写回 YAML；关闭实时调参时不允许误保存。"""
        if not self.live_tuning_enabled:
            self.runtime_hint_label.setText("提示: 只有启用实时调参时才能保存当前参数。")
            return

        self.detection_config = copy.deepcopy(self.active_detection_config)
        self.config["detection"] = self.strip_runtime_detection_config(self.active_detection_config)
        save_config(self.config, self.config_path)
        self.runtime_hint_label.setText("提示: 当前实时参数已写回 config.yaml。")
        self.config_status_label.setText(f"配置: 已保存 {self.config_path.name}")
        print("当前实时参数已保存到 config.yaml")

    @staticmethod
    def make_odd(value: int) -> int:
        """将形态学核尺寸限制为至少 1 的奇数。"""
        value = max(int(value), 1)
        return value if value % 2 == 1 else value + 1

    @staticmethod
    def strip_runtime_detection_config(detection_config: dict) -> dict:
        """移除运行时注入的 calibration，避免把顶层标定重复写入 detection。"""
        saved_detection_config = copy.deepcopy(detection_config)
        saved_detection_config.pop("calibration", None)
        return saved_detection_config

    def update_param_summary(self) -> None:
        """把当前生效的检测阈值压缩成一行，显示在主窗口状态区。"""
        detection = self.active_detection_config if self.active_detection_config else self.detection_config
        hsv = detection.get("hsv", {})
        contour = detection.get("contour", {})
        brightness = detection.get("brightness", {})
        circle = detection.get("circle", {})
        morphology = detection.get("morphology", {})
        angle = detection.get("angle", {})
        calibration = detection.get("calibration", {})
        mode = "Live" if self.live_tuning_enabled else "Config"
        self.param_summary_label.setText(
            "检测参数  "
            f"Mode={mode}  "
            f"HSV={hsv.get('lower')}->{hsv.get('upper')}  "
            f"亮度下限={brightness.get('min_v')}  "
            f"面积={contour.get('min_area')}~{contour.get('max_area')}  "
            f"远阈值>={contour.get('far_min_area')}  "
            f"近阈值>={contour.get('near_min_area')}  "
            f"圆度>={contour.get('min_circularity')}  "
            f"填充率>={contour.get('min_fill_ratio')}  "
            f"半径={circle.get('min_radius')}~{circle.get('max_radius')}  "
            f"fx={calibration.get('fx')}  "
            f"Angle={angle.get('focal_length_mm', 50.0)}mm/{angle.get('pixel_size_um', 3.45)}um  "
            f"Blur={morphology.get('blur_kernel')} Open={morphology.get('open_kernel')} Close={morphology.get('close_kernel')}"
        )

    def reload_runtime_config(self) -> None:
        """重新读取配置并刷新运行时组件；异常只显示错误，不让界面直接退出。"""
        try:
            config = load_config(self.config_path)
            self.apply_runtime_config(config, reconnect_serial=True)
            print("配置已重新加载")
        except Exception as exc:
            print(f"重载配置失败: {exc}")

    def setup_camera(self) -> None:
        """枚举并启动第一台海康相机，同时准备 SDK 帧缓冲区。"""
        # 这里只选择第一台 GigE/USB 海康设备；现场多相机选择逻辑仍由设备顺序决定。
        device_list = MV_CC_DEVICE_INFO_LIST()  # noqa: F405
        tlayer_type = MV_GIGE_DEVICE | MV_USB_DEVICE  # noqa: F405
        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)  # noqa: F405
        if ret != 0 or device_list.nDeviceNum == 0:
            self.camera = None
            self.enable_debug_mode("未找到海康相机，已进入视频调试模式。")
            return

        # 句柄创建和打开设备分成两步，任一步失败都进入无相机调试模式。
        self.camera = MvCamera()  # noqa: F405
        device = cast(device_list.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents  # noqa: F405
        if self.camera.MV_CC_CreateHandle(device) != 0:
            self.camera = None
            self.enable_debug_mode("创建相机句柄失败。")
            return
        if self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0) != 0:  # noqa: F405
            self.camera = None
            self.enable_debug_mode("打开相机失败。")
            return

        # 曝光和增益必须在开始取流前设置，避免首帧使用未配置的相机参数。
        self.apply_camera_parameters()
        if self.camera.MV_CC_StartGrabbing() != 0:
            self.camera = None
            self.enable_debug_mode("启动相机取流失败。")
            return

        # PayloadSize 决定 SDK 每次写入 p_data 的最大字节数。
        st_param = MVCC_INTVALUE_EX()  # noqa: F405
        memset(byref(st_param), 0, sizeof(MVCC_INTVALUE_EX))  # noqa: F405
        if self.camera.MV_CC_GetIntValueEx("PayloadSize", st_param) != 0:
            self.camera = None
            self.enable_debug_mode("读取 PayloadSize 失败。")
            return

        self.data_size = st_param.nCurValue
        self.p_data = (c_ubyte * self.data_size)()
        self.st_frame_info = MV_FRAME_OUT_INFO_EX()  # noqa: F405
        memset(byref(self.st_frame_info), 0, sizeof(self.st_frame_info))
        self.last_pixel_type = None
        self.camera_status_label.setText("相机: 已连接")
        print("相机初始化完成")

    def apply_camera_parameters(self) -> None:
        """将配置中的曝光和增益写入相机，并回读实际生效值显示给用户。"""
        if self.camera is None:
            return
        exposure = float(self.camera_config.get("exposure", 16000.0))
        gain = float(self.camera_config.get("gain", 15.9))
        set_camera_value(self.camera, "float_value", "ExposureTime", exposure)
        set_camera_value(self.camera, "float_value", "Gain", gain)
        current_exposure = get_camera_value(self.camera, "float_value", "ExposureTime")
        current_gain = get_camera_value(self.camera, "float_value", "Gain")
        self.camera_status_label.setText(
            f"相机: 已连接  曝光={current_exposure or exposure:.1f}  增益={current_gain or gain:.1f}"
        )

    def update_pixel_type_status(self, pixel_type: int) -> None:
        """仅在像素格式变化时刷新状态，避免每帧重复打印日志。"""
        if self.last_pixel_type == pixel_type:
            return
        self.last_pixel_type = pixel_type
        pixel_type_name = PIXEL_TYPE_NAMES.get(pixel_type, f"Unknown({pixel_type})")
        print(f"Camera PixelType: {pixel_type_name} ({pixel_type})")
        self.camera_status_label.setText(f"相机: PixelType={pixel_type_name} ({pixel_type})")

    def enable_debug_mode(self, reason: str) -> None:
        """标记相机不可用；视频/图片模式仍可继续复用检测链路。"""
        self.debug_mode = True
        self.camera_status_label.setText(f"相机: 调试模式  {reason}")
        print(reason)

    def load_test_source_from_config(self) -> None:
        """解析 test.source/test.path，并根据类型加载视频或静态图片。"""
        source = str(self.test_config.get("source", "video")).lower()
        path_value = str(self.test_config.get("path", "")).strip()
        if not path_value:
            self.runtime_hint_label.setText("提示: test 模式已启用，但 test.path 为空。")
            return

        target = Path(path_value)
        if not target.is_absolute():
            target = (PROJECT_ROOT / target).resolve()
        if not target.exists():
            self.runtime_hint_label.setText(f"提示: 测试文件不存在: {target}")
            print(f"测试文件不存在: {target}")
            return

        if source == "video":
            self.load_video(str(target))
        elif source == "image":
            self.load_image_test(str(target))
        else:
            self.runtime_hint_label.setText(f"提示: 不支持的 test.source: {source}")

    def load_image_test(self, image_path: str) -> None:
        """加载单张 BGR 图片，转换为 RGB 后执行一次检测。"""
        frame_bgr = cv2.imread(image_path)
        if frame_bgr is None:
            print(f"无法读取测试图片: {image_path}")
            self.runtime_hint_label.setText(f"提示: 无法读取测试图片: {image_path}")
            return
        self.is_video_mode = False
        self.video_play_button.setEnabled(False)
        self.video_slider.setEnabled(False)
        self.frame_timer.stop()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        self.process_frame(frame_rgb, source_name="image", fps=None)
        self.runtime_hint_label.setText(f"提示: 已加载测试图片 {Path(image_path).name}")

    def setup_serial(self) -> None:
        """根据当前串口配置创建连接；失败时保留状态并等待定时器重试。"""
        self._close_serial()
        if not self.serial_config.get("enabled", False):
            self.connection_status_label.setText("串口: 已禁用")
            return
        if not self.serial_send_supported():
            self.connection_status_label.setText("串口: 当前平台不支持")
            return

        port = str(self.serial_config.get("port", "/dev/ttyACM0")).strip()
        if not port:
            self.connection_status_label.setText("串口: port 为空")
            return

        # SerialCommunication 的构造函数会立即尝试打开设备。
        serial_comm = SerialCommunication(
            port=port,
            baudrate=int(self.serial_config.get("baudrate", 115200)),
            data_bits=int(self.serial_config.get("data_bits", 8)),
            parity=str(self.serial_config.get("parity", "none")),
            stop_bits=int(self.serial_config.get("stop_bits", 1)),
            write_timeout_ms=int(self.serial_config.get("write_timeout_ms", 200)),
        )

        if serial_comm.is_connected():
            self.serial_comm = serial_comm
            self.connection_status_label.setText(
                f"串口: 已连接 {port} baudrate={serial_comm.baudrate}"
            )
            return

        error_detail = f" ({serial_comm.last_error})" if serial_comm.last_error else ""
        self.connection_status_label.setText(f"串口: 连接失败{error_detail}")

    def _close_serial(self) -> None:
        """关闭现有串口对象并清空引用，供重连和退出流程复用。"""
        if self.serial_comm is not None:
            self.serial_comm.close()
            self.serial_comm = None

    def refresh_connection(self) -> None:
        """串口重连定时器和“重连串口”按钮共用的入口。"""
        if not self.serial_config.get("enabled", False):
            self.connection_status_label.setText("串口: 已禁用")
            return
        if not self.serial_send_supported():
            self.connection_status_label.setText("串口: 当前平台不支持")
            return
        if self.serial_comm is not None and self.serial_comm.is_connected():
            name = getattr(self.serial_comm, "connection_name", "unknown")
            self.connection_status_label.setText(f"串口: 已连接 {name}")
            return
        self.setup_serial()

    def _convert_raw_frame_to_rgb(
        self,
        frame_data: np.ndarray,
        width: int,
        height: int,
        pixel_type: int,
    ) -> np.ndarray | None:
        """按 SDK 的 PixelType 将原始帧统一转换成 RGB 数组。"""
        mono_size = width * height
        if width <= 0 or height <= 0 or frame_data.size < mono_size:
            return None

        if pixel_type == 17301505:  # Mono8
            # 单通道图像需要扩展为三通道，才能和 RGB/Bayer 输入走同一检测流程。
            gray = frame_data[:mono_size].reshape((height, width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        # Bayer 原始数据仍是单通道，颜色排列由 PixelType 决定。
        bayer_to_rgb = {
            17301513: cv2.COLOR_BAYER_BG2RGB,  # BayerRG8
            17301514: cv2.COLOR_BAYER_GB2RGB,  # BayerGB8
            17301512: cv2.COLOR_BAYER_GR2RGB,  # BayerGR8
            17301515: cv2.COLOR_BAYER_RG2RGB,  # BayerBG8
        }
        if pixel_type in bayer_to_rgb:
            raw = frame_data[:mono_size].reshape((height, width))
            return cv2.cvtColor(raw, bayer_to_rgb[pixel_type])

        if pixel_type == 35127316:  # RGB8 Packed
            rgb_size = mono_size * 3
            if frame_data.size < rgb_size:
                return None
            return frame_data[:rgb_size].reshape((height, width, 3))

        if pixel_type == 35127317:  # BGR8 Packed
            bgr_size = mono_size * 3
            if frame_data.size < bgr_size:
                return None
            bgr = frame_data[:bgr_size].reshape((height, width, 3))
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        if pixel_type == 34603039:  # YUV422
            yuv_size = mono_size * 2
            if frame_data.size < yuv_size:
                return None
            yuv = frame_data[:yuv_size].reshape((height, width, 2))
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_Y422)

        # 兜底：未知单通道格式按 BayerBG 尝试，再回退灰度。
        raw = frame_data[:mono_size].reshape((height, width))
        try:
            return cv2.cvtColor(raw, cv2.COLOR_BAYER_BG2RGB)
        except cv2.error:
            return cv2.cvtColor(raw, cv2.COLOR_GRAY2RGB)

    def read_frame(self) -> tuple[bool, np.ndarray | None]:
        """从 SDK 取一帧并转换为检测链路使用的 RGB 图像。"""
        if self.camera is None or self.p_data is None or self.st_frame_info is None:
            return False, None

        # SDK 将图像写入预分配缓冲区；超时 1000ms 时本次不更新界面。
        ret = self.camera.MV_CC_GetOneFrameTimeout(self.p_data, self.data_size, self.st_frame_info, 1000)
        if ret != 0:
            return False, None

        # 某些 SDK 版本的 nFrameLen 可能为空，回退到 PayloadSize 避免截断。
        frame_len = int(getattr(self.st_frame_info, "nFrameLen", 0) or self.data_size)
        frame_data = np.frombuffer(self.p_data, dtype=np.uint8, count=frame_len)
        height = int(self.st_frame_info.nHeight)
        width = int(self.st_frame_info.nWidth)
        pixel_type = self.st_frame_info.enPixelType
        self.update_pixel_type_status(pixel_type)
        image_rgb = self._convert_raw_frame_to_rgb(frame_data, width, height, pixel_type)
        if image_rgb is None:
            return False, None
        return True, image_rgb

    def update_camera_frame(self) -> None:
        """定时器回调：取相机帧、录制原始结果并交给统一处理入口。"""
        if self.is_video_mode:
            return
        if self.camera is None:
            self.show_debug_placeholder()
            return

        # FPS 只用于界面显示，不参与检测阈值或串口限频。
        current_time = time.time()
        fps = 0.0 if self.prev_frame_time == 0 else 1.0 / max(current_time - self.prev_frame_time, 1e-6)
        self.prev_frame_time = current_time
        success, frame = self.read_frame()
        if not success or frame is None:
            return
        record_hik_stream_frame(frame)
        self.process_frame(frame, source_name="camera", fps=fps)

    def show_debug_placeholder(self) -> None:
        """相机未连接时显示占位图，保持主窗口布局和检测状态可见。"""
        placeholder = np.zeros((540, 720, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Camera not connected", (170, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(placeholder, "Use video mode for field debugging", (120, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.set_label_image(self.camera_label, placeholder)
        self.set_label_image(self.mask_label, np.zeros_like(placeholder))

    def process_frame(self, frame_rgb: np.ndarray, source_name: str, fps: float | None = None) -> None:
        """处理单帧：检测、限频发包、绘制调试信息并刷新界面。"""
        self.processed_frame_count += 1
        # 相机帧和视频帧都必须从这里进入，确保检测、显示、日志和串口行为一致。
        targets = detect_green_targets(frame_rgb)
        self.send_detection_result(targets)

        # 左侧显示原图叠加检测信息，右侧显示检测器最近一次生成的掩膜图。
        result_image = self.draw_detection_overlay(
            frame_rgb,
            source_name=source_name,
            fps=fps,
            targets=targets,
        )
        mask_image, _ = get_debug_images()

        mask_vis = np.zeros_like(frame_rgb) if mask_image is None else cv2.cvtColor(mask_image, cv2.COLOR_GRAY2RGB)
        self.set_label_image(self.camera_label, result_image)
        self.set_label_image(self.mask_label, mask_vis)
        self.update_detection_status(targets, source_name)
        self.maybe_log_detection(targets, source_name)

    def send_detection_result(self, targets: list[dict]) -> None:
        """按帧率和最小间隔限频，将检测结果编码成串口协议帧。"""
        if not self.serial_config.get("enabled", False):
            return

        # 两层限频：按帧数抽样，再按时间间隔限流，避免高帧率下堵塞下位机。
        send_every_n = max(int(self.serial_config.get("send_every_n_frames", 1)), 1)
        if self.processed_frame_count % send_every_n != 0:
            return
        min_send_interval_ms = max(int(self.serial_config.get("min_send_interval_ms", 120)), 0)
        now = time.time()
        if min_send_interval_ms > 0 and (now - self.last_serial_send_ts) * 1000.0 < min_send_interval_ms:
            return

        payload = self.build_detection_packet(targets)
        if self.serial_comm is None or not self.serial_comm.is_connected():
            # 未连接时仍打印完整帧，便于现场确认检测结果和协议内容，但不丢给 os.write。
            self.last_serial_send_ts = now
            self.print_unsent_serial_packet(targets, payload)
            return

        if self.serial_comm.send_frame(payload):
            self.last_serial_send_ts = now
            name = getattr(self.serial_comm, "connection_name", "unknown")
            self.connection_status_label.setText(
                f"串口: 已连接 {name} baudrate={self.serial_comm.baudrate}"
            )
        else:
            # 无论超时还是设备断开，本帧都视为失败并等待下一次定时器/检测周期。
            self.last_serial_send_ts = now
            if "超时" in self.serial_comm.last_error or "timeout" in self.serial_comm.last_error.lower():
                self.connection_status_label.setText("串口: 写入超时，当前帧已丢弃")
            else:
                error_detail = f" ({self.serial_comm.last_error})" if self.serial_comm.last_error else ""
                self.connection_status_label.setText(f"串口: 发送失败{error_detail}")

    def print_unsent_serial_packet(self, targets: list[dict], packet: bytes) -> None:
        """打印未发送帧的业务字段和十六进制内容，支持无硬件联调。"""
        values = self.detection_packet_values(targets)
        port = str(self.serial_config.get("port", "")).strip() or "未配置"
        print(
            "[串口未连接] 未发送检测帧 "
            f"port={port} "
            f"target_count={values[0]:.0f} "
            f"near_flag={values[1]:.0f} near_angle_deg={values[2]:.3f} "
            f"far_flag={values[3]:.0f} far_angle_deg={values[4]:.3f} "
            f"packet_len={len(packet)} "
            f"packet_hex={packet.hex(' ')}"
        )

    @staticmethod
    def detection_packet_values(targets: list[dict]) -> list[float]:
        """按协议固定顺序提取数量、近目标标志/角度和远目标标志/角度。"""
        near_target = MainWindow.target_by_role(targets, "near")
        far_target = MainWindow.target_by_role(targets, "far")
        return [
            float(len(targets)),
            1.0 if near_target is not None else 0.0,
            float(near_target.get("angle_deg", 0.0)) if near_target is not None else 0.0,
            1.0 if far_target is not None else 0.0,
            float(far_target.get("angle_deg", 0.0)) if far_target is not None else 0.0,
        ]

    @staticmethod
    def build_detection_packet(targets: list[dict]) -> bytes:
        """构造 ``A5 + 长度 + 5 个 float32 + CRC16 + 5A`` 二进制帧。"""
        values = MainWindow.detection_packet_values(targets)
        # 使用小端序固定协议字段，payload 长度当前为 5 * 4 = 20 字节。
        payload = struct.pack("<5f", *values)
        length = len(payload)
        # CRC 覆盖帧头、长度和 payload，不包含 CRC 自身及帧尾。
        frame = bytearray([0xA5, length & 0xFF])
        frame.extend(payload)
        crc = crc16_ccitt(bytes(frame))
        frame.extend(struct.pack("<H", crc))
        frame.append(0x5A)
        return bytes(frame)

    @staticmethod
    def target_by_role(targets: list[dict], role: str) -> dict | None:
        """从检测结果中找到指定角色（``near`` 或 ``far``）的第一个目标。"""
        return next((target for target in targets if target.get("role") == role), None)

    def draw_detection_overlay(self, image_rgb, source_name, fps, targets):
        """在原图上绘制中心线、目标框、连线、角度和评分信息。"""
        result = image_rgb.copy()
        height, width = result.shape[:2]
        center = (width // 2, height // 2)
        center_color = (255, 80, 80)
        cv2.line(result, (center[0], 0), (center[0], height), center_color, 1)
        cv2.circle(result, center, 8, center_color, 2)
        cv2.line(result, (center[0] - 14, center[1]), (center[0] + 14, center[1]), center_color, 2)

        found = bool(targets)
        lines = [
            f"Source: {source_name}",
            f"Detect: {'ON' if found else 'OFF'}",
            f"Target Count: {len(targets)}",
        ]
        if fps is not None:
            lines.append(f"FPS: {fps:.1f}")
        for target in targets:
            metrics = target.get("metrics", {})
            x_pixel = target.get("x_pixel", target.get("center", (0, 0))[0])
            x_offset = float(target.get("x_offset", 0.0))
            angle_deg = float(target.get("angle_deg", 0.0))
            lines.append(
                f"{target['role'].upper()}: x={x_pixel} dx={x_offset:.1f}px angle={angle_deg:.3f}deg area={target['area']:.1f}"
            )
            lines.append(
                f"{target['role'].upper()} score={metrics.get('score', 0)} bright={metrics.get('mean_brightness', 0)} circ={metrics.get('circularity', 0)}"
            )

        text_y = 30
        for line in lines:
            cv2.putText(result, line, (12, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
            text_y += 28

        # 目标颜色用于区分近/远类别，中心到目标的连线用于直观看水平偏差方向。
        colors = {"near": (0, 255, 0), "far": (255, 165, 0)}
        line_colors = {"near": (255, 0, 0), "far": (255, 0, 0)}
        for target in targets:
            x, y, w, h = target["bbox"]
            target_center = target["center"]
            color = colors.get(target["role"], (0, 255, 255))
            line_color = line_colors.get(target["role"], (255, 255, 0))
            x_offset = float(target.get("x_offset", 0.0))
            angle_deg = float(target.get("angle_deg", 0.0))
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            cv2.drawContours(result, [target["contour"]], -1, color, 2)
            cv2.circle(result, target_center, 7, color, 2)
            cv2.circle(result, target_center, 3, (255, 255, 255), -1)
            cv2.line(result, center, target_center, line_color, 2, cv2.LINE_AA)
            cv2.line(result, (target_center[0], 0), (target_center[0], height), line_color, 1)
            cv2.putText(
                result,
                target["role"].upper(),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
            )
            label_x = min(max(12, (center[0] + target_center[0]) // 2 + 8), max(12, width - 220))
            label_y = min(max(24, (center[1] + target_center[1]) // 2 - 8), max(24, height - 12))
            cv2.putText(
                result,
                f"dx={x_offset:.1f}px angle={angle_deg:.3f}deg",
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                line_color,
                2,
            )
            if "circle" in target:
                circle_x, circle_y, radius = target["circle"]
                cv2.circle(result, (circle_x, circle_y), radius, line_color, 2)
        return result

    def update_detection_status(self, targets, source_name) -> None:
        """更新底部文本状态；存在近目标时优先显示近目标角度。"""
        if targets:
            near_target = self.target_by_role(targets, "near") or targets[0]
            near_angle = float(near_target.get("angle_deg", 0.0))
            text = (
                f"Detect: hit {len(targets)} target(s) source={source_name}  "
                f"near_x={near_target.get('x_pixel')} angle={near_angle:.3f}deg area={near_target['area']:.1f}"
            )
            far_target = self.target_by_role(targets, "far")
            if far_target is not None:
                far_angle = float(far_target.get("angle_deg", 0.0))
                text += f"  far_x={far_target.get('x_pixel')} angle={far_angle:.3f}deg area={far_target['area']:.1f}"
            self.detect_status_label.setText(text)
        else:
            self.detect_status_label.setText(f"Detect: miss source={source_name}")

    def maybe_log_detection(self, targets, source_name) -> None:
        """按配置的帧间隔输出检测日志，避免控制台被每帧信息淹没。"""
        interval = max(int(self.debug_config.get("console_log_interval_frames", 20)), 1)
        if self.processed_frame_count % interval != 0:
            return
        if targets:
            print(f"[{source_name}] 检测命中 {len(targets)} 个目标")
            for target in targets:
                metrics = target.get("metrics", {})
                print(
                    f"  - {target['role']}: x={target.get('x_pixel')} dx={target.get('x_offset')} angle={target.get('angle_deg')}deg "
                    f"area={target['area']:.1f} bright={metrics.get('mean_brightness')} "
                    f"circ={metrics.get('circularity')} fill={metrics.get('fill_ratio')} score={metrics.get('score')}"
                )
        else:
            print(f"[{source_name}] 未检测到满足阈值的绿光目标")

    def set_label_image(self, label: QLabel, image_rgb: np.ndarray) -> None:
        """将 RGB NumPy 图像包装为 Qt 图片并按 QLabel 比例缩放显示。"""
        # contiguous 保证 QImage 读取到连续内存；这里不改变原始检测帧。
        rgb_image = np.ascontiguousarray(image_rgb)
        height, width, channels = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, channels * width, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        label.setPixmap(
            pixmap.scaled(
                label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def open_video_file(self) -> None:
        """打开文件选择框，选择成功后交给 load_video 初始化视频状态。"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择视频文件",
            "",
            "视频文件 (*.mp4 *.avi *.mkv *.mov *.webm);;所有文件 (*.*)",
        )
        if file_path:
            self.load_video(file_path)

    def load_video(self, video_path: str) -> None:
        """打开视频并切换到视频模式；首帧会立即用于检测和显示。"""
        if self.video_cap is not None:
            self.video_cap.release()
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"无法打开视频: {video_path}")
            return

        self.video_cap = cap
        self.video_fps = cap.get(cv2.CAP_PROP_FPS) or 30
        if self.video_fps <= 1:
            self.video_fps = 30
        self.video_total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.current_frame_idx = 0
        self.is_video_mode = True
        self.is_video_playing = False
        self.video_play_button.setEnabled(True)
        self.video_play_button.setText("播放视频")
        self.video_slider.setEnabled(True)
        self.video_slider.setMaximum(max(self.video_total_frames - 1, 0))
        self.video_slider.setValue(0)
        self.frame_timer.stop()
        print(f"视频已加载: {video_path}")
        self.process_video_frame()

    def toggle_video_playback(self) -> None:
        """启动或暂停视频定时器；播放间隔由视频 FPS 换算得到。"""
        if not self.is_video_mode or self.video_cap is None:
            return
        if self.is_video_playing:
            self.video_timer.stop()
            self.is_video_playing = False
            self.video_play_button.setText("播放视频")
            return
        interval_ms = max(int(1000 / max(self.video_fps, 1.0)), 1)
        self.video_timer.start(interval_ms)
        self.is_video_playing = True
        self.video_play_button.setText("暂停视频")

    def process_video_frame(self) -> None:
        """读取视频下一帧；播放到末尾后从第一帧循环。"""
        if not self.is_video_mode or self.video_cap is None:
            return
        success, frame_bgr = self.video_cap.read()
        if not success:
            self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.current_frame_idx = 0
            self.video_slider.setValue(0)
            return

        self.current_frame_idx = int(self.video_cap.get(cv2.CAP_PROP_POS_FRAMES))
        self.video_slider.blockSignals(True)
        self.video_slider.setValue(self.current_frame_idx)
        self.video_slider.blockSignals(False)
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        self.process_frame(frame_rgb, source_name="video", fps=self.video_fps)

    def on_video_slider_changed(self, value: int) -> None:
        """响应拖动跳帧；播放中先暂停，避免定时器和手动定位同时修改位置。"""
        if not self.is_video_mode or self.video_cap is None:
            return
        if self.is_video_playing:
            self.toggle_video_playback()
        self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, value)
        self.current_frame_idx = value
        self.process_video_frame()

    def switch_to_camera_mode(self) -> None:
        """释放视频句柄并恢复相机定时器。"""
        if self.video_timer.isActive():
            self.video_timer.stop()
        self.is_video_playing = False
        self.is_video_mode = False
        self.video_play_button.setText("播放视频")
        self.video_slider.setEnabled(False)
        self.video_slider.setValue(0)
        if self.video_cap is not None:
            self.video_cap.release()
            self.video_cap = None
        self.frame_timer.start(30)
        print("已切换回相机模式")

    def closeEvent(self, event) -> None:  # noqa: N802
        """Qt 窗口关闭事件：先释放所有外部资源，再接受关闭请求。"""
        self.shutdown()
        event.accept()

    def shutdown(self) -> None:
        """幂等释放定时器、视频、串口、海康设备和录制会话。"""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        self.frame_timer.stop()
        self.video_timer.stop()
        self.reconnect_timer.stop()
        if self.video_cap is not None:
            self.video_cap.release()
            self.video_cap = None
        self._close_serial()
        if self.camera is not None:
            self.camera.MV_CC_StopGrabbing()
            self.camera.MV_CC_CloseDevice()
            self.camera.MV_CC_DestroyHandle()
            self.camera = None

        if self.tuning_window is not None:
            self.tuning_window.hide()

        stop_system_recording()


def main() -> None:
    """创建 Qt 应用，按配置启动主窗口，并统一处理退出清理。"""
    config_path = PROJECT_ROOT / "config.yaml"
    _ = load_config(config_path)

    # 即使窗口异常退出，也尝试关闭录制器，避免最后一个会话长期占用资源。
    atexit.register(stop_system_recording)
    app = QApplication(sys.argv)

    # 让 Ctrl+C/SIGTERM 能在 Qt 事件循环中被可靠处理；sig_timer 用于唤醒事件循环。
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    sig_timer = QTimer()
    sig_timer.start(200)
    sig_timer.timeout.connect(lambda: None)

    # MainWindow 构造阶段完成 UI/配置/串口初始化，runtime.mode 决定是否启动相机。
    window = MainWindow(config_path)
    window.startup_by_runtime_mode()
    app.aboutToQuit.connect(window.shutdown)
    exit_code = 0
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在退出...")
        exit_code = 0
    finally:
        # aboutToQuit、closeEvent 和 KeyboardInterrupt 可能走不同路径，shutdown 本身保持幂等。
        window.shutdown()
        stop_system_recording()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
