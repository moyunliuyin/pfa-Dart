import sys
import cv2
import numpy as np
import threading
import time
from ctypes import *
from PyQt6.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout,
                             QWidget, QSlider, QGroupBox)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap

# 根据系统选择正确的库路径
if sys.platform.startswith("win"):
    sys.path.append("./MvImport")
    from MvImport.MvCameraControl_class import *
else:
    sys.path.append("./MvImport_Linux")
    from MvImport_Linux.MvCameraControl_class import *


def get_Value(cam, param_type="float_value", node_name=""):
    """获取相机参数"""
    if param_type == "float_value":
        stParam = MVCC_FLOATVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_FLOATVALUE))
        ret = cam.MV_CC_GetFloatValue(node_name, stParam)
        if ret != 0:
            print("获取参数 %s 失败! ret[0x%x]" % (node_name, ret))
            return None
        return stParam.fCurValue
    elif param_type == "enum_value":
        stParam = MVCC_ENUMVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_ENUMVALUE))
        ret = cam.MV_CC_GetEnumValue(node_name, stParam)
        if ret != 0:
            print("获取参数 %s 失败! ret[0x%x]" % (node_name, ret))
            return None
        return stParam.nCurValue


def set_Value(cam, param_type="float_value", node_name="", node_value=0):
    """设置相机参数"""
    if param_type == "float_value":
        ret = cam.MV_CC_SetFloatValue(node_name, node_value)
        if ret != 0:
            print("设置参数 %s 失败! ret[0x%x]" % (node_name, ret))
    elif param_type == "enum_value":
        ret = cam.MV_CC_SetEnumValue(node_name, node_value)
        if ret != 0:
            print("设置参数 %s 失败! ret[0x%x]" % (node_name, ret))


class CameraThread(threading.Thread):
    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.running = True
        self.frame = None
        self.lock = threading.Lock()
        self.stFrameInfo = MV_FRAME_OUT_INFO_EX()
        self.data_size = None
        self.pData = None

    def run(self):
        # 获取数据包大小
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = self.camera.MV_CC_GetIntValueEx("PayloadSize", stParam)
        if ret != 0:
            print("获取数据包大小失败! ret[0x%x]" % ret)
            return

        self.data_size = stParam.nCurValue
        self.pData = (c_ubyte * self.data_size)()
        memset(byref(self.stFrameInfo), 0, sizeof(self.stFrameInfo))

        while self.running:
            ret = self.camera.MV_CC_GetOneFrameTimeout(self.pData, self.data_size, self.stFrameInfo, 1000)
            if ret == 0:
                # 转换图像格式
                data = np.frombuffer(self.pData, dtype=np.uint8)

                # 根据像素格式处理图像
                frame = data.reshape((self.stFrameInfo.nHeight, self.stFrameInfo.nWidth))

                if self.stFrameInfo.enPixelType == 17301505:  # Mono8
                    img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif self.stFrameInfo.enPixelType == 17301513:  # BayerGB8
                    img = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                elif self.stFrameInfo.enPixelType == 17301514:  # BayerRG8
                    img = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                elif self.stFrameInfo.enPixelType == 17301515:  # BayerGR8
                    img = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                elif self.stFrameInfo.enPixelType == 17301516:  # BayerBG8
                    img = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                else:
                    try:
                        img = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                    except:
                        try:
                            img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                        except:
                            print(f"未知的像素格式: {self.stFrameInfo.enPixelType}，无法转换")
                            continue

                with self.lock:
                    self.frame = img.copy()
            else:
                print("获取一帧图像失败: ret[0x%x]" % ret)

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
            return None

    def stop(self):
        self.running = False


class CameraControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化相机变量
        self.camera = None
        self.camera_thread = None
        self.prev_frame_time = 0

        # 设置相机
        self.setup_camera()

        # 初始化UI
        self.initUI()

        # 启动图像更新定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # 30ms刷新一次，约33FPS

    def setup_camera(self):
        """初始化相机"""
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("枚举设备失败! ret[0x%x]" % ret)
            return False

        if deviceList.nDeviceNum == 0:
            print("没有找到设备!")
            return False

        print("找到 %d 个设备!" % deviceList.nDeviceNum)

        # 创建相机实例
        self.camera = MvCamera()
        stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.camera.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("创建句柄失败! ret[0x%x]" % ret)
            return False

        # 打开设备
        ret = self.camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("打开设备失败! ret[0x%x]" % ret)
            return False

        # 获取曝光和增益的范围
        self.exposure_min = 1000
        self.exposure_max = 50000  # 默认较大值
        self.gain_min = 0
        self.gain_max = 17.0  # 按要求，增益上限为17.0

        # 获取当前曝光和增益值
        self.current_exposure = get_Value(self.camera, "float_value", "ExposureTime") or 10000
        self.current_gain = get_Value(self.camera, "float_value", "Gain") or 5.0

        print(f"当前曝光值: {self.current_exposure}微秒")
        print(f"当前增益值: {self.current_gain}")

        # 开始取流
        ret = self.camera.MV_CC_StartGrabbing()
        if ret != 0:
            print("开始取流失败! ret[0x%x]" % ret)
            return False

        # 创建相机线程
        self.camera_thread = CameraThread(self.camera)
        self.camera_thread.start()

        return True

    def initUI(self):
        """初始化UI界面"""
        self.setWindowTitle('海康相机控制器')
        self.setGeometry(100, 100, 1200, 800)

        # 创建主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()

        # 图像显示区域
        self.image_label = QLabel()
        self.image_label.setMinimumSize(1024, 600)
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setStyleSheet("border: 1px solid #cccccc;")
        main_layout.addWidget(self.image_label)

        # 控制区域
        control_layout = QHBoxLayout()

        # 曝光控制组
        exposure_group = QGroupBox("曝光控制 (微秒)")
        exposure_layout = QVBoxLayout()

        # 曝光滑动条 - 按1000微秒为步长
        self.exposure_slider = QSlider(Qt.Orientation.Horizontal)
        self.exposure_slider.setMinimum(int(self.exposure_min / 1000))
        self.exposure_slider.setMaximum(int(self.exposure_max / 1000))
        self.exposure_slider.setValue(int(self.current_exposure / 1000))
        self.exposure_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.exposure_slider.setTickInterval(10)  # 每10个单位显示一个刻度
        self.exposure_slider.valueChanged.connect(self.exposure_changed)

        # 曝光值显示标签
        self.exposure_label = QLabel(f"曝光时间: {self.current_exposure}微秒")

        exposure_layout.addWidget(self.exposure_label)
        exposure_layout.addWidget(self.exposure_slider)
        exposure_group.setLayout(exposure_layout)

        # 增益控制组
        gain_group = QGroupBox("增益控制")
        gain_layout = QVBoxLayout()

        # 增益滑动条 - 精度为0.1
        self.gain_slider = QSlider(Qt.Orientation.Horizontal)
        self.gain_slider.setMinimum(int(self.gain_min * 10))
        self.gain_slider.setMaximum(int(self.gain_max * 10))
        self.gain_slider.setValue(int(self.current_gain * 10))
        self.gain_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.gain_slider.setTickInterval(10)  # 每10个单位(相当于增益1.0)显示一个刻度
        self.gain_slider.valueChanged.connect(self.gain_changed)

        # 增益值显示标签
        self.gain_label = QLabel(f"增益值: {self.current_gain:.1f}")

        gain_layout.addWidget(self.gain_label)
        gain_layout.addWidget(self.gain_slider)
        gain_group.setLayout(gain_layout)

        # 添加到控制布局
        control_layout.addWidget(exposure_group)
        control_layout.addWidget(gain_group)

        # FPS显示标签
        self.fps_label = QLabel("FPS: 0.0")
        self.fps_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        control_layout.addWidget(self.fps_label)

        # 将控制布局添加到主布局
        main_layout.addLayout(control_layout)

        # 设置主布局
        main_widget.setLayout(main_layout)

    def exposure_changed(self, value):
        """曝光滑动条值变化回调"""
        # 转换回微秒，以1000为单位
        exposure_value = value * 1000
        self.current_exposure = exposure_value
        self.exposure_label.setText(f"曝光时间: {exposure_value}微秒")

        # 设置相机曝光值
        set_Value(self.camera, "float_value", "ExposureTime", exposure_value)

    def gain_changed(self, value):
        """增益滑动条值变化回调"""
        # 转换回实际增益值，精度为0.1
        gain_value = value / 10.0
        self.current_gain = gain_value
        self.gain_label.setText(f"增益值: {gain_value:.1f}")

        # 设置相机增益值
        set_Value(self.camera, "float_value", "Gain", gain_value)

    def update_frame(self):
        """更新图像帧显示"""
        if self.camera_thread and self.camera_thread.running:
            frame = self.camera_thread.get_frame()
            if frame is not None:
                # 计算FPS
                current_time = time.time()
                fps = 1 / (current_time - self.prev_frame_time) if self.prev_frame_time > 0 else 0
                self.prev_frame_time = current_time
                self.fps_label.setText(f"FPS: {fps:.1f}")

                # 绘制信息到图像上
                h, w, _ = frame.shape
                frame_copy = frame.copy()

                # 添加曝光和增益信息
                cv2.putText(frame_copy, f"Exposure: {self.current_exposure}us", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame_copy, f"Gain: {self.current_gain:.1f}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame_copy, f"FPS: {fps:.1f}", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # 转换为Qt图像并显示
                frame_rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image)
                scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
                self.image_label.setPixmap(scaled_pixmap)

    def closeEvent(self, event):
        """窗口关闭事件处理"""
        # 停止相机线程
        if self.camera_thread:
            self.camera_thread.stop()
            self.camera_thread.join()

        # 停止取流
        if self.camera:
            self.camera.MV_CC_StopGrabbing()
            # 关闭设备
            self.camera.MV_CC_CloseDevice()
            # 销毁句柄
            self.camera.MV_CC_DestroyHandle()

        event.accept()


def main():
    app = QApplication(sys.argv)
    window = CameraControlWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()