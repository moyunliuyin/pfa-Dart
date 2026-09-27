"""兼容 USB 摄像头与海康相机的简单工厂封装。"""

import cv2
import numpy as np
import sys

# 根据系统选择正确的库路径
if sys.platform.startswith("win"):
    sys.path.append("./MvImport")
    from MvImport.MvCameraControl_class import *
else:
    sys.path.append("./MvImport_Linux")
    from MvImport_Linux.MvCameraControl_class import *

class CameraFactory:
    """根据配置创建对应的相机实现。"""

    @staticmethod
    def create_camera(camera_type='usb', device_id=0):
        """创建 USB 或海康相机；未知类型直接报错，避免静默使用错误设备。"""
        if camera_type.lower() == 'usb':
            return UsbCamera(device_id)
        elif camera_type.lower() == 'hik':
            return HikCamera()
        else:
            raise ValueError(f"Unsupported camera type: {camera_type}")

class BaseCamera:
    """相机实现的最小统一接口。"""

    def __init__(self):
        self.cap = None

    def read(self):
        """读取一帧，具体格式由子类决定。"""
        raise NotImplementedError

    def release(self):
        """释放 OpenCV 相机句柄。"""
        if self.cap is not None:
            self.cap.release()

class UsbCamera(BaseCamera):
    """通过 OpenCV ``VideoCapture`` 读取 USB 摄像头。"""

    def __init__(self, device_id=0):
        super().__init__()
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera with device ID: {device_id}")

    def read(self):
        """返回 OpenCV 的 ``(success, frame)`` 结果。"""
        return self.cap.read()

class HikCamera(BaseCamera):
    """通过海康 SDK 读取第一台枚举到的相机。"""

    def __init__(self):
        super().__init__()
        # 枚举设备
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            raise RuntimeError("枚举设备失败! ret[0x%x]" % ret)

        if deviceList.nDeviceNum == 0:
            raise RuntimeError("没有找到设备!")

        print("找到 %d 个设备!" % deviceList.nDeviceNum)

        # 创建相机实例
        self.cam = MvCamera()
        stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            raise RuntimeError("创建句柄失败! ret[0x%x]" % ret)

        # 打开设备
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            raise RuntimeError("打开设备失败! ret[0x%x]" % ret)

        # 开始取流
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            raise RuntimeError("开始取流失败! ret[0x%x]" % ret)

    def read(self):
        """拉取一帧 BayerGB8 图像并转换为 BGR；不支持的格式返回空帧。"""
        try:
            stOutFrame = MV_FRAME_OUT()
            ret = self.cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
            
            if ret == 0:
                # 转换图像格式
                data_size = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight
                data = bytes(bytearray(stOutFrame.pBufAddr[0:data_size]))
                
                if stOutFrame.stFrameInfo.enPixelType == 17301513:  # BayerGB8
                    pixel_data = np.frombuffer(data, dtype=np.uint8)
                    frame = pixel_data.reshape((stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth))
                    frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
                    self.cam.MV_CC_FreeImageBuffer(stOutFrame)
                    return True, frame
                
                self.cam.MV_CC_FreeImageBuffer(stOutFrame)
            
            return False, None
            
        except Exception as e:
            print(f"Error reading from HIK camera: {str(e)}")
            return False, None

    def release(self):
        """按停止取流、关闭设备、销毁句柄的顺序释放 SDK 资源。"""
        if self.cam is not None:
            # 停止取流
            ret = self.cam.MV_CC_StopGrabbing()
            if ret != 0:
                print("停止取流失败! ret[0x%x]" % ret)

            # 关闭设备
            ret = self.cam.MV_CC_CloseDevice()
            if ret != 0:
                print("关闭设备失败! ret[0x%x]" % ret)

            # 销毁句柄
            ret = self.cam.MV_CC_DestroyHandle()
            if ret != 0:
                print("销毁句柄失败! ret[0x%x]" % ret)
