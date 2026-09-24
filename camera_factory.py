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
    @staticmethod
    def create_camera(camera_type='usb', device_id=0):
        if camera_type.lower() == 'usb':
            return UsbCamera(device_id)
        elif camera_type.lower() == 'hik':
            return HikCamera()
        else:
            raise ValueError(f"Unsupported camera type: {camera_type}")

class BaseCamera:
    def __init__(self):
        self.cap = None

    def read(self):
        raise NotImplementedError

    def release(self):
        if self.cap is not None:
            self.cap.release()

class UsbCamera(BaseCamera):
    def __init__(self, device_id=0):
        super().__init__()
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera with device ID: {device_id}")

    def read(self):
        return self.cap.read()

class HikCamera(BaseCamera):
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
