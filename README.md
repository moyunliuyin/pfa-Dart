# DartSystem

用于检测明亮的圆形绿光，并将识别结果通过 **串口** 发送到下位机。

当前运行/通信边界：

- Linux 串口设备（如 `/dev/ttyACM0` 或 `/dev/ttyUSB0`）
- 原生串口字节流发送
- 应用层 payload 发送水平转角（度）
- Windows 可运行相机/视频检测和调参界面；当前发送层使用 termios，Windows 会跳过发送层

## 功能

- 海康相机实时采集
- 视频/图片测试模式
- 双目标绿光检测
- 近目标 / 远目标面积阈值分类
- 实时调参与保存到 `config.yaml`
- 启动后可选海康取流录制，可保存到固定 `video/` 目录
- Linux 下通过串口向下位机发送结果

## 文件

- [main.py](main.py)：主程序、GUI、相机、串口发送
- [system_recorder.py](system_recorder.py)：海康 SDK 帧的 OpenCV 录制
- [opencv_green_detection.py](opencv_green_detection.py)：绿光检测算法
- [app_config.py](app_config.py)：默认配置与配置保存
- [config.yaml](config.yaml)：主配置文件

## Python 环境

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 串口准备

程序不会自动猜测串口号，要先确认下位机在系统中的设备名。

常见设备名：

- STM32 USB CDC：`/dev/ttyACM0`
- USB 转串口模块：`/dev/ttyUSB0`
- Windows：`COM1`、`COM3` 等

Linux 可用下面命令查看：

```bash
dmesg | grep -E "ttyACM|ttyUSB"
ls /dev/ttyACM* /dev/ttyUSB*
```

如果串口设备权限不足，可临时授权或把当前用户加入对应组：

```bash
sudo chmod a+rw /dev/ttyACM0
# 或：sudo usermod -aG dialout $USER，然后重新登录
```

## 运行

```bash
source .venv/bin/activate
python main.py
```

如果先做算法联调，建议先用测试模式：

```yaml
runtime:
  mode: "test"

test:
  source: "video"
  path: "test.mp4"
  auto_start: true
```

说明：Linux 下开启 `serial.enabled=true` 后，`hik` 和 `test` 两种运行模式都会发送串口数据；Windows 下当前会自动跳过发送层。

## 海康相机录制

在 `recording.enabled: true` 时，程序把 **海康 SDK 已解码的 RGB 帧**（与界面/检测同源）写入 `video/`，文件名形如 `hik_camera_recording_*.mp4`。首帧到达后才创建文件；退出时关闭 `VideoWriter` 并保存。单路海康独占时**不再用 ffmpeg 另开设备**。

依赖：随 **OpenCV**（`requirements.txt`）的 **VideoWriter**；Linux 上写 mp4 常依赖系统 **ffmpeg** 相关库。若 `mp4` 打不开或创建失败，可将 `format` 改为 **`avi`**。

[config.yaml](config.yaml) 示例：

```yaml
recording:
  enabled: true
  format: "mp4"
  output_dir: "video"
  frame_rate: 15
  resolution: null
```

- `frame_rate`：写入容器的标称帧率。
- `resolution`：`null` 为原始分辨率；可写 `"1280x720"` 缩放后再写文件。

## 串口配置

[config.yaml](config.yaml) 中的推荐配置：

```yaml
serial:
  enabled: true
  port: "/dev/ttyACM0"
  baudrate: 115200
  data_bits: 8
  parity: "none"
  stop_bits: 1
  write_timeout_ms: 200
  reconnect_interval_ms: 5000
  min_send_interval_ms: 120
  send_every_n_frames: 2
```

参数说明：

- `enabled`：是否启用串口发送
- `port`：串口设备名，按实际设备修改
- `baudrate`：串口波特率，需与下位机一致
- `data_bits`：数据位，通常为 `8`
- `parity`：校验位，支持 `none`、`even`、`odd`
- `stop_bits`：停止位，支持 `1` 或 `2`
- `write_timeout_ms`：单帧写入超时时间（毫秒）
- `min_send_interval_ms`：两次发送之间的最小间隔（毫秒）
- `send_every_n_frames`：每几帧发送一次

## 检测数据模拟

`simulate_dart.py` 用与主程序相同的 25 字节协议，模拟“绿光检测结果发送给下位机”。
默认只打印帧，不打开串口：

```bash
python simulate_dart.py --near-angle 2.5 --far-angle -1.2
```

指定串口后才会实际发送到该串口连接的下位机（Linux 下通常是 `/dev/ttyACM0`）：

```bash
python simulate_dart.py --port /dev/ttyACM0 --near-angle 2.5 --count 0
```

未指定 `--near-angle` 或 `--far-angle` 时，对应目标标志为 0、角度为 0。发送帧中的 5 个字段顺序为：
`target_count`、`near_flag`、`near_angle_deg`、`far_flag`、`far_angle_deg`。

## 发送内容

发送给下位机的应用层数据先构造成 25 字节协议帧，只包含水平转角：

```text
Byte0:    0xA5
Byte1:    Length = 20
Byte2-21: 5个 float32，小端序
Byte22-23: CRC16，小端序
Byte24:   0x5A
```

5 个 `float32` 的含义：

1. `target_count`
2. `near_flag`
3. `near_angle_deg`
4. `far_flag`
5. `far_angle_deg`

说明：

- 只使用目标中心的 x 轴像素位置：`x_offset = target_x - center_x`
- 标定优先公式：`angle_deg = atan(x_offset / fx) * 180 / pi`
- `calibration.image_width_px` 用于在当前帧宽不同于标定宽度时同步缩放 `fx/cx`
- 如果没有 `calibration.fx`，回退到镜头估算：`angle_deg = atan((x_offset * pixel_size_um / 1000) / focal_length_mm) * 180 / pi`
- 当前默认 `fx=15989.09`，小角度下每像素约 `0.003583°`

串口线上实际发送的是以上 25 字节原始协议帧。文档和终端中的 `a5 14 ... 5a` 是十六进制显示方式，不是 ASCII 字符串；线上没有空格、没有换行。

## 调参

高频调参参数都在 `config.yaml` 的 `detection` 段。

最常调：

- `detection.hsv.lower / upper`
- `detection.brightness.min_v`
- `detection.contour.min_area / max_area`
- `detection.contour.far_min_area / near_min_area`
- `detection.contour.min_circularity`
- `detection.contour.min_fill_ratio`
- `detection.circle.min_radius / max_radius`
- `calibration.fx / fy / cx / cy`
- `detection.angle.focal_length_mm / pixel_size_um / center_x_px / invert_x`

界面支持：

- 重载配置
- 实时调参开关
- 保存当前实时参数到配置

## 联调顺序

1. 先确认下位机串口设备名，并写入 `serial.port`
2. 用 `runtime.mode=test` 跑视频，确认检测结果稳定
3. 打开 `serial.enabled=true`
4. 确认下位机串口波特率、数据位、校验位、停止位一致
5. 再切海康相机实时模式

## 常见问题

`串口: 连接失败 (No such file or directory)`

- 说明配置的 `serial.port` 不存在，检查 `/dev/ttyACM*` 或 `/dev/ttyUSB*`

`串口: 连接失败 (Permission denied)`

- 当前用户没有串口读写权限，可临时 `chmod` 或加入 `dialout` 组

`不支持的串口波特率`

- `serial.baudrate` 不在系统 termios 支持列表中，改为常见值如 `115200`、`921600`

`串口发送失败: 串口写入超时`

- 说明串口写入阻塞或下位机/驱动处理不过来
- 建议先调大 `serial.min_send_interval_ms`（如 120~300）或 `serial.send_every_n_frames`（如 2~5）
- 检查串口线、USB 连接和下位机接收任务是否正常

## 当前限制

- 当前发送层使用 termios 串口配置，主要面向 Linux 部署
- Windows 可用于检测/调参，不启用发送层
- Linux 部署时需要系统已正确提供串口设备并授予读写权限



#near 6300
#far 3000
