"""模拟 DartSystem 向下位机发送绿光检测结果。

该脚本复用 DartSystem 的应用层协议，但不导入 ``main.py``，因此不会启动
Qt、相机或检测模型。默认只打印协议帧；只有显式传入 ``--port`` 才会打开
串口并发送。

协议帧：
    A5 | length=20 | 5 个 little-endian float32 | CRC16-CCITT | 5A

五个 float32 的顺序与 ``MainWindow.build_detection_packet`` 一致：
    target_count, near_flag, near_angle_deg, far_flag, far_angle_deg
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time
from pathlib import Path

try:
    import termios
except ImportError:  # pragma: no cover - Windows 没有 termios。
    termios = None


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 115200
DEFAULT_INTERVAL_MS = 200


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """计算 DartSystem 使用的 CRC-16/CCITT（多项式 0x1021）。"""
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_detection_packet(
    near_angle_deg: float | None = None,
    far_angle_deg: float | None = None,
    target_count: int | None = None,
) -> bytes:
    """构造与 DartSystem 主程序完全一致的 25 字节检测帧。

    ``near_angle_deg`` 或 ``far_angle_deg`` 传入后，对应目标标志为 1；
    未传入时标志为 0，角度填 0。目标数量默认由两个目标标志计算，也可以
    用 ``target_count`` 显式覆盖来测试下位机的异常/边界处理。
    """
    near_flag = 1.0 if near_angle_deg is not None else 0.0
    far_flag = 1.0 if far_angle_deg is not None else 0.0
    near_angle = float(near_angle_deg) if near_angle_deg is not None else 0.0
    far_angle = float(far_angle_deg) if far_angle_deg is not None else 0.0

    if target_count is None:
        target_count_value = near_flag + far_flag
    else:
        if target_count < 0:
            raise ValueError("target_count 不能为负数")
        target_count_value = float(target_count)

    payload = struct.pack(
        "<5f",
        target_count_value,
        near_flag,
        near_angle,
        far_flag,
        far_angle,
    )
    frame = bytearray((0xA5, len(payload)))
    frame.extend(payload)
    frame.extend(struct.pack("<H", crc16_ccitt(bytes(frame))))
    frame.append(0x5A)
    return bytes(frame)


def baudrate_flag(baudrate: int) -> int:
    if termios is None:
        raise OSError("当前平台没有 termios 串口支持")
    name = f"B{baudrate}"
    if not hasattr(termios, name):
        raise OSError(f"不支持的串口波特率: {baudrate}")
    return getattr(termios, name)


def configure_port(fd: int, baudrate: int) -> None:
    """按 DartSystem 的 8N1 原始模式配置串口。"""
    if termios is None:
        raise OSError("当前平台没有 termios 串口支持")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = baudrate_flag(baudrate)
    attrs[5] = baudrate_flag(baudrate)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def write_all(fd: int, packet: bytes, timeout_ms: int) -> None:
    """处理部分写入，确保整帧写完或明确报错。"""
    deadline = None if timeout_ms <= 0 else time.monotonic() + timeout_ms / 1000.0
    pending = memoryview(packet)
    while pending:
        timeout = None if deadline is None else max(deadline - time.monotonic(), 0.0)
        _, writable, _ = select.select([], [fd], [], timeout)
        if not writable:
            raise TimeoutError("串口写入超时")
        written = os.write(fd, pending)
        if written <= 0:
            raise TimeoutError("串口写入超时")
        pending = pending[written:]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="模拟 DartSystem 向下位机发送检测结果")
    parser.add_argument(
        "--port",
        help=f"实际串口设备；不传则只打印。Linux 默认配置通常是 {DEFAULT_PORT}",
    )
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="波特率，默认 115200")
    parser.add_argument("--near-angle", type=float, help="近目标水平角度（度）；指定后 near_flag=1")
    parser.add_argument("--far-angle", type=float, help="远目标水平角度（度）；指定后 far_flag=1")
    parser.add_argument("--target-count", type=int, help="显式指定目标数量；默认按 near/far 标志计算")
    parser.add_argument("--count", type=int, default=1, help="发送帧数；0 表示持续发送，默认 1")
    parser.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS, help="发送间隔，默认 200ms")
    parser.add_argument("--write-timeout-ms", type=int, default=200, help="单帧写入超时，0 表示不限制")
    parser.add_argument("--dry-run", action="store_true", help="只打印数据，不打开串口")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.count < 0:
        print("错误: --count 不能为负数", file=sys.stderr)
        return 2
    if args.interval_ms < 0 or args.write_timeout_ms < 0:
        print("错误: --interval-ms 和 --write-timeout-ms 不能为负数", file=sys.stderr)
        return 2
    if args.baudrate <= 0:
        print("错误: --baudrate 必须为正数", file=sys.stderr)
        return 2
    if args.target_count is not None and args.target_count < 0:
        print("错误: --target-count 不能为负数", file=sys.stderr)
        return 2

    packet = build_detection_packet(args.near_angle, args.far_angle, args.target_count)
    serial_fd: int | None = None
    if args.port is None or args.dry_run:
        print("运行模式: dry-run（不会打开串口）")
        if args.port:
            print(f"提示: 已忽略 --port {args.port}，因为指定了 --dry-run")
    else:
        if termios is None:
            print("错误: 当前平台不支持 termios 串口发送；请在 Linux 上运行", file=sys.stderr)
            return 2
        try:
            serial_fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            configure_port(serial_fd, args.baudrate)
        except (OSError, ValueError) as exc:
            if serial_fd is not None:
                os.close(serial_fd)
            print(f"错误: 无法打开或配置串口 {args.port}: {exc}", file=sys.stderr)
            return 1
        print(f"运行模式: serial（发送到 {args.port}，{args.baudrate} baud，8N1）")

    print(
        f"target_count={struct.unpack_from('<f', packet, 2)[0]:.0f} "
        f"near_flag={struct.unpack_from('<f', packet, 6)[0]:.0f} "
        f"near_angle_deg={struct.unpack_from('<f', packet, 10)[0]:.3f} "
        f"far_flag={struct.unpack_from('<f', packet, 14)[0]:.0f} "
        f"far_angle_deg={struct.unpack_from('<f', packet, 18)[0]:.3f} "
        f"packet_len={len(packet)} packet_hex={packet.hex(' ')}"
    )

    try:
        send_index = 0
        while args.count == 0 or send_index < args.count:
            if serial_fd is not None:
                write_all(serial_fd, packet, args.write_timeout_ms)
            send_index += 1
            print(f"已{'发送' if serial_fd is not None else '模拟'}第 {send_index} 帧")
            if args.count == 0 or send_index < args.count:
                time.sleep(args.interval_ms / 1000.0)
    except KeyboardInterrupt:
        print("\n已停止")
        return 130
    except (OSError, TimeoutError) as exc:
        print(f"串口发送失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if serial_fd is not None:
            os.close(serial_fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
