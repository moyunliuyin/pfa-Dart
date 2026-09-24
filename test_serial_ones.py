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
except ImportError:  # pragma: no cover - termios is unavailable on Windows.
    termios = None

from app_config import PROJECT_ROOT, load_config


ONES_VALUES = (1.0, 1.0, 1.0, 1.0, 1.0)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_ones_packet() -> bytes:
    payload = struct.pack("<5f", *ONES_VALUES)
    frame = bytearray([0xA5, len(payload) & 0xFF])
    frame.extend(payload)
    crc = crc16_ccitt(bytes(frame))
    frame.extend(struct.pack("<H", crc))
    frame.append(0x5A)
    return bytes(frame)


def baudrate_flag(baudrate: int) -> int:
    assert termios is not None
    baudrate_name = f"B{baudrate}"
    if not hasattr(termios, baudrate_name):
        raise OSError(f"不支持的串口波特率: {baudrate}")
    return getattr(termios, baudrate_name)


def data_bits_flag(data_bits: int) -> int:
    assert termios is not None
    data_bits_map = {
        5: termios.CS5,
        6: termios.CS6,
        7: termios.CS7,
        8: termios.CS8,
    }
    if data_bits not in data_bits_map:
        raise OSError(f"不支持的数据位: {data_bits}")
    return data_bits_map[data_bits]


def parity_flag(parity: str) -> int:
    assert termios is not None
    normalized_parity = parity.lower()
    if normalized_parity in {"none", "n", ""}:
        return 0
    if normalized_parity in {"even", "e"}:
        return termios.PARENB
    if normalized_parity in {"odd", "o"}:
        return termios.PARENB | termios.PARODD
    raise OSError(f"不支持的校验位: {parity}")


def stop_bits_flag(stop_bits: int) -> int:
    assert termios is not None
    if stop_bits == 1:
        return 0
    if stop_bits == 2:
        return termios.CSTOPB
    raise OSError(f"不支持的停止位: {stop_bits}")


def configure_port(fd: int, baudrate: int, data_bits: int, parity: str, stop_bits: int) -> None:
    assert termios is not None
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = data_bits_flag(data_bits) | parity_flag(parity) | stop_bits_flag(stop_bits) | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = baudrate_flag(baudrate)
    attrs[5] = baudrate_flag(baudrate)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def write_all(fd: int, payload: bytes, timeout_ms: int) -> None:
    deadline = None if timeout_ms <= 0 else time.monotonic() + (timeout_ms / 1000.0)
    pending = memoryview(payload)
    while pending:
        timeout = None if deadline is None else max(deadline - time.monotonic(), 0.0)
        _, writable, _ = select.select([], [fd], [], timeout)
        if not writable:
            raise TimeoutError("串口写入超时")
        written = os.write(fd, pending)
        if written <= 0:
            raise TimeoutError("串口写入超时")
        pending = pending[written:]


def serial_config_from_file(config_path: Path) -> dict:
    config = load_config(config_path)
    serial_config = config.get("serial", {})
    if not isinstance(serial_config, dict):
        raise ValueError("配置文件 serial 段无效")
    return serial_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="发送一帧业务字段全为 1.0 的串口测试数据")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config.yaml", help="配置文件路径")
    parser.add_argument("--port", help="串口设备名，默认读取 config.yaml 的 serial.port")
    parser.add_argument("--baudrate", type=int, help="串口波特率，默认读取配置")
    parser.add_argument("--data-bits", type=int, help="数据位，默认读取配置")
    parser.add_argument("--parity", help="校验位，默认读取配置")
    parser.add_argument("--stop-bits", type=int, help="停止位，默认读取配置")
    parser.add_argument("--write-timeout-ms", type=int, help="写入超时，默认读取配置")
    parser.add_argument("--count", type=int, default=1, help="发送次数；0 表示一直发送，直到 Ctrl+C")
    parser.add_argument("--interval-ms", type=int, help="多次发送的间隔，默认读取 serial.min_send_interval_ms")
    parser.add_argument("--dry-run", action="store_true", help="只打印测试帧，不打开串口")
    return parser.parse_args()


def main() -> int:
    if termios is None:
        print("当前平台不支持 termios 串口配置。", file=sys.stderr)
        return 2

    args = parse_args()
    serial_config = serial_config_from_file(args.config)
    port = args.port or str(serial_config.get("port", "/dev/ttyACM0"))
    baudrate = args.baudrate or int(serial_config.get("baudrate", 115200))
    data_bits = args.data_bits or int(serial_config.get("data_bits", 8))
    parity = args.parity or str(serial_config.get("parity", "none"))
    stop_bits = args.stop_bits or int(serial_config.get("stop_bits", 1))
    write_timeout_ms = args.write_timeout_ms if args.write_timeout_ms is not None else int(serial_config.get("write_timeout_ms", 200))
    interval_ms = args.interval_ms if args.interval_ms is not None else int(serial_config.get("min_send_interval_ms", 200))
    packet = build_ones_packet()

    print("串口测试帧: 5 个业务 float32 均为 1.0，串口实际发送 25 字节原始帧")
    print(f"packet_len={len(packet)} packet_hex={packet.hex(' ')}")
    timeout_text = "关闭" if write_timeout_ms <= 0 else f"{write_timeout_ms}ms"
    print(f"port={port} baudrate={baudrate} data_bits={data_bits} parity={parity} stop_bits={stop_bits} write_timeout={timeout_text}")

    if args.dry_run:
        return 0

    fd: int | None = None
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        configure_port(fd, baudrate, data_bits, parity, stop_bits)
        send_index = 0
        while args.count == 0 or send_index < args.count:
            write_all(fd, packet, write_timeout_ms)
            send_index += 1
            print(f"已发送第 {send_index} 帧")
            if args.count == 0 or send_index < args.count:
                time.sleep(max(interval_ms, 0) / 1000.0)
        return 0
    except KeyboardInterrupt:
        print("已停止发送")
        return 130
    except OSError as exc:
        print(f"串口测试发送失败: {exc}", file=sys.stderr)
        return 1
    except TimeoutError as exc:
        print(f"串口测试发送失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if fd is not None:
            os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())