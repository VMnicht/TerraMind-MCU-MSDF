#!/usr/bin/env python3
"""Epson M-G366PDG0 four-mode UART upper computer.

Mode switching intentionally does not write SMPL_CTRL, FILTER_CTRL,
UART_CTRL, MSC_CTRL, POL_CTRL, or GLOB_CMD3. DRDY has a separate control that
only updates MSC_CTRL.DRDY_ON/DRDY_POL. Flash backup is available only as a
separate, explicit, confirmed user action.
Delta scaling has a separate readback-verified low-byte configuration action.
"""

from __future__ import annotations

import ctypes
from ctypes import wintypes
from collections import deque
from dataclasses import dataclass
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
import winreg


BAUD_RATE = 230400
CR = 0x0D
ADDRESS = 0x80


def delta_range_text(angle_code: int, rate: float | None) -> str:
    """G365PDF1 32-bit representation range, not the physical gyro range."""
    limit = (2**31 - 1) * (1 / 66 / 2000) * 2**angle_code / 65536
    text = f"G365PDF1 角增量表示范围 ±{limit:.6f}°/帧"
    if rate is not None:
        text += f"；{rate:g} Hz 下等效 ±{limit * rate:.2f}°/s"
    return text + "（不扩大传感器物理量程）"


@dataclass(frozen=True)
class ModeProfile:
    key: str
    title: str
    description: str
    burst1: int
    burst2: int
    atti_high: int
    packet_length: int
    temp_bits: int = 16
    gyro_bits: int = 0
    accel_bits: int = 0
    delta_bits: int = 0
    attitude_bits: int = 0


MODES = {
    "raw16": ModeProfile(
        "raw16", "1  原始 IMU 16-bit", "通用兼容；严格全 16-bit",
        0xF007, 0x0000, 0x00, 24, gyro_bits=16, accel_bits=16,
    ),
    "raw32": ModeProfile(
        "raw32", "2  原始 IMU 32-bit", "推荐的 RTK/INS 主模式",
        0xF007, 0x3000, 0x00, 36, gyro_bits=32, accel_bits=32,
    ),
    "delta32": ModeProfile(
        "delta32", "3  增量导航 32-bit", "自研捷联惯导：Delta Angle / Velocity",
        0xCC07, 0x0C00, 0x02, 36, delta_bits=32,
    ),
    "attitude16": ModeProfile(
        "attitude16", "4  欧拉姿态角 16-bit", "原始 IMU + Roll / Pitch / Yaw",
        0xF107, 0x0000, 0x0C, 30,
        gyro_bits=16, accel_bits=16, attitude_bits=16,
    ),
}

MODE_DATA_DETAILS = {
    "raw16": (
        "24字节：FLAG 16位；温度 16位；三轴角速度各16位；三轴加速度各16位；"
        "GPIO 16位；COUNT 16位；CHECKSUM 16位。"
    ),
    "raw32": (
        "36字节：FLAG 16位；温度 16位；三轴角速度各32位；三轴加速度各32位；"
        "GPIO 16位；COUNT 16位；CHECKSUM 16位。"
    ),
    "delta32": (
        "36字节：FLAG 16位；温度 16位；三轴增量角各32位；三轴增量速度各32位；"
        "GPIO 16位；COUNT 16位；CHECKSUM 16位。不发送原始角速度/加速度。"
    ),
    "attitude16": (
        "30字节：FLAG 16位；温度 16位；三轴角速度各16位；三轴加速度各16位；"
        "Roll/Pitch/Yaw各16位；GPIO 16位；COUNT 16位；CHECKSUM 16位。"
        "不发送四元数或增量量。"
    ),
}

# Factory/example format observed on some units: temperature, gyro and accel are
# all 32-bit. It is monitor-only and is deliberately not a fifth switch mode.
OBSERVED_RAW32_ALL = ModeProfile(
    "observed_raw32_all", "设备当前：原始 IMU 全 32-bit",
    "被动检测到的 38 字节格式", 0xF007, 0x7000, 0x00, 38,
    temp_bits=32, gyro_bits=32, accel_bits=32,
)


DOUT_RATES = {
    0x00: 2000.0, 0x01: 1000.0, 0x02: 500.0, 0x03: 250.0,
    0x04: 125.0, 0x05: 62.5, 0x06: 31.25, 0x07: 15.625,
    0x08: 400.0, 0x09: 200.0, 0x0A: 100.0, 0x0B: 80.0,
    0x0C: 50.0, 0x0D: 40.0, 0x0E: 25.0, 0x0F: 20.0,
}


# Valid DOUT_RATE / FILTER_SEL combinations when ATTI_ON == 10.
ATTITUDE_FILTERS = {
    0x02: {0x04, 0x05, 0x08, 0x09},
    0x08: {0x04, 0x05, 0x08, 0x09},
    0x03: {0x04, 0x05, 0x08, 0x09},
    0x09: {0x04, 0x05, 0x08, 0x09},
    0x04: {0x04, 0x05, 0x08},
    0x0A: {0x05, 0x08},
    0x0B: {0x05},
    0x05: {0x05},
}


ATTI_CONVERSIONS = [
    (0x00, "X前 / Y左 / Z上（FLU，标准安装）"),
    (0x01, "X前 / Z左 / -Y上"), (0x02, "X前 / -Y左 / -Z上"),
    (0x03, "X前 / -Z左 / Y上"), (0x04, "Y前 / Z左 / X上"),
    (0x05, "Y前 / X左 / -Z上"), (0x06, "Y前 / -Z左 / -X上"),
    (0x07, "Y前 / -X左 / Z上"), (0x08, "Z前 / X左 / Y上"),
    (0x09, "Z前 / Y左 / -X上"), (0x0A, "Z前 / -X左 / -Y上"),
    (0x0B, "Z前 / -Y左 / X上"), (0x0C, "-X前 / Y左 / -Z上"),
    (0x0D, "-X前 / -Z左 / -Y上"), (0x0E, "-X前 / -Y左 / Z上"),
    (0x0F, "-X前 / Z左 / Y上"), (0x10, "-Y前 / Z左 / -X上"),
    (0x11, "-Y前 / -X左 / -Z上"), (0x12, "-Y前 / -Z左 / X上"),
    (0x13, "-Y前 / X左 / Z上"), (0x14, "-Z前 / X左 / -Y上"),
    (0x15, "-Z前 / -Y左 / -X上"), (0x16, "-Z前 / -X左 / Y上"),
    (0x17, "-Z前 / Y左 / X上"),
]

MOTION_PROFILES = [
    (0, "modeA：通用，约 3 m/s"),
    (1, "modeB：车辆，约 20 m/s（推荐）"),
    (2, "modeC：工程机械，约 1 m/s"),
]

DRDY_FUNCTIONS = ("Data Ready（DRDY）", "GPIO1（DRDY关闭）")
DRDY_POLARITIES = ("高电平有效（Active High）", "低电平有效（Active Low）")
ND_ENABLE_MASK = 0xFEFC


class SerialError(RuntimeError):
    pass


class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
        ("flags", wintypes.DWORD), ("wReserved", wintypes.WORD),
        ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
        ("ByteSize", wintypes.BYTE), ("Parity", wintypes.BYTE),
        ("StopBits", wintypes.BYTE), ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
        ("wReserved1", wintypes.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [
        ("ReadIntervalTimeout", wintypes.DWORD),
        ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
        ("ReadTotalTimeoutConstant", wintypes.DWORD),
        ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
        ("WriteTotalTimeoutConstant", wintypes.DWORD),
    ]


class WindowsSerial:
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    FILE_ATTRIBUTE_NORMAL = 0x80
    PURGE_TXABORT = 0x0001
    PURGE_RXABORT = 0x0002
    PURGE_TXCLEAR = 0x0004
    PURGE_RXCLEAR = 0x0008

    def __init__(self, port: str, baud: int):
        self.port = port.upper().strip()
        self.baud = baud
        self.handle = None
        self.lock = threading.RLock()
        self.k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self._bind()

    def _bind(self):
        self.k32.CreateFileW.argtypes = [
            wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
            wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
        ]
        self.k32.CreateFileW.restype = wintypes.HANDLE
        self.k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
        self.k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(DCB)]
        self.k32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
        self.k32.SetupComm.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD]
        self.k32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        self.k32.ReadFile.argtypes = [
            wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID,
        ]
        self.k32.WriteFile.argtypes = [
            wintypes.HANDLE, wintypes.LPCVOID, wintypes.DWORD,
            ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID,
        ]
        self.k32.CloseHandle.argtypes = [wintypes.HANDLE]

    def _fail(self, action: str):
        code = ctypes.get_last_error()
        raise SerialError(f"{action}失败（Win32 错误 {code}）")

    @property
    def is_open(self):
        return self.handle not in (None, 0, wintypes.HANDLE(-1).value)

    def open(self):
        path = self.port if self.port.startswith("\\\\.\\") else "\\\\.\\" + self.port
        handle = self.k32.CreateFileW(
            path, self.GENERIC_READ | self.GENERIC_WRITE, 0, None,
            self.OPEN_EXISTING, self.FILE_ATTRIBUTE_NORMAL, None,
        )
        if handle == wintypes.HANDLE(-1).value:
            self._fail(f"打开 {self.port}")
        self.handle = handle
        try:
            self.k32.SetupComm(handle, 65536, 65536)
            dcb = DCB()
            dcb.DCBlength = ctypes.sizeof(DCB)
            if not self.k32.GetCommState(handle, ctypes.byref(dcb)):
                self._fail("读取串口参数")
            dcb.BaudRate = self.baud
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            dcb.flags = 0x00000001  # fBinary; no HW/SW flow control
            if not self.k32.SetCommState(handle, ctypes.byref(dcb)):
                self._fail("设置串口参数")
            timeouts = COMMTIMEOUTS(30, 0, 40, 0, 200)
            if not self.k32.SetCommTimeouts(handle, ctypes.byref(timeouts)):
                self._fail("设置串口超时")
            self.purge_all()
        except Exception:
            self.close()
            raise

    def close(self):
        with self.lock:
            if self.is_open:
                self.k32.CloseHandle(self.handle)
            self.handle = None

    def purge_all(self):
        with self.lock:
            if self.is_open:
                flags = self.PURGE_RXABORT | self.PURGE_RXCLEAR | self.PURGE_TXABORT | self.PURGE_TXCLEAR
                if not self.k32.PurgeComm(self.handle, flags):
                    self._fail("清空串口缓冲区")

    def purge_rx(self):
        with self.lock:
            if self.is_open and not self.k32.PurgeComm(
                self.handle, self.PURGE_RXABORT | self.PURGE_RXCLEAR
            ):
                self._fail("清空串口接收缓冲区")

    def write(self, data: bytes):
        with self.lock:
            if not self.is_open:
                raise SerialError("串口未连接")
            sent = wintypes.DWORD()
            buf = ctypes.create_string_buffer(data)
            if not self.k32.WriteFile(self.handle, buf, len(data), ctypes.byref(sent), None):
                self._fail("串口写入")
            if sent.value != len(data):
                raise SerialError(f"串口只写入 {sent.value}/{len(data)} 字节")

    def read(self, size=4096) -> bytes:
        with self.lock:
            if not self.is_open:
                return b""
            buf = ctypes.create_string_buffer(size)
            got = wintypes.DWORD()
            if not self.k32.ReadFile(self.handle, buf, size, ctypes.byref(got), None):
                self._fail("串口读取")
            return buf.raw[:got.value]


class G366Protocol:
    def __init__(self, port: WindowsSerial):
        self.port = port

    def write8(self, address: int, value: int):
        self.port.write(bytes([(address & 0x7F) | 0x80, value & 0xFF, CR]))
        time.sleep(0.001)

    def select_window(self, window: int):
        self.write8(0x7E, window)

    def _read_response(self, expected_address: int, timeout=0.35) -> int:
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.port.read(256)
            if chunk:
                buf.extend(chunk)
                while CR in buf:
                    end = buf.index(CR)
                    frame = bytes(buf[:end + 1])
                    del buf[:end + 1]
                    if len(frame) == 4 and frame[0] == expected_address:
                        return (frame[1] << 8) | frame[2]
            else:
                time.sleep(0.002)
        raise SerialError(f"读取寄存器 0x{expected_address:02X} 超时")

    def read16(self, even_address: int) -> int:
        address = even_address & 0x7E
        self.port.write(bytes([address, 0x00, CR]))
        return self._read_response(address)

    def enter_configuration(self):
        self.select_window(0)
        self.write8(0x03, 0x02)
        time.sleep(0.05)
        self.port.purge_rx()
        mode = self.read16(0x02)
        if not (mode & 0x0400):
            raise SerialError(f"设备未进入 Configuration Mode（MODE_CTRL=0x{mode:04X}）")

    def enter_sampling(self):
        self.select_window(0)
        self.write8(0x03, 0x01)
        time.sleep(0.03)

    def snapshot(self) -> dict[str, int]:
        self.select_window(1)
        regs = {
            "sig": self.read16(0x00),
            "msc": self.read16(0x02),
            "smpl": self.read16(0x04),
            "filter": self.read16(0x06),
            "uart": self.read16(0x08),
            "burst1": self.read16(0x0C),
            "burst2": self.read16(0x0E),
            "pol": self.read16(0x10),
            "glob3": self.read16(0x12),
            "atti": self.read16(0x14),
            "glob2": self.read16(0x16),
        }
        return regs

    def apply_profile(self, profile: ModeProfile, atti_conv: int, motion_profile: int):
        self.select_window(1)
        self.write8(0x0C, profile.burst1 & 0xFF)
        self.write8(0x0D, profile.burst1 >> 8)
        # BURST_CTRL2 low byte is reserved. Only write its documented high byte.
        self.write8(0x0F, profile.burst2 >> 8)
        self.write8(0x15, profile.atti_high)
        if profile.key == "attitude16":
            self.write8(0x14, atti_conv & 0x1F)
            self.write8(0x16, (motion_profile & 0x03) << 4)
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline:
                if not (self.read16(0x16) & 0x0040):
                    break
            else:
                raise SerialError("姿态运动模型设置超时")

    def restore_mode_registers(self, old: dict[str, int]):
        self.select_window(1)
        self.write8(0x0C, old["burst1"] & 0xFF)
        self.write8(0x0D, old["burst1"] >> 8)
        self.write8(0x0F, old["burst2"] >> 8)
        self.write8(0x14, old["atti"] & 0x1F)
        self.write8(0x15, (old["atti"] >> 8) & 0x0E)
        self.write8(0x16, old["glob2"] & 0x30)

    def configure_drdy(self, current_msc: int, enabled: bool, active_high: bool) -> int:
        """Update only MSC_CTRL low-byte DRDY bits and return its readback.

        EXT_SEL[7:6] is preserved, reserved bits are written as zero as required,
        and address 0x03 (FLASH_TEST/SELF_TEST) is never written.
        """
        new_low = current_msc & 0x00C0
        if enabled:
            new_low |= 0x04
        if active_high:
            new_low |= 0x02
        self.select_window(1)
        self.write8(0x02, new_low)
        return self.read16(0x02)

    def configure_delta_scale(self, angle: int, velocity: int) -> int:
        if type(angle) is not int or type(velocity) is not int or not (
            0 <= angle <= 15 and 0 <= velocity <= 15
        ):
            raise ValueError("增量比例代码必须为 0–15 的整数")
        self.select_window(1)
        # W1:0x13 is NOT written: preserve high-byte fields/reserved bits.
        self.write8(0x12, (angle << 4) | velocity)
        return self.read16(0x12)

    def restore_drdy(self, old_msc: int):
        """Restore the documented MSC_CTRL low-byte fields without touching 0x03."""
        self.select_window(1)
        self.write8(0x02, old_msc & 0x00C6)

    def flash_backup(self) -> int:
        """Save all registers marked ○ in Table 6.1 and return DIAG_STAT."""
        self.select_window(1)
        self.write8(0x0A, 0x08)  # GLOB_CMD.FLASH_BACKUP
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            status = self.read16(0x0A)
            if not (status & 0x0008):
                break
            time.sleep(0.01)
        else:
            raise SerialError("FLASH_BACKUP 超时（超过 800 ms）")

        self.select_window(0)
        diag = self.read16(0x04)
        if diag & 0x0001:
            raise SerialError(f"FLASH_BACKUP 失败：FLASH_BU_ERR=1，DIAG_STAT=0x{diag:04X}")
        return diag


def signed16(data: bytes) -> int:
    value = int.from_bytes(data, "big", signed=False)
    return value - 0x10000 if value & 0x8000 else value


def signed32(data: bytes) -> int:
    value = int.from_bytes(data, "big", signed=False)
    return value - 0x100000000 if value & 0x80000000 else value


class FrameDecoder:
    def __init__(self):
        self.profile: ModeProfile | None = None
        self.buffer = bytearray()
        self.accel_16g = False
        self.delta_angle_code = 0
        self.delta_velocity_code = 0

    def configure(self, profile: ModeProfile | None, glob3: int):
        self.profile = profile
        self.buffer.clear()
        self.accel_16g = bool(glob3 & 0x0100)
        self.delta_angle_code = (glob3 >> 4) & 0x0F
        self.delta_velocity_code = glob3 & 0x0F

    @staticmethod
    def checksum_ok(frame: bytes) -> bool:
        if len(frame) < 6 or frame[-1] != CR:
            return False
        expected = int.from_bytes(frame[-3:-1], "big")
        content = frame[1:-3]
        if len(content) % 2:
            return False
        total = sum(int.from_bytes(content[i:i + 2], "big") for i in range(0, len(content), 2))
        return (total & 0xFFFF) == expected

    def feed(self, chunk: bytes):
        if not self.profile:
            return [], 0
        self.buffer.extend(chunk)
        frames = []
        bad = 0
        length = self.profile.packet_length
        while True:
            try:
                start = self.buffer.index(ADDRESS)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < length:
                break
            candidate = bytes(self.buffer[:length])
            if candidate[-1] != CR or not self.checksum_ok(candidate):
                del self.buffer[0]
                bad += 1
                continue
            del self.buffer[:length]
            frames.append((candidate, self.decode(candidate)))
        return frames, bad

    def decode(self, frame: bytes) -> dict[str, float | int]:
        p = self.profile
        assert p is not None
        out: dict[str, float | int] = {}
        i = 1

        def u16():
            nonlocal i
            v = int.from_bytes(frame[i:i + 2], "big")
            i += 2
            return v

        def s16():
            nonlocal i
            v = signed16(frame[i:i + 2])
            i += 2
            return v

        def s32():
            nonlocal i
            v = signed32(frame[i:i + 4])
            i += 4
            return v

        out["flag"] = u16()
        if p.temp_bits == 32:
            out["temp_c"] = 25.0 + s32() * 0.00390625 / 65536.0
        else:
            out["temp_c"] = 25.0 + s16() * 0.00390625

        if p.gyro_bits:
            read = s16 if p.gyro_bits == 16 else s32
            divisor = 66.0 if p.gyro_bits == 16 else 66.0 * 65536.0
            for axis in "xyz":
                out[f"gyro_{axis}"] = read() / divisor

        if p.accel_bits:
            read = s16 if p.accel_bits == 16 else s32
            lsb_per_mg = 2.0 if self.accel_16g else 4.0
            divisor = lsb_per_mg * 1000.0
            if p.accel_bits == 32:
                divisor *= 65536.0
            for axis in "xyz":
                out[f"accel_{axis}"] = read() / divisor

        if p.delta_bits:
            angle_sf = 7.576e-6 * (2 ** self.delta_angle_code)
            velocity_sf = (2.452e-6 if self.accel_16g else 1.226e-6) * (2 ** self.delta_velocity_code)
            for axis in "xyz":
                raw = s32()
                out[f"delta_angle_{axis}"] = raw * angle_sf / 65536.0
                if raw in (-2147483648, 2147483647):
                    out["angle_saturated"] = 1
            for axis in "xyz":
                raw = s32()
                out[f"delta_velocity_{axis}"] = raw * velocity_sf / 65536.0
                if raw in (-2147483648, 2147483647):
                    out["velocity_saturated"] = 1

        if p.attitude_bits:
            for name in ("roll", "pitch", "yaw"):
                out[name] = s16() * 0.00699411

        out["gpio"] = u16()
        out["count"] = u16()
        out["checksum"] = u16()
        return out


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.total = 0
        self.bad = 0
        self.latest = {}
        self.raw_hex = ""
        self.times = deque()
        self.angle_saturated = self.velocity_saturated = self.flag_errors = 0

    def reset(self):
        with self.lock:
            self.total = 0
            self.bad = 0
            self.latest = {}
            self.raw_hex = ""
            self.times.clear()
            self.angle_saturated = self.velocity_saturated = self.flag_errors = 0

    def add(self, frames, bad):
        now = time.monotonic()
        with self.lock:
            self.bad += bad
            for raw, decoded in frames:
                self.total += 1
                self.angle_saturated += bool(decoded.get("angle_saturated"))
                self.velocity_saturated += bool(decoded.get("velocity_saturated"))
                self.flag_errors += bool(decoded.get("flag", 0) & 0x0101)
                self.latest = decoded
                self.raw_hex = raw.hex(" ").upper()
                self.times.append(now)
            while self.times and self.times[0] < now - 1.0:
                self.times.popleft()

    def snapshot(self):
        with self.lock:
            return self.total, self.bad, len(self.times), dict(self.latest), self.raw_hex

    def health_snapshot(self):
        with self.lock:
            return self.angle_saturated, self.velocity_saturated, self.flag_errors


class HostBiasCalibrator:
    """Collect stationary decoded samples without writing anything to the IMU."""

    NAMES = (
        "gyro_x", "gyro_y", "gyro_z",
        "accel_x", "accel_y", "accel_z", "temp_c",
    )

    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.started = 0.0
        self.duration = 0.0
        self.count = 0
        self.sums = {name: 0.0 for name in self.NAMES}
        self.sumsq = {name: 0.0 for name in self.NAMES}
        self.result = None

    def start(self, duration: float):
        with self.lock:
            self.active = True
            self.started = time.monotonic()
            self.duration = duration
            self.count = 0
            self.sums = {name: 0.0 for name in self.NAMES}
            self.sumsq = {name: 0.0 for name in self.NAMES}
            self.result = None

    def cancel(self):
        with self.lock:
            self.active = False
            self.result = None

    def add(self, decoded: dict):
        with self.lock:
            if not self.active or not all(name in decoded for name in self.NAMES):
                return
            for name in self.NAMES:
                value = float(decoded[name])
                self.sums[name] += value
                self.sumsq[name] += value * value
            self.count += 1
            now = time.monotonic()
            if now - self.started >= self.duration:
                means = {name: self.sums[name] / self.count for name in self.NAMES}
                std = {}
                for name in self.NAMES:
                    if self.count > 1:
                        variance = max(
                            0.0,
                            (self.sumsq[name] - self.sums[name] ** 2 / self.count)
                            / (self.count - 1),
                        )
                        std[name] = variance ** 0.5
                    else:
                        std[name] = float("inf")
                self.result = {
                    "count": self.count,
                    "duration": now - self.started,
                    "mean": means,
                    "std": std,
                }
                self.active = False

    def status(self):
        with self.lock:
            elapsed = time.monotonic() - self.started if self.active else 0.0
            return self.active, elapsed, self.duration, self.count

    def take_result(self):
        with self.lock:
            result = self.result
            self.result = None
            return result


def available_ports():
    ports = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            i = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, i)
                    ports.append(str(value).upper())
                    i += 1
                except OSError:
                    break
    except OSError:
        pass
    return sorted(set(ports), key=lambda x: (len(x), x))


def identify_mode(regs: dict[str, int]) -> ModeProfile | None:
    burst1 = regs["burst1"] & 0xFF07
    burst2 = regs["burst2"] & 0x7F00
    atti_high = (regs["atti"] >> 8) & 0x0E
    for mode in MODES.values():
        if burst1 == mode.burst1 and burst2 == mode.burst2 and atti_high == mode.atti_high:
            return mode
    return None


def detect_passive_mode(data: bytes) -> tuple[ModeProfile | None, bytes | None]:
    """Recognize an unambiguous known packet from a receive-only stream."""
    candidates = list(MODES.values()) + [OBSERVED_RAW32_ALL]
    for profile in candidates:
        length = profile.packet_length
        for start in range(0, max(0, len(data) - length + 1)):
            frame = data[start:start + length]
            if frame[0] == ADDRESS and frame[-1] == CR and FrameDecoder.checksum_ok(frame):
                # 36 bytes is intentionally ambiguous between raw32 and delta32.
                if length == 36:
                    return None, frame
                return profile, frame
    return None, None


class G366App:
    PROTECTED = ("smpl", "filter", "uart", "msc", "pol", "glob3")

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Epson M-G366PDG0 四模式上位机")
        self.root.geometry("1200x960")
        self.root.minsize(1020, 800)
        self.events = queue.Queue()
        self.port: WindowsSerial | None = None
        self.protocol: G366Protocol | None = None
        self.reader_thread: threading.Thread | None = None
        self.reader_stop = threading.Event()
        self.reader_pause = threading.Event()
        self.reader_io_lock = threading.Lock()
        self.busy_lock = threading.Lock()
        self.decoder = FrameDecoder()
        self.monitor = MonitorState()
        self.calibrator = HostBiasCalibrator()
        self.gyro_bias = {axis: 0.0 for axis in "xyz"}
        self.current_regs: dict[str, int] = {}
        self.current_mode: ModeProfile | None = None
        self.connected = False
        self.tx_ready = False
        self._build_ui()
        self._refresh_ports()
        self.root.after(100, self._poll_ui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Mode.TLabelframe.Label", font=("Microsoft YaHei UI", 10, "bold"))
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="M-G366PDG0 四模式上位机", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            outer,
            text="模式与DRDY分开配置；不改采样率/滤波器/波特率；零偏仅在上位机计算；Flash须单独确认。",
            foreground="#555555",
        ).pack(anchor="w", pady=(2, 10))

        connection = ttk.LabelFrame(outer, text="连接", padding=8)
        connection.pack(fill="x")
        ttk.Label(connection, text="串口").grid(row=0, column=0, padx=(0, 5))
        self.port_var = tk.StringVar(value="COM4")
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, width=10)
        self.port_combo.grid(row=0, column=1)
        ttk.Button(connection, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Label(connection, text="波特率 230400 · 8N1 · 无流控").grid(row=0, column=3, padx=15)
        self.connect_btn = ttk.Button(connection, text="连接", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5)
        self.burst_btn = ttk.Button(connection, text="手动请求一帧", command=self._request_burst, state="disabled")
        self.burst_btn.grid(row=0, column=5, padx=5)
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(connection, textvariable=self.status_var, foreground="#005A9C").grid(row=0, column=6, padx=15, sticky="w")
        connection.columnconfigure(6, weight=1)

        body = ttk.Panedwindow(outer, orient="horizontal")
        body.pack(fill="both", expand=True, pady=10)
        controls_pane = ttk.Frame(body, padding=(0, 0, 6, 0))
        right = ttk.Frame(body, padding=(6, 0, 0, 0))
        body.add(controls_pane, weight=2)
        body.add(right, weight=3)
        # Configuration can grow vertically without hiding the live monitor.
        controls_canvas = tk.Canvas(controls_pane, width=510, highlightthickness=0)
        controls_scroll = ttk.Scrollbar(controls_pane, orient="vertical",
                                        command=controls_canvas.yview)
        controls_scroll.pack(side="right", fill="y")
        controls_canvas.pack(side="left", fill="both", expand=True)
        controls_canvas.configure(yscrollcommand=controls_scroll.set)
        left = ttk.Frame(controls_canvas)
        controls_window = controls_canvas.create_window((0, 0), window=left, anchor="nw")
        left.bind("<Configure>", lambda event: controls_canvas.configure(
            scrollregion=controls_canvas.bbox("all")))
        controls_canvas.bind("<Configure>", lambda event: controls_canvas.itemconfigure(
            controls_window, width=event.width))

        modes_frame = ttk.LabelFrame(left, text="输出模式", padding=8, style="Mode.TLabelframe")
        modes_frame.pack(fill="x")
        self.mode_var = tk.StringVar(value="raw16")
        for row, mode in enumerate(MODES.values()):
            ttk.Radiobutton(
                modes_frame, text=mode.title, value=mode.key, variable=self.mode_var,
                command=self._update_mode_details,
            ).grid(row=row * 2, column=0, sticky="w", pady=(4, 0))
            ttk.Label(modes_frame, text=f"{mode.description}；{mode.packet_length} 字节/帧", foreground="#666666").grid(
                row=row * 2 + 1, column=0, sticky="w", padx=(24, 0), pady=(0, 3)
            )
        ttk.Separator(modes_frame).grid(row=8, column=0, sticky="ew", pady=(6, 5))
        self.mode_details_var = tk.StringVar(value=MODE_DATA_DETAILS["raw16"])
        ttk.Label(
            modes_frame, textvariable=self.mode_details_var, wraplength=400,
            justify="left", foreground="#1F4E79",
        ).grid(row=9, column=0, sticky="ew")
        modes_frame.columnconfigure(0, weight=1)

        attitude = ttk.LabelFrame(left, text="姿态角模式选项", padding=8)
        attitude.pack(fill="x", pady=8)
        ttk.Label(attitude, text="安装方向").grid(row=0, column=0, sticky="w")
        self.conv_var = tk.StringVar(value=ATTI_CONVERSIONS[0][1])
        self.conv_combo = ttk.Combobox(
            attitude, textvariable=self.conv_var,
            values=[label for _, label in ATTI_CONVERSIONS], state="readonly", width=34,
        )
        self.conv_combo.grid(row=1, column=0, sticky="ew", pady=(2, 7))
        ttk.Label(attitude, text="运动模型").grid(row=2, column=0, sticky="w")
        self.motion_var = tk.StringVar(value=MOTION_PROFILES[1][1])
        self.motion_combo = ttk.Combobox(
            attitude, textvariable=self.motion_var,
            values=[label for _, label in MOTION_PROFILES], state="readonly", width=34,
        )
        self.motion_combo.grid(row=3, column=0, sticky="ew", pady=(2, 0))
        attitude.columnconfigure(0, weight=1)

        self.apply_btn = ttk.Button(left, text="切换到所选模式", command=self._switch_mode, state="disabled")
        self.apply_btn.pack(fill="x", pady=(0, 5))

        drdy = ttk.LabelFrame(left, text="DRDY / GPIO1 独立配置", padding=8)
        drdy.pack(fill="x", pady=(0, 8))
        ttk.Label(drdy, text="GPIO1引脚功能").grid(row=0, column=0, sticky="w")
        ttk.Label(drdy, text="DRDY极性").grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.drdy_function_var = tk.StringVar(value=DRDY_FUNCTIONS[0])
        self.drdy_function_combo = ttk.Combobox(
            drdy, textvariable=self.drdy_function_var, values=DRDY_FUNCTIONS,
            state="readonly", width=20,
        )
        self.drdy_function_combo.grid(row=1, column=0, sticky="ew", pady=(2, 5))
        self.drdy_polarity_var = tk.StringVar(value=DRDY_POLARITIES[0])
        self.drdy_polarity_combo = ttk.Combobox(
            drdy, textvariable=self.drdy_polarity_var, values=DRDY_POLARITIES,
            state="readonly", width=24,
        )
        self.drdy_polarity_combo.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=(2, 5))
        self.drdy_apply_btn = ttk.Button(
            drdy, text="应用DRDY配置（不保存Flash）",
            command=self._apply_drdy, state="disabled",
        )
        self.drdy_apply_btn.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.drdy_status_var = tk.StringVar(value="等待读取DRDY配置")
        ttk.Label(
            drdy, textvariable=self.drdy_status_var, wraplength=410,
            justify="left", foreground="#1F4E79",
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(5, 0))
        drdy.columnconfigure(0, weight=1)
        drdy.columnconfigure(1, weight=1)

        self.flash_btn = ttk.Button(
            left, text="保存当前全部可备份设置到 Flash…",
            command=self._save_flash, state="disabled",
        )
        self.flash_btn.pack(fill="x", pady=(0, 8))

        calibration = ttk.LabelFrame(left, text="上位机陀螺零偏校准（IMU保持静止）", padding=8)
        calibration.pack(fill="x", pady=(0, 8))
        controls = ttk.Frame(calibration)
        controls.pack(fill="x")
        ttk.Label(controls, text="采样时间").pack(side="left")
        self.calib_duration_var = tk.StringVar(value="10")
        ttk.Combobox(
            controls, textvariable=self.calib_duration_var,
            values=("3", "5", "10", "20", "30"), state="readonly", width=5,
        ).pack(side="left", padx=(5, 3))
        ttk.Label(controls, text="秒").pack(side="left")
        self.calib_btn = ttk.Button(
            controls, text="开始静止校准", command=self._start_bias_calibration,
            state="disabled",
        )
        self.calib_btn.pack(side="left", padx=(12, 4))
        self.clear_calib_btn = ttk.Button(
            controls, text="清除", command=self._clear_host_bias, state="disabled",
        )
        self.clear_calib_btn.pack(side="left")
        self.calib_var = tk.StringVar(
            value="未校准。只计算陀螺XYZ零偏；加速度静止均值仅显示，不扣除重力。"
        )
        ttk.Label(
            calibration, textvariable=self.calib_var, wraplength=410,
            justify="left", foreground="#704214",
        ).pack(anchor="w", pady=(6, 0))
        self.config_var = tk.StringVar(value="等待读取设备配置")
        config = ttk.LabelFrame(left, text="设备配置（实际读回）", padding=8)
        config.pack(fill="x", pady=(0, 8))
        ttk.Label(config, textvariable=self.config_var, wraplength=500, justify="left").pack(anchor="w")
        ttk.Button(config, text="复制配置", command=self._copy_config).pack(anchor="e", pady=(4, 0))

        delta = ttk.LabelFrame(left, text="G365PDF1 增量比例（独立配置）", padding=8)
        delta.pack(fill="x", pady=(0, 8))
        self.delta_angle_var = tk.StringVar(value="0")
        self.delta_velocity_var = tk.StringVar(value="8")
        for column, (label, variable) in enumerate((("角增量代码", self.delta_angle_var),
                                                    ("速度增量代码", self.delta_velocity_var))):
            ttk.Label(delta, text=label).grid(row=0, column=column * 2, sticky="w")
            combo = ttk.Combobox(delta, textvariable=variable, values=tuple(range(16)),
                                 state="readonly", width=4)
            combo.grid(row=0, column=column * 2 + 1, padx=(3, 8))
            combo.bind("<<ComboboxSelected>>", lambda event: self._delta_preview())
        self.delta_preview_var = tk.StringVar()
        ttk.Label(delta, textvariable=self.delta_preview_var, wraplength=500,
                  foreground="#1F4E79").grid(row=1, column=0, columnspan=4, sticky="w", pady=4)
        buttons = ttk.Frame(delta)
        buttons.grid(row=2, column=0, columnspan=4, sticky="ew")
        ttk.Button(buttons, text="填入推荐值 4 / 8", command=self._delta_recommended).pack(side="left")
        self.delta_apply_btn = ttk.Button(buttons, text="应用比例（不保存Flash）",
                                         command=self._apply_delta, state="disabled")
        self.delta_apply_btn.pack(side="left", padx=4)
        self.delta_read_btn = ttk.Button(buttons, text="重新读取",
                                        command=lambda: self._apply_delta(read_only=True), state="disabled")
        self.delta_read_btn.pack(side="left")
        self.delta_status_var = tk.StringVar(value="等待读回；推荐值仅填入，不自动写设备。")
        ttk.Label(delta, textvariable=self.delta_status_var, wraplength=500).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self._delta_preview()

        live = ttk.LabelFrame(right, text="实时数据与操作日志", padding=8)
        live.pack(fill="both", expand=True)
        self.stats_var = tk.StringVar(value="帧 0 · 校验错误 0 · 0 Hz")
        ttk.Label(live, textvariable=self.stats_var).pack(anchor="w", pady=(0, 8))
        self.health_var = tk.StringVar(value="角增量饱和 0 · 速度增量饱和 0 · 错误FLAG 0")
        ttk.Label(live, textvariable=self.health_var, foreground="#B03020",
                  wraplength=500).pack(anchor="w", pady=(0, 4))
        self.tree = ttk.Treeview(live, columns=("x", "y", "z", "unit"), show="tree headings", height=8)
        self.tree.heading("#0", text="数据")
        self.tree.column("#0", width=105, anchor="w")
        for name, text, width in (("x", "X / Roll", 115), ("y", "Y / Pitch", 115), ("z", "Z / Yaw", 115), ("unit", "单位", 80)):
            self.tree.heading(name, text=text)
            self.tree.column(name, width=width, anchor="center")
        self.tree.pack(fill="x")
        self.tree.insert("", "end", iid="gyro_raw", text="陀螺 原始", values=("—", "—", "—", "°/s"))
        self.tree.insert("", "end", iid="gyro_cal", text="陀螺 校准后", values=("—", "—", "—", "°/s"))
        self.tree.insert("", "end", iid="accel", text="加速度 原始", values=("—", "—", "—", "g"))
        self.tree.insert("", "end", iid="dangle", text="增量角", values=("—", "—", "—", "°"))
        self.tree.insert("", "end", iid="dvelocity", text="增量速度", values=("—", "—", "—", "m/s"))
        self.tree.insert("", "end", iid="attitude", text="姿态角", values=("—", "—", "—", "°"))

        self.misc_var = tk.StringVar(value="温度 —   COUNT —   FLAG —   GPIO —")
        ttk.Label(live, textvariable=self.misc_var).pack(anchor="w", pady=8)
        ttk.Label(live, text="最新有效帧（HEX）").pack(anchor="w")
        self.hex_text = tk.Text(live, height=4, wrap="word", state="disabled", font=("Consolas", 9))
        self.hex_text.pack(fill="x", pady=(3, 8))
        ttk.Label(live, text="操作 / 运行日志").pack(anchor="w")
        log_frame = ttk.Frame(live)
        log_frame.pack(fill="both", expand=True, pady=(3, 0))
        self.log_text = tk.Text(log_frame, height=10, wrap="word", state="disabled", font=("Consolas", 9))
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)

        def scroll_controls(event):
            controls_canvas.yview_scroll(-int(event.delta / 120), "units")
            return "break"

        def bind_controls_scroll(widget):
            if widget.winfo_class() != "TCombobox":
                widget.bind("<MouseWheel>", scroll_controls)
            for child in widget.winfo_children():
                bind_controls_scroll(child)

        bind_controls_scroll(left)
        controls_canvas.bind("<MouseWheel>", scroll_controls)

    def _refresh_ports(self):
        ports = available_ports()
        self.port_combo["values"] = ports
        if "COM4" in ports:
            self.port_var.set("COM4")

    def _update_mode_details(self):
        self.mode_details_var.set(MODE_DATA_DETAILS[self.mode_var.get()])

    def _log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {text}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _post(self, kind, payload=None):
        self.events.put((kind, payload))

    def _async(self, func):
        threading.Thread(target=func, daemon=True).start()

    def _toggle_connection(self):
        if self.busy_lock.locked():
            return
        if self.connected:
            self._disconnect()
        else:
            port_name = self.port_var.get().strip().upper()
            if not port_name:
                messagebox.showerror("连接失败", "请输入串口号。")
                return
            self.connect_btn.configure(state="disabled")
            self.status_var.set("正在连接…")
            self._async(lambda: self._connect_worker(port_name))

    def _connect_worker(self, port_name):
        if not self.busy_lock.acquire(blocking=False):
            return
        port = None
        try:
            port = WindowsSerial(port_name, BAUD_RATE)
            port.open()
            protocol = G366Protocol(port)
            warning = None
            tx_ready = True
            frame = None
            try:
                protocol.enter_configuration()
                regs = protocol.snapshot()
                protocol.enter_sampling()
                mode = identify_mode(regs)
                glob3 = regs["glob3"]
            except Exception as handshake_exc:
                # Keep the receive monitor useful when PC TX -> IMU SIN is not
                # connected. Do not attempt blind register writes.
                tx_ready = False
                regs = {}
                port.purge_rx()
                captured = bytearray()
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline and len(captured) < 8192:
                    chunk = port.read(4096)
                    if chunk:
                        captured.extend(chunk)
                mode, frame = detect_passive_mode(bytes(captured))
                if frame is None:
                    raise SerialError(
                        f"寄存器握手失败，且没有检测到有效数据帧：{handshake_exc}"
                    )
                glob3 = 0x00CC  # factory default, used only for approximate display
                warning = (
                    f"只能接收数据，IMU 未响应写入/寄存器命令：{handshake_exc}。"
                    "请检查 USB-UART TX -> IMU SIN 与共地；修复前已禁用模式切换。"
                )
            self.port = port
            self.protocol = protocol
            self.current_regs = regs
            self.current_mode = mode
            self.tx_ready = tx_ready
            self.decoder.configure(mode, glob3)
            self.monitor.reset()
            if not tx_ready and frame is not None and mode is not None:
                frames, bad = self.decoder.feed(frame)
                self.monitor.add(frames, bad)
            self.reader_stop.clear()
            self.reader_pause.clear()
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            self._post("connected", (port_name, warning, tx_ready))
        except Exception as exc:
            if port:
                port.close()
            self.port = None
            self.protocol = None
            self._post("error", f"连接失败：{exc}")
        finally:
            self.busy_lock.release()

    def _disconnect(self):
        self.reader_stop.set()
        self.reader_pause.clear()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.3)
        if self.port:
            self.port.close()
        self.port = None
        self.protocol = None
        self.connected = False
        self.tx_ready = False
        self.current_mode = None
        self.current_regs = {}
        self.delta_apply_btn.configure(state="disabled")
        self.delta_read_btn.configure(state="disabled")
        self.delta_status_var.set("未连接；待重新读回配置。")
        self.calibrator.cancel()
        self.gyro_bias = {axis: 0.0 for axis in "xyz"}
        self.apply_btn.configure(state="disabled")
        self.flash_btn.configure(state="disabled")
        self.drdy_apply_btn.configure(state="disabled")
        self.burst_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.clear_calib_btn.configure(state="disabled")
        self.calib_var.set("未校准。只计算陀螺XYZ零偏；加速度静止均值仅显示，不扣除重力。")
        self.connect_btn.configure(text="连接", state="normal")
        self.status_var.set("未连接")
        self.drdy_status_var.set("等待读取DRDY配置")
        self._log("串口已断开。")

    def _reader_loop(self):
        while not self.reader_stop.is_set():
            if self.reader_pause.is_set():
                time.sleep(0.005)
                continue
            try:
                with self.reader_io_lock:
                    if self.reader_pause.is_set():
                        continue
                    chunk = self.port.read(4096) if self.port else b""
                    if chunk:
                        frames, bad = self.decoder.feed(chunk)
                        self.monitor.add(frames, bad)
                        for _, decoded in frames:
                            self.calibrator.add(decoded)
            except Exception as exc:
                if not self.reader_stop.is_set():
                    self._post("error", f"接收失败：{exc}")
                return

    def _request_burst(self):
        if not self.port or not self.connected or self.busy_lock.locked():
            return
        try:
            self.port.write(bytes([0x80, 0x00, CR]))
            self._log("已发送一次 UART Burst 请求。")
        except Exception as exc:
            messagebox.showerror("发送失败", str(exc))

    def _selected_conversion(self):
        label = self.conv_var.get()
        return next(value for value, text in ATTI_CONVERSIONS if text == label)

    def _selected_motion(self):
        label = self.motion_var.get()
        return next(value for value, text in MOTION_PROFILES if text == label)

    def _mode_has_gyro(self):
        return bool(self.current_mode and self.current_mode.gyro_bits)

    def _delta_preview(self):
        angle, velocity = int(self.delta_angle_var.get()), int(self.delta_velocity_var.get())
        high = self.current_regs.get("glob3", 0) & 0xFF00
        rate = DOUT_RATES.get((self.current_regs.get("smpl", 0xFF00) >> 8) & 255)
        self.delta_preview_var.set(
            f"待写入：0x{high | angle << 4 | velocity:04X}（高字节保持实际值）\n"
            + delta_range_text(angle, rate))

    def _delta_recommended(self):
        self.delta_angle_var.set("4")
        self.delta_velocity_var.set("8")
        self._delta_preview()

    def _sync_delta_ui(self):
        if self.current_regs:
            value = self.current_regs["glob3"]
            self.delta_angle_var.set(str((value >> 4) & 15))
            self.delta_velocity_var.set(str(value & 15))
        self._delta_preview()

    def _apply_delta(self, read_only=False):
        if not self.connected or not self.tx_ready or self.busy_lock.locked():
            return
        angle, velocity = int(self.delta_angle_var.get()), int(self.delta_velocity_var.get())
        self.calibrator.cancel()
        self.gyro_bias = {axis: 0.0 for axis in "xyz"}
        self.clear_calib_btn.configure(state="disabled")
        self.calib_var.set("配置操作已清除上位机标定；完成后请重新静止标定。板端接回后也须重新标定。")
        for button in (self.apply_btn, self.flash_btn, self.drdy_apply_btn, self.calib_btn,
                       self.delta_apply_btn, self.delta_read_btn, self.connect_btn, self.burst_btn):
            button.configure(state="disabled")
        self.status_var.set("正在读取增量配置…" if read_only else "正在应用增量比例…")
        self._async(lambda: self._delta_worker(None if read_only else (angle, velocity)))

    def _delta_worker(self, codes):
        if not self.busy_lock.acquire(blocking=False):
            self._post("delta_result", (False, "设备正在执行其他操作，未写入。"))
            return
        self.reader_pause.set()
        before = None
        attempted = False
        try:
            with self.reader_io_lock:
                try:
                    assert self.port and self.protocol
                    self.port.purge_rx()
                    self.protocol.enter_configuration()
                    before = self.protocol.snapshot()
                    expected = dict(before)
                    if codes is not None:
                        expected["glob3"] = (before["glob3"] & 0xFF00) | (codes[0] << 4) | codes[1]
                        attempted = True  # A failed write may still reach the sensor.
                        value = self.protocol.configure_delta_scale(*codes)
                        if value != expected["glob3"]:
                            raise SerialError(f"DLT_CTRL 回读不一致：0x{value:04X}")
                    after = self.protocol.snapshot()
                    if after != expected:
                        raise SerialError("配置快照不一致：比例或其他寄存器发生意外变化")
                    self.protocol.enter_sampling()
                    self.current_regs = after
                    self.current_mode = identify_mode(after)
                    self.decoder.configure(self.current_mode, after["glob3"])
                    self.monitor.reset()
                    self.port.purge_rx()
                    action = "已读回" if codes is None else "已应用，尚未保存Flash"
                    self._post("delta_result", (True,
                        f"{action}：DLT_CTRL=0x{after['glob3']:04X}。板端和采集参数须与此值一致。"))
                except Exception as exc:
                    try:
                        if before is None:
                            raise SerialError("未取得完整配置快照")
                        self.protocol.enter_configuration()
                        if attempted:
                            self.protocol.configure_delta_scale((before["glob3"] >> 4) & 15,
                                                                before["glob3"] & 15)
                        restored = self.protocol.snapshot()
                        if restored != before:
                            raise SerialError("恢复读回与原配置不一致")
                        self.protocol.enter_sampling()
                        self.current_regs = restored
                        self.current_mode = identify_mode(restored)
                        self.decoder.configure(self.current_mode, restored["glob3"])
                        self.monitor.reset()
                        self.port.purge_rx()
                        detail = "已读回确认原配置并恢复采样。"
                    except Exception as restore_exc:
                        self.tx_ready = False
                        self.current_regs = {}
                        self.current_mode = None
                        self.decoder.configure(None, 0)
                        self.monitor.reset()
                        detail = f"无法确认设备状态：{restore_exc}；已停用解码和写入，请重新连接。"
                    self._post("delta_result", (False, f"操作失败：{exc}；{detail}"))
        finally:
            self.reader_pause.clear()
            self.busy_lock.release()

    def _sync_drdy_ui(self):
        self._sync_delta_ui()
        if not self.current_regs:
            self.drdy_status_var.set("寄存器不可读；DRDY配置已禁用。")
            return
        msc = self.current_regs["msc"]
        enabled = bool(msc & 0x0004)
        active_high = bool(msc & 0x0002)
        self.drdy_function_var.set(DRDY_FUNCTIONS[0 if enabled else 1])
        self.drdy_polarity_var.set(DRDY_POLARITIES[0 if active_high else 1])
        pin_text = "Data Ready" if enabled else "GPIO1"
        polarity_text = "高有效" if active_high else "低有效"
        ext_sel = (msc >> 6) & 0x03
        warning = ""
        if enabled and not (self.current_regs.get("sig", 0) & ND_ENABLE_MASK):
            warning = "；警告：全部ND_EN均关闭，DRDY不会触发"
        self.drdy_status_var.set(
            f"当前：{pin_text}，{polarity_text}；EXT_SEL={ext_sel:02b} 已保护；"
            f"MSC_CTRL=0x{msc:04X}{warning}。应用后不会自动写Flash。"
        )

    def _start_bias_calibration(self):
        if not self.connected or not self._mode_has_gyro():
            messagebox.showwarning("无法校准", "当前数据包没有原始三轴角速度，请切换到原始IMU或姿态角模式。")
            return
        if self.current_regs and not (self.current_regs.get("uart", 0) & 0x0001):
            messagebox.showwarning("无法校准", "设备当前是UART Manual模式，没有连续数据流。")
            return
        duration = float(self.calib_duration_var.get())
        self.calibrator.start(duration)
        self.calib_btn.configure(state="disabled")
        self.apply_btn.configure(state="disabled")
        self.flash_btn.configure(state="disabled")
        self.drdy_apply_btn.configure(state="disabled")
        self.calib_var.set(f"正在采集静止数据：0.0/{duration:g} 秒，样本 0。请勿移动IMU。")
        self._log(f"开始上位机陀螺零偏校准：{duration:g}秒。此操作不向IMU写入任何校准值。")

    def _clear_host_bias(self):
        self.calibrator.cancel()
        self.gyro_bias = {axis: 0.0 for axis in "xyz"}
        self.calib_var.set("已清除上位机零偏；校准后值现在等于原始值。")
        self.calib_btn.configure(state="normal" if self.connected and self._mode_has_gyro() else "disabled")
        self.clear_calib_btn.configure(state="disabled")
        if self.tx_ready:
            self.apply_btn.configure(state="normal")
            self.flash_btn.configure(state="normal")
            self.drdy_apply_btn.configure(state="normal")
        self._log("已清除上位机陀螺零偏，不影响IMU内部设置。")

    def _finish_bias_calibration(self, result):
        count = result["count"]
        mean = result["mean"]
        std = result["std"]
        min_samples = 50
        gyro_noise = max(std[f"gyro_{axis}"] for axis in "xyz")
        accel_noise = max(std[f"accel_{axis}"] for axis in "xyz")
        if count < min_samples or gyro_noise > 0.10 or accel_noise > 0.03:
            reason = (
                f"样本{count}，陀螺最大标准差{gyro_noise:.5f}°/s，"
                f"加速度最大标准差{accel_noise:.5f}g。检测到移动/振动或样本不足，未应用新零偏。"
            )
            self.calib_var.set("校准失败：" + reason)
            self._log("零偏校准失败：" + reason)
            messagebox.showwarning("零偏校准失败", reason)
        else:
            self.gyro_bias = {axis: mean[f"gyro_{axis}"] for axis in "xyz"}
            accel_mean = tuple(mean[f"accel_{axis}"] for axis in "xyz")
            self.calib_var.set(
                "校准完成："
                f"原始静止均值/零偏 X={self.gyro_bias['x']:+.7f}, "
                f"Y={self.gyro_bias['y']:+.7f}, Z={self.gyro_bias['z']:+.7f} °/s；"
                "校准后静止均值≈(0,0,0) °/s。"
                f" 样本{count}，温度{mean['temp_c']:.2f}°C；"
                f"加速度静止均值=({accel_mean[0]:+.5f},{accel_mean[1]:+.5f},{accel_mean[2]:+.5f})g（未扣除）。"
            )
            self._log(
                "上位机零偏已应用："
                f"X={self.gyro_bias['x']:+.7f}, Y={self.gyro_bias['y']:+.7f}, "
                f"Z={self.gyro_bias['z']:+.7f} °/s；样本{count}。"
            )
            self.clear_calib_btn.configure(state="normal")
        self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
        if self.tx_ready:
            self.apply_btn.configure(state="normal")
            self.flash_btn.configure(state="normal")
            self.drdy_apply_btn.configure(state="normal")

    def _apply_drdy(self):
        if not self.connected or not self.tx_ready:
            return
        enabled = self.drdy_function_var.get() == DRDY_FUNCTIONS[0]
        active_high = self.drdy_polarity_var.get() == DRDY_POLARITIES[0]
        if self.calibrator.status()[0]:
            self.calibrator.cancel()
            self.calib_var.set("DRDY配置已取消正在进行的零偏校准。")
        self.apply_btn.configure(state="disabled")
        self.flash_btn.configure(state="disabled")
        self.drdy_apply_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.status_var.set("正在应用独立DRDY配置…")
        self._async(lambda: self._drdy_worker(enabled, active_high))

    def _switch_mode(self):
        if not self.connected or not self.tx_ready:
            return
        profile = MODES[self.mode_var.get()]
        if self.calibrator.status()[0]:
            self.calibrator.cancel()
            self.calib_var.set("模式切换已取消正在进行的零偏校准。")
        self.apply_btn.configure(state="disabled")
        self.flash_btn.configure(state="disabled")
        self.drdy_apply_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.status_var.set(f"正在切换：{profile.title}…")
        conv = self._selected_conversion()
        motion = self._selected_motion()
        self._async(lambda: self._switch_worker(profile, conv, motion))

    def _save_flash(self):
        if not self.connected or not self.tx_ready:
            return
        confirmed = messagebox.askyesno(
            "确认写入 Flash",
            "FLASH_BACKUP 会把所有带 ○ 标记的当前寄存器一起写入非易失存储器，\n"
            "包括当前采样率、滤波器、波特率、UART Auto/Auto Start、数据包、量程和姿态设置。\n\n"
            "上位机不会改变这些受保护设置，但会把它们的当前值一起保存。\n"
            "GLOB_CMD2 的姿态运动模型不属于普通 FLASH_BACKUP 范围，断电后需由上位机重新应用。\n\n"
            "保存后还会读取 DIAG_STAT 检查 FLASH_BU_ERR；该读取会清除已锁存的诊断标志。\n"
            "不要频繁重复写入。是否继续？",
            icon="warning",
        )
        if not confirmed:
            return
        self.apply_btn.configure(state="disabled")
        self.flash_btn.configure(state="disabled")
        self.drdy_apply_btn.configure(state="disabled")
        self.calib_btn.configure(state="disabled")
        self.status_var.set("正在保存当前设置到 Flash…")
        self._async(self._flash_worker)

    @staticmethod
    def _preflight(profile: ModeProfile, regs: dict[str, int]):
        rate_code = (regs["smpl"] >> 8) & 0xFF
        rate = DOUT_RATES.get(rate_code)
        if rate is None:
            raise SerialError(f"未知输出率编码 0x{rate_code:02X}")
        baud_code = (regs["uart"] >> 8) & 0x03
        if baud_code != 0x01:
            raise SerialError(
                f"设备 BAUD_RATE 位不是 230400（编码 0x{baud_code:X}）；为保护设置，拒绝写入"
            )
        uart_auto = bool(regs["uart"] & 0x0001)
        required = profile.packet_length * rate * 10.0
        if uart_auto and required > BAUD_RATE * 0.95:
            raise SerialError(
                f"当前 {rate:g} Sps、{profile.packet_length} 字节/帧需要 {required:.0f} bit/s，"
                f"超过 230400 的安全带宽；按要求不改变频率和波特率，因此拒绝切换"
            )
        if profile.key == "attitude16":
            if rate > 500:
                raise SerialError("姿态角功能要求输出率不高于 500 Sps")
            filter_code = regs["filter"] & 0x1F
            if filter_code not in ATTITUDE_FILTERS.get(rate_code, set()):
                raise SerialError(
                    f"当前输出率编码 0x{rate_code:02X} 与滤波器 0x{filter_code:02X} 不支持姿态角；"
                    "不会擅自修改它们"
                )
            ext_sel = (regs["msc"] >> 6) & 0x03
            if ext_sel in (1, 3):
                raise SerialError("当前启用了外部计数复位/外部触发，手册规定不能启用姿态角")
            if regs["pol"] & 0x007E:
                raise SerialError("当前 POL_CTRL 存在轴反相，手册要求姿态功能下这些位全部为 0")

    def _switch_worker(self, profile, conv, motion):
        if not self.busy_lock.acquire(blocking=False):
            self._post("error", "设备正在执行其他操作。")
            return
        self.reader_pause.set()
        time.sleep(0.07)
        before = None
        changed = False
        try:
            assert self.port and self.protocol
            self.port.purge_rx()
            self.protocol.enter_configuration()
            before = self.protocol.snapshot()
            self._preflight(profile, before)
            self.protocol.apply_profile(profile, conv, motion)
            changed = True
            after = self.protocol.snapshot()

            if (after["burst1"] & 0xFF07) != profile.burst1:
                raise SerialError(f"BURST_CTRL1 回读不一致：0x{after['burst1']:04X}")
            if (after["burst2"] & 0x7F00) != profile.burst2:
                raise SerialError(f"BURST_CTRL2 回读不一致：0x{after['burst2']:04X}")
            if ((after["atti"] >> 8) & 0x0E) != profile.atti_high:
                raise SerialError(f"ATTI_CTRL 回读不一致：0x{after['atti']:04X}")
            for name in self.PROTECTED:
                if after[name] != before[name]:
                    raise SerialError(
                        f"受保护寄存器 {name.upper()} 发生变化："
                        f"0x{before[name]:04X} -> 0x{after[name]:04X}"
                    )

            self.protocol.enter_sampling()
            self.current_regs = after
            self.current_mode = profile
            self.decoder.configure(profile, after["glob3"])
            self.monitor.reset()
            self.port.purge_rx()
            self._post("switched", profile)
        except Exception as exc:
            try:
                if before and changed and self.protocol:
                    self.protocol.restore_mode_registers(before)
                if self.protocol:
                    self.protocol.enter_sampling()
            except Exception as rollback_exc:
                self._post("error", f"切换失败：{exc}；恢复原设置也失败：{rollback_exc}")
            else:
                self._post("error", f"切换失败，已恢复原模式：{exc}")
        finally:
            time.sleep(0.03)
            self.reader_pause.clear()
            self.busy_lock.release()

    def _flash_worker(self):
        if not self.busy_lock.acquire(blocking=False):
            self._post("error", "设备正在执行其他操作。")
            return
        self.reader_pause.set()
        time.sleep(0.07)
        try:
            assert self.port and self.protocol
            self.port.purge_rx()
            self.protocol.enter_configuration()
            before = self.protocol.snapshot()
            diag = self.protocol.flash_backup()
            after = self.protocol.snapshot()
            for name in (*self.PROTECTED, "burst1", "burst2", "atti"):
                if after[name] != before[name]:
                    raise SerialError(
                        f"Flash 保存过程中寄存器 {name.upper()} 意外变化："
                        f"0x{before[name]:04X} -> 0x{after[name]:04X}"
                    )
            self.protocol.enter_sampling()
            self.current_regs = after
            self.port.purge_rx()
            self._post("flash_saved", diag)
        except Exception as exc:
            try:
                if self.protocol:
                    self.protocol.enter_sampling()
            except Exception as resume_exc:
                self._post("error", f"Flash 保存失败：{exc}；恢复采样也失败：{resume_exc}")
            else:
                self._post("error", f"Flash 保存失败：{exc}")
        finally:
            time.sleep(0.03)
            self.reader_pause.clear()
            self.busy_lock.release()

    def _drdy_worker(self, enabled: bool, active_high: bool):
        if not self.busy_lock.acquire(blocking=False):
            self._post("error", "设备正在执行其他操作。")
            return
        self.reader_pause.set()
        time.sleep(0.07)
        before = None
        changed = False
        try:
            assert self.port and self.protocol
            self.port.purge_rx()
            self.protocol.enter_configuration()
            before = self.protocol.snapshot()
            readback = self.protocol.configure_drdy(before["msc"], enabled, active_high)
            changed = True
            expected_drdy = (0x04 if enabled else 0) | (0x02 if active_high else 0)
            if (readback & 0x0006) != expected_drdy:
                raise SerialError(f"DRDY位回读不一致：MSC_CTRL=0x{readback:04X}")
            after = self.protocol.snapshot()
            if (after["msc"] & 0x0006) != expected_drdy:
                raise SerialError(f"DRDY配置未保持：MSC_CTRL=0x{after['msc']:04X}")
            if (after["msc"] & ~0x0006) != (before["msc"] & ~0x0006):
                raise SerialError(
                    "MSC_CTRL中DRDY以外的位发生变化："
                    f"0x{before['msc']:04X} -> 0x{after['msc']:04X}"
                )
            for name in before:
                if name != "msc" and after[name] != before[name]:
                    raise SerialError(
                        f"其他寄存器 {name.upper()} 发生变化："
                        f"0x{before[name]:04X} -> 0x{after[name]:04X}"
                    )
            self.protocol.enter_sampling()
            self.current_regs = after
            self.port.purge_rx()
            self._post("drdy_updated", (before["msc"], after["msc"]))
        except Exception as exc:
            try:
                if before and changed and self.protocol:
                    self.protocol.restore_drdy(before["msc"])
                if self.protocol:
                    self.protocol.enter_sampling()
            except Exception as rollback_exc:
                self._post("error", f"DRDY配置失败：{exc}；恢复原DRDY配置也失败：{rollback_exc}")
            else:
                self._post("error", f"DRDY配置失败，已恢复原DRDY配置：{exc}")
        finally:
            time.sleep(0.03)
            self.reader_pause.clear()
            self.busy_lock.release()

    def _config_text(self):
        if not self.current_regs:
            return "等待读取设备配置"
        r = self.current_regs
        rate_code = (r["smpl"] >> 8) & 0xFF
        rate = DOUT_RATES.get(rate_code, float("nan"))
        filter_code = r["filter"] & 0x1F
        auto = "UART Auto" if r["uart"] & 1 else "UART Manual"
        auto_start = "开" if r["uart"] & 2 else "关"
        accel_range = "±16 g" if r["glob3"] & 0x0100 else "±8 g"
        return (
            f"输出率：{rate:g} Sps（0x{rate_code:02X}）\n"
            f"滤波器：0x{filter_code:02X}\n"
            f"串口：230400，{auto}，Auto Start {auto_start}\n"
            f"加速度量程：{accel_range}\n"
            f"BURST_CTRL1：0x{r['burst1']:04X}\n"
            f"BURST_CTRL2：0x{r['burst2']:04X}\n"
            f"ATTI_CTRL（W1:0x14）：0x{r['atti']:04X}\n"
            f"DLT_CTRL / GLOB_CMD3（W1:0x12）：0x{r['glob3']:04X}\n"
            f"角增量比例代码：{(r['glob3'] >> 4) & 0x0F}"
            f"（0x{(r['glob3'] >> 4) & 0x0F:X}），"
            f"速度增量比例代码：{r['glob3'] & 0x0F}（0x{r['glob3'] & 0x0F:X}）"
            "\n" + delta_range_text((r['glob3'] >> 4) & 15, DOUT_RATES.get(rate_code))
        )

    def _copy_config(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.config_var.get())

    def _poll_ui(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
                    port_name, warning, tx_ready = payload
                    self.connected = True
                    self.connect_btn.configure(text="断开", state="normal")
                    self.apply_btn.configure(state="normal" if tx_ready else "disabled")
                    self.flash_btn.configure(state="normal" if tx_ready else "disabled")
                    self.drdy_apply_btn.configure(state="normal" if tx_ready else "disabled")
                    self.burst_btn.configure(state="normal" if tx_ready else "disabled")
                    if self.current_mode:
                        if self.current_mode.key in MODES:
                            self.mode_var.set(self.current_mode.key)
                            self._update_mode_details()
                        mode_text = self.current_mode.title
                    else:
                        mode_text = "未知数据包配置"
                    self.gyro_bias = {axis: 0.0 for axis in "xyz"}
                    self.calibrator.cancel()
                    self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self.clear_calib_btn.configure(state="disabled")
                    self.calib_var.set("未校准。保持IMU静止后点击“开始静止校准”。零偏只在上位机内存中生效。")
                    suffix = "" if tx_ready else " · 仅监听（TX无响应）"
                    self.status_var.set(f"已连接 {port_name} · {mode_text}{suffix}")
                    self.config_var.set(
                        self._config_text() if self.current_regs else
                        "寄存器不可读；下方物理量按出厂 ±8 g / 默认比例近似显示。"
                    )
                    self._sync_drdy_ui()
                    self.delta_status_var.set("已读取当前设备配置；填入推荐值后点击应用才会写入。"
                                              if tx_ready else "仅监听：不能确认比例，配置已禁用。")
                    self._log(f"已连接 {port_name}；当前模式：{mode_text}。")
                    if warning:
                        self._log(warning)
                        messagebox.showwarning("串口只能接收", warning)
                elif kind == "switched":
                    self.status_var.set(f"已切换：{payload.title}")
                    self.apply_btn.configure(state="normal")
                    self.flash_btn.configure(state="normal")
                    self.drdy_apply_btn.configure(state="normal")
                    self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self.config_var.set(self._config_text())
                    self._sync_drdy_ui()
                    self._log(
                        f"切换完成：{payload.title}，BURST_CTRL1=0x{payload.burst1:04X}，"
                        f"BURST_CTRL2=0x{payload.burst2:04X}。尚未写 Flash；如需掉电保存请单独点击保存按钮。"
                    )
                elif kind == "flash_saved":
                    self.status_var.set("Flash 保存成功")
                    self.delta_status_var.set(
                        f"Flash 保存成功；当前读回 DLT_CTRL=0x{self.current_regs['glob3']:04X}。")
                    self.apply_btn.configure(state="normal")
                    self.flash_btn.configure(state="normal")
                    self.drdy_apply_btn.configure(state="normal")
                    self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self.config_var.set(self._config_text())
                    self._sync_drdy_ui()
                    self._log(
                        f"FLASH_BACKUP 完成，FLASH_BU_ERR=0，DIAG_STAT=0x{payload:04X}。"
                        "当前带 ○ 标记的寄存器已保存；GLOB_CMD2 姿态运动模型不在普通备份范围内。"
                    )
                    messagebox.showinfo("Flash 保存成功", "当前可备份设置已写入 Flash，并通过 FLASH_BU_ERR 检查。")
                elif kind == "drdy_updated":
                    before_msc, after_msc = payload
                    self.status_var.set("DRDY独立配置成功")
                    self.apply_btn.configure(state="normal")
                    self.flash_btn.configure(state="normal")
                    self.drdy_apply_btn.configure(state="normal")
                    self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self.config_var.set(self._config_text())
                    self._sync_drdy_ui()
                    self._log(
                        f"DRDY配置完成：MSC_CTRL 0x{before_msc:04X} -> 0x{after_msc:04X}；"
                        "只修改DRDY_ON/DRDY_POL，其他寄存器已逐项回读确认未变。尚未写Flash。"
                    )
                elif kind == "delta_result":
                    success, message = payload
                    self.status_var.set("增量配置操作完成" if success else "增量配置操作失败")
                    self.delta_status_var.set(message)
                    self.config_var.set(self._config_text())
                    self._sync_drdy_ui()
                    self.connect_btn.configure(state="normal")
                    for button in (self.apply_btn, self.flash_btn, self.drdy_apply_btn, self.burst_btn):
                        button.configure(state="normal" if self.tx_ready else "disabled")
                    self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self._log(message)
                    if not success:
                        messagebox.showerror("增量比例配置失败", message)
                elif kind == "error":
                    self.status_var.set("操作失败")
                    self.connect_btn.configure(state="normal")
                    if self.connected:
                        self.apply_btn.configure(state="normal" if self.tx_ready else "disabled")
                        self.flash_btn.configure(state="normal" if self.tx_ready else "disabled")
                        self.drdy_apply_btn.configure(state="normal" if self.tx_ready else "disabled")
                        self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                    self._log(payload)
                    messagebox.showerror("M-G366PDG0", payload)
        except queue.Empty:
            pass

        total, bad, hz, data, raw_hex = self.monitor.snapshot()
        self.stats_var.set(f"有效帧 {total} · 校验/同步错误 {bad} · 最近 {hz} Hz")
        angle_sat, velocity_sat, flag_errors = self.monitor.health_snapshot()
        self.health_var.set(
            f"角增量饱和 {angle_sat} · 速度增量饱和 {velocity_sat} · 错误FLAG {flag_errors}"
            + ("\n检测到削顶：请检查增量比例并重新采集。" if angle_sat or velocity_sat else ""))
        delta_ready = self.connected and self.tx_ready and not self.busy_lock.locked()
        for button in (self.delta_apply_btn, self.delta_read_btn):
            button.configure(state="normal" if delta_ready else "disabled")
        active, elapsed, duration, sample_count = self.calibrator.status()
        if active:
            if elapsed > duration + 2.0:
                self.calibrator.cancel()
                self.calib_var.set("校准失败：超过采样时间仍未收到足够的原始陀螺数据。")
                self.calib_btn.configure(state="normal" if self._mode_has_gyro() else "disabled")
                if self.tx_ready:
                    self.apply_btn.configure(state="normal")
                    self.flash_btn.configure(state="normal")
                    self.drdy_apply_btn.configure(state="normal")
                self._log("零偏校准失败：数据流超时。")
            else:
                self.calib_var.set(
                    f"正在采集静止数据：{min(elapsed, duration):.1f}/{duration:g} 秒，"
                    f"样本 {sample_count}。请勿移动IMU。"
                )
        result = self.calibrator.take_result()
        if result is not None:
            self._finish_bias_calibration(result)
        dash = "—"
        f = lambda value: f"{value:.7g}" if isinstance(value, float) else dash
        raw_gyro = tuple(data.get(f"gyro_{axis}") for axis in "xyz")
        calibrated_gyro = tuple(
            value - self.gyro_bias[axis] if isinstance(value, float) else None
            for axis, value in zip("xyz", raw_gyro)
        )
        self.tree.item("gyro_raw", values=tuple(f(value) for value in raw_gyro) + ("°/s",))
        self.tree.item("gyro_cal", values=tuple(f(value) for value in calibrated_gyro) + ("°/s",))
        self.tree.item("accel", values=tuple(f(data.get(f"accel_{a}")) for a in "xyz") + ("g",))
        self.tree.item("dangle", values=tuple(f(data.get(f"delta_angle_{a}")) for a in "xyz") + ("°",))
        self.tree.item("dvelocity", values=tuple(f(data.get(f"delta_velocity_{a}")) for a in "xyz") + ("m/s",))
        self.tree.item("attitude", values=tuple(f(data.get(n)) for n in ("roll", "pitch", "yaw")) + ("°",))
        if data:
            self.misc_var.set(
                f"温度 {data.get('temp_c', 0):.3f} °C   COUNT {data.get('count', 0)}   "
                f"FLAG 0x{data.get('flag', 0):04X}   GPIO 0x{data.get('gpio', 0):04X}"
            )
        self.hex_text.configure(state="normal")
        self.hex_text.delete("1.0", "end")
        self.hex_text.insert("1.0", raw_hex)
        self.hex_text.configure(state="disabled")
        self.root.after(100, self._poll_ui)

    def _on_close(self):
        if self.busy_lock.locked():
            self.status_var.set("正在完成设备操作，请稍后关闭。")
            return
        if self.connected:
            self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    G366App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
