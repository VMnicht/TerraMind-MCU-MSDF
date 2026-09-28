"""Parser and conservative acquisition readiness checks for USART6 monitor logs."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple


STATUS_FIELDS = (
    "mcu_ms", "imu_frames", "imu_crc", "imu_framing", "imu_count_jumps",
    "imu_rx_overflow", "imu_uart_errors", "imu_rearm_errors", "gnss_bestnav",
    "gnss_heading", "gnss_crc", "gnss_format", "gnss_line_overflows",
    "gnss_rx_overflow", "gnss_uart_errors", "gnss_rearm_errors", "tx_dropped",
    "tx_discarded_bytes", "tx_dma_start_errors", "tx_uart_errors",
    "tx_queue_high_water", "format_errors",
)
ERROR_FIELDS = (
    "imu_crc", "imu_framing", "imu_rx_overflow", "imu_uart_errors",
    "imu_rearm_errors", "gnss_crc", "gnss_format", "gnss_line_overflows",
    "gnss_rx_overflow", "gnss_uart_errors", "gnss_rearm_errors", "tx_dropped",
    "tx_discarded_bytes", "tx_dma_start_errors", "tx_uart_errors", "format_errors",
)
SYNC_FIELDS = (
    "mcu_ms", "timer_hz_nominal", "drdy_captures", "pps_captures",
    "drdy_queue_drops", "pps_queue_drops", "drdy_overcaptures",
    "pps_overcaptures", "imu_unmatched",
)
SYNC_ERROR_FIELDS = SYNC_FIELDS[4:]
ANCHOR_FIELDS = (
    "mcu_ms", "pps_sequence", "gps_week", "gps_tow_ms", "tick_hi",
    "tick_lo", "period_ticks", "serial_lag_ticks", "consecutive", "locked",
)


def unicore_crc32(text: str) -> int:
    crc = 0
    for byte in text.encode("ascii"):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


@dataclass
class Imu:
    mcu_ms: int
    mode: int
    count: int
    flag: int
    temp_raw: int
    gpio: int
    values: Tuple[int, ...]

    @property
    def gyro_dps(self) -> Tuple[float, ...]:
        return tuple(v / (66 * 65536) for v in self.values[:3]) if self.mode == 2 else ()

    @property
    def accel_g(self) -> Tuple[float, ...]:
        return tuple(v / (2500 * 65536) for v in self.values[3:]) if self.mode == 2 else ()


@dataclass
class Position:
    mcu_ms: int
    week: int
    tow_ms: int
    time_status: str
    status: str
    solution: str
    lat: float
    lon: float
    height_m: float
    lat_std_m: float
    lon_std_m: float
    height_std_m: float
    satellites: int
    speed_mps: float


@dataclass
class Heading:
    mcu_ms: int
    week: int
    tow_ms: int
    time_status: str
    status: str
    solution: str
    baseline_m: float
    heading_deg: float
    pitch_deg: float
    heading_std_deg: float


class MonitorModel:
    def __init__(self) -> None:
        self.version: Optional[str] = None
        self.imu: Optional[Imu] = None
        self.position: Optional[Position] = None
        self.heading: Optional[Heading] = None
        self.imu_at = self.position_at = self.heading_at = self.status_at = -1e9
        self.imu_times: Deque[Tuple[float, int]] = deque()
        self.position_history: Deque[Tuple[float, Position]] = deque()
        self.heading_history: Deque[Tuple[float, Heading]] = deque()
        self.heading_baselines: Deque[Tuple[float, float]] = deque()
        self.clock_pairs: Deque[Tuple[float, int, int]] = deque()
        self.pos_std_history: Deque[Tuple[float, float]] = deque()
        self.hdg_std_history: Deque[Tuple[float, float]] = deque()
        self.status: Dict[str, int] = {}
        self.sync_status: Dict[str, int] = {}
        self.previous_sync_status: Dict[str, int] = {}
        self.pps_at = self.sync_at = -1e9
        self.pps_period_ticks: Optional[int] = None
        self.pps_sequence = 0
        self.imu_timed = 0
        self.imu_timed_missing = 0
        self.anchor: Dict[str, int] = {}
        self.anchor_at = -1e9
        self.previous_status: Dict[str, int] = {}
        self.status_fault_at = -1e9
        self.local_fault_at = -1e9
        self.imu_count_gaps = 0
        self.record_errors = 0
        self.unknown_lines = 0
        self.counts: Counter[str] = Counter()

    def feed(self, line: str, at: float) -> None:
        line = line.strip("\r\n")
        if not line:
            return
        try:
            if line.startswith("V,"):
                self.version = line
            elif line.startswith("I,"):
                fields = line.split(",")
                if len(fields) != 13:
                    raise ValueError("IMU field count")
                data = tuple(int(x) for x in fields[1:])
                sample = Imu(*data[:6], data[6:])
                if sample.mode not in (2, 3) or not 0 <= sample.count <= 65535:
                    raise ValueError("IMU mode/count")
                if self.imu is not None:
                    delta_ms = sample.mcu_ms - self.imu.mcu_ms
                    delta_count = (sample.count - self.imu.count) & 0xFFFF
                    if delta_ms > 8 or delta_ms <= 0 or delta_count not in (312, 313):
                        self.imu_count_gaps += 1
                        self.local_fault_at = at
                self.imu = sample
                self.imu_at = at
                self.imu_times.append((at, sample.mcu_ms))
                self.counts["I"] += 1
            elif line.startswith("N,"):
                _, mcu_text, raw = line.split(",", 2)
                mcu_ms = int(mcu_text)
                if raw.startswith("#BESTNAVA,") or raw.startswith("#UNIHEADINGA,"):
                    payload, checksum = raw.rsplit("*", 1)
                    if len(checksum) != 8 or unicore_crc32(payload[1:]) != int(checksum, 16):
                        raise ValueError("GNSS CRC")
                    header_text, body_text = payload[1:].split(";", 1)
                    header, body = header_text.split(","), body_text.split(",")
                    if len(header) != 10:
                        raise ValueError("GNSS header")
                    week, tow_ms = int(header[4]), int(header[5])
                    if raw.startswith("#BESTNAVA,"):
                        if len(body) != 30:
                            raise ValueError("BESTNAVA fields")
                        self.position = Position(
                            mcu_ms, week, tow_ms, header[3], body[0], body[1], float(body[2]),
                            float(body[3]), float(body[4]), float(body[7]),
                            float(body[8]), float(body[9]), int(body[14]), float(body[25]),
                        )
                        self.position_at = at
                        self.position_history.append((at, self.position))
                        self.counts["BESTNAVA"] += 1
                        horizontal = max(self.position.lat_std_m, self.position.lon_std_m)
                        self.pos_std_history.append((at, horizontal))
                        self.clock_pairs.append((at, tow_ms, mcu_ms))
                    else:
                        if len(body) != 17:
                            raise ValueError("UNIHEADINGA fields")
                        self.heading = Heading(
                            mcu_ms, week, tow_ms, header[3], body[0], body[1], float(body[2]),
                            float(body[3]), float(body[4]), float(body[6]),
                        )
                        self.heading_at = at
                        self.heading_history.append((at, self.heading))
                        self.counts["UNIHEADINGA"] += 1
                        self.hdg_std_history.append((at, self.heading.heading_std_deg))
                        if self.heading.status == "SOL_COMPUTED":
                            self.heading_baselines.append((at, self.heading.baseline_m))
                elif raw.startswith("$"):
                    self.counts["NMEA"] += 1
                else:
                    self.counts["other_N"] += 1
                self.counts["N"] += 1
            elif line.startswith("S,"):
                fields = line.split(",")[1:]
                if len(fields) != len(STATUS_FIELDS):
                    raise ValueError("status field count")
                new_status = dict(zip(STATUS_FIELDS, (int(x) for x in fields)))
                if self.previous_status and any(
                    new_status[key] > self.previous_status[key] for key in ERROR_FIELDS
                ):
                    self.status_fault_at = at
                self.previous_status = new_status
                self.status = new_status
                self.status_at = at
                self.counts["S"] += 1
            elif line.startswith("D,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != 4 or values[0] <= 0:
                    raise ValueError("DRDY record")
                self.counts["D"] += 1
            elif line.startswith("P,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != 5 or values[0] <= 0:
                    raise ValueError("PPS record")
                self.pps_sequence = values[0]
                self.pps_at = at
                self.pps_period_ticks = values[4] or None
                if self.pps_period_ticks is not None and not 3_500_000 <= self.pps_period_ticks <= 4_500_000:
                    self.local_fault_at = at
                self.counts["P"] += 1
            elif line.startswith("T,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != 7:
                    raise ValueError("IMU timing record")
                self.imu_timed += 1
                if values[2] == 0:
                    self.imu_timed_missing += 1
                    self.local_fault_at = at
                self.counts["T"] += 1
            elif line.startswith("R,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != 3:
                    raise ValueError("GNSS timing record")
                self.counts["R"] += 1
            elif line.startswith("H,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != len(SYNC_FIELDS):
                    raise ValueError("sync status field count")
                new_sync = dict(zip(SYNC_FIELDS, values))
                if self.previous_sync_status and any(
                    new_sync[key] > self.previous_sync_status[key] for key in SYNC_ERROR_FIELDS
                ):
                    self.status_fault_at = at
                self.previous_sync_status = new_sync
                self.sync_status = new_sync
                self.sync_at = at
                self.counts["H"] += 1
            elif line.startswith("A,"):
                values = [int(x) for x in line.split(",")[1:]]
                if len(values) != len(ANCHOR_FIELDS) or values[-1] not in (0, 1):
                    raise ValueError("GPS/PPS anchor")
                self.anchor = dict(zip(ANCHOR_FIELDS, values))
                self.anchor_at = at
                self.counts["A"] += 1
            else:
                self.unknown_lines += 1
                self.local_fault_at = at
        except (ValueError, IndexError, UnicodeError):
            self.record_errors += 1
            self.local_fault_at = at
        self._trim(at)

    def _trim(self, at: float) -> None:
        for queue in (self.imu_times, self.position_history, self.heading_history,
                      self.heading_baselines, self.clock_pairs,
                      self.pos_std_history, self.hdg_std_history):
            while queue and queue[0][0] < at - 30.0:
                queue.popleft()

    def imu_rate(self, now: float) -> float:
        times = [mcu for at, mcu in self.imu_times if at >= now - 3.0]
        return 1000.0 * (len(times) - 1) / (times[-1] - times[0]) if len(times) >= 2 and times[-1] > times[0] else 0.0

    def clock_drift_ms_per_s(self, now: float) -> Optional[float]:
        pairs = [(tow, mcu) for at, tow, mcu in self.clock_pairs if at >= now - 15.0]
        if len(pairs) < 30 or pairs[-1][0] - pairs[0][0] < 5000:
            return None
        return 1000.0 * ((pairs[-1][1] - pairs[0][1]) /
                         (pairs[-1][0] - pairs[0][0]) - 1.0)

    def assess(self, now: float, pos_std_limit: float = 0.2,
               heading_std_limit: float = 2.0,
               expected_baseline: Optional[float] = None) -> Tuple[bool, List[str], List[str]]:
        issues: List[str] = []
        warnings: List[str] = []
        def good_pos(pos: Position) -> bool:
            return (pos.time_status == "FINE" and pos.status == "SOL_COMPUTED" and pos.solution == "NARROW_INT"
                    and max(pos.lat_std_m, pos.lon_std_m) <= pos_std_limit
                    and pos.height_std_m <= 0.5)

        def good_hdg(hdg: Heading) -> bool:
            return (hdg.time_status == "FINE" and hdg.status == "SOL_COMPUTED" and hdg.solution == "NARROW_INT"
                    and 0 < hdg.heading_std_deg <= heading_std_limit
                    and hdg.baseline_m > 0)

        if now - self.imu_at > 0.25:
            issues.append("IMU 数据超时")
        elif not 190 <= self.imu_rate(now) <= 210:
            issues.append("IMU 速率未稳定在 200 Hz")
        if now - self.position_at > 0.5 or self.position is None:
            issues.append("GNSS 位置数据超时")
        else:
            pos = self.position
            if not good_pos(pos):
                issues.append("位置未达到 RTK 固定和精度阈值")
        if now - self.heading_at > 0.5 or self.heading is None:
            issues.append("双天线航向数据超时")
        else:
            hdg = self.heading
            if not good_hdg(hdg):
                issues.append("航向未达到固定解和精度阈值")
            recent_baselines = [v for t, v in self.heading_baselines if t >= now - 5]
            if good_hdg(hdg) and len(recent_baselines) >= 20 and max(recent_baselines) - min(recent_baselines) > 0.1:
                issues.append("双天线基线长度波动超过 0.1 m")
            if good_hdg(hdg) and expected_baseline is not None and abs(hdg.baseline_m - expected_baseline) > 0.1:
                issues.append("基线长度与实测安装值偏差超过 0.1 m")
        if now - self.status_at > 2.5:
            issues.append("固件 S 状态统计超时")
        if now - max(self.local_fault_at, self.status_fault_at) < 5:
            issues.append("近 5 秒出现记录损坏或接收/发送错误")
        recent_pos = [(t, p) for t, p in self.position_history if t >= now - 5]
        recent_hdg = [(t, h) for t, h in self.heading_history if t >= now - 5]
        if (len(recent_pos) < 45 or len(recent_hdg) < 45 or
                recent_pos[0][0] > now - 4.8 or recent_hdg[0][0] > now - 4.8 or
                not all(good_pos(p) for _, p in recent_pos) or
                not all(good_hdg(h) for _, h in recent_hdg)):
            issues.append("位置和航向尚未连续稳定 5 秒")
        drift = self.clock_drift_ms_per_s(now)
        if drift is not None and abs(drift) > 0.5:
            warnings.append(f"MCU 相对 GPS 时间漂移约 {drift:+.2f} ms/s")
        if self.version and self.version.startswith("V,2,"):
            if now - self.pps_at > 2.5:
                issues.append("PPS 捕获超时或未接入")
            if now - self.sync_at > 2.5:
                issues.append("硬件时间戳状态超时")
            if self.pps_period_ticks is None:
                issues.append("尚未测得完整 PPS 周期")
            if now - self.anchor_at > 2.5 or not self.anchor.get("locked", 0):
                issues.append("GPS 整秒与 PPS 尚未连续配对")
            warnings.append("PPS 已用于同一 TIM2 时基；GPS 整秒归属和固定输出延迟仍需核对")
        else:
            warnings.append("无 PPS 同步证明：就绪仅表示传感器采集条件，不代表组合导航已收敛")
        return not issues, issues, warnings
