"""Replay mode-3 logs with original KF-GINS or optional offline aiding."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from bisect import bisect_left
from math import cos, degrees, isfinite, pi, radians, sin, sqrt
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Optional

import numpy as np

from fusion2d import FusionResult, GnssPoint, ReplayFrame
from monitor_protocol import unicore_crc32
from capture_profile import CaptureProfile, load_capture, delta_scales
from startup_calibration import load_startup_calibration
from sync_timeline import GPS_WEEK_MS, analyze_sync


ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "KF-GINS"
DEFAULT_CONFIG = UPSTREAM / "config" / "kf-gins.yaml"
VEHICLE_CONFIG = ROOT / "tools" / "config" / "kf_gins_g365_vehicle.yaml"
VALID_SOLUTIONS = {"NARROW_INT", "NARROW_FLOAT", "L1_INT", "L1_FLOAT", "PSRDIFF", "SINGLE"}


@dataclass
class KfOptions:
    # Match the capture default; historical logs may require a different value.
    delta_ctrl: int = CaptureProfile().delta_ctrl
    axis_forward: str = "+x"
    axis_right: str = "+y"
    axis_down: str = "+z"
    lever_forward_m: float = 0.0
    lever_right_m: float = 0.0
    lever_down_m: float = 0.0
    heading_offset_deg: float = 0.0
    initial_roll_deg: float = 0.0
    initial_pitch_deg: float = 0.0
    initial_heading_deg: Optional[float] = None
    gnss_delay_ms: float = 0.0
    config_path: str = ""
    apply_startup_calibration: bool = True
    heading_aiding: bool = False
    velocity_aiding: bool = False

    def validate(self) -> None:
        if type(self.apply_startup_calibration) is not bool:
            raise ValueError("应用启动标定必须为布尔值")
        if type(self.heading_aiding) is not bool or type(self.velocity_aiding) is not bool:
            raise ValueError("航向 / 速度更新开关必须为布尔值")
        if not 0 <= self.delta_ctrl <= 0xFFFF:
            raise ValueError("DLT_CTRL 必须是 0x0000–0xFFFF")
        axes = [self.axis_forward, self.axis_right, self.axis_down]
        if any(not re.fullmatch(r"[+-][xyz]", axis) for axis in axes):
            raise ValueError("IMU 轴映射须为 +x、-x 等带符号的轴名")
        if {axis[1] for axis in axes} != {"x", "y", "z"}:
            raise ValueError("前、右、下必须各使用一个不同的 IMU 轴")
        # A signed permutation must preserve handedness for FRD and sensor XYZ.
        permutation = ["xyz".index(axis[1]) for axis in axes]
        inversions = sum(permutation[i] > permutation[j] for i in range(3) for j in range(i + 1, 3))
        signs = sum(axis[0] == "-" for axis in axes)
        if (inversions + signs) % 2:
            raise ValueError("前右下轴映射必须保持右手系；请检查三个轴的方向")
        for key, value in asdict(self).items():
            if isinstance(value, (float, int)) and not isfinite(value):
                raise ValueError(f"{key} 必须是有限数值")
        if max(abs(self.lever_forward_m), abs(self.lever_right_m), abs(self.lever_down_m)) > 100:
            raise ValueError("天线杆臂超过 100 m，请检查单位")
        if abs(self.gnss_delay_ms) > 500:
            raise ValueError("GNSS 延迟修正限于 ±500 ms")
        if abs(self.initial_roll_deg) > 90 or abs(self.initial_pitch_deg) > 90:
            raise ValueError("初始横滚/俯仰应在 ±90° 内")
        if self.config_path and not Path(self.config_path).is_file():
            raise ValueError("KF-GINS 配置模板不存在")


@dataclass
class DeltaSample:
    mcu_ms: int
    count: int
    angle_raw: tuple[int, int, int]
    velocity_raw: tuple[int, int, int]
    capture_ticks: Optional[int] = None
    drdy_sequence: int = 0
    flag: int = 0
    gps_ms: float = 0.0


@dataclass
class Position:
    gps_ms: int
    mcu_ms: int
    latitude: float
    longitude: float
    height: float
    north_std: float
    east_std: float
    down_std: float
    speed: float
    track: float
    vertical_speed: float
    velocity_valid: bool
    solution: str
    velocity_latency_s: float = 0.0
    velocity_age_s: float = 0.0
    vertical_speed_std: float = 0.0
    horizontal_speed_std: float = 0.0


@dataclass
class Heading:
    gps_ms: int
    degrees: float
    std: float
    solution: str = ""
    baseline_m: float = 0.0


@dataclass
class KfFusionResult(FusionResult):
    heading_aiding: bool = False
    velocity_aiding: bool = False


@dataclass
class KfReplayFrame(ReplayFrame):
    # Diagnostic additions only. The inherited navigation fields remain the IMU solution.
    antenna_east_m: Optional[float] = None
    antenna_north_m: Optional[float] = None
    antenna_height_m: Optional[float] = None
    gnss_vehicle_heading_deg: Optional[float] = None
    gnss_heading_std_deg: Optional[float] = None
    heading_residual_deg: Optional[float] = None


def _with_diagnostics(frames: list[ReplayFrame], headings: list[Heading],
                      week: int, options: KfOptions) -> list[KfReplayFrame]:
    """Compare on GPS epochs without feeding extra observations to the solver.

    Circular interpolation is allowed only between consecutive 10 Hz observations
    (at most 150 ms apart). Never extrapolate or bridge a heading outage.
    """
    by_time = {h.gps_ms - week * GPS_WEEK_MS: h for h in headings}
    times = sorted(by_time)
    lever = np.array([options.lever_forward_m, options.lever_right_m, options.lever_down_m])
    annotated = []
    for frame in frames:
        result = KfReplayFrame(**vars(frame))
        ned = _body_to_ned(frame.heading_deg, frame.pitch_deg or 0.0,
                           frame.roll_deg or 0.0) @ lever
        result.antenna_east_m = frame.east_m + float(ned[1])
        result.antenna_north_m = frame.north_m + float(ned[0])
        result.antenna_height_m = (frame.height_m - float(ned[2])
                                   if frame.height_m is not None else None)
        index = bisect_left(times, frame.mcu_ms)
        reference = std = None
        if index < len(times) and abs(times[index] - frame.mcu_ms) < 1e-4:
            h = by_time[times[index]]
            reference, std = h.degrees, h.std
        elif 0 < index < len(times) and 0 < times[index] - times[index - 1] <= 150:
            first, last = by_time[times[index - 1]], by_time[times[index]]
            weight = (frame.mcu_ms - times[index - 1]) / (times[index] - times[index - 1])
            delta = (last.degrees - first.degrees + 180) % 360 - 180
            reference = first.degrees + weight * delta
            std = max(first.std, last.std)  # Conservative display, not a filter covariance.
        if reference is not None:
            result.gnss_vehicle_heading_deg = (reference + options.heading_offset_deg) % 360
            result.gnss_heading_std_deg = std
            result.heading_residual_deg = (frame.heading_deg - result.gnss_vehicle_heading_deg + 180) % 360 - 180
        annotated.append(result)
    return annotated


@dataclass
class Prepared:
    imu_rows: list[tuple[float, ...]]
    gnss_rows: list[tuple[float, ...]]
    positions: list[Position]
    headings: list[Heading]
    first_position: Position
    initial_heading: float
    initial_velocity: tuple[float, float, float]
    hardware_sync: bool
    pps_count: int
    fit_rms_ms: float
    rejected: int
    warnings: list[str]
    imu_count: int
    week: int
    startup_calibration_applied: bool = False


def _parse_log(path: Path) -> tuple[list[DeltaSample], list[Position], list[Heading], int, int]:
    imu: list[DeltaSample] = []
    positions: list[Position] = []
    headings: list[Heading] = []
    rejected = raw_count = 0
    with path.open("r", encoding="ascii", errors="replace") as source:
        for line in source:
            line = line.strip()
            try:
                if line.startswith("I,"):
                    f = line.split(",")
                    if len(f) != 13:
                        raise ValueError("IMU field count")
                    if f[2] == "2":
                        raw_count += 1
                        continue
                    if f[2] != "3":
                        raise ValueError("IMU mode")
                    values = tuple(int(v) for v in f[7:13])
                    if any(not -(2**31) <= v < 2**31 for v in values):
                        raise ValueError("IMU raw range")
                    flag = int(f[4])
                    if not 0 <= flag <= 65535:
                        raise ValueError("IMU flag range")
                    imu.append(DeltaSample(int(f[1]), int(f[3]), values[:3], values[3:], flag=flag))
                elif line.startswith("T,"):
                    f = line.split(",")
                    if len(f) != 8:
                        raise ValueError("IMU timing fields")
                    if imu and imu[-1].mcu_ms == int(f[1]) and imu[-1].count == int(f[2]) and int(f[3]):
                        imu[-1].capture_ticks = (int(f[4]) << 32) | int(f[5])
                        imu[-1].drdy_sequence = int(f[3])
                elif line.startswith("N,"):
                    _, mcu_text, raw = line.split(",", 2)
                    if not raw.startswith(("#BESTNAVA,", "#UNIHEADINGA,")):
                        continue
                    payload, checksum = raw.rsplit("*", 1)
                    if len(checksum) != 8 or unicore_crc32(payload[1:]) != int(checksum, 16):
                        raise ValueError("GNSS CRC")
                    header_text, body_text = payload[1:].split(";", 1)
                    header, body = header_text.split(","), body_text.split(",")
                    if len(header) != 10 or header[3] != "FINE":
                        continue
                    gps_ms = int(header[4]) * GPS_WEEK_MS + int(header[5])
                    if raw.startswith("#BESTNAVA,"):
                        if len(body) != 30:
                            raise ValueError("BESTNAVA fields")
                        if body[0] != "SOL_COMPUTED" or body[1] not in VALID_SOLUTIONS:
                            continue
                        p = Position(gps_ms, int(mcu_text), float(body[2]), float(body[3]),
                                     float(body[4]) + float(body[5]),
                                     float(body[7]), float(body[8]), float(body[9]),
                                     float(body[25]), float(body[26]), float(body[27]),
                                     body[21] == "SOL_COMPUTED" and body[22] == "DOPPLER_VELOCITY",
                                     body[1], float(body[23]), float(body[24]),
                                     float(body[28]), float(body[29]))
                        if (-90 <= p.latitude <= 90 and -180 <= p.longitude <= 180 and
                                isfinite(p.height) and
                                all(isfinite(v) and v > 0 for v in (p.north_std, p.east_std, p.down_std))):
                            positions.append(p)
                        else:
                            rejected += 1
                    else:
                        if len(body) != 17:
                            raise ValueError("UNIHEADINGA fields")
                        if (body[0] == "SOL_COMPUTED" and body[1] in VALID_SOLUTIONS and
                                float(body[2]) > 0 and 0 < float(body[6]) < 20):
                            headings.append(Heading(gps_ms, float(body[3]), float(body[6]),
                                                    body[1], float(body[2])))
            except (ValueError, IndexError, UnicodeError):
                rejected += 1
    return imu, positions, headings, rejected, raw_count


def _axis_values(values: tuple[float, float, float], options: KfOptions) -> tuple[float, float, float]:
    source = dict(zip("xyz", values))
    return tuple(source[axis[1]] * (1 if axis[0] == "+" else -1)
                 for axis in (options.axis_forward, options.axis_right, options.axis_down))


def prepare_log(path: Path, options: KfOptions) -> Prepared:
    options.validate()
    metadata = load_capture(path, verify_log=True)
    startup = load_startup_calibration(path, metadata)
    if startup and (startup.mode != 3 or startup.delta_ctrl != options.delta_ctrl):
        raise ValueError("回放模式/DLT_CTRL 与板端启动标定记录冲突")
    if metadata and metadata["profile"]["delta_ctrl_confirmed"]:
        declared = metadata["profile"]["delta_ctrl"]
        if options.delta_ctrl != declared:
            raise ValueError(f"DLT_CTRL 与采集时已核对值 0x{declared:04X} 冲突，请核对回放参数")
    imu, positions, headings, rejected, raw_count = _parse_log(path)
    if raw_count:
        raise ValueError(f"日志混入 {raw_count} 帧模式 2 原始 IMU；请整段使用模式 3 增量采集")
    if len(imu) < 20 or len(positions) < 3:
        raise ValueError("需要至少 20 帧模式 3 增量 IMU 和 3 条有效 BESTNAVA")
    saturated_angle = sum(any(v in (-2147483648, 2147483647) for v in s.angle_raw) for s in imu)
    saturated_velocity = sum(any(v in (-2147483648, 2147483647) for v in s.velocity_raw) for s in imu)
    faulty = sum(bool(s.flag & 0x0101) for s in imu)
    if saturated_angle or saturated_velocity or faulty:
        raise ValueError(
            f"IMU 输入无效：角增量满量程 {saturated_angle} 帧，速度增量满量程 {saturated_velocity} 帧，"
            f"FLAG 错误/超量程 {faulty} 帧。截顶数据无法通过标定或修改回放比例恢复；"
            "请检查传感器诊断状态，按实际运动设置传感器增量量程并同步采集配置后重新采集。")
    if options.apply_startup_calibration and startup is None:
        raise ValueError("未找到有效启动标定 B/JSON；请使用带标定的日志，或明确取消“应用板端启动标定”以分析原始数据")
    imu.sort(key=lambda x: x.mcu_ms)
    positions.sort(key=lambda x: x.gps_ms)
    headings.sort(key=lambda x: x.gps_ms)
    if len({p.gps_ms for p in positions}) != len(positions):
        raise ValueError("BESTNAVA 历元重复，无法构造 KF-GINS 输入")
    gps0 = positions[0].gps_ms
    x = np.array([p.gps_ms - gps0 for p in positions], dtype=float)
    y = np.array([p.mcu_ms for p in positions], dtype=float)
    slope, offset = np.polyfit(x, y, 1)
    fit_rms = float(np.sqrt(np.mean((y - (slope * x + offset)) ** 2)))
    if not 0.999 <= slope <= 1.001:
        raise ValueError("MCU/GPS 时钟拟合斜率异常，请检查日志时间戳")
    sync = analyze_sync(path)
    captured = sum(sample.capture_ticks is not None for sample in imu)
    hardware = sync.locked and captured >= 0.98 * len(imu)
    if hardware:
        assert sync.anchor_pps is not None and sync.tick_rate is not None
        known = [(s.mcu_ms, s.capture_ticks) for s in imu if s.capture_ticks is not None]
        tick_slope, tick_offset = np.polyfit([v[0] - known[0][0] for v in known],
                                              [v[1] for v in known], 1)
        for sample in imu:
            ticks = sample.capture_ticks
            if ticks is None:
                ticks = int(tick_slope * (sample.mcu_ms - known[0][0]) + tick_offset)
            sample.gps_ms = sync.tick_to_gps_ms(ticks)
    else:
        for sample in imu:
            sample.gps_ms = (sample.mcu_ms - offset + options.gnss_delay_ms) / slope + gps0
    weeks = {int(s.gps_ms // GPS_WEEK_MS) for s in imu} | {p.gps_ms // GPS_WEEK_MS for p in positions}
    if len(weeks) != 1:
        raise ValueError("本次采集跨 GPS 周界；KF-GINS 的周内秒输入不能跨周，请分段运行")
    week = weeks.pop()
    times = [(s.gps_ms - week * GPS_WEEK_MS) / 1000 for s in imu]
    gaps = np.diff(times)
    if np.any(gaps <= 0):
        raise ValueError("IMU 时间非严格递增；请检查 DRDY/串口时间记录")
    if np.any(gaps > 0.1):
        raise ValueError(f"IMU 数据间隔最大 {max(gaps):.3f} s，疑似丢帧；请重新采集或分段处理")
    median_dt = float(np.median(gaps))
    if not 0.004 <= median_dt <= 0.006:
        raise ValueError(f"IMU 中位采样间隔 {median_dt * 1000:.2f} ms，需约 200 Hz 模式 3 数据")
    if times[-1] - times[0] < 0.1:
        raise ValueError("IMU 时长过短")
    start, end = times[0], times[-1]
    selected = [p for p in positions if start + 0.001 < (p.gps_ms - week * GPS_WEEK_MS) / 1000 < end - 0.001]
    if len(selected) < 2 or (selected[0].gps_ms - week * GPS_WEEK_MS) / 1000 - start > 1.5:
        raise ValueError("IMU 起点后 1.5 秒内需有有效 GNSS，且处理区间内至少有 2 个 GNSS 历元")
    first = selected[0]
    if options.initial_heading_deg is not None:
        heading = options.initial_heading_deg
    else:
        near = min(headings, key=lambda h: abs(h.gps_ms - first.gps_ms), default=None)
        if near is None or abs(near.gps_ms - first.gps_ms) > 250:
            raise ValueError("起点附近无有效双天线航向；请填初始航向或采集 UNIHEADINGA")
        heading = near.degrees + options.heading_offset_deg
    heading %= 360
    if first.velocity_valid and first.speed >= 0:
        track = radians(first.track)
        velocity = (first.speed * cos(track), first.speed * sin(track), -first.vertical_speed)
    else:
        velocity = (0.0, 0.0, 0.0)
    angle_degrees, velocity_scale = delta_scales(options.delta_ctrl)
    angle_scale = radians(angle_degrees)
    imu_rows = []
    fallback_intervals = 0
    for index, (sample, time_s) in enumerate(zip(imu, times)):
        angle_xyz = tuple(v * angle_scale for v in sample.angle_raw)
        if options.apply_startup_calibration:
            previous = imu[index-1] if index else None
            if previous and ((sample.count - previous.count) & 0xFFFF) not in (312, 313):
                raise ValueError("标定补偿遇到 IMU COUNT 不连续，请分段处理，不能把丢帧时长作为单帧增量间隔")
            if previous and previous.capture_ticks is not None and sample.capture_ticks is not None:
                if sample.drdy_sequence != previous.drdy_sequence + 1:
                    raise ValueError("标定补偿遇到 DRDY 序号不连续，请分段处理")
                dt = (sample.capture_ticks - previous.capture_ticks) / 4_000_000
            else:
                dt = times[index] - times[index-1] if index else median_dt
                fallback_intervals += 1
            if not 0.004 <= dt <= 0.006:
                raise ValueError("标定补偿的单帧间隔不在 4–6 ms，请检查时间记录")
            angle_xyz = tuple(a-b*dt for a,b in zip(angle_xyz, startup.stationary_rate))
        angle = _axis_values(angle_xyz, options)
        dv = _axis_values(tuple(v * velocity_scale for v in sample.velocity_raw), options)
        imu_rows.append((time_s, *angle, *dv))
    if first.velocity_valid:
        nearby = [row[1:4] for row, sample in zip(imu_rows, imu)
                  if abs(sample.gps_ms - first.gps_ms) <= 50]
        if nearby:
            omega_body = np.mean(nearby, axis=0) / median_dt
            lever_body = np.array([options.lever_forward_m, options.lever_right_m,
                                   options.lever_down_m])
            rotation = _body_to_ned(heading, options.initial_pitch_deg, options.initial_roll_deg)
            rotational_velocity = np.cross(rotation @ omega_body, rotation @ lever_body)
            velocity = tuple(float(v) for v in (np.array(velocity) - rotational_velocity))
    gnss_rows = [((p.gps_ms - week * GPS_WEEK_MS) / 1000, p.latitude, p.longitude, p.height,
                  max(p.north_std, 0.03), max(p.east_std, 0.03), max(p.down_std, 0.05))
                 for p in selected]
    warnings = []
    if options.apply_startup_calibration:
        mean = " / ".join(f"{degrees(v):+.6f}" for v in startup.stationary_rate)
        warnings.append(f"已应用板端启动标定：{startup.samples} 帧 / {startup.duration_us/1e6:.3f} s，"
                        f"扣除传感器 XYZ 静止均值 {mean} °/s；仅补偿角增量，原始日志不改写")
        warnings.append("静止均值包含地球自转；当前采用与监视器一致的去均值输入，并非分离地球自转后的真实零偏标定。"
                        "KF 初始陀螺残余零偏设为 0，21 状态残余误差估计保持启用")
        if fallback_intervals:
            warnings.append(f"{fallback_intervals} 帧补偿间隔无完整相邻 DRDY，使用回放时间差（首帧用中位间隔）")
    else:
        warnings.append("未应用启动标定：明确选择了原始 IMU 输入，使用 YAML 的陀螺零偏初值")
    if hardware:
        warnings.append(f"使用 {len(sync.pps)} 个 PPS 与 {captured} 帧 DRDY 时间戳；GPS 整秒归属仍需实测核验")
    else:
        warnings.append(f"未建立完整 PPS/DRDY 同步，使用串口到达拟合；残差 {fit_rms:.1f} ms，固定延迟未知")
    if not first.velocity_valid:
        warnings.append("首个 GNSS 无有效多普勒速度，初始速度设为 0")
    jumps = sum(((b.count - a.count) & 0xFFFF) not in (312, 313) for a, b in zip(imu, imu[1:]))
    if jumps:
        warnings.append(f"IMU COUNT 有 {jumps} 处非 312/313 步长，请检查丢帧")
    if not metadata or not metadata["profile"]["delta_ctrl_confirmed"]:
        warnings.append(f"DLT_CTRL=0x{options.delta_ctrl:04X} 无采集时已核对记录；须核对传感器实际寄存器")
    if metadata and not metadata.get("finished"):
        warnings.append("采集未正常结束，配套配置未完成日志完整性校验")
    if options.heading_aiding or options.velocity_aiding:
        warnings.append("离线增强模式：完整 21 状态，保留 GNSS 位置更新；持续航向="
                        f"{options.heading_aiding}，多普勒速度={options.velocity_aiding}。"
                        "同源 GNSS 残差用于一致性比较，不代表独立真值精度")
    else:
        warnings.append("KF-GINS 仅使用 GNSS 位置更新；双天线航向只用于初始航向，初始横滚/俯仰和安装轴向须核对")
    return Prepared(imu_rows, gnss_rows, selected, headings, first, heading, velocity,
                    bool(hardware), len(sync.pps), fit_rms, rejected, warnings, len(imu), week,
                    options.apply_startup_calibration)


def _yaml_path(path: Path) -> str:
    # Quoted YAML scalar, also accepted by yaml-cpp on Windows.
    return '"' + str(path).replace("\\", "/").replace('"', '\\"') + '"'


def _body_to_ned(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    yaw = radians(yaw_deg)
    pitch = radians(pitch_deg)
    roll = radians(roll_deg)
    cy, sy = cos(yaw), sin(yaw)
    cp, sp = cos(pitch), sin(pitch)
    cr, sr = cos(roll), sin(roll)
    # Body (forward/right/down) to navigation (north/east/down).
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _initial_imu_position(prepared: Prepared, options: KfOptions) -> tuple[float, float, float]:
    """Move the initial GNSS antenna geodetic position to the IMU origin."""
    p = prepared.first_position
    rotation = _body_to_ned(prepared.initial_heading, options.initial_pitch_deg,
                            options.initial_roll_deg)
    lever_ned = rotation @ np.array([options.lever_forward_m,
                                      options.lever_right_m, options.lever_down_m])
    latitude = radians(p.latitude)
    denominator = 1 - 0.00669437999014 * sin(latitude) ** 2
    prime_vertical = 6378137 / sqrt(denominator)
    meridian = 6378137 * (1 - 0.00669437999014) / denominator ** 1.5
    imu_lat = p.latitude - lever_ned[0] / (meridian + p.height) * 180 / pi
    imu_lon = p.longitude - lever_ned[1] / ((prime_vertical + p.height) * cos(latitude)) * 180 / pi
    imu_height = p.height + lever_ned[2]
    return float(imu_lat), float(imu_lon), float(imu_height)


def _config(prepared: Prepared, options: KfOptions, folder: Path) -> str:
    template = Path(options.config_path) if options.config_path else DEFAULT_CONFIG
    config = template.read_text(encoding="utf-8")
    init_lat, init_lon, init_height = _initial_imu_position(prepared, options)
    changes = {
        "imupath": _yaml_path(folder / "imu.txt"),
        "gnsspath": _yaml_path(folder / "gnss.txt"),
        "outputpath": _yaml_path(folder),
        "imudatalen": "7",
        "imudatarate": "200",
        "starttime": f"{max(prepared.imu_rows[0][0], prepared.gnss_rows[0][0] - 0.001):.9f}",
        "endtime": "-1",
        "initpos": f"[{init_lat:.10f}, {init_lon:.10f}, {init_height:.4f}]",
        "initvel": "[" + ", ".join(f"{v:.8f}" for v in prepared.initial_velocity) + "]",
        "initatt": f"[{options.initial_roll_deg:.8f}, {options.initial_pitch_deg:.8f}, {prepared.initial_heading:.8f}]",
        "antlever": f"[{options.lever_forward_m:.8f}, {options.lever_right_m:.8f}, {options.lever_down_m:.8f}]",
    }
    if prepared.startup_calibration_applied:
        changes["initgyrbias"] = "[0, 0, 0]"
    for key, value in changes.items():
        pattern = rf"(?m)^{re.escape(key)}\s*:.*$"
        config, count = re.subn(pattern, f"{key}: {value}", config)
        if count != 1:
            raise ValueError(f"KF-GINS 配置模板缺少唯一的 {key} 字段")
    return config


def _write_rows(path: Path, rows: list[tuple[float, ...]], format_spec: str = ".12f") -> None:
    with path.open("w", encoding="ascii", newline="\n") as output:
        for row in rows:
            output.write(" ".join(f"{value:{format_spec}}" for value in row) + "\n")


def _executable() -> Path:
    candidates = [UPSTREAM / "bin" / "KF-GINS.exe", UPSTREAM / "bin" / "Release" / "KF-GINS.exe",
                  UPSTREAM / "bin" / "KF-GINS"]
    result = next((path for path in candidates if path.is_file()), None)
    if result is None:
        raise ValueError("未找到 KF-GINS 可执行文件。先运行 tools/build_kf_gins.ps1 编译原算法。")
    return result


def _aided_executable() -> Path:
    folder = ROOT / "build-navigation" / "aided"
    result = next((p for p in (folder / "Release" / "kf_gins_aided.exe",
                               folder / "kf_gins_aided.exe", folder / "kf_gins_aided") if p.is_file()), None)
    if result is None:
        raise ValueError("未找到离线增强程序，请先运行 tools/build_kf_gins_aided.ps1。取消两个更新开关可运行原模式。")
    return result


def _aiding_rows(prepared: Prepared, options: KfOptions) -> list[tuple[float, ...]]:
    """Actual GPS epochs only; no interpolation or course-as-heading observations.

    Rows: time, pos-valid, BLH(rad/rad/m), pos-std, vel-valid, vNED, vel-std,
    heading-valid, measured baseline azimuth(rad), std(rad), baseline body azimuth.
    BESTNAVA tail: latency, age, horizontal speed, track, up speed, v-std, h-std.
    """
    events: dict[int, list[float]] = {}
    def event(gps_ms: int) -> list[float]:
        if gps_ms not in events:
            events[gps_ms] = [(gps_ms - prepared.week * GPS_WEEK_MS) / 1000] + [0.] * 18
        return events[gps_ms]

    velocity_used = velocity_rejected = heading_used = heading_rejected = 0
    for p, original in zip(prepared.positions, prepared.gnss_rows):
        row = event(p.gps_ms)
        row[1:8] = [1, radians(p.latitude), radians(p.longitude), p.height, *original[4:7]]
        if options.velocity_aiding:
            valid = (p.velocity_valid and all(isfinite(v) for v in (
                p.speed, p.track, p.vertical_speed, p.velocity_latency_s, p.velocity_age_s,
                p.horizontal_speed_std, p.vertical_speed_std)) and 0 <= p.speed < 200 and
                abs(p.vertical_speed) < 100 and 0 < p.horizontal_speed_std <= 1 and
                0 < p.vertical_speed_std <= 2 and abs(p.velocity_latency_s) < 1e-6 and
                abs(p.velocity_age_s) < 1e-6)
            if valid:
                track = radians(p.track)
                row[8:15] = [1, p.speed*cos(track), p.speed*sin(track), -p.vertical_speed,
                              max(.05, p.horizontal_speed_std), max(.05, p.horizontal_speed_std),
                              max(.05, p.vertical_speed_std)]
                velocity_used += 1
            else:
                velocity_rejected += 1
    if options.heading_aiding:
        start = max(prepared.imu_rows[0][0], prepared.gnss_rows[0][0] - .001)
        end = prepared.imu_rows[-1][0]
        headings = [h for h in prepared.headings
                    if start < (h.gps_ms - prepared.week * GPS_WEEK_MS) / 1000 <= end]
        fixed = {"NARROW_INT", "L1_INT"}
        lengths = [h.baseline_m for h in headings if h.solution in fixed and
                   isfinite(h.baseline_m) and .1 <= h.baseline_m <= 100]
        median_length = float(np.median(lengths)) if lengths else 0
        seen = set()
        for h in headings:
            if h.gps_ms in seen:
                raise ValueError("UNIHEADINGA 历元重复，增强模式不能重复使用航向观测")
            seen.add(h.gps_ms)
            valid = (h.solution in fixed and all(isfinite(v) for v in (h.degrees, h.std, h.baseline_m))
                     and 0 < h.std <= 5 and median_length > 0 and
                     abs(h.baseline_m - median_length) <= max(.05, .2 * median_length))
            if valid:
                event(h.gps_ms)[15:19] = [1, radians(h.degrees), radians(max(.5, h.std)),
                                         -radians(options.heading_offset_deg)]
                heading_used += 1
            else:
                heading_rejected += 1
    prepared.warnings.append(f"新增观测质量检查：速度通过 {velocity_used} / 拒绝 {velocity_rejected}；"
                             f"航向通过 {heading_used} / 拒绝 {heading_rejected}。滤波还会进行 NIS 残差检查")
    if velocity_rejected:
        prepared.warnings.append("速度拒绝原因包括状态/标准差异常或非零速度延迟/龄期；本增量仅接受当前 GPS 历元的多普勒速度")
    return [tuple(events[t]) for t in sorted(events)]


def _local_en(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    phi = radians(lat0)
    denominator = 1 - 0.00669437999014 * sin(phi) ** 2
    east = (lon - lon0) * 6378137 * cos(phi) / sqrt(denominator) * pi / 180
    north = (lat - lat0) * 6378137 * (1 - 0.00669437999014) / denominator ** 1.5 * pi / 180
    return east, north


def run_kf_gins(path: Path, options: KfOptions) -> FusionResult:
    prepared = prepare_log(path, options)
    aided = options.heading_aiding or options.velocity_aiding
    executable = _aided_executable() if aided else _executable()
    with tempfile.TemporaryDirectory(prefix="terramind_kfgins_") as temporary:
        folder = Path(temporary)
        if not str(folder).isascii():
            raise ValueError("KF-GINS Windows 路径须为 ASCII；请将临时目录 TMP 设置为纯英文路径")
        _write_rows(folder / "imu.txt", prepared.imu_rows)
        _write_rows(folder / "gnss.txt", prepared.gnss_rows)
        (folder / "run.yaml").write_text(_config(prepared, options, folder), encoding="utf-8")
        command = [str(executable), str(folder / "run.yaml")]
        if aided:
            _write_rows(folder / "aiding.txt", _aiding_rows(prepared, options), ".17g")
            command.append(str(folder / "aiding.txt"))
        try:
            completed = subprocess.run(command, cwd=UPSTREAM,
                                       capture_output=True, text=True, errors="replace", timeout=1800)
        except subprocess.TimeoutExpired as exc:
            raise ValueError("KF-GINS 运行超过 30 分钟，已停止。请缩短日志处理区间。") from exc
        nav = folder / "KF_GINS_Navresult.nav"
        if completed.returncode != 0 or not nav.is_file():
            tail = (completed.stdout + "\n" + completed.stderr)[-2000:]
            raise ValueError("KF-GINS 运行失败：\n" + tail)
        if aided:
            summary = re.search(r"aiding: velocity_used=(\d+) velocity_rejected=(\d+) heading_used=(\d+) heading_rejected=(\d+)",
                                completed.stdout)
            if summary is None:
                raise ValueError("增强程序缺少观测更新统计，请重新编译")
            vu, vr, hu, hr = summary.groups()
            prepared.warnings.append(f"滤波新增观测：速度采用 {vu} / NIS 拒绝 {vr}；航向采用 {hu} / NIS 拒绝 {hr}")
        frames = []
        p0 = prepared.first_position
        with nav.open("r", encoding="ascii") as source:
            for line in source:
                values = [float(v) for v in line.split()]
                if len(values) != 11 or not all(isfinite(v) for v in values):
                    raise ValueError("KF-GINS 导航结果含无效行")
                _, tow, lat, lon, height, vn, ve, vd, roll, pitch, heading = values
                east, north = _local_en(lat, lon, p0.latitude, p0.longitude)
                frames.append(ReplayFrame(tow * 1000, east, north, ve, vn, heading % 360, 0,
                                          height, roll, pitch, lat, lon, vd, prepared.week))
        if not frames:
            raise ValueError("KF-GINS 没有产生导航结果")
        frames = _with_diagnostics(frames, prepared.headings, prepared.week, options)
    gnss = []
    for p in prepared.positions:
        east, north = _local_en(p.latitude, p.longitude, p0.latitude, p0.longitude)
        gnss.append(GnssPoint((p.gps_ms - prepared.week * GPS_WEEK_MS), east, north,
                              east, north, p.solution))
    return KfFusionResult(frames, gnss, p0.latitude, p0.longitude, 1.0, prepared.fit_rms_ms,
                        None, prepared.imu_count, len(prepared.positions), len(prepared.headings),
                        prepared.rejected, prepared.warnings, prepared.hardware_sync,
                        prepared.pps_count, heading_aiding=options.heading_aiding,
                        velocity_aiding=options.velocity_aiding)
