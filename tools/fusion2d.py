"""Experimental 2-D loose GNSS/IMU EKF for offline USART6 monitor logs.

This is a planar replay model. It does not compensate roll/pitch or establish
absolute hardware synchronization, so its output is not ground truth.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import asdict, dataclass
from math import cos, hypot, pi, radians, sin, sqrt
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from monitor_protocol import unicore_crc32
from sync_timeline import MAX_GNSS_OUTPUT_LATENCY_MS, analyze_sync


@dataclass
class Geometry:
    # Master antenna relative to IMU origin, in IMU X/Y coordinates.
    lever_x_m: float = 0.0
    lever_y_m: float = 0.0
    # IMU +X heading = GNSS master-to-slave heading + this clockwise angle.
    heading_offset_deg: float = 0.0
    # +1 when IMU +Z angular rate agrees with clockwise GNSS heading change.
    gyro_heading_sign: int = 1
    # +1 when IMU +Y points to the right of +X; -1 when it points left.
    imu_y_right_sign: int = 1
    # Shift GNSS measurement epoch earlier than the affine UART arrival fit.
    gnss_delay_ms: float = 0.0

    def validate(self) -> None:
        if self.gyro_heading_sign not in (-1, 1) or self.imu_y_right_sign not in (-1, 1):
            raise ValueError("角速度方向和 IMU Y 方向只能填 +1 或 -1")
        for key, value in asdict(self).items():
            if not np.isfinite(value):
                raise ValueError(f"{key} 必须是有限数值")
        if abs(self.lever_x_m) > 100 or abs(self.lever_y_m) > 100:
            raise ValueError("杆臂超过 100 m，请核对单位")
        if abs(self.gnss_delay_ms) > 500:
            raise ValueError("GNSS 时间平移限于 ±500 ms")


@dataclass
class ImuObs:
    mcu_ms: int
    count: int
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    accel_x_mps2: float
    accel_y_mps2: float
    capture_ticks: Optional[int] = None


@dataclass
class PosObs:
    mcu_ms: int
    gps_ms: int
    status: str
    solution: str
    lat_deg: float
    lon_deg: float
    lat_std_m: float
    lon_std_m: float
    speed_mps: float
    track_deg: float
    speed_std_mps: float
    vel_valid: bool


@dataclass
class HeadingObs:
    mcu_ms: int
    gps_ms: int
    status: str
    solution: str
    heading_deg: float
    heading_std_deg: float
    baseline_m: float


@dataclass
class ReplayFrame:
    mcu_ms: float
    east_m: float
    north_m: float
    ve_mps: float
    vn_mps: float
    heading_deg: float
    gyro_bias_dps: float


@dataclass
class GnssPoint:
    mcu_ms: float
    antenna_e_m: float
    antenna_n_m: float
    imu_e_m: float
    imu_n_m: float
    solution: str


@dataclass
class StaticImuCalibration:
    applied: bool
    reason: str
    sample_count: int = 0
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0
    accel_x_mps2: float = 0.0
    accel_y_mps2: float = 0.0


@dataclass
class FusionResult:
    frames: List[ReplayFrame]
    gnss: List[GnssPoint]
    origin_lat_deg: float
    origin_lon_deg: float
    gps_to_mcu_slope: float
    gps_arrival_fit_rms_ms: float
    gyro_heading_correlation: Optional[float]
    imu_count: int
    position_count: int
    heading_count: int
    rejected_lines: int
    warnings: List[str]
    hardware_sync: bool = False
    pps_count: int = 0
    static_calibration: Optional[StaticImuCalibration] = None


def _wrap_rad(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _lever_en(heading_rad: float, geometry: Geometry) -> Tuple[float, float]:
    x, y = geometry.lever_x_m, geometry.lever_y_m
    return x * sin(heading_rad) + y * cos(heading_rad), x * cos(heading_rad) - y * sin(heading_rad)


def _parse_log(path: Path) -> Tuple[List[ImuObs], List[PosObs], List[HeadingObs], int]:
    imu: List[ImuObs] = []
    positions: List[PosObs] = []
    headings: List[HeadingObs] = []
    rejected = 0
    with path.open("r", encoding="ascii", errors="replace") as source:
        for line in source:
            line = line.strip()
            try:
                if line.startswith("I,"):
                    f = line.split(",")
                    if len(f) != 13 or f[2] != "2":
                        raise ValueError("only raw mode 2 is supported")
                    imu.append(ImuObs(
                        int(f[1]), int(f[3]),
                        int(f[7]) / (66 * 65536),
                        int(f[8]) / (66 * 65536),
                        int(f[9]) / (66 * 65536),
                        int(f[10]) * 9.80665 / (2500 * 65536),
                        int(f[11]) * 9.80665 / (2500 * 65536),
                    ))
                elif line.startswith("T,"):
                    f = line.split(",")
                    if len(f) != 8:
                        raise ValueError("IMU timing fields")
                    if imu and imu[-1].mcu_ms == int(f[1]) and imu[-1].count == int(f[2]) and int(f[3]) != 0:
                        imu[-1].capture_ticks = (int(f[4]) << 32) | int(f[5])
                elif line.startswith("N,"):
                    _, mcu_text, raw = line.split(",", 2)
                    if not raw.startswith(("#BESTNAVA,", "#UNIHEADINGA,")):
                        continue
                    payload, check = raw.rsplit("*", 1)
                    if len(check) != 8 or unicore_crc32(payload[1:]) != int(check, 16):
                        raise ValueError("GNSS CRC")
                    header_text, body_text = payload[1:].split(";", 1)
                    header = header_text.split(",")
                    body = body_text.split(",")
                    if len(header) != 10:
                        raise ValueError("GNSS header")
                    gps_ms = int(header[4]) * 604800000 + int(header[5])
                    if header[3] != "FINE":
                        continue
                    if raw.startswith("#BESTNAVA,"):
                        if len(body) != 30:
                            raise ValueError("BESTNAVA fields")
                        if body[0] != "SOL_COMPUTED" or body[1] == "NONE":
                            continue
                        positions.append(PosObs(
                            int(mcu_text), gps_ms, body[0], body[1],
                            float(body[2]), float(body[3]), float(body[7]),
                            float(body[8]), float(body[25]), float(body[26]),
                            float(body[29]), body[21] == "SOL_COMPUTED" and
                            body[22] == "DOPPLER_VELOCITY",
                        ))
                    else:
                        if len(body) != 17:
                            raise ValueError("UNIHEADINGA fields")
                        headings.append(HeadingObs(
                            int(mcu_text), gps_ms, body[0], body[1],
                            float(body[3]), float(body[6]), float(body[2]),
                        ))
            except (ValueError, IndexError, UnicodeError):
                rejected += 1
    return imu, positions, headings, rejected


class _Ekf:
    # State: east, north, v_e, v_n, heading clockwise from north,
    # accelerometer X/Y bias, gyro Z bias (rad/s).
    def __init__(self, east: float, north: float, ve: float, vn: float,
                 heading: float) -> None:
        self.x = np.array([east, north, ve, vn, heading, 0., 0., 0.], dtype=float)
        self.P = np.diag([0.4**2, 0.4**2, 0.3**2, 0.3**2,
                          radians(8)**2, 0.3**2, 0.3**2, radians(3)**2])
        self.rejected = 0

    def propagate(self, dt: float, accel_x: float, accel_y: float,
                  gyro_rad_s: float) -> None:
        if dt <= 0:
            return
        e, n, ve, vn, h, bax, bay, bg = self.x
        ax, ay = accel_x - bax, accel_y - bay
        sh, ch = sin(h), cos(h)
        ae = ax * sh + ay * ch
        an = ax * ch - ay * sh
        self.x[0] += ve * dt + 0.5 * ae * dt * dt
        self.x[1] += vn * dt + 0.5 * an * dt * dt
        self.x[2] += ae * dt
        self.x[3] += an * dt
        self.x[4] = _wrap_rad(h + (gyro_rad_s - bg) * dt)
        F = np.eye(8)
        F[0, 2] = F[1, 3] = dt
        F[0, 4], F[1, 4] = 0.5 * an * dt * dt, -0.5 * ae * dt * dt
        F[2, 4], F[3, 4] = an * dt, -ae * dt
        F[0, 5], F[0, 6] = -0.5 * sh * dt * dt, -0.5 * ch * dt * dt
        F[1, 5], F[1, 6] = -0.5 * ch * dt * dt, 0.5 * sh * dt * dt
        F[2, 5], F[2, 6] = -sh * dt, -ch * dt
        F[3, 5], F[3, 6] = -ch * dt, sh * dt
        F[4, 7] = -dt
        q = np.zeros(8)
        q[0:2] = (0.5 * 0.7 * dt * dt) ** 2
        q[2:4] = (0.7 * dt) ** 2
        q[4] = (radians(0.5) * dt) ** 2
        q[5:7] = 0.015**2 * dt
        q[7] = radians(0.015)**2 * dt
        self.P = F @ self.P @ F.T + np.diag(q)

    def update(self, index: int, value: float, variance: float,
               max_innovation: float = float("inf"), circular: bool = False) -> None:
        residual = value - self.x[index]
        if circular:
            residual = _wrap_rad(residual)
        if abs(residual) > max_innovation:
            self.rejected += 1
            return
        column = self.P[:, index].copy()
        gain = column / (self.P[index, index] + variance)
        self.x += gain * residual
        if circular:
            self.x[index] = _wrap_rad(self.x[index])
        eye_kh = np.eye(8)
        eye_kh[:, index] -= gain
        self.P = eye_kh @ self.P @ eye_kh.T + np.outer(gain, gain) * variance


def _estimate_static_imu(imu: List[ImuObs], positions: List[PosObs],
                         imu_event_ms: Callable[[ImuObs], float],
                         gps_event_ms: Callable[[int], float]) -> StaticImuCalibration:
    """Only remove the first 3 s apparent X/Y and Z offsets when stillness is credible."""
    start_ms = imu_event_ms(imu[0])
    samples = [obs for obs in imu if start_ms <= imu_event_ms(obs) < start_ms + 3000]

    def reject(reason: str) -> StaticImuCalibration:
        return StaticImuCalibration(False, reason, len(samples))

    if len(samples) < 400 or imu_event_ms(samples[-1]) - start_ms < 2950:
        return reject("前 3 秒 IMU 样本不足或时间覆盖不完整")
    sample_times = np.array([imu_event_ms(obs) for obs in samples])
    if np.max(np.diff(sample_times)) > 100:
        return reject("前 3 秒 IMU 有超过 100 ms 的采样空缺")
    rates = np.array([[obs.gyro_x_dps, obs.gyro_y_dps, obs.gyro_z_dps]
                      for obs in samples])
    accel_x = np.array([obs.accel_x_mps2 for obs in samples])
    accel_y = np.array([obs.accel_y_mps2 for obs in samples])
    if not all(np.isfinite(values).all() for values in (rates, accel_x, accel_y)):
        return reject("IMU 样本含无效数值")
    if np.max(np.abs(np.mean(rates, axis=0))) > 0.5 or np.max(np.std(rates, axis=0)) > 0.15:
        return reject("三轴角速度存在转动或波动过大，无法确认静止")
    if max(float(np.std(accel_x)), float(np.std(accel_y))) > 0.12:
        return reject("水平加速度波动过大，无法确认静止")

    valid_pos = [p for p in positions
                 if start_ms <= gps_event_ms(p.gps_ms) < start_ms + 3000
                 and p.vel_valid and np.isfinite(p.speed_mps)
                 and np.isfinite(p.speed_std_mps) and 0 <= p.speed_std_mps <= 0.25]
    if len(valid_pos) < 5 or gps_event_ms(valid_pos[-1].gps_ms) - gps_event_ms(valid_pos[0].gps_ms) < 1500:
        return reject("前 3 秒缺少足够的可信 GNSS 多普勒速度，不能排除匀速运动")
    speeds = np.array([p.speed_mps for p in valid_pos])
    if not np.isfinite(speeds).all() or np.min(speeds) < 0:
        return reject("GNSS 速度含无效数值")
    if float(np.quantile(speeds, 0.9)) > 0.2 or float(np.max(speeds)) > 0.35:
        return reject("GNSS 速度显示前 3 秒存在运动")
    lat0, lon0 = valid_pos[0].lat_deg, valid_pos[0].lon_deg
    east_scale = 111320 * cos(radians(lat0))
    east = np.array([(p.lon_deg - lon0) * east_scale for p in valid_pos])
    north = np.array([(p.lat_deg - lat0) * 111000 for p in valid_pos])
    if not np.isfinite(east).all() or not np.isfinite(north).all():
        return reject("GNSS 位置含无效数值")
    if float(np.max(np.hypot(east, north))) > 0.25 or hypot(east[-1], north[-1]) > 0.15:
        return reject("GNSS 位置显示前 3 秒存在位移或定位波动过大")
    gyro_mean = np.mean(rates, axis=0)
    return StaticImuCalibration(True, "前 3 秒静止检查通过", len(samples),
                                float(gyro_mean[0]), float(gyro_mean[1]),
                                float(gyro_mean[2]), float(np.mean(accel_x)),
                                float(np.mean(accel_y)))


def run_fusion(path: Path, geometry: Geometry,
               calibrate_static_imu: bool = False) -> FusionResult:
    geometry.validate()
    imu, positions, headings, rejected = _parse_log(path)
    if len(imu) < 20 or len(positions) < 2:
        raise ValueError("需要至少 20 帧模式 2 IMU 和 2 条有效 BESTNAVA")
    imu.sort(key=lambda x: x.mcu_ms)
    positions.sort(key=lambda x: x.gps_ms)
    headings.sort(key=lambda x: x.gps_ms)
    gps0 = positions[0].gps_ms
    xfit = np.array([p.gps_ms - gps0 for p in positions], dtype=float)
    yfit = np.array([p.mcu_ms for p in positions], dtype=float)
    slope, offset = np.polyfit(xfit, yfit, 1)
    fit_rms = float(np.sqrt(np.mean((yfit - (slope * xfit + offset)) ** 2)))

    sync = analyze_sync(path)
    captured = [obs for obs in imu if obs.capture_ticks is not None]
    use_hardware = sync.locked and len(captured) >= 0.98 * len(imu)
    if use_hardware:
        assert sync.tick_rate is not None
        local_capture_ms = np.array([sync.tick_to_local_ms(obs.capture_ticks) for obs in captured])
        local_mcu_ms = np.array([obs.mcu_ms for obs in captured], dtype=float)
        capture_slope, capture_offset = np.polyfit(local_mcu_ms - local_mcu_ms[0],
                                                  local_capture_ms, 1)

    def imu_event_ms(obs: ImuObs) -> float:
        if use_hardware:
            if obs.capture_ticks is not None:
                return sync.tick_to_local_ms(obs.capture_ticks)
            return float(capture_slope * (obs.mcu_ms - local_mcu_ms[0]) + capture_offset)
        return float(obs.mcu_ms)

    def event_ms(gps_ms: int) -> float:
        if use_hardware:
            return sync.gps_ms_to_local_ms(gps_ms)
        return slope * (gps_ms - gps0) + offset - geometry.gnss_delay_ms

    calibration = (_estimate_static_imu(imu, positions, imu_event_ms, event_ms)
                   if calibrate_static_imu else None)
    if calibration and calibration.applied:
        for obs in imu:
            obs.gyro_x_dps -= calibration.gyro_x_dps
            obs.gyro_y_dps -= calibration.gyro_y_dps
            obs.gyro_z_dps -= calibration.gyro_z_dps
            obs.accel_x_mps2 -= calibration.accel_x_mps2
            obs.accel_y_mps2 -= calibration.accel_y_mps2

    def corrected_control(obs: ImuObs) -> Tuple[float, float, float]:
        return (obs.accel_x_mps2,
                obs.accel_y_mps2 * geometry.imu_y_right_sign,
                radians(obs.gyro_z_dps * geometry.gyro_heading_sign))

    lat0, lon0 = positions[0].lat_deg, positions[0].lon_deg
    lat_rad = radians(lat0)
    a, eccentricity2 = 6378137.0, 0.00669437999014
    denom = 1 - eccentricity2 * sin(lat_rad)**2
    east_scale = a * cos(lat_rad) / sqrt(denom) * pi / 180
    north_scale = a * (1 - eccentricity2) / denom**1.5 * pi / 180

    def en(p: PosObs) -> Tuple[float, float]:
        return (p.lon_deg - lon0) * east_scale, (p.lat_deg - lat0) * north_scale

    heading_by_gps: Dict[int, HeadingObs] = {
        h.gps_ms: h for h in headings
        if h.status == "SOL_COMPUTED" and h.solution in ("NARROW_INT", "NARROW_FLOAT", "L1_INT", "L1_FLOAT")
        and 0 < h.heading_std_deg < 20 and h.baseline_m > 0
    }
    imu_times = [imu_event_ms(sample) for sample in imu]
    gyro_prefix = np.concatenate(([0.0], np.cumsum(
        [sample.gyro_z_dps * geometry.gyro_heading_sign for sample in imu])))
    paired_rates = []
    valid_headings = sorted(heading_by_gps.values(), key=lambda h: h.gps_ms)
    for prior, current in zip(valid_headings, valid_headings[1:]):
        elapsed_s = (current.gps_ms - prior.gps_ms) / 1000.0
        if not 0.05 <= elapsed_s <= 0.25:
            continue
        start = bisect_right(imu_times, event_ms(prior.gps_ms))
        end = bisect_right(imu_times, event_ms(current.gps_ms))
        if end <= start:
            continue
        gnss_rate = np.degrees(_wrap_rad(radians(current.heading_deg - prior.heading_deg))) / elapsed_s
        gyro_rate = float((gyro_prefix[end] - gyro_prefix[start]) / (end - start))
        paired_rates.append((gnss_rate, gyro_rate))
    correlation: Optional[float] = None
    if len(paired_rates) >= 20:
        rates = np.array(paired_rates)
        if np.std(rates[:, 0]) > 0.5 and np.std(rates[:, 1]) > 0.5:
            correlation = float(np.corrcoef(rates[:, 0], rates[:, 1])[0, 1])
    first = positions[0]
    first_h = heading_by_gps.get(first.gps_ms)
    if first_h is None:
        candidates = [(abs(h.gps_ms - first.gps_ms), h) for h in heading_by_gps.values()]
        if candidates and min(candidates, key=lambda item: item[0])[0] <= 200:
            first_h = min(candidates, key=lambda item: item[0])[1]
    warnings: List[str] = []
    if first_h is not None:
        heading_rad = radians(first_h.heading_deg + geometry.heading_offset_deg)
    elif first.speed_mps > 1:
        heading_rad = radians(first.track_deg + geometry.heading_offset_deg)
        warnings.append("起始航向使用 GNSS 运动方向；低速或倒车时可能错误")
    else:
        heading_rad = radians(geometry.heading_offset_deg)
        warnings.append("起始时无可信双天线航向，初始角度仅为安装偏角")
    antenna_e, antenna_n = en(first)
    lever_e, lever_n = _lever_en(heading_rad, geometry)
    track_rad = radians(first.track_deg)
    ve, vn = first.speed_mps * sin(track_rad), first.speed_mps * cos(track_rad)
    imu_index = max(0, bisect_right(imu_times, event_ms(first.gps_ms)) - 1)
    initial_omega = corrected_control(imu[imu_index])[2]
    ve -= initial_omega * lever_n
    vn += initial_omega * lever_e
    ekf = _Ekf(antenna_e - lever_e, antenna_n - lever_n, ve, vn, heading_rad)
    first_t = event_ms(first.gps_ms)
    events = []
    for obs in imu:
        events.append((imu_event_ms(obs), 2, obs))
    for obs in headings:
        events.append((event_ms(obs.gps_ms), 0, obs))
    for obs in positions:
        events.append((event_ms(obs.gps_ms), 1, obs))
    events.sort(key=lambda x: (x[0], x[1]))
    control = (0.0, 0.0, 0.0)
    previous_time = first_t
    frames: List[ReplayFrame] = []
    gnss_points: List[GnssPoint] = []
    for event_t, kind, obs in events:
        if event_t < first_t:
            if kind == 2:
                control = corrected_control(obs)
            continue
        if event_t > previous_time:
            remaining = (event_t - previous_time) / 1000.0
            while remaining > 1e-9:
                step = min(remaining, 0.02)
                ekf.propagate(step, *control)
                remaining -= step
            previous_time = event_t
        if kind == 2:
            control = corrected_control(obs)
            frames.append(ReplayFrame(
                event_t, float(ekf.x[0]), float(ekf.x[1]), float(ekf.x[2]),
                float(ekf.x[3]), float(np.degrees(ekf.x[4]) % 360),
                float(np.degrees(ekf.x[7])),
            ))
        elif kind == 0:
            h = obs
            if h.gps_ms in heading_by_gps:
                sigma = max(radians(h.heading_std_deg), radians(1.0))
                ekf.update(4, radians(h.heading_deg + geometry.heading_offset_deg),
                           sigma**2, max_innovation=radians(120), circular=True)
        else:
            p = obs
            ant_e, ant_n = en(p)
            h_obs = heading_by_gps.get(p.gps_ms)
            lever_h = (radians(h_obs.heading_deg + geometry.heading_offset_deg)
                       if h_obs else float(ekf.x[4]))
            le, ln = _lever_en(lever_h, geometry)
            pe, pn = ant_e - le, ant_n - ln
            floor = {"NARROW_INT": 0.08, "L1_INT": 0.1,
                     "NARROW_FLOAT": 0.3, "L1_FLOAT": 0.5,
                     "PSRDIFF": 2.0, "SINGLE": 5.0}.get(p.solution, 5.0)
            ekf.update(0, pe, max(p.lon_std_m, floor)**2, max_innovation=10)
            ekf.update(1, pn, max(p.lat_std_m, floor)**2, max_innovation=10)
            if p.vel_valid and p.speed_mps >= 0:
                track = radians(p.track_deg)
                v_e = p.speed_mps * sin(track)
                v_n = p.speed_mps * cos(track)
                # GNSS reports antenna velocity. Correct rotational lever-arm velocity.
                omega = control[2] - float(ekf.x[7])
                v_e -= omega * ln
                v_n += omega * le
                v_sigma = max(p.speed_std_mps, 0.05 if p.solution == "NARROW_INT" else 0.2)
                ekf.update(2, v_e, v_sigma**2, max_innovation=3)
                ekf.update(3, v_n, v_sigma**2, max_innovation=3)
            gnss_points.append(GnssPoint(event_t, ant_e, ant_n, pe, pn, p.solution))
    if not frames:
        raise ValueError("GNSS 首历元后没有可回放的 IMU 帧")
    if use_hardware:
        warnings.append(f"已用 {len(sync.pps)} 个 PPS 和 {len(captured)} 个 DRDY 时间戳；GPS 整秒归属假设 BESTNAVA 串口延迟小于 {MAX_GNSS_OUTPUT_LATENCY_MS} ms")
        warnings.append("请核实 UM982 消息固定延迟；当前整秒归属仅为条件配对，仍可能差整整 1 秒")
    elif sync.pps:
        warnings.append("检测到 PPS 记录，但 GPS 整秒或 IMU/DRDY 配对不足，已退回串口到达时间拟合")
    if not use_hardware and abs((slope - 1) * 1000) > 0.5:
        warnings.append(f"MCU/GPS 漂移约 {(slope - 1) * 1000:+.2f} ms/s；已拟合频率，固定时间偏移仍未知")
    if correlation is not None and correlation < 0.3:
        warnings.append(f"GNSS 航向变化与所选 Z 陀螺方向相关性 {correlation:+.2f}，请核对安装方向/符号")
    if not use_hardware and geometry.gnss_delay_ms == 0:
        warnings.append("GNSS 时间平移为 0 ms，实际串口/解算延迟未经标定")
    warnings.append("二维模型未补偿横滚、俯仰及重力投影；轨迹不能作为精度真值")
    if ekf.rejected:
        warnings.append(f"滤波器拒绝 {ekf.rejected} 次大残差观测")
    return FusionResult(frames, gnss_points, lat0, lon0, float(slope), fit_rms, correlation,
                        len(imu), len(positions), len(headings), rejected, warnings,
                        use_hardware, len(sync.pps), calibration)
