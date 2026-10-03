"""B,1 startup stationary gyro mean. I records remain uncorrected sensor data."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class StartupCalibration:
    mcu_ms: int
    mode: int
    delta_ctrl: int
    samples: int
    duration_us: int
    rate_nrad_s: tuple[int, int, int]
    std_nrad_s: tuple[int, int, int]
    temperature_mc: int

    @property
    def stationary_rate(self):
        return tuple(v * 1e-9 for v in self.rate_nrad_s)

    def identity(self):
        return (self.mode, self.delta_ctrl, self.samples, self.duration_us,
                self.rate_nrad_s, self.std_nrad_s, self.temperature_mc)


def parse_calibration(line: str) -> StartupCalibration:
    fields = line.strip().split(",")
    if len(fields) != 14 or fields[:2] != ["B", "1"]:
        raise ValueError("启动标定记录格式/版本错误")
    v = [int(x) for x in fields[2:]]
    result = StartupCalibration(*v[:5], tuple(v[5:8]), tuple(v[8:11]), v[11])
    if (result.mode not in (2, 3) or not 0 <= result.delta_ctrl <= 65535 or
            result.samples < 167 or not 1_000_000 <= result.duration_us <= 30_006_000 or
            any(abs(x) > 100_000_000 for x in result.rate_nrad_s) or
            any(x < 0 or x > 100_000_000 for x in result.std_nrad_s)):
        raise ValueError("启动标定记录数值无效")
    return result


def read_calibration(path: Path) -> StartupCalibration | None:
    result = None
    with path.open("r", encoding="ascii", errors="replace") as source:
        for line in source:
            if line.startswith("B,"):
                current = parse_calibration(line)
                if result and result.identity() != current.identity():
                    raise ValueError("日志包含不同启动标定结果，请分段处理")
                result = current
    return result


def load_startup_calibration(path: Path, metadata: dict | None = None) -> StartupCalibration | None:
    """Read B and cross-check the capture sidecar; never guess missing means.

    Callers must verify the sidecar's log hash with load_capture(verify_log=True).
    A JSON-only fallback requires a successfully completed capture.
    """
    record = read_calibration(path)
    data = metadata.get("startup_calibration") if metadata else None
    if data is None:
        return record
    try:
        rates, std = data["rate_nrad_s"], data["std_nrad_s"]
        if len(rates) != 3 or len(std) != 3:
            raise ValueError("标定向量必须有三个轴")
        values = [data[key] for key in ("mcu_ms", "mode", "delta_ctrl", "samples", "duration_us")]
        values += list(rates) + list(std) + [data["temperature_mc"]]
        if any(type(v) is not int for v in values):
            raise ValueError("标定字段必须为整数")
        saved = parse_calibration("B,1," + ",".join(map(str, values)))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"JSON 启动标定记录无效：{exc}") from exc
    if record and record.identity() != saved.identity():
        raise ValueError("日志 B 与 JSON 启动标定记录冲突")
    if record is None and metadata.get("finished") is not True:
        raise ValueError("仅 JSON 含启动标定，但采集未完成，无法确认适用于本日志")
    return record or saved
