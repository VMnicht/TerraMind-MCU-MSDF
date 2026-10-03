"""Operator-declared G365 settings and a sidecar for untouched serial bytes.

No setting here is a sensor-register readback or a command sent to the MCU.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from startup_calibration import parse_calibration


@dataclass(frozen=True)
class CaptureProfile:
    expected_mode: int = 3
    delta_ctrl: int = 0x0048  # Planned dynamic-capture setting; confirm physical readback.
    delta_ctrl_confirmed: bool = False
    sample_hz: int = 200

    def validate(self) -> None:
        if type(self.expected_mode) is not int or self.expected_mode not in (2, 3):
            raise ValueError("期望 IMU 模式必须为 2 或 3")
        if type(self.delta_ctrl) is not int or not 0 <= self.delta_ctrl <= 0xFFFF:
            raise ValueError("DLT_CTRL 必须为 0x0000–0xFFFF")
        if type(self.delta_ctrl_confirmed) is not bool:
            raise ValueError("DLT_CTRL 核对状态必须为布尔值")
        if type(self.sample_hz) is not int or self.sample_hz != 200:
            raise ValueError("当前采集配置仅支持 200 Hz")


def delta_scales(delta_ctrl: int) -> tuple[float, float]:
    CaptureProfile(delta_ctrl=delta_ctrl).validate()
    return ((1 / 66 / 2000) * (1 << ((delta_ctrl >> 4) & 15)) / 65536,
            (0.4 / 1000 * 9.80665 / 2000) * (1 << (delta_ctrl & 15)) / 65536)


def sidecar_path(log: Path) -> Path:
    return log.with_suffix(log.suffix + ".capture.json")


def load_capture(log: Path, verify_log: bool = False) -> dict | None:
    path = sidecar_path(log)
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema") != 1:
        raise ValueError("不支持的采集配置文件版本")
    try:
        profile = CaptureProfile(**data["profile"])
        profile.validate()
    except (KeyError, TypeError) as exc:
        raise ValueError("采集配置文件缺少有效 profile") from exc
    data["profile"] = asdict(profile)
    if verify_log and data.get("finished"):
        digest = hashlib.sha256()
        size = 0
        with log.open("rb") as source:
            for block in iter(lambda: source.read(65536), b""):
                size += len(block)
                digest.update(block)
        if size != data.get("bytes") or digest.hexdigest() != data.get("sha256"):
            raise ValueError("采集配置与日志内容不匹配（字节数或 SHA256 不同）")
    return data


class RawCapture:
    """Called under InputWorker's lock; GUI queue loss cannot affect this file."""

    def __init__(self, path: Path, profile: CaptureProfile) -> None:
        profile.validate()
        self.path = path
        self.profile = profile
        self.pending = bytearray()
        self.digest = hashlib.sha256()
        self.modes: set[int] = set()
        self.versions: set[str] = set()
        self.data = {"schema": 1, "profile": asdict(profile),
                     "configuration_source": "operator_declared_not_sensor_readback",
                     "started_utc": datetime.now(timezone.utc).isoformat(),
                     "finished": False, "bytes": 0}
        # Make a sidecar failure visible before truncating the chosen log.
        self._save_metadata()
        self.file = path.open("wb")

    def _save_metadata(self) -> None:
        self.data.update(observed_modes=sorted(self.modes),
                         observed_versions=sorted(self.versions),
                         sha256=self.digest.hexdigest())
        target = sidecar_path(self.path)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")
        temporary.replace(target)

    def write(self, block: bytes) -> None:
        self.file.write(block)
        self.file.flush()
        self.digest.update(block)
        self.data["bytes"] += len(block)
        self.pending.extend(block)
        while b"\n" in self.pending:
            row, _, self.pending = self.pending.partition(b"\n")
            try:
                fields = row.decode("ascii").strip().split(",")
                if fields[0] == "I" and len(fields) == 13:
                    mode = int(fields[2])
                    if mode in (2, 3):
                        self.modes.add(mode)
                elif fields[0] == "V" and len(row) < 128:
                    self.versions.add(row.decode("ascii").strip())
                elif fields[0] == "B":
                    self.data["startup_calibration"] = asdict(parse_calibration(row.decode("ascii")))
            except (ValueError, UnicodeError):
                pass  # Corrupt bytes are still preserved above.
        if len(self.pending) > 2048:
            self.pending.clear()

    def close(self, error: str = "") -> None:
        self.file.close()
        self.data.update(finished=not bool(error), error=error,
                         ended_utc=datetime.now(timezone.utc).isoformat())
        self._save_metadata()
