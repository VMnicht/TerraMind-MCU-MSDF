"""Run four offline aiding modes with identical IMU, geometry and noise settings.

Reports consistency with the input GNSS, NOT independent ground-truth accuracy.
No changes to source logs; outputs contain GNSS positions and should be kept locally.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

import numpy as np

from kf_gins_bridge import KfOptions, prepare_log, run_kf_gins, _local_en
from kf_gins_diagnostics import write_diagnostics_csv
from sync_timeline import GPS_WEEK_MS

MODES = {"original": (False, False), "heading": (True, False),
         "velocity": (False, True), "heading_velocity": (True, True)}


def distribution(values) -> dict:
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"count": 0}
    return {"count": len(values), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values)),
            "rms": float(np.sqrt(np.mean(values**2)))}


def metrics(prepared, result) -> dict:
    frames = result.frames
    t = np.array([f.mcu_ms / 1000 for f in frames])
    gt = np.array([(p.gps_ms - prepared.week * GPS_WEEK_MS) / 1000 for p in prepared.positions])
    inside = (gt >= t[0]) & (gt <= t[-1])
    gt = gt[inside]
    p0 = prepared.first_position
    xy = np.array([_local_en(p.latitude, p.longitude, p0.latitude, p0.longitude)
                   for p in prepared.positions])[inside]
    est = np.column_stack([np.interp(gt, t, [getattr(f, key) for f in frames])
                           for key in ("antenna_east_m", "antenna_north_m")])
    error = np.linalg.norm(est-xy, axis=1)
    imu = np.array(prepared.imu_rows)
    dt = np.diff(imu[:, 0], prepend=imu[0, 0] - np.median(np.diff(imu[:, 0])))
    # Mask is identical across modes and independent of the estimated trajectory.
    turning = np.abs(np.interp(gt, imu[:, 0], np.degrees(imu[:, 3] / dt))) > 10
    yaw = np.array([f.heading_residual_deg for f in frames if f.heading_residual_deg is not None])
    return {"frames": len(frames), "antenna_gnss_horizontal_m": distribution(error),
            "turn_horizontal_m": distribution(error[turning]),
            "straight_horizontal_m": distribution(error[~turning]),
            "abs_heading_residual_deg": distribution(np.abs(yaw)), "warnings": result.warnings}


def compare(paths: list[Path], options: KfOptions, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    report = {"interpretation": "Same-input GNSS consistency; not independent truth. Turn mask: abs(IMU FRD Z rate)>10 deg/s.",
              "options": asdict(options), "logs": {}}
    for path in paths:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        prepared = prepare_log(path, options)
        runs = {}
        for mode, (heading, velocity) in MODES.items():
            result = run_kf_gins(path, replace(options, heading_aiding=heading, velocity_aiding=velocity))
            write_diagnostics_csv(result, output / f"{path.stem}_{mode}.csv")
            runs[mode] = metrics(prepared, result)
            print(path.name, mode, json.dumps({k: v for k, v in runs[mode].items() if k != "warnings"}), flush=True)
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError(f"Source log changed during comparison: {path}")
        report["logs"][path.name] = {"path": str(path.resolve()), "sha256": digest, "runs": runs}
    (output / "comparison.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--config", required=True, type=Path, help="KF replay installation JSON")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    options = KfOptions(**json.loads(args.config.read_text(encoding="utf-8")))
    options.validate()
    compare(args.logs, options, args.output)


if __name__ == "__main__":
    main()
