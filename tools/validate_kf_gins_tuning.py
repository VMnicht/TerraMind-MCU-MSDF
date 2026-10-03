"""Reproduce KF tuning checks without changing source logs or capture sidecars.

Compare upstream noise / position only, upstream noise / all aiding, and the
vehicle profile. Remove every fifth complete aiding epoch, then independently
remove three 1 s windows centred on turn-time quantiles. Held-out GNSS is still
from the same receiver: these are causal prediction checks, not external truth.
Requires the offline native runner built by build_kf_gins_aided.ps1.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from compare_kf_gins import distribution, metrics
from fusion2d import ReplayFrame
from kf_gins_bridge import (DEFAULT_CONFIG, VEHICLE_CONFIG, KfOptions, _aided_executable,
                            _aiding_rows, _config, _local_en, _with_diagnostics,
                            _write_rows, prepare_log)
from kf_gins_diagnostics import write_diagnostics_csv
from types import SimpleNamespace


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def omission_masks(observations: np.ndarray, imu: np.ndarray) -> dict[str, np.ndarray]:
    """Keep initialization intact and omit complete epochs, including heading."""
    times = observations[:, 0]
    masks = {"full": np.zeros(len(times), dtype=bool),
             "holdout_20pct": np.arange(len(times)) % 5 == 3}
    dt = np.diff(imu[:, 0], prepend=imu[0, 0] - np.median(np.diff(imu[:, 0])))
    turn = np.abs(np.interp(times, imu[:, 0], imu[:, 3] / dt)) > np.radians(10)
    eligible = times[turn & (times > times[0] + 2) & (times < times[-1] - .5)]
    if len(eligible):
        for index, centre in enumerate(np.quantile(eligible, [.2, .5, .8])):
            masks[f"outage_1s_{index+1}"] = (times >= centre-.5) & (times < centre+.5)
    return masks


def evaluate(path: Path, options: KfOptions, output: Path) -> dict:
    executable = _aided_executable()
    hashes = {str(p.resolve()): sha256(p) for p in (path, Path(str(path)+".capture.json")) if p.is_file()}
    prepared = prepare_log(path, options)
    reference_options = replace(options, heading_aiding=True, velocity_aiding=True)
    observations = np.array(_aiding_rows(prepared, reference_options))
    masks = omission_masks(observations, np.array(prepared.imu_rows))
    report = {"input_sha256": hashes, "installation": asdict(options),
              "executable_sha256": sha256(executable), "cases": {}, "omitted_epochs": {
                  key: observations[mask, 0].tolist() for key, mask in masks.items()}}
    profiles = {"original": replace(options, config_path="", heading_aiding=False, velocity_aiding=False),
                "aided_default": replace(reference_options, config_path=""),
                "vehicle": replace(reference_options, config_path=str(VEHICLE_CONFIG))}
    for tag, config in profiles.items():
        template = Path(config.config_path) if config.config_path else DEFAULT_CONFIG
        case = {"options": asdict(config), "template_sha256": sha256(template), "runs": {}}
        rows = observations.copy()
        if not config.heading_aiding:
            rows[:, 15] = 0
        if not config.velocity_aiding:
            rows[:, 8] = 0
        for mode, omitted in masks.items():
            folder = (output / path.stem / tag / mode).resolve()
            folder.mkdir(parents=True, exist_ok=True)
            if not str(folder).isascii():
                raise ValueError("Native runner output directory must have an ASCII path")
            _write_rows(folder / "imu.txt", prepared.imu_rows)
            _write_rows(folder / "gnss.txt", prepared.gnss_rows)
            active = (rows[:, 1] != 0) | (rows[:, 8] != 0) | (rows[:, 15] != 0)
            _write_rows(folder / "aiding.txt", rows[~omitted & active], ".17g")
            (folder / "run.yaml").write_text(_config(prepared, config, folder)+"\ndiagnostics: true\n", encoding="utf-8")
            completed = subprocess.run([str(executable), str(folder / "run.yaml"), str(folder / "aiding.txt")],
                                       capture_output=True, text=True, check=True, timeout=1800)
            nav = np.loadtxt(folder / "KF_GINS_Navresult.nav", ndmin=2)
            if nav.shape[1] != 11 or not np.all(np.isfinite(nav)):
                raise ValueError("Invalid navigation output")
            p0 = prepared.first_position
            frames = []
            for _, t, lat, lon, h, vn, ve, vd, roll, pitch, heading in nav:
                east, north = _local_en(lat, lon, p0.latitude, p0.longitude)
                frames.append(ReplayFrame(t*1000, east, north, ve, vn, heading % 360, 0,
                                          h, roll, pitch, lat, lon, vd, prepared.week))
            frames = _with_diagnostics(frames, prepared.headings, prepared.week, config)
            result = SimpleNamespace(frames=frames, warnings=prepared.warnings)
            values = metrics(prepared, result)
            write_diagnostics_csv(result, folder / "trajectory.csv")
            # Evaluate only GNSS positions actually withheld (not all frames).
            gt = np.array([r[0] for r in prepared.gnss_rows])
            held = np.isin(np.round(gt, 6), np.round(observations[omitted, 0], 6))
            held &= (gt >= nav[0, 1]) & (gt <= nav[-1, 1])
            xy = np.array([_local_en(p.latitude, p.longitude, p0.latitude, p0.longitude) for p in prepared.positions])
            estimated = np.column_stack([np.interp(gt, nav[:, 1], [getattr(f, key) for f in frames])
                                         for key in ("antenna_east_m", "antenna_north_m")])
            values["withheld_horizontal_m"] = distribution(np.linalg.norm(estimated[held]-xy[held], axis=1))
            state = np.loadtxt(folder / "KF_GINS_State.csv", delimiter=",", skiprows=1, ndmin=2)
            if not np.all(np.isfinite(state)) or np.any(state[:, 13:34] < 0):
                raise ValueError("Invalid state diagnostics / negative covariance diagonal")
            values["final_gyro_bias_deg_h"] = state[-1, 1:4].tolist()
            values["final_acc_bias_m_s2"] = state[-1, 4:7].tolist()
            values["minimum_covariance_diagonal"] = float(state[:, 13:34].min())
            values["native_counts"] = completed.stdout.strip()
            case["runs"][mode] = values
        report["cases"][tag] = case
        print(path.name, tag, json.dumps({k: {"turn_median_m": v["turn_horizontal_m"].get("median"),
              "withheld_rms_m": v["withheld_horizontal_m"].get("rms")} for k, v in case["runs"].items()}), flush=True)
    if any(sha256(Path(name)) != digest for name, digest in hashes.items()):
        raise RuntimeError("Input log / sidecar changed while evaluating")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--config", required=True, type=Path, help="Saved KF installation JSON; noise/mode are compared separately")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    options = KfOptions(**json.loads(args.config.read_text(encoding="utf-8")))
    options.validate()
    report = {"interpretation": "Same-receiver consistency and omitted-observation prediction; not independent accuracy. "
              "Short logs used for tuning, not an independent generalization dataset. Full 21 states, no smoothing.",
              "logs": {}}
    args.output.mkdir(parents=True, exist_ok=True)
    for path in args.logs:
        report["logs"][path.name] = evaluate(path, options, args.output)
        (args.output / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
