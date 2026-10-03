"""Check the sensor conversion and the native KF-GINS integration boundary."""

from pathlib import Path
from dataclasses import asdict
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from kf_gins_bridge import KfOptions, _config, _initial_imu_position, prepare_log, run_kf_gins
from monitor_protocol import unicore_crc32
from startup_calibration import parse_calibration
from capture_profile import CaptureProfile, RawCapture, sidecar_path


def gnss_line(name: str, mcu_ms: int, tow_ms: int, body: list[str]) -> str:
    header = f"#{name},COM1,0,FINE,2300,{tow_ms},0,0,0,0"
    payload = header + ";" + ",".join(body)
    return f"N,{mcu_ms},{payload}*{unicore_crc32(payload[1:]):08X}\n"


def example_log(path: Path, mode: int = 3) -> None:
    lines = ["V,2,921600,4000000\n"]
    for index in range(201):
        mcu_ms = 1000 + index * 5
        count = (index * 312 + index // 2) & 0xFFFF
        lines.append(f"I,{mcu_ms},{mode},{count},0,2634,0,0,0,0,0,0,-400\n")
        if index in (20, 40, 60, 80, 100, 120, 140, 160, 180):
            p = ["0"] * 30
            p[0], p[1] = "SOL_COMPUTED", "NARROW_INT"
            p[2], p[3] = "30.0", "114.0"
            p[4], p[5] = "20.0", "2.5"
            p[7], p[8], p[9] = "0.03", "0.03", "0.05"
            p[21], p[22] = "SOL_COMPUTED", "DOPPLER_VELOCITY"
            p[25], p[26], p[27] = "0.0", "0.0", "0.0"
            lines.append(gnss_line("BESTNAVA", mcu_ms, 100000 + index * 5, p))
            h = ["0"] * 17
            h[0], h[1] = "SOL_COMPUTED", "NARROW_INT"
            h[2], h[3], h[6] = "1.0", "0.0", "0.5"
            lines.append(gnss_line("UNIHEADINGA", mcu_ms, 100000 + index * 5, h))
    path.write_text("".join(lines), encoding="ascii")


def hardware_log(path: Path) -> None:
    lines = ["V,2,921600,4000000\n"]
    anchor_tick = 10_000_000
    for second in range(5):
        tick = anchor_tick + second * 4_000_000
        lines.append(f"P,{second + 1},{1000 + second * 1000},{tick >> 32},{tick & 0xFFFFFFFF},4000000\n")
    for index in range(900):
        mcu_ms = 900 + index * 5
        tick = anchor_tick - 400_000 + index * 20_000
        count = (index * 312 + index // 2) & 0xFFFF
        lines.append(f"I,{mcu_ms},3,{count},0,2634,0,0,0,0,0,0,-400\n")
        lines.append(f"T,{mcu_ms},{count},{index + 1},{tick >> 32},{tick & 0xFFFFFFFF},0,0\n")
    for second in range(5):
        p = ["0"] * 30
        p[0], p[1] = "SOL_COMPUTED", "NARROW_INT"
        p[2], p[3], p[4], p[5] = "30", "114", "20", "2.5"
        p[7], p[8], p[9] = "0.03", "0.03", "0.05"
        p[21], p[22] = "SOL_COMPUTED", "DOPPLER_VELOCITY"
        p[25], p[26], p[27] = "0", "0", "0"
        lines.append(gnss_line("BESTNAVA", 1000 + second * 1000 + 100,
                               100000 + second * 1000, p))
        tick = anchor_tick + second * 4_000_000 + 400_000
        lines.append(f"R,{1100 + second * 1000},{tick >> 32},{tick & 0xFFFFFFFF}\n")
        h = ["0"] * 17
        h[0], h[1], h[2], h[3], h[6] = "SOL_COMPUTED", "NARROW_INT", "1", "0", "0.5"
        lines.append(gnss_line("UNIHEADINGA", 1100 + second * 1000,
                               100000 + second * 1000, h))
    path.write_text("".join(lines), encoding="ascii")


class KfGinsBridgeTest(unittest.TestCase):
    def test_rejects_saturated_or_flagged_imu_even_without_calibration(self):
        for field, value in ((9, 2147483647), (9, -2147483648), (12, 2147483647), (4, 1), (4, 256)):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/"log.txt"
                example_log(path)
                rows = path.read_text(encoding="ascii").splitlines()
                index = next(i for i, line in enumerate(rows) if line.startswith("I,"))
                fields = rows[index].split(",")
                fields[field] = str(value)
                rows[index] = ",".join(fields)
                path.write_text("\n".join(rows)+"\n", encoding="ascii")
                with self.assertRaisesRegex(ValueError, "IMU 输入无效"):
                    prepare_log(path, KfOptions(apply_startup_calibration=False))

    def test_startup_mean_uses_each_drdy_interval_and_axis_mapping(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            rows = ["B,1,3005,3,72,600,3000000,1000000,-2000000,3000000,100,100,100,25000"]
            index = 0
            for line in path.read_text(encoding="ascii").splitlines():
                rows.append(line)
                if line.startswith("I,"):
                    f = line.split(",")
                    tick = 12000000 + index*20000 + (2000 if index%2 else 0)
                    rows.append(f"T,{f[1]},{f[3]},{index+1},0,{tick},0,{tick+6000}")
                    index += 1
            path.write_text("\n".join(rows)+"\n", encoding="ascii")
            original = path.read_bytes()
            options = KfOptions(axis_forward="+z", axis_right="+x", axis_down="+y")
            corrected = prepare_log(path, options)
            raw = prepare_log(path, KfOptions(axis_forward="+z", axis_right="+x", axis_down="+y",
                                              apply_startup_calibration=False))
            for index, dt in ((1, .0055), (2, .0045)):
                for axis, rate in enumerate((.003, .001, -.002), start=1):
                    self.assertAlmostEqual(raw.imu_rows[index][axis]-corrected.imu_rows[index][axis], rate*dt, places=13)
                self.assertEqual(raw.imu_rows[index][4:], corrected.imu_rows[index][4:])
            self.assertTrue(corrected.startup_calibration_applied)
            self.assertEqual(path.read_bytes(), original)
            template = Path(temp)/"custom.yaml"
            default = Path(__file__).resolve().parents[1]/"KF-GINS/config/kf-gins.yaml"
            template.write_text(default.read_text(encoding="utf-8").replace(
                "initgyrbias: [ 0, 0, 0 ]", "initgyrbias: [ 100, 200, 300 ]"), encoding="utf-8")
            options.config_path = str(template)
            self.assertIn("initgyrbias: [0, 0, 0]", _config(corrected, options, Path(temp)))
            self.assertIn("initgyrbias: [ 100, 200, 300 ]", _config(raw, options, Path(temp)))

    def test_calibration_missing_json_fallback_and_conflict(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"log.txt"
            example_log(path)
            with self.assertRaisesRegex(ValueError, "未找到有效启动标定"):
                prepare_log(path, KfOptions())
            original = path.read_bytes()
            capture = RawCapture(path, CaptureProfile(delta_ctrl_confirmed=True))
            capture.write(original); capture.close()
            meta_path = sidecar_path(path)
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            record = "B,1,3005,3,72,600,3000000,1000000,-2000000,3000000,100,100,100,25000"
            meta["startup_calibration"] = asdict(parse_calibration(record))
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            self.assertTrue(prepare_log(path, KfOptions()).startup_calibration_applied)
            meta["finished"] = False
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "采集未完成"):
                prepare_log(path, KfOptions())
            capture = RawCapture(path, CaptureProfile(delta_ctrl_confirmed=True))
            capture.write((record+"\n").encode()+original); capture.close()
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta["startup_calibration"]["rate_nrad_s"][0] += 1
            meta_path.write_text(json.dumps(meta), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "B 与 JSON"):
                prepare_log(path, KfOptions())

    def test_prepare_converts_delta_and_ellipsoid_height(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            data = prepare_log(path, KfOptions(apply_startup_calibration=False))
            self.assertEqual(len(data.imu_rows), 201)
            self.assertEqual(len(data.gnss_rows), 9)
            self.assertAlmostEqual(data.gnss_rows[0][3], 22.5)
            self.assertAlmostEqual(data.imu_rows[0][6], -400 *
                                   (0.4 / 1000 * 9.80665 / 2000) * 256 / 65536)
            self.assertAlmostEqual(data.initial_heading, 0)
            self.assertIn("starttime:", _config(data, KfOptions(), Path(temp)))
            shifted = _initial_imu_position(data, KfOptions(lever_forward_m=1, lever_down_m=0.2))
            self.assertLess(shifted[0], 30.0)
            self.assertAlmostEqual(shifted[2], 22.7)

    def test_rejects_raw_mode_and_left_handed_axis_map(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path, mode=2)
            with self.assertRaisesRegex(ValueError, "模式 2"):
                prepare_log(path, KfOptions(apply_startup_calibration=False))
        with self.assertRaisesRegex(ValueError, "右手系"):
            KfOptions(axis_down="-z").validate()

    def test_hardware_time_map(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            hardware_log(path)
            data = prepare_log(path, KfOptions(apply_startup_calibration=False))
            self.assertTrue(data.hardware_sync)
            self.assertEqual(data.pps_count, 5)
            self.assertAlmostEqual(data.imu_rows[20][0], 100.0, places=6)

    @unittest.skipUnless(any((Path(__file__).resolve().parents[1] / "KF-GINS" / "bin").glob("**/KF-GINS.exe")),
                         "native KF-GINS executable not built")
    def test_native_solver_produces_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            path.write_text("B,1,3005,3,72,600,3000000,0,0,0,100,100,100,25000\n" + path.read_text(encoding="ascii"), encoding="ascii")
            result = run_kf_gins(path, KfOptions())
            self.assertGreater(len(result.frames), 100)
            self.assertAlmostEqual(result.frames[0].height_m, 22.5, delta=2)


if __name__ == "__main__":
    unittest.main()
