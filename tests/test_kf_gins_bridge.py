"""Check the sensor conversion and the native KF-GINS integration boundary."""

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from kf_gins_bridge import KfOptions, _config, _initial_imu_position, prepare_log, run_kf_gins
from monitor_protocol import unicore_crc32


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
    def test_prepare_converts_delta_and_ellipsoid_height(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            data = prepare_log(path, KfOptions())
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
                prepare_log(path, KfOptions())
        with self.assertRaisesRegex(ValueError, "右手系"):
            KfOptions(axis_down="-z").validate()

    def test_hardware_time_map(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            hardware_log(path)
            data = prepare_log(path, KfOptions())
            self.assertTrue(data.hardware_sync)
            self.assertEqual(data.pps_count, 5)
            self.assertAlmostEqual(data.imu_rows[20][0], 100.0, places=6)

    @unittest.skipUnless(any((Path(__file__).resolve().parents[1] / "KF-GINS" / "bin").glob("**/KF-GINS.exe")),
                         "native KF-GINS executable not built")
    def test_native_solver_produces_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            result = run_kf_gins(path, KfOptions())
            self.assertGreater(len(result.frames), 100)
            self.assertAlmostEqual(result.frames[0].height_m, 22.5, delta=2)


if __name__ == "__main__":
    unittest.main()
