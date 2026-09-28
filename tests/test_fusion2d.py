import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from fusion2d import Geometry, run_fusion
from monitor_protocol import unicore_crc32


def sentence(name, body, tow):
    payload = f"{name},93,GPS,FINE,2438,{tow},0,0,18,3;{','.join(body)}"
    return f"#{payload}*{unicore_crc32(payload):08x}"


def log_for_motion(seconds, position_at):
    """position_at(t) -> antenna east/north, speed, track, heading, gyro."""
    lines = []
    count = 0
    lat0 = 23.0
    east_scale = 111320 * math.cos(math.radians(lat0))
    north_scale = 110758.0
    for ms in range(0, seconds * 1000 + 1, 5):
        t = ms / 1000
        e, n, speed, track, heading, gyro = position_at(t)
        count = (count + (312 if ms % 10 == 0 else 313)) & 65535
        gyro_raw = round(gyro * 66 * 65536)
        lines.append(f"I,{100000+ms},2,{count},0,1000,0,0,0,{gyro_raw},0,0,-163840000\r\n")
        if ms % 100:
            continue
        tow = 31000000 + ms
        body = ["0"] * 30
        body[0:2] = ["SOL_COMPUTED", "NARROW_INT"]
        body[2], body[3] = f"{lat0+n/north_scale:.11f}", f"{113+e/east_scale:.11f}"
        body[4] = "30.0"
        body[7:10] = ["0.02", "0.02", "0.03"]
        body[14] = "20"
        body[21:23] = ["SOL_COMPUTED", "DOPPLER_VELOCITY"]
        body[25:27] = [f"{speed:.5f}", f"{track:.5f}"]
        body[29] = "0.02"
        lines.append(f"N,{100020+ms},{sentence('BESTNAVA',body,tow)}\r\n")
        hdg = ["0"] * 17
        hdg[0:5] = ["SOL_COMPUTED", "NARROW_INT", "1.0", f"{heading%360:.5f}", "0"]
        hdg[6] = "0.5"
        lines.append(f"N,{100040+ms},{sentence('UNIHEADINGA',hdg,tow)}\r\n")
    return "".join(lines)


def with_hardware_timing(text, pps_offset_ms=0):
    lines = ["V,2,921600,4000000\r\n"]
    drdy_seq = pps_seq = 0
    for line in text.splitlines():
        fields = line.split(",")
        if fields[0] == "I":
            ms, count = int(fields[1]), int(fields[3])
            if ms % 1000 == 0:
                pps_seq += 1
                lines.append(f"P,{pps_seq},{ms + pps_offset_ms},0,{(ms + pps_offset_ms) * 4000},{0 if pps_seq == 1 else 4000000}\r\n")
            drdy_seq += 1
            lines.append(f"D,{drdy_seq},{ms},0,{ms * 4000}\r\n")
            lines.append(line + "\r\n")
            lines.append(f"T,{ms},{count},{drdy_seq},0,{ms * 4000},0,{ms * 4000 + 6000}\r\n")
        elif fields[0] == "N":
            ms = int(fields[1])
            lines.append(line + "\r\n")
            lines.append(f"R,{ms},0,{ms * 4000}\r\n")
    return "".join(lines)


class Fusion2DTests(unittest.TestCase):
    def _run(self, text, geometry, calibrate_static_imu=False):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.txt"
            path.write_text(text, encoding="ascii")
            return run_fusion(path, geometry, calibrate_static_imu=calibrate_static_imu)

    @staticmethod
    def _add_imu_offsets(text, gyro_dps, accel_x, accel_y,
                         gyro_x_dps=0.0, gyro_y_dps=0.0):
        output = []
        for line in text.splitlines():
            if line.startswith("I,"):
                fields = line.split(",")
                fields[7] = str(int(fields[7]) + round(gyro_x_dps * 66 * 65536))
                fields[8] = str(int(fields[8]) + round(gyro_y_dps * 66 * 65536))
                fields[9] = str(int(fields[9]) + round(gyro_dps * 66 * 65536))
                fields[10] = str(int(fields[10]) + round(accel_x / 9.80665 * 2500 * 65536))
                fields[11] = str(int(fields[11]) + round(accel_y / 9.80665 * 2500 * 65536))
                line = ",".join(fields)
            output.append(line + "\r\n")
        return "".join(output)

    def test_constant_east_motion(self):
        text = log_for_motion(10, lambda t: (t, 0, 1, 90, 90, 0))
        result = self._run(text, Geometry())
        self.assertGreater(len(result.frames), 1900)
        self.assertAlmostEqual(result.frames[-1].east_m, 10, delta=0.25)
        self.assertAlmostEqual(result.frames[-1].north_m, 0, delta=0.15)
        self.assertAlmostEqual(result.frames[-1].heading_deg, 90, delta=2)
        self.assertAlmostEqual(result.gps_to_mcu_slope, 1.0, delta=1e-5)

    def test_rotating_lever_arm_is_removed(self):
        omega = math.radians(9)
        def motion(t):
            heading = math.radians(90 + 9 * t)
            e, n = math.sin(heading), math.cos(heading)
            ve, vn = omega * n, -omega * e
            return e, n, omega, math.degrees(math.atan2(ve, vn)) % 360, math.degrees(heading), 9
        text = log_for_motion(10, motion)
        corrected = self._run(text, Geometry(lever_x_m=1.0))
        uncorrected = self._run(text, Geometry())
        corr_displacement = math.hypot(corrected.frames[-1].east_m - corrected.frames[0].east_m,
                                       corrected.frames[-1].north_m - corrected.frames[0].north_m)
        raw_displacement = math.hypot(uncorrected.frames[-1].east_m - uncorrected.frames[0].east_m,
                                      uncorrected.frames[-1].north_m - uncorrected.frames[0].north_m)
        self.assertLess(corr_displacement, 0.2)
        self.assertGreater(raw_displacement, 0.8)

    def test_hardware_pps_drdy_timeline(self):
        text = with_hardware_timing(log_for_motion(10, lambda t: (t, 0, 1, 90, 90, 0)))
        result = self._run(text, Geometry())
        self.assertTrue(result.hardware_sync)
        self.assertEqual(result.pps_count, 11)
        self.assertAlmostEqual(result.frames[-1].east_m, 10, delta=0.25)

    def test_late_pps_edge_is_not_treated_as_gps_second(self):
        text = with_hardware_timing(log_for_motion(10, lambda t: (t, 0, 1, 90, 90, 0)),
                                    pps_offset_ms=500)
        result = self._run(text, Geometry())
        self.assertFalse(result.hardware_sync)
        self.assertTrue(any("退回" in note for note in result.warnings))

    def test_static_start_calibrates_all_three_planar_offsets(self):
        text = log_for_motion(6, lambda t: (0, 0, 0, 90, 90, 0))
        text = self._add_imu_offsets(text, 0.08, 0.10, -0.12,
                                     gyro_x_dps=0.03, gyro_y_dps=-0.04)
        result = self._run(text, Geometry(), calibrate_static_imu=True)
        calibration = result.static_calibration
        self.assertIsNotNone(calibration)
        self.assertTrue(calibration.applied, calibration.reason)
        self.assertGreater(calibration.sample_count, 590)
        self.assertAlmostEqual(calibration.gyro_x_dps, 0.03, delta=0.001)
        self.assertAlmostEqual(calibration.gyro_y_dps, -0.04, delta=0.001)
        self.assertAlmostEqual(calibration.gyro_z_dps, 0.08, delta=0.001)
        self.assertAlmostEqual(calibration.accel_x_mps2, 0.10, delta=0.001)
        self.assertAlmostEqual(calibration.accel_y_mps2, -0.12, delta=0.001)
        self.assertLess(math.hypot(result.frames[-1].east_m, result.frames[-1].north_m), 0.1)

    def test_slow_constant_motion_is_not_calibrated_as_stationary(self):
        text = log_for_motion(6, lambda t: (0.1 * t, 0, 0.1, 90, 90, 0))
        result = self._run(text, Geometry(), calibrate_static_imu=True)
        self.assertIsNotNone(result.static_calibration)
        self.assertFalse(result.static_calibration.applied)
        self.assertIn("位移", result.static_calibration.reason)


if __name__ == "__main__":
    unittest.main()
