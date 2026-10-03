from dataclasses import asdict
from pathlib import Path
import csv
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fusion2d import ReplayFrame
from kf_gins_bridge import KfOptions, Heading, _with_diagnostics
from kf_gins_diagnostics import write_diagnostics_csv
from sync_timeline import GPS_WEEK_MS


def frame(t=1000, yaw=0, roll=0, pitch=0):
    return ReplayFrame(t, 10, 20, 2, 3, yaw, 0, 100, roll, pitch, 30, 114, .1, 2300)


class KfDiagnosticsTests(unittest.TestCase):
    def test_full_attitude_lever_projection_and_unchanged_navigation(self):
        source = frame(yaw=90)
        original = asdict(source)
        options = KfOptions(lever_forward_m=1, lever_right_m=-.15, lever_down_m=.2)
        f = _with_diagnostics([source], [], 2300, options)[0]
        self.assertAlmostEqual(f.antenna_east_m, 11)
        self.assertAlmostEqual(f.antenna_north_m, 20.15)
        self.assertAlmostEqual(f.antenna_height_m, 99.8)
        self.assertEqual(asdict(source), original)
        self.assertEqual({k: getattr(f, k) for k in original}, original)
        tilted = _with_diagnostics([frame(roll=90)], [], 2300,
                                  KfOptions(lever_down_m=1))[0]
        self.assertAlmostEqual(tilted.antenna_east_m, 9)
        self.assertAlmostEqual(tilted.antenna_height_m, 100)
        pitched = _with_diagnostics([frame(pitch=90)], [], 2300,
                                   KfOptions(lever_forward_m=1))[0]
        self.assertAlmostEqual(pitched.antenna_height_m, 101)

    def test_heading_wrap_offset_and_missing_observation_gaps(self):
        origin = 2300 * GPS_WEEK_MS
        headings = [Heading(origin + 1000, 89, 1), Heading(origin + 1100, 91, 2),
                    Heading(origin + 2000, 100, 1)]
        frames = [frame(t, yaw=359) for t in (995, 1000, 1050, 1100, 1200, 2000, 2005)]
        options = KfOptions(heading_offset_deg=-90)
        values = _with_diagnostics(frames, headings, 2300, options)
        self.assertEqual(values[1].gnss_vehicle_heading_deg, 359)
        self.assertEqual(values[2].gnss_vehicle_heading_deg, 0)
        self.assertEqual(values[2].heading_residual_deg, -1)
        self.assertEqual(values[2].gnss_heading_std_deg, 2)
        for index in (0, 4, 6):
            self.assertIsNone(values[index].heading_residual_deg)
        options.heading_offset_deg = 0
        self.assertEqual(values[2].gnss_vehicle_heading_deg, 0)  # Frozen run geometry.

    def test_zero_lever_and_diagnostic_csv_keep_both_references(self):
        f = _with_diagnostics([frame()], [], 2300, KfOptions())[0]
        self.assertEqual((f.antenna_east_m, f.antenna_north_m), (f.east_m, f.north_m))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "diagnostics.csv"
            write_diagnostics_csv(SimpleNamespace(frames=[f]), path)
            with path.open(encoding="utf-8", newline="") as source:
                row = next(csv.DictReader(source))
            self.assertEqual(float(row["imu_east_m"]), float(row["fused_antenna_east_m"]))
            self.assertEqual(row["heading_residual_deg"], "")


if __name__ == "__main__":
    unittest.main()
