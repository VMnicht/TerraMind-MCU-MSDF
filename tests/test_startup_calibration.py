from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from startup_calibration import parse_calibration, read_calibration
from monitor_protocol import MonitorModel
from capture_profile import RawCapture, CaptureProfile, load_capture
from kf_gins_bridge import KfOptions, prepare_log
from test_kf_gins_bridge import example_log

B = "B,1,3005,3,204,600,3000000,1000000,-2000000,3000000,100,100,100,25000"


class StartupCalibrationTests(unittest.TestCase):
    def test_record_units_repetition_and_validation(self):
        result = parse_calibration(B)
        self.assertEqual(result.samples, 600)
        self.assertEqual(result.stationary_rate, (0.001, -0.002, 0.003))
        for bad in (B.replace("B,1,", "B,2,"), B + ",0", B.replace(",3000000,1000000", ",0,1000000")):
            with self.assertRaises(ValueError): parse_calibration(bad)
        model = MonitorModel()
        model.feed(B, 3.005)
        model.feed(B.replace(",3005,", ",4005,"), 4.005)
        self.assertEqual(model.record_errors, 0)
        self.assertEqual(model.unknown_lines, 0)
        self.assertEqual(model.counts["B"], 2)
        issues = model.assess(4.005, profile=CaptureProfile(delta_ctrl=0xCD))[1]
        self.assertTrue(any("板端标定配置" in s for s in issues))

    def test_drdy_interval_for_display_correction(self):
        model = MonitorModel()
        model.feed(B, 3.005)
        model.feed("I,3010,3,100,0,0,0,1,2,3,4,5,6", 3.010)
        model.feed("T,3010,100,1,0,12040000,0,12046000", 3.010)
        self.assertIsNone(model.imu_dt_s)
        model.feed("I,3015,3,413,0,0,0,1,2,3,4,5,6", 3.015)
        model.feed("T,3015,413,2,0,12060000,0,12066000", 3.015)
        self.assertEqual(model.imu_dt_s, 0.005)
        model.feed("I,3020,3,725,0,0,0,1,2,3,4,5,6", 3.020)
        model.feed("T,3020,725,0,0,0,0,12086000", 3.020)
        self.assertIsNone(model.imu_dt_s)

    def test_capture_and_kf_keep_raw_and_reject_changed_calibration(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"log.txt"
            example_log(path)
            original = path.read_bytes()
            data = (B + "\r\n").encode() + original
            capture = RawCapture(path, CaptureProfile(delta_ctrl=0xCC))
            capture.write(data[:19]);capture.write(data[19:]);capture.close()
            self.assertEqual(path.read_bytes(), data)
            meta = load_capture(path, verify_log=True)
            self.assertEqual(meta["startup_calibration"]["samples"], 600)
            self.assertEqual(read_calibration(path).stationary_rate[0], .001)
            prepared = prepare_log(path, KfOptions(delta_ctrl=0xCC))
            self.assertTrue(any("地球自转" in text for text in prepared.warnings))
            with self.assertRaisesRegex(ValueError, "启动标定记录冲突"):
                prepare_log(path, KfOptions(delta_ctrl=0xCD))
            with path.open("ab") as output:
                output.write((B.replace(",1000000,", ",1000001,")+"\n").encode())
            with self.assertRaisesRegex(ValueError, "不同启动标定"):
                read_calibration(path)


if __name__ == "__main__": unittest.main()
