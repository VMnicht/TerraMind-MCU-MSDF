import json
from pathlib import Path
import queue
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from capture_profile import CaptureProfile, RawCapture, load_capture, sidecar_path
from monitor_gui import InputWorker
from monitor_protocol import Imu, MonitorModel
import test_monitor_protocol
from test_kf_gins_bridge import example_log
from kf_gins_bridge import KfOptions, prepare_log


class IncrementalCaptureTests(unittest.TestCase):
    def clean_model(self, mode=3):
        model = MonitorModel()
        original_feed = model.feed
        def feed(line, at):
            if line.startswith("I,"):
                fields = line.split(",")
                fields[2] = str(mode)
                line = ",".join(fields)
            original_feed(line, at)
        model.feed = feed
        test_monitor_protocol.MonitorProtocolTests()._feed_clean(model)
        model.feed = original_feed
        return model

    def test_mode_and_confirmation_gate_readiness(self):
        model = self.clean_model()
        self.assertFalse(model.assess(6, profile=CaptureProfile())[0])
        self.assertTrue(model.assess(6, profile=CaptureProfile(delta_ctrl_confirmed=True))[0])
        raw = self.clean_model(2)
        self.assertFalse(raw.assess(6, profile=CaptureProfile(delta_ctrl_confirmed=True))[0])
        self.assertTrue(raw.assess(6, profile=CaptureProfile(expected_mode=2))[0])
        model.feed("I,6005,2,48096,0,1000,0,1,2,3,4,5,6", 6.005)
        self.assertTrue(any("混有" in issue for issue in model.assess(12)[1]))

    def test_sensor_flags_and_scale(self):
        for flag in (1, 256):
            model = self.clean_model()
            model.feed(f"I,6005,3,48096,{flag},1000,0,1,2,3,4,5,6", 6.005)
            self.assertTrue(any("传感器错误" in issue for issue in model.assess(6.005)[1]))
        sample = Imu(0, 3, 0, 0, 0, 0, (1000, -2000, 3000, -4000, 5000, -6000))
        angle, velocity = sample.increments(0xCC)
        self.assertAlmostEqual(angle[0], 0.0004734848484848485)
        self.assertAlmostEqual(velocity[0], -0.0004903325)
        angle2, velocity2 = sample.increments(0xCD)
        self.assertEqual(angle, angle2)
        self.assertEqual(velocity2[0], 2 * velocity[0])

    def test_raw_serial_bytes_survive_ui_queue_overflow(self):
        blocks = [b"V,2,921600,4000000\r\nI,5,3,312,0,1,0,-2147483648,",
                  b"2147483647,3,4,5,6\r\n\xffbroken\r\n",
                  b"I,10,2,625,0,1,0,1,2,3,4,5,6\npartial"]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "capture.txt"
            worker = InputWorker(queue.Queue(maxsize=1), port="fake")
            class Serial:
                def __init__(self, *args, **kwargs):
                    self.blocks = iter(blocks)
                def __enter__(self): return self
                def __exit__(self, *args): pass
                def read(self, count):
                    block = next(self.blocks, b"")
                    if not block:
                        worker.stop_event.set()
                    return block
            worker.start_capture(path, CaptureProfile(delta_ctrl=0xCD, delta_ctrl_confirmed=True))
            with patch("serial.Serial", Serial):
                worker.run()
            self.assertEqual(worker.error, "")
            self.assertGreater(worker.dropped, 0)
            self.assertEqual(path.read_bytes(), b"".join(blocks))
            metadata = load_capture(path, verify_log=True)
            self.assertTrue(metadata["finished"])
            self.assertEqual(metadata["observed_modes"], [2, 3])
            self.assertEqual(metadata["profile"]["delta_ctrl"], 0xCD)
            path.write_bytes(path.read_bytes() + b"modified")
            with self.assertRaisesRegex(ValueError, "不匹配"):
                load_capture(path, verify_log=True)

    def test_sidecar_failure_reported_and_raw_file_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            worker = InputWorker(queue.Queue())
            path = Path(temp) / "log.txt"
            worker.start_capture(path, CaptureProfile())
            capture = worker.capture
            capture.write(b"exact bytes\x00\xff")
            with patch.object(capture, "_save_metadata", side_effect=OSError("disk full")):
                worker.stop_capture()
            self.assertIn("disk full", worker.error)
            self.assertIsNone(worker.capture)
            self.assertTrue(capture.file.closed)
            self.assertEqual(path.read_bytes(), b"exact bytes\x00\xff")

    def test_replay_checks_confirmed_scale_and_preserves_legacy_logs(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "log.txt"
            example_log(path)
            raw = path.read_bytes()
            legacy = prepare_log(path, KfOptions(delta_ctrl=0xCC, apply_startup_calibration=False))
            capture = RawCapture(path, CaptureProfile(delta_ctrl=0xCD, delta_ctrl_confirmed=True))
            capture.write(raw)
            capture.close()
            with self.assertRaisesRegex(ValueError, "冲突"):
                prepare_log(path, KfOptions())
            checked = prepare_log(path, KfOptions(delta_ctrl=0xCD, apply_startup_calibration=False))
            self.assertEqual(checked.imu_rows[0][6], 2 * legacy.imu_rows[0][6])
            self.assertEqual(path.read_bytes(), raw)
            metadata = json.loads(sidecar_path(path).read_text(encoding="utf-8"))
            metadata["profile"]["delta_ctrl_confirmed"] = "false"
            sidecar_path(path).write_text(json.dumps(metadata), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_capture(path)


if __name__ == "__main__":
    unittest.main()
