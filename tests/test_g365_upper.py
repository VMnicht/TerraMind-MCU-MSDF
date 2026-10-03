"""No serial device is opened by these register-transaction tests."""
import queue
import sys
import threading
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "g365"))
from g366_upper import G366Protocol, G366App, FrameDecoder, MonitorState, MODES, delta_range_text


class RegisterProtocol(G366Protocol):
    def __init__(self, fail=False, cannot_restore=False):
        self.regs = dict(sig=0, msc=6, smpl=0x0900, filter=5, uart=0x0103,
                         burst1=0xCC07, burst2=0x0C00, pol=0, glob3=0x0108,
                         atti=0x0200, glob2=0)
        self.writes = []
        self.fail = fail
        self.cannot_restore = cannot_restore
        self.samples = 0

    def select_window(self, window):
        assert window == 1

    def write8(self, address, value):
        self.writes.append((address, value))
        assert address == 0x12
        self.regs["glob3"] = (self.regs["glob3"] & 0xFF00) | value
        if self.fail:
            self.fail = False
            if self.cannot_restore:
                self.regs["smpl"] = 0
            raise OSError("simulated ACK loss after write")

    def read16(self, address):
        assert address == 0x12
        return self.regs["glob3"]

    def enter_configuration(self): pass
    def enter_sampling(self): self.samples += 1
    def snapshot(self): return dict(self.regs)


def run_transaction(protocol, codes):
    app = G366App.__new__(G366App)
    app.protocol = protocol
    app.port = Mock()
    app.events = queue.Queue()
    app.busy_lock = threading.Lock()
    app.reader_io_lock = threading.Lock()
    app.reader_pause = threading.Event()
    app.decoder = FrameDecoder()
    app.monitor = MonitorState()
    app.tx_ready = True
    app._delta_worker(codes)
    return app, app.events.get_nowait()[1]


class DeltaConfigurationTests(unittest.TestCase):
    def test_writes_only_low_byte_and_checks_codes(self):
        p = RegisterProtocol()
        self.assertEqual(p.configure_delta_scale(4, 8), 0x0148)
        self.assertEqual(p.writes, [(0x12, 0x48)])
        for codes in ((-1, 8), (16, 8), (4, 16), (4, True)):
            with self.assertRaises(ValueError): p.configure_delta_scale(*codes)
        self.assertEqual(len(p.writes), 1)

    def test_apply_preserves_all_other_registers_and_reconfigures_decoder(self):
        p = RegisterProtocol()
        expected = dict(p.regs, glob3=0x0148)
        app, result = run_transaction(p, (4, 8))
        self.assertTrue(result[0])
        self.assertEqual(app.current_regs, expected)
        self.assertEqual(app.decoder.delta_angle_code, 4)
        self.assertEqual(app.decoder.delta_velocity_code, 8)
        self.assertFalse(app.reader_pause.is_set())
        self.assertFalse(app.busy_lock.locked())

    def test_read_only_never_writes(self):
        p = RegisterProtocol()
        app, result = run_transaction(p, None)
        self.assertTrue(result[0])
        self.assertEqual(p.writes, [])
        self.assertEqual(app.decoder.delta_angle_code, 0)

    def test_partial_write_is_restored_and_verified(self):
        p = RegisterProtocol(fail=True)
        app, result = run_transaction(p, (4, 8))
        self.assertFalse(result[0])
        self.assertTrue(app.tx_ready)
        self.assertEqual(p.writes, [(0x12, 0x48), (0x12, 8)])
        self.assertEqual(app.current_regs["glob3"], 0x0108)
        self.assertEqual(app.decoder.delta_angle_code, 0)

    def test_unverified_restore_disables_decoding(self):
        app, result = run_transaction(RegisterProtocol(fail=True, cannot_restore=True), (4, 8))
        self.assertFalse(result[0])
        self.assertFalse(app.tx_ready)
        self.assertIsNone(app.decoder.profile)
        self.assertEqual(app.current_regs, {})

    def test_readback_mismatch_is_rejected(self):
        p = RegisterProtocol()
        p.read16 = lambda address: 0
        app, result = run_transaction(p, (4, 8))
        self.assertFalse(result[0])
        self.assertIn("回读不一致", result[1])
        self.assertEqual(app.current_regs["glob3"], 0x0108)

    def test_raw_rail_detection_and_counter_reset(self):
        d = FrameDecoder()
        d.configure(MODES["delta32"], 0x48)
        values = [2147483647, -2147483648, 0, 0, 123, -2147483648]
        raw = b"\x80\xfe\xfd\x00\x00" + b"".join(v.to_bytes(4, "big", signed=True) for v in values)
        raw += b"\x00" * 6 + b"\x0d"
        sample = d.decode(raw)
        monitor = MonitorState()
        monitor.add([(raw, sample)], 0)
        self.assertEqual(monitor.health_snapshot(), (1, 1, 1))
        before = sample["delta_angle_x"]
        d.configure(MODES["delta32"], 8)
        self.assertAlmostEqual(before, 16 * d.decode(raw)["delta_angle_x"])
        monitor.reset()
        self.assertEqual(monitor.health_snapshot(), (0, 0, 0))

    def test_g365_representation_range(self):
        self.assertIn("49.65", delta_range_text(0, 200))
        self.assertIn("794.38", delta_range_text(4, 200))
        self.assertNotIn("Hz", delta_range_text(4, None))

    def test_legacy_log_keeps_original_scale(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
        from kf_gins_bridge import KfOptions, prepare_log
        from test_kf_gins_bridge import example_log
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "old.txt"
            example_log(path)
            path.write_text("B,1,3005,3,8,600,3000000,0,0,0,100,100,100,25000\n"
                            + path.read_text(encoding="ascii"), encoding="ascii")
            original = path.read_bytes()
            self.assertTrue(prepare_log(path, KfOptions(delta_ctrl=8)).startup_calibration_applied)
            with self.assertRaises(ValueError): prepare_log(path, KfOptions())
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__": unittest.main()
