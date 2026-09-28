import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from monitor_protocol import MonitorModel, unicore_crc32


def gnss(name, body, tow):
    payload = f"{name},93,GPS,FINE,2438,{tow},0,0,18,3;{','.join(body)}"
    return f"#{payload}*{unicore_crc32(payload):08x}"


def position_body(solution="NARROW_INT"):
    body = ["0"] * 30
    for index, value in {
        0: "SOL_COMPUTED", 1: solution, 2: "23.0", 3: "113.0", 4: "30.0",
        7: "0.02", 8: "0.02", 9: "0.04", 14: "18", 25: "0.0",
    }.items():
        body[index] = value
    return body


def heading_body(solution="NARROW_INT", std="0.3"):
    body = ["0"] * 17
    for index, value in {
        0: "SOL_COMPUTED", 1: solution, 2: "1.200", 3: "150.0",
        4: "0.0", 6: std,
    }.items():
        body[index] = value
    return body


class MonitorProtocolTests(unittest.TestCase):
    def _feed_clean(self, model, duration_ms=6000, bad_at=None):
        count = 0
        for ms in range(0, duration_ms + 1, 5):
            count = (count + (312 if ms % 10 == 0 else 313)) & 0xffff
            model.feed(f"I,{ms},2,{count},0,1000,0,1,2,3,4,5,6", ms / 1000)
            if ms % 100 == 0:
                tow = 31000000 + ms
                model.feed(f"N,{ms},{gnss('BESTNAVA', position_body(), tow)}", ms / 1000)
                std = "5.0" if ms == bad_at else "0.3"
                model.feed(f"N,{ms},{gnss('UNIHEADINGA', heading_body(std=std), tow)}", ms / 1000)
            if ms % 1000 == 0:
                model.feed("S," + ",".join([str(ms)] + ["0"] * 21), ms / 1000)

    def test_clean_fixed_stream_becomes_ready(self):
        model = MonitorModel()
        self._feed_clean(model)
        ready, issues, _ = model.assess(6.0, expected_baseline=1.2)
        self.assertTrue(ready, issues)
        self.assertEqual(model.imu_count_gaps, 0)
        self.assertEqual(model.record_errors, 0)

    def test_intermediate_bad_heading_blocks_ready(self):
        model = MonitorModel()
        self._feed_clean(model, bad_at=3500)
        ready, issues, _ = model.assess(6.0)
        self.assertFalse(ready)
        self.assertTrue(any("连续稳定" in issue for issue in issues))

    def test_broken_record_and_status_error_block_ready(self):
        model = MonitorModel()
        self._feed_clean(model)
        model.feed("N,6000,#UNIHEADINGA,broken", 6.0)
        self.assertEqual(model.record_errors, 1)
        self.assertFalse(model.assess(6.0)[0])
        model = MonitorModel()
        self._feed_clean(model)
        fields = [0] * 22
        fields[0] = 6000
        fields[16] = 1  # tx_dropped
        model.feed("S," + ",".join(map(str, fields)), 6.0)
        self.assertFalse(model.assess(6.0)[0])

    def test_v2_pps_records_and_missing_capture(self):
        model = MonitorModel()
        model.feed("V,2,921600,4000000", 0.0)
        self._feed_clean(model)
        model.feed("P,1,5000,0,20000000,0", 5.0)
        model.feed("P,2,6000,0,24000000,4000000", 6.0)
        model.feed("D,1,6000,0,24000000", 6.0)
        model.feed("T,6000,100,1,0,24000000,0,24006000", 6.0)
        model.feed("R,6000,0,24008000", 6.0)
        model.feed("H,6000,4000000,1200,2,0,0,0,0,0", 6.0)
        model.feed("A,6000,2,2438,31006000,0,24000000,4000000,80000,3,1", 6.0)
        self.assertEqual(model.record_errors, 0)
        self.assertEqual(model.pps_period_ticks, 4000000)
        self.assertTrue(model.assess(6.0)[0])
        model.feed("T,6005,101,0,0,0,0,24026000", 6.005)
        self.assertFalse(model.assess(6.005)[0])


if __name__ == "__main__":
    unittest.main()
