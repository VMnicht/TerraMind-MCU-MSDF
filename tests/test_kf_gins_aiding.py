"""Quality gates, observation units and compatibility of the offline aiding path."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from kf_gins_bridge import (KfOptions, Heading, _aiding_rows, _aided_executable, _config,
                            _executable, _parse_log, _write_rows, prepare_log, run_kf_gins)
from test_kf_gins_bridge import example_log, gnss_line


class AidingTests(unittest.TestCase):
    def test_optional_switches_and_original_route(self):
        self.assertFalse(KfOptions().heading_aiding)
        self.assertFalse(KfOptions().velocity_aiding)
        KfOptions(**json.loads('{"delta_ctrl":72}')).validate()
        with self.assertRaises(ValueError):
            KfOptions(heading_aiding=1).validate()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"log.txt";example_log(path)
            try:
                _executable()
            except ValueError:
                self.skipTest("Original executable not built")
            with patch("kf_gins_bridge._aided_executable", side_effect=AssertionError("Original mode must not need enhanced binary")):
                result = run_kf_gins(path, KfOptions(apply_startup_calibration=False))
                self.assertFalse(result.heading_aiding)

    def test_bestnav_tail_and_heading_quality_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = ["0"]*30
            p[0:6] = ["SOL_COMPUTED", "NARROW_INT", "30", "114", "20", "2"]
            p[7:10] = [".03", ".04", ".05"]
            p[21:30] = ["SOL_COMPUTED", "DOPPLER_VELOCITY", ".01", ".02", "1.2", "90", ".3", ".12", ".07"]
            h = ["0"]*17;h[0:4] = ["SOL_COMPUTED", "L1_INT", ".3", "359"]
            h[6] = "1.5"
            path = Path(tmp)/"fields.txt"
            path.write_text(gnss_line("BESTNAVA", 1000, 100000, p) + gnss_line("UNIHEADINGA", 1000, 100000, h), encoding="ascii")
            _, positions, headings, rejected, _ = _parse_log(path)
            self.assertEqual(rejected, 0)
            self.assertEqual((positions[0].velocity_latency_s, positions[0].velocity_age_s,
                              positions[0].vertical_speed_std, positions[0].horizontal_speed_std), (.01, .02, .12, .07))
            self.assertEqual((headings[0].solution, headings[0].baseline_m), ("L1_INT", .3))

    def test_observation_units_and_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"log.txt";example_log(path)
            options = KfOptions(apply_startup_calibration=False, heading_aiding=True,
                                velocity_aiding=True, heading_offset_deg=-90)
            data = prepare_log(path, options)
            p = data.positions[0]
            data.positions[0] = replace(p, speed=2, track=90, vertical_speed=.3,
                                         horizontal_speed_std=.04, vertical_speed_std=.12)
            rows = _aiding_rows(data, options)
            self.assertEqual(rows[0][8], 1)
            np.testing.assert_allclose(rows[0][9:15], [0, 2, -.3, .05, .05, .12], atol=1e-15)
            self.assertAlmostEqual(rows[0][2], np.pi/6)
            self.assertEqual(rows[0][15], 1)
            self.assertAlmostEqual(rows[0][18], np.pi/2)
            self.assertTrue(all(r[8] == 0 for r in rows[1:]))  # Missing velocity std never gets zero variance.
            data.positions[0].velocity_latency_s = .05
            data.headings[0] = replace(data.headings[0], solution="NARROW_FLOAT")
            data.headings[1] = replace(data.headings[1], std=8)
            data.headings[2] = replace(data.headings[2], baseline_m=2)
            data.headings[3] = replace(data.headings[3], degrees=float("nan"))
            rows = _aiding_rows(data, options)
            self.assertEqual(rows[0][8], 0)
            self.assertTrue(all(r[15] == 0 for r in rows[:4]))
            data.headings.append(data.headings[-1])
            with self.assertRaisesRegex(ValueError, "历元重复"):
                _aiding_rows(data, options)

    def test_full_configuration_and_position_only_match_original(self):
        try:
            original, aided = _executable(), _aided_executable()
        except ValueError:
            self.skipTest("Build both native executables for configuration equivalence test")
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp);log = folder/"log.txt";example_log(log)
            options = KfOptions(apply_startup_calibration=False, lever_forward_m=.2, lever_right_m=-.15,
                                initial_roll_deg=.2, initial_pitch_deg=-.3)
            data = prepare_log(log, options)
            _write_rows(folder/"imu.txt", data.imu_rows)
            _write_rows(folder/"gnss.txt", data.gnss_rows)
            _write_rows(folder/"aiding.txt", _aiding_rows(data, options), ".17g")
            config = _config(data, options, folder)
            config = config.replace("initgyrbias: [ 0, 0, 0 ]", "initgyrbias: [ 15, -20, 30 ]")
            config = config.replace("initaccbias: [ 0, 0, 0 ]", "initaccbias: [ -45, 20, 18 ]")
            config = config.replace("initgyrscale: [ 0, 0, 0 ]", "initgyrscale: [ 1200, -2100, 3400 ]")
            config = config.replace("initaccscale: [ 0, 0, 0 ]", "initaccscale: [ 800, 2100, -1000 ]")
            config += "\ninitbgstd: [60, 40, 30]\ninitbastd: [180, 220, 200]\ninitsgstd: [800, 900, 1100]\ninitsastd: [900, 1300, 1000]\n"
            (folder/"run.yaml").write_text(config, encoding="utf-8")
            subprocess.run([str(original), str(folder/"run.yaml")], check=True, capture_output=True)
            reference = np.loadtxt(folder/"KF_GINS_Navresult.nav")
            subprocess.run([str(aided), str(folder/"run.yaml"), str(folder/"aiding.txt")], check=True, capture_output=True)
            result = np.loadtxt(folder/"KF_GINS_Navresult.nav")
            self.assertEqual(reference.shape, result.shape)
            # The upstream saver rounds to 9 decimals; fixed-size core arithmetic
            # also differs slightly. Require <0.07 mm BLH and <1e-7 m/s / deg.
            np.testing.assert_allclose(reference[:, :5], result[:, :5], rtol=0, atol=6e-10)
            np.testing.assert_allclose(reference[:, 5:], result[:, 5:], rtol=0, atol=1e-7)
            # Optional state diagnostics must not perturb the solver or its NAV
            # contract, and all 21 covariance components must be recorded.
            (folder/"run.yaml").write_text(config + "\ndiagnostics: true\n", encoding="utf-8")
            subprocess.run([str(aided), str(folder/"run.yaml"), str(folder/"aiding.txt")], check=True, capture_output=True)
            np.testing.assert_array_equal(result, np.loadtxt(folder/"KF_GINS_Navresult.nav"))
            state = np.loadtxt(folder/"KF_GINS_State.csv", delimiter=",", skiprows=1)
            self.assertEqual(state.shape, (len(result), 40))
            self.assertTrue(np.isfinite(state).all())
            self.assertTrue((state[:, 13:34] >= 0).all())
            np.testing.assert_allclose(state[:, 0], result[:, 1], atol=1e-10, rtol=0)


if __name__ == "__main__":
    unittest.main()
