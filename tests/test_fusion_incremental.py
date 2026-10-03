"""Mode-3 conversion, interval ownership, calibration and raw-mode compatibility."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from capture_profile import CaptureProfile, RawCapture, delta_scales
from fusion2d import (Geometry, ImuInputOptions, _Ekf, _convert_incremental_imu,
                     _parse_log, incremental_settings, run_fusion)
import test_fusion2d as raw_tests
from test_fusion2d import log_for_motion, with_hardware_timing


def as_increments(text, ctrl=72, mean_dps=0., include_calibration=True):
    angle_scale, velocity_scale = delta_scales(ctrl)
    lines = ([f"B,1,3005,3,{ctrl},600,3000000,0,0,{round(math.radians(mean_dps)*1e9)},100,100,100,25000"]
             if include_calibration else [])
    for line in text.splitlines():
        if line.startswith("I,"):
            f = line.split(",");f[2] = "3"
            for index in range(7, 10):
                f[index] = str(round(int(f[index]) / (66*65536) * .005 / angle_scale))
            for index in range(10, 13):
                f[index] = str(round(int(f[index]) * 9.80665 / (2500*65536) * .005 / velocity_scale))
            line = ",".join(f)
        lines.append(line)
    return "\n".join(lines) + "\n"


class FusionIncrementalTests(unittest.TestCase):
    def run_text(self, text, **kwargs):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"data.txt";path.write_text(text, encoding="ascii")
            before = path.read_bytes()
            result = run_fusion(path, kwargs.pop("geometry", Geometry()), **kwargs)
            self.assertEqual(before, path.read_bytes())
            return result

    def test_raw_path_still_uses_same_input_and_output(self):
        text = log_for_motion(2, lambda t: (t, 0, 1, 90, 90, 0))
        plain = self.run_text(text)
        configured = self.run_text(text, imu_options=ImuInputOptions(delta_ctrl=8, apply_startup_calibration=True))
        self.assertEqual(plain.frames, configured.frames)
        self.assertEqual(plain.imu_input_mode, 2)
        self.assertIsNone(plain.board_startup_calibration)

    def test_stationary_and_straight_incremental_replay(self):
        for speed in (0., 1.):
            text = as_increments(with_hardware_timing(log_for_motion(5, lambda t: (speed*t, 0, speed, 90, 90, 0))))
            result = self.run_text(text)
            self.assertEqual(result.imu_input_mode, 3)
            self.assertEqual(result.delta_ctrl, 72)
            self.assertTrue(result.hardware_sync)
            self.assertEqual(result.board_startup_calibration.samples, 600)
            self.assertAlmostEqual(result.frames[-1].east_m, speed*5, delta=.2)
            self.assertAlmostEqual(result.frames[-1].north_m, 0, delta=.1)
            self.assertGreater(len(result.frames), 990)

    def test_interval_integrals_are_consumed_once_with_gnss_inside_frames(self):
        text = log_for_motion(2, lambda t: (0, 0, 0, 0, 0, 20*t))
        # Also exercise varying planar velocity increments, not just delta angle.
        lines = []
        for line in text.splitlines():
            if line.startswith("I,"):
                f = line.split(",");t=(int(f[1])-100000)/1000
                f[10] = str(round(t * 2500*65536 / 9.80665))
                f[11] = str(round(-t*.5 * 2500*65536 / 9.80665))
                line = ",".join(f)
            lines.append(line)
        text = as_increments("\n".join(lines))
        integral = np.zeros(3)
        original = _Ekf.propagate
        def record(ekf, dt, ax, ay, gyro):
            integral[:] += np.array([ax, ay, gyro])*dt
            original(ekf, dt, ax, ay, gyro)
        with patch.object(_Ekf, "propagate", record):
            # UART arrival is +20 ms; advance 18 ms -> first GNSS is +2 ms,
            # inside the interval ending at +5 ms. Only 3/5 of that increment is used.
            result = self.run_text(text, geometry=Geometry(gnss_delay_ms=18))
        samples = [line.split(",") for line in text.splitlines() if line.startswith("I,")]
        a, v = delta_scales(72)
        increments = np.array([[int(f[10])*v, int(f[11])*v, math.radians(int(f[9])*a)] for f in samples])
        expected = increments[1]*.6 + increments[2:].sum(axis=0)
        np.testing.assert_allclose(integral, expected, atol=1e-9, rtol=0)
        self.assertAlmostEqual(result.frames[-1].mcu_ms, 102000)
        self.assertLessEqual(result.gnss[-1].mcu_ms, result.frames[-1].mcu_ms)

    def test_variable_intervals_and_stationary_mean_match_increment_units(self):
        text = as_increments(log_for_motion(2, lambda t: (0, 0, 0, 0, 0, .08)), mean_dps=.08)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"data.txt";path.write_text(text,encoding="ascii")
            imu, _, _, _ = _parse_log(path)
            ctrl, startup, _ = incremental_settings(path, ImuInputOptions())
            times = [100000+i*5+(.5 if i%2 else 0) for i in range(len(imu))]
            _convert_incremental_imu(imu, times, ctrl, startup)
            angle_scale, velocity_scale = delta_scales(ctrl)
            for index in (1, 2):
                dt=(times[index]-times[index-1])/1000
                self.assertAlmostEqual(math.radians(imu[index].gyro_z_dps)*dt,
                    math.radians(imu[index].angle_raw[2]*angle_scale)-startup.stationary_rate[2]*dt, places=14)
                self.assertAlmostEqual(imu[index].accel_x_mps2*dt, imu[index].velocity_raw[0]*velocity_scale, places=14)

    def test_board_and_additional_static_calibration_do_not_subtract_twice(self):
        text=raw_tests.Fusion2DTests._add_imu_offsets(log_for_motion(5, lambda t:(0,0,0,90,90,0)), .08,.1,-.12)
        result=self.run_text(as_increments(text,mean_dps=.08), calibrate_static_imu=True)
        self.assertTrue(result.static_calibration.applied, result.static_calibration.reason)
        self.assertAlmostEqual(result.static_calibration.gyro_z_dps, 0, delta=1e-4)
        self.assertAlmostEqual(result.static_calibration.accel_x_mps2, .1, delta=1e-4)
        self.assertAlmostEqual(result.static_calibration.accel_y_mps2, -.12, delta=1e-4)

    def test_missing_configuration_and_json_only_calibration(self):
        text=as_increments(log_for_motion(2, lambda t:(0,0,0,0,0,0)), include_calibration=False)
        with self.assertRaisesRegex(ValueError,"DLT_CTRL"):
            self.run_text(text)
        with self.assertRaisesRegex(ValueError,"未找到有效"):
            self.run_text(text, imu_options=ImuInputOptions(delta_ctrl=72))
        result=self.run_text(text, imu_options=ImuInputOptions(72,False))
        self.assertIsNone(result.board_startup_calibration)
        with tempfile.TemporaryDirectory() as tmp:
            import json
            from dataclasses import asdict
            from startup_calibration import parse_calibration
            path=Path(tmp)/"log.txt"
            capture=RawCapture(path,CaptureProfile(delta_ctrl_confirmed=True))
            capture.write(text.encode());capture.close()
            sidecar=Path(str(path)+".capture.json")
            metadata=json.loads(sidecar.read_text(encoding="utf-8"))
            metadata['startup_calibration']=asdict(parse_calibration('B,1,3005,3,72,600,3000000,0,0,0,100,100,100,25000'))
            sidecar.write_text(json.dumps(metadata),encoding="utf-8")
            self.assertIsNotNone(run_fusion(path,Geometry()).board_startup_calibration)
            with self.assertRaisesRegex(ValueError,"冲突"):
                run_fusion(path,Geometry(),imu_options=ImuInputOptions(8,False))
            with path.open('a') as f:f.write('changed')
            with self.assertRaisesRegex(ValueError,"SHA256"):
                run_fusion(path,Geometry())

    def test_discontinuities_bad_frames_and_mixed_modes_are_rejected(self):
        text=as_increments(log_for_motion(2,lambda t:(0,0,0,0,0,0)))
        for change in ('gap','count','flag','angle_rail','velocity_rail','mixed','malformed','drdy'):
            with self.subTest(change=change):
                rows=(with_hardware_timing(text) if change=='drdy' else text).splitlines()
                index=[i for i,line in enumerate(rows) if line.startswith('I,')][50]
                f=rows[index].split(',')
                if change=='gap':rows.pop(index)
                else:
                    if change=='count':f[3]=str((int(f[3])+10)&65535)
                    if change=='flag':f[4]='256'
                    if change=='angle_rail':f[7]='2147483647'
                    if change=='velocity_rail':f[12]='-2147483648'
                    if change=='mixed':f[2]='2'
                    if change=='malformed':f.pop()
                    if change=='drdy':
                        t=rows[index+1].split(',');t[3]=str(int(t[3])+1);rows[index+1]=','.join(t)
                    rows[index]=','.join(f)
                with self.assertRaises(ValueError):self.run_text('\n'.join(rows)+'\n')
        tail=self.run_text(text+'I,102005,3,12,0,4')
        self.assertEqual(tail.rejected_lines,1)
        self.assertTrue(any('末行' in w for w in tail.warnings))

    def test_drdy_without_pps_and_mode3_raw_configuration_conflict(self):
        text=as_increments(with_hardware_timing(log_for_motion(3,lambda t:(t,0,1,90,90,0))))
        without_pps='\n'.join(row for row in text.splitlines() if not row.startswith('P,'))+'\n'
        result=self.run_text(without_pps)
        self.assertFalse(result.hardware_sync)
        self.assertTrue(any('仅使用 DRDY' in w for w in result.warnings))
        self.assertAlmostEqual(result.frames[-1].east_m,3,delta=.2)
        with self.assertRaisesRegex(ValueError,'冲突'):
            self.run_text(text,imu_options=ImuInputOptions(8))


if __name__=='__main__':
    unittest.main()
