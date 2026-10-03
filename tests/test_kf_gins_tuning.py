"""Ensure the offline validation really withholds complete observation epochs."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from validate_kf_gins_tuning import omission_masks


class TuningValidationTests(unittest.TestCase):
    def test_whole_epochs_and_no_initialization_or_tail_outage(self):
        t = np.arange(0, 20, .005)
        imu = np.zeros((len(t), 7));imu[:, 0] = t
        imu[:, 3] = np.radians(30)*.005
        observations = np.zeros((200, 19));observations[:, 0] = np.arange(200)/10
        observations[:, [1, 8, 15]] = 1
        masks = omission_masks(observations, imu)
        self.assertFalse(masks["full"].any())
        self.assertEqual(masks["holdout_20pct"].sum(), 40)
        self.assertFalse(masks["holdout_20pct"][0])
        for name, mask in masks.items():
            kept = observations[~mask]
            self.assertFalse(np.isin(observations[mask, 0], kept[:, 0]).any())
            if name.startswith("outage"):
                omitted = observations[mask, 0]
                self.assertEqual(len(omitted), 10)
                self.assertGreater(omitted[0], 1)
                self.assertLess(omitted[-1], observations[-1, 0])
        imu[:, 3] = 0
        self.assertEqual(set(omission_masks(observations, imu)), {"full", "holdout_20pct"})


if __name__ == "__main__":
    unittest.main()
