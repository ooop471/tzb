"""Regress the capture-time pairing failure measured in the rover bag."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_estimator.navigation_observations import NavigationCloudHistory


class NavigationTimingTest(unittest.TestCase):
    def test_delayed_detector_uses_exposure_cloud_instead_of_latest_cloud(self):
        history = NavigationCloudHistory()
        exposure = object()
        history.add(exposure, 10.)
        # The measured median inference/transport delay is 219 ms. Newest depth
        # fails the old 150 ms gate even when the correct depth is available.
        for i in range(1, 9):
            history.add(object(), 10. + i/30.)
        self.assertGreater(history.clouds[-1][0] - 10., .15)
        self.assertIs(history.match(10.), exposure)

    def test_chooses_nearest_cloud_for_each_independent_detection(self):
        history = NavigationCloudHistory()
        first, second = object(), object()
        history.add(first, 10.)
        history.add(second, 10.167)
        self.assertIs(history.match(10.004), first)
        self.assertIs(history.match(10.17), second)

    def test_old_exposure_is_removed_and_cannot_match_unrelated_depth(self):
        history = NavigationCloudHistory()
        history.add(object(), 10.)
        history.add(object(), 11.)
        self.assertEqual(len(history.clouds), 1)
        self.assertIsNone(history.match(10.))

    def test_restarted_capture_clock_drops_previous_epoch(self):
        history = NavigationCloudHistory()
        history.add(object(), 20.)
        restarted = object()
        history.add(restarted, 1.)
        self.assertIs(history.match(1.), restarted)
        self.assertIsNone(history.match(20.))


if __name__ == '__main__':
    unittest.main()
