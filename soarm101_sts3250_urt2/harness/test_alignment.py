"""Offline alignment math and sampling tests; no serial access."""

import unittest
from unittest.mock import patch

from align_real import single_pose_mapping, stable_sample
from keyboard_control import NAMES


class AlignmentTests(unittest.TestCase):
    def test_zero_pose(self):
        measured = dict(zip(NAMES[:5], [35, -10, 20, 7, -8]))
        result = single_pose_mapping(measured, [0]*5, [1]*5, [-0.1, 1.7])
        self.assertEqual(result['offset_deg'], [35, -10, 20, 7, -8])
        self.assertFalse(result['verified'])

    def test_nonzero_pose_and_preserved_mapping(self):
        measured = dict(zip(NAMES[:5], [35, -10, 20, 7, -8]))
        signs, reference, grip = [-1, 1, -1, 1, 1], [10, 5, 20, 0, -10], [1.7, -0.1]
        result = single_pose_mapping(measured, reference, signs, grip)
        self.assertEqual(result['offset_deg'], [45, -15, 40, 7, 2])
        self.assertEqual(result['sign'], signs)
        self.assertEqual(result['gripper_radians_at_0_100'], grip)
        for i, name in enumerate(NAMES[:5]):
            self.assertAlmostEqual((measured[name] - result['offset_deg'][i]) / signs[i], reference[i])

    def test_invalid_reference(self):
        with self.assertRaises(ValueError):
            single_pose_mapping(dict.fromkeys(NAMES[:5], 0), [float('nan')]*5, [1]*5, [0, 1])

    def test_sample_stability(self):
        class Viewer:
            def is_running(self):
                return True
            def sync(self):
                pass

        class Bus:
            def __init__(self, step):
                self.value, self.step = 10, step
            def sync_read(self, register, **kwargs):
                assert register == 'Present_Position'
                self.value += self.step
                return {'shoulder_pan': self.value}

        with patch('align_real.time.sleep'):
            result = stable_sample(Bus(0), Viewer(), ['shoulder_pan'])
            self.assertEqual(result['shoulder_pan'], 10)
            with self.assertRaises(ValueError):
                stable_sample(Bus(1), Viewer(), ['shoulder_pan'])


if __name__ == '__main__':
    unittest.main()
