import math
import unittest

from keyboard_control import check_pose_limits, send_target


class KeyboardControlTests(unittest.TestCase):
    def test_reports_motor_id_in_hardware_coordinates(self):
        q = [0, 0, 1.2, 0, 0, 0]
        with self.assertRaises(RuntimeError) as context:
            check_pose_limits(q, [[-1, 1]] * 6, [1, 1, -1, 1, 1],
                              [0, 0, 90, 0, 0], [-1, 1], '启动姿态超限')
        text = str(context.exception)
        self.assertIn('ID 3 (elbow_flex)', text)
        self.assertIn('低于下限', text)
        self.assertIn(f'{90-math.degrees(1.2):.3f}', text)

    def test_multiple_bad_readings(self):
        with self.assertRaises(RuntimeError) as context:
            check_pose_limits([float('nan'), 0, 0, 0, 0, 2], [[-1, 1]] * 6,
                              [1]*5, [0]*5, [-1, 1], '反馈超限')
        self.assertIn('ID 1', str(context.exception))
        self.assertIn('ID 6', str(context.exception))

    def test_hold_does_not_follow_clamped_feedback(self):
        class Arm:
            def __init__(self):
                self.goals = []
            def send_action(self, target):
                self.goals.append(target)
                return target - 0.02
        arm = Arm()
        target = 0.5
        for _ in range(10):
            target = send_target(arm, target, False, lambda x: x, lambda x: x)
        self.assertEqual(arm.goals, [0.5]*10)
        self.assertEqual(send_target(arm, target, True, lambda x: x, lambda x: x), 0.48)


if __name__ == '__main__':
    unittest.main()
