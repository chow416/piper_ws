#!/usr/bin/env python3
"""Regression tests for the board-to-press-target bridge."""

import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import press_board_centre  # noqa: E402


def rotation_from_unit_quaternion(quaternion):
    """Return a rotation matrix from an already unit-length (x, y, z, w)."""
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


class DirectionQuaternionTest(unittest.TestCase):
    def test_negative_trace_rotation_preserves_press_direction(self):
        # This is the direction that exposed the old negative-trace branch:
        # it produced a non-unit quaternion and changed the commanded +Z axis.
        direction = np.array([0.96626951, 0.00459985, -0.25749188])
        direction /= np.linalg.norm(direction)

        quaternion = np.asarray(press_board_centre.matrix_to_quat(
            press_board_centre.frame_from_axis(direction)))

        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0, places=12)
        actual_direction = rotation_from_unit_quaternion(quaternion)[:, 2]
        np.testing.assert_allclose(actual_direction, direction, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
