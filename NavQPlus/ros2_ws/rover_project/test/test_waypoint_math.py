"""
Unit tests for the pure math static methods on WaypointFollowerNode.
No ROS2 context is required — all ROS2 imports are mocked.
"""

import sys
import math
import types
import unittest
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock ROS2 / rover_project dependencies before importing the module
# ---------------------------------------------------------------------------
_mock_rclpy = types.ModuleType("rclpy")
_mock_rclpy.node = types.ModuleType("rclpy.node")
_mock_rclpy.node.Node = object
sys.modules.setdefault("rclpy", _mock_rclpy)
sys.modules.setdefault("rclpy.node", _mock_rclpy.node)

for _mod in (
    "std_msgs", "std_msgs.msg",
    "rover_project", "rover_project.msg",
):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

_std_msg = sys.modules["std_msgs.msg"]
for _cls in ("Int32", "Bool", "Float32"):
    setattr(_std_msg, _cls, MagicMock)

_rover_msg = sys.modules["rover_project.msg"]
for _cls in ("GpsFix", "ImuOrientation", "WaypointList"):
    setattr(_rover_msg, _cls, MagicMock)

import importlib.util, os
_script = os.path.join(
    os.path.dirname(__file__),
    "..", "scripts", "waypoint_follower_node.py"
)
_spec = importlib.util.spec_from_file_location("waypoint_follower_node", _script)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

WaypointFollowerNode = _mod.WaypointFollowerNode

haversine = WaypointFollowerNode.haversine
bearing   = WaypointFollowerNode.bearing
angle_diff = WaypointFollowerNode.angle_diff


# ===========================================================================
# haversine() Tests
# ===========================================================================

class TestHaversine(unittest.TestCase):

    def test_same_point_is_zero(self):
        self.assertAlmostEqual(haversine(33.97, -117.33, 33.97, -117.33), 0.0, places=3)

    def test_known_distance_approx(self):
        """UCR campus to Riverside Convention Center (~3 km apart)."""
        d = haversine(33.9737, -117.3281, 33.9800, -117.3750)
        self.assertGreater(d, 3500)
        self.assertLess(d, 5000)

    def test_symmetry(self):
        """haversine(A, B) == haversine(B, A)."""
        d1 = haversine(33.97, -117.33, 34.05, -117.50)
        d2 = haversine(34.05, -117.50, 33.97, -117.33)
        self.assertAlmostEqual(d1, d2, places=6)

    def test_north_south_one_degree(self):
        """One degree of latitude ≈ 111,000 m."""
        d = haversine(33.0, -117.0, 34.0, -117.0)
        self.assertGreater(d, 110_000)
        self.assertLess(d, 112_000)

    def test_east_west_at_ucr_latitude(self):
        """One degree of longitude at ~34° ≈ 92,000 m."""
        d = haversine(34.0, -117.0, 34.0, -118.0)
        self.assertGreater(d, 90_000)
        self.assertLess(d, 95_000)

    def test_returns_metres(self):
        """Result should be in metres — Earth radius used is 6,371,000 m."""
        d = haversine(0.0, 0.0, 0.0, 1.0)
        # ~111 km at equator
        self.assertGreater(d, 100_000)
        self.assertLess(d, 120_000)


# ===========================================================================
# bearing() Tests
# ===========================================================================

class TestBearing(unittest.TestCase):

    def test_due_north(self):
        """Moving straight north → bearing ≈ 0°."""
        b = bearing(33.0, -117.0, 34.0, -117.0)
        self.assertAlmostEqual(b, 0.0, delta=0.5)

    def test_due_south(self):
        """Moving straight south → bearing ≈ 180°."""
        b = bearing(34.0, -117.0, 33.0, -117.0)
        self.assertAlmostEqual(b, 180.0, delta=0.5)

    def test_due_east(self):
        """Moving straight east → bearing ≈ 90°."""
        b = bearing(33.0, -118.0, 33.0, -117.0)
        self.assertAlmostEqual(b, 90.0, delta=1.0)

    def test_due_west(self):
        """Moving straight west → bearing ≈ 270°."""
        b = bearing(33.0, -117.0, 33.0, -118.0)
        self.assertAlmostEqual(b, 270.0, delta=1.0)

    def test_northeast_quadrant(self):
        """NE destination should give a bearing between 0 and 90."""
        b = bearing(33.0, -117.5, 33.5, -117.0)
        self.assertGreater(b, 0.0)
        self.assertLess(b, 90.0)

    def test_output_range(self):
        """bearing() must always return a value in [0, 360)."""
        test_cases = [
            (33.0, -117.0, 34.0, -116.0),
            (34.0, -116.0, 33.0, -117.0),
            (33.0, -117.0, 33.0, -118.0),
            (33.97, -117.33, 33.98, -117.34),
        ]
        for args in test_cases:
            with self.subTest(args=args):
                b = bearing(*args)
                self.assertGreaterEqual(b, 0.0)
                self.assertLess(b, 360.0)


# ===========================================================================
# angle_diff() Tests
# ===========================================================================

class TestAngleDiff(unittest.TestCase):
    """angle_diff(current_heading, target_bearing) → signed error in (-180, 180]."""

    def test_no_difference(self):
        self.assertAlmostEqual(angle_diff(90.0, 90.0), 0.0)

    def test_positive_right_turn(self):
        """Target is 30° clockwise → positive error."""
        self.assertAlmostEqual(angle_diff(0.0, 30.0), 30.0)

    def test_negative_left_turn(self):
        """Target is 30° counter-clockwise → negative error."""
        self.assertAlmostEqual(angle_diff(30.0, 0.0), -30.0)

    def test_wrap_across_zero(self):
        """Heading 350°, target 10° → only 20° right turn, not -340°."""
        self.assertAlmostEqual(angle_diff(350.0, 10.0), 20.0)

    def test_wrap_other_direction(self):
        """Heading 10°, target 350° → only -20° left turn, not 340°."""
        self.assertAlmostEqual(angle_diff(10.0, 350.0), -20.0)

    def test_exact_180_ambiguous(self):
        """180° difference resolves to +180 (the modulo convention)."""
        result = angle_diff(0.0, 180.0)
        self.assertAlmostEqual(abs(result), 180.0, places=6)

    def test_output_within_range(self):
        """Result must always be in (-180, 180]."""
        for a in range(0, 360, 15):
            for b in range(0, 360, 15):
                with self.subTest(a=a, b=b):
                    d = angle_diff(float(a), float(b))
                    self.assertGreater(d, -180.0 - 1e-9)
                    self.assertLessEqual(d, 180.0 + 1e-9)


if __name__ == "__main__":
    unittest.main()
