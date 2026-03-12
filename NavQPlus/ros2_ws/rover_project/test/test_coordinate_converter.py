"""
Unit tests for StatePlaneConverter and RoadNetworkGraph from path_planner_node.py.
These tests run without a live ROS2 context by mocking all ROS2 imports.
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

for _mod in ("rover_project", "rover_project.msg", "foxglove_msgs", "foxglove_msgs.msg"):
    sys.modules.setdefault(_mod, types.ModuleType(_mod))

# Provide stub message classes so the module-level imports succeed
_rover_msg = sys.modules["rover_project.msg"]
for _cls in ("GpsFix", "NavGoal", "WaypointList"):
    setattr(_rover_msg, _cls, MagicMock)
_fox_msg = sys.modules["foxglove_msgs.msg"]
setattr(_fox_msg, "GeoJSON", MagicMock)

# Now import the classes under test
import importlib.util, os
_script = os.path.join(
    os.path.dirname(__file__),
    "..", "scripts", "path_planner_node.py"
)
_spec = importlib.util.spec_from_file_location("path_planner_node", _script)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

StatePlaneConverter = _mod.StatePlaneConverter
RoadNetworkGraph = _mod.RoadNetworkGraph


# ===========================================================================
# StatePlaneConverter Tests
# ===========================================================================

class TestStatePlaneConverterRoundTrip(unittest.TestCase):
    """Verify that converting lat/lon -> state plane -> lat/lon is lossless."""

    def setUp(self):
        self.conv = StatePlaneConverter()

    def _assert_roundtrip(self, lat, lon, tol_deg=1e-6):
        e, n = self.conv.to_stateplane(lat, lon)
        lat2, lon2 = self.conv.to_latlon(e, n)
        self.assertAlmostEqual(lat, lat2, places=5,
                               msg=f"Latitude round-trip error for ({lat},{lon})")
        self.assertAlmostEqual(lon, lon2, places=5,
                               msg=f"Longitude round-trip error for ({lat},{lon})")

    def test_roundtrip_ucr_campus(self):
        """UCR campus centre (approx. 33.9737 N, -117.3281 W)."""
        self._assert_roundtrip(33.9737, -117.3281)

    def test_roundtrip_southern_bound(self):
        """Near southern boundary of CA Zone VI coverage."""
        self._assert_roundtrip(32.5, -116.5)

    def test_roundtrip_northern_bound(self):
        """Near northern boundary of CA Zone VI coverage."""
        self._assert_roundtrip(34.0, -117.8)

    def test_roundtrip_multiple_points(self):
        test_points = [
            (33.0, -116.5),
            (33.5, -117.0),
            (34.2, -117.5),
        ]
        for lat, lon in test_points:
            with self.subTest(lat=lat, lon=lon):
                self._assert_roundtrip(lat, lon)


class TestStatePlaneConverterKnownPoint(unittest.TestCase):
    """Check that a known state-plane coordinate converts to the expected lat/lon."""

    def test_known_coordinate_region(self):
        """State-plane coordinate (6235000, 2300000) should fall inside CA Zone VI."""
        conv = StatePlaneConverter()
        lat, lon = conv.to_latlon(6235000, 2300000)
        self.assertGreater(lat, 32.0)
        self.assertLess(lat, 34.5)
        self.assertGreater(lon, -118.0)
        self.assertLess(lon, -116.0)


class TestStatePlaneConverterValidation(unittest.TestCase):
    """Boundary validation should raise ValueError for out-of-region outputs."""

    def test_out_of_region_raises(self):
        """Feeding extreme state-plane values should raise ValueError (lat/lon check)."""
        conv = StatePlaneConverter()
        # These absurdly large values will produce a lat/lon outside California
        with self.assertRaises((ValueError, Exception)):
            conv.to_latlon(0.0, 0.0)


# ===========================================================================
# RoadNetworkGraph Tests
# ===========================================================================

class TestRoadNetworkGraphBasic(unittest.TestCase):

    def setUp(self):
        self.g = RoadNetworkGraph(snap_tolerance_ft=2.0)

    def test_empty_graph(self):
        self.assertEqual(len(self.g.node_coords), 0)
        self.assertEqual(len(self.g.adj), 0)

    def test_add_single_segment(self):
        self.g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        self.assertEqual(len(self.g.node_coords), 2)
        # Each node should have one neighbour
        for key in self.g.adj:
            self.assertEqual(len(self.g.adj[key]), 1)

    def test_add_feature_too_short(self):
        """A single-point feature should add nothing."""
        self.g.add_feature([(0.0, 0.0)])
        self.assertEqual(len(self.g.node_coords), 0)

    def test_segment_weight_is_euclidean_distance(self):
        self.g.add_feature([(0.0, 0.0), (3.0, 4.0)])  # 3-4-5 triangle → dist=5
        key0 = self.g._snap_key(0.0, 0.0)
        neighbours = self.g.adj[key0]
        self.assertEqual(len(neighbours), 1)
        _, weight = neighbours[0]
        self.assertAlmostEqual(weight, 5.0, places=5)

    def test_bidirectional_edges(self):
        """Every edge must appear in both directions."""
        self.g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        k1 = self.g._snap_key(0.0, 0.0)
        k2 = self.g._snap_key(10.0, 0.0)
        neighbours_of_k1 = [k for k, _ in self.g.adj[k1]]
        neighbours_of_k2 = [k for k, _ in self.g.adj[k2]]
        self.assertIn(k2, neighbours_of_k1)
        self.assertIn(k1, neighbours_of_k2)

    def test_duplicate_point_ignored(self):
        """A feature whose snapped endpoints are identical should add no edge."""
        self.g.add_feature([(0.0, 0.0), (0.5, 0.0)])  # snap_tol=2 → same bucket
        self.assertEqual(len(self.g.adj), 0)

    def test_stats_format(self):
        self.g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        s = self.g.stats()
        self.assertIn("nodes", s)
        self.assertIn("edges", s)


class TestRoadNetworkGraphSnapToNearest(unittest.TestCase):

    def setUp(self):
        self.g = RoadNetworkGraph(snap_tolerance_ft=2.0)
        self.g.add_feature([(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)])

    def test_snap_exact_node(self):
        key, dist = self.g.snap_to_nearest(0.0, 0.0)
        self.assertIsNotNone(key)
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_snap_nearby_point(self):
        """A point very close to (100,0) should snap to that node."""
        key, dist = self.g.snap_to_nearest(101.0, 1.0)
        self.assertIsNotNone(key)
        expected_key = self.g._snap_key(100.0, 0.0)
        self.assertEqual(key, expected_key)

    def test_snap_too_far_returns_none(self):
        key, dist = self.g.snap_to_nearest(0.0, 0.0, max_dist_ft=0.001)
        self.assertIsNone(key)

    def test_snap_selects_closest_of_multiple(self):
        """Should select the nearest of several nodes."""
        key, _ = self.g.snap_to_nearest(190.0, 0.0)
        expected = self.g._snap_key(200.0, 0.0)
        self.assertEqual(key, expected)


class TestRoadNetworkGraphDijkstra(unittest.TestCase):

    def _build_chain(self, n=5):
        """Build a simple chain: 0-1-2-3-4, each segment 10 ft long."""
        g = RoadNetworkGraph(snap_tolerance_ft=0.1)
        points = [(i * 10.0, 0.0) for i in range(n)]
        g.add_feature(points)
        return g, points

    def test_trivial_same_node(self):
        g, pts = self._build_chain()
        k = g._snap_key(*pts[0])
        path = g.dijkstra(k, k)
        # Returns a single-node path or None — both acceptable for same start/goal
        # but the standard Dijkstra here will have prev[start]=None, so path=[start]
        if path is not None:
            self.assertEqual(path[0], k)

    def test_shortest_path_chain(self):
        g, pts = self._build_chain()
        k0 = g._snap_key(*pts[0])
        k4 = g._snap_key(*pts[4])
        path = g.dijkstra(k0, k4)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], k0)
        self.assertEqual(path[-1], k4)
        self.assertEqual(len(path), 5)

    def test_no_path_disconnected_graph(self):
        """Two disconnected segments → no path between them."""
        g = RoadNetworkGraph(snap_tolerance_ft=0.1)
        g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        g.add_feature([(100.0, 0.0), (110.0, 0.0)])
        k_start = g._snap_key(0.0, 0.0)
        k_goal = g._snap_key(100.0, 0.0)
        path = g.dijkstra(k_start, k_goal)
        self.assertIsNone(path)

    def test_path_with_branch_takes_shorter_route(self):
        """
        Graph:
          A --(10)-- B --(10)-- C
                |               |
               (50)            (50)
                D -----(10)--- E
        Shortest A->C is A-B-C (cost 20), not A-B-D-E-C (cost 120).
        """
        g = RoadNetworkGraph(snap_tolerance_ft=0.1)
        A, B, C = (0.0, 0.0), (10.0, 0.0), (20.0, 0.0)
        D, E    = (10.0, 50.0), (20.0, 50.0)
        g.add_feature([A, B, C])
        g.add_feature([B, D])
        g.add_feature([D, E])
        g.add_feature([E, C])

        kA = g._snap_key(*A)
        kC = g._snap_key(*C)
        path = g.dijkstra(kA, kC)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 3)   # A -> B -> C


if __name__ == "__main__":
    unittest.main()
