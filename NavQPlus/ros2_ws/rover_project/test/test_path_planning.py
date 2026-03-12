"""
Unit tests for RoadNetworkGraph from path_planner_node.py.
These tests run without a live ROS2 context by mocking all ROS2 imports.

Note: StatePlaneConverter was removed from path_planner_node.py; only
      RoadNetworkGraph (GPS / Haversine-based) is tested here.
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

# Mock ament_index_python so the module-level get_package_share_directory call succeeds
_mock_ament = types.ModuleType("ament_index_python")
_mock_ament_pkgs = types.ModuleType("ament_index_python.packages")
_mock_ament_pkgs.get_package_share_directory = lambda pkg: "/tmp/mock_pkg_share"
sys.modules.setdefault("ament_index_python", _mock_ament)
sys.modules.setdefault("ament_index_python.packages", _mock_ament_pkgs)

# Now import the classes under test
import importlib.util, os
_script = os.path.join(
    os.path.dirname(__file__),
    "..", "scripts", "path_planner_node.py"
)
_spec = importlib.util.spec_from_file_location("path_planner_node", _script)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)

RoadNetworkGraph = _mod.RoadNetworkGraph
haversine_m = _mod.haversine_m


# ===========================================================================
# RoadNetworkGraph Tests
# ===========================================================================

class TestRoadNetworkGraphBasic(unittest.TestCase):

    def setUp(self):
        self.g = RoadNetworkGraph()  # default snap_decimals=6

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

    def test_segment_weight_is_haversine_distance(self):
        """Edge weight must be a positive Haversine distance in meters."""
        self.g.add_feature([(0.0, 0.0), (3.0, 4.0)])
        key0 = self.g._snap_key(0.0, 0.0)
        neighbours = self.g.adj[key0]
        self.assertEqual(len(neighbours), 1)
        _, weight = neighbours[0]
        expected = haversine_m(0.0, 0.0, 4.0, 3.0)  # add_feature passes (lon, lat)
        self.assertAlmostEqual(weight, expected, places=3)
        self.assertGreater(weight, 0.0)

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
        """A feature whose snapped endpoints round to the same key should add no edge."""
        # With snap_decimals=6, coordinates differing in the 7th decimal place
        # round to the same 6-decimal key and are treated as identical nodes.
        self.g.add_feature([(0.0, 0.0), (0.0000001, 0.0)])
        self.assertEqual(len(self.g.adj), 0)

    def test_stats_format(self):
        self.g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        s = self.g.stats()
        self.assertIn("nodes", s)
        self.assertIn("edges", s)


class TestRoadNetworkGraphSnapToNearest(unittest.TestCase):

    def setUp(self):
        self.g = RoadNetworkGraph()  # default snap_decimals=6
        self.g.add_feature([(0.0, 0.0), (100.0, 0.0), (200.0, 0.0)])

    def test_snap_exact_node(self):
        key, dist = self.g.snap_to_nearest(0.0, 0.0)
        self.assertIsNotNone(key)
        self.assertAlmostEqual(dist, 0.0, places=3)

    def test_snap_nearby_point(self):
        """A point very close to (100, 0) should snap to that node."""
        # Pass max_dist_m=inf since these are not real GPS coords — just
        # verifying nearest-node selection independent of distance threshold.
        key, dist = self.g.snap_to_nearest(101.0, 1.0, max_dist_m=float('inf'))
        self.assertIsNotNone(key)
        expected_key = self.g._snap_key(100.0, 0.0)
        self.assertEqual(key, expected_key)

    def test_snap_too_far_returns_none(self):
        """A point far from all nodes beyond max_dist_m must return None."""
        # Use a point not on any node and a very tight threshold (1 m).
        # (50.0, 0.0) is not a graph node; Haversine to nearest node is >>1 m.
        key, dist = self.g.snap_to_nearest(50.0, 0.0, max_dist_m=1.0)
        self.assertIsNone(key)

    def test_snap_selects_closest_of_multiple(self):
        """Should select the nearest of several nodes."""
        key, _ = self.g.snap_to_nearest(190.0, 0.0, max_dist_m=float('inf'))
        expected = self.g._snap_key(200.0, 0.0)
        self.assertEqual(key, expected)


class TestRoadNetworkGraphDijkstra(unittest.TestCase):

    def _build_chain(self, n=5):
        """Build a simple chain: 0-1-2-3-4, each segment 10 degrees apart."""
        g = RoadNetworkGraph(snap_decimals=6)
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
        g = RoadNetworkGraph(snap_decimals=6)
        g.add_feature([(0.0, 0.0), (10.0, 0.0)])
        g.add_feature([(100.0, 0.0), (110.0, 0.0)])
        k_start = g._snap_key(0.0, 0.0)
        k_goal = g._snap_key(100.0, 0.0)
        path = g.dijkstra(k_start, k_goal)
        self.assertIsNone(path)

    def test_path_with_branch_takes_shorter_route(self):
        """
        Graph:
          A --(short)-- B --(short)-- C
                |                     |
             (long)                (long)
                D -----(short)---- E
        Shortest A->C is A-B-C (2 hops), not A-B-D-E-C (4 hops).
        """
        g = RoadNetworkGraph(snap_decimals=6)
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
