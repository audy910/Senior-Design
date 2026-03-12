#!/usr/bin/env python3
"""
path_planner_node.py

Builds a walkable road/path network graph from a GeoJSON file (UCR_Centerlines.json)
and computes shortest paths using Dijkstra's algorithm.

APPROACH:
  Reads standard WGS84 [Longitude, Latitude] coordinates from GeoJSON.
  Each line segment becomes an edge in the graph, weighted by Haversine distance (meters).
  Intersections become nodes. Snaps the rover's GPS position and goal to the nearest 
  point on the network and runs Dijkstra.

Subscribes:
  can/gps    (rover_project/GpsFix)    — current rover position
  nav/goal   (rover_project/NavGoal)   — target destination

Publishes:
  nav/waypoints (rover_project/WaypointList) — planned GPS waypoints
  nav/path_geojson (foxglove_msgs/GeoJSON)   — path for foxglove visualization
"""

import rclpy
from rclpy.node import Node
from rover_project.msg import GpsFix, NavGoal, WaypointList
from foxglove_msgs.msg import GeoJSON
from ament_index_python.packages import get_package_share_directory

import math
import heapq
import os
import time
import json


# ——— MATH UTILITIES ———

def haversine_m(lat1, lon1, lat2, lon2):
    """Calculate the great-circle distance between two points in meters."""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ——— ROAD NETWORK GRAPH ———

class RoadNetworkGraph:
    """
    Nodes = unique [lon, lat] coordinates (snapped by decimal precision to merge intersections).
    Edges = line segments weighted by actual distance in meters.
    """
    def __init__(self, snap_decimals=6):
        self.snap_decimals = snap_decimals
        self.adj = {}           # node_key -> [(neighbor_key, distance_m)]
        self.node_coords = {}   # node_key -> (lon, lat)

    def _snap_key(self, lon, lat):
        return (round(lon, self.snap_decimals), round(lat, self.snap_decimals))

    def add_feature(self, points):
        """Adds a list of [lon, lat] points as edges to the graph."""
        if len(points) < 2:
            return
        for i in range(len(points) - 1):
            lon1, lat1 = points[i]
            lon2, lat2 = points[i + 1]

            k1 = self._snap_key(lon1, lat1)
            k2 = self._snap_key(lon2, lat2)

            if k1 == k2:
                continue

            self.node_coords[k1] = (lon1, lat1)
            self.node_coords[k2] = (lon2, lat2)

            dist_m = haversine_m(lat1, lon1, lat2, lon2)

            self.adj.setdefault(k1, []).append((k2, dist_m))
            self.adj.setdefault(k2, []).append((k1, dist_m))

    def snap_to_nearest(self, lon, lat, max_dist_m=150.0):
        """Find the closest node in the graph to the given coordinates."""
        best_key = None
        best_dist = float('inf')
        for key, (n_lon, n_lat) in self.node_coords.items():
            d = haversine_m(lat, lon, n_lat, n_lon)
            if d < best_dist:
                best_dist = d
                best_key = key

        if best_dist > max_dist_m:
            return None, best_dist
        return best_key, best_dist

    def dijkstra(self, start_key, goal_key):
        if start_key not in self.adj or goal_key not in self.adj:
            return None

        dist = {start_key: 0.0}
        prev = {start_key: None}
        pq = [(0.0, start_key)]
        visited = set()

        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if u == goal_key:
                break

            for v, w in self.adj.get(u, []):
                if v in visited:
                    continue
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if goal_key not in prev:
            return None

        path = []
        node = goal_key
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return path

    def stats(self):
        return f"{len(self.node_coords)} nodes, {sum(len(v) for v in self.adj.values())//2} edges"


# ——— PATH UTILITIES ———

def downsample_path(lats, lons, min_spacing_m=10.0):
    """
    Reduce waypoint density by keeping only points that are at least
    min_spacing_m apart. Always keeps the first and last point.
    """
    if len(lats) < 2:
        return lats, lons

    out_lats = [lats[0]]
    out_lons = [lons[0]]

    for i in range(1, len(lats) - 1):
        d = haversine_m(out_lats[-1], out_lons[-1], lats[i], lons[i])
        if d >= min_spacing_m:
            out_lats.append(lats[i])
            out_lons.append(lons[i])

    # Always include the final goal
    out_lats.append(lats[-1])
    out_lons.append(lons[-1])

    return out_lats, out_lons


# ——— ROS 2 NODE ———

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 1. Resolve Map Path
        try:
            package_share = get_package_share_directory('rover_project')
            default_geojson = os.path.join(package_share, 'UCR_Centerlines.json')
        except Exception as e:
            self.get_logger().error(f"Failed to find package share: {e}")
            default_geojson = 'UCR_Centerlines.json'

        # 2. Parameters
        self.declare_parameter('geojson_path', default_geojson)
        self.declare_parameter('walk_paths_only', False)
        self.declare_parameter('max_snap_distance_m', 150.0)
        self.declare_parameter('replan_deviation_m', 5.0)
        self.declare_parameter('replan_cooldown_s', 20.0)
        self.declare_parameter('max_h_acc_m', 10.0)
        self.declare_parameter('waypoint_spacing_m', 10.0)   # FIX 4: downsample spacing
        self.declare_parameter('gps_timeout_s', 5.0)         # FIX 2: staleness threshold

        geojson_path = self.get_parameter('geojson_path').value
        self.walk_only = self.get_parameter('walk_paths_only').value
        self.max_snap_m = self.get_parameter('max_snap_distance_m').value
        self.replan_dev = self.get_parameter('replan_deviation_m').value
        self.replan_cooldown = self.get_parameter('replan_cooldown_s').value
        self.max_h_acc_m = self.get_parameter('max_h_acc_m').value
        self.waypoint_spacing_m = self.get_parameter('waypoint_spacing_m').value
        self.gps_timeout_s = self.get_parameter('gps_timeout_s').value

        # 3. State
        self.graph = RoadNetworkGraph()
        self.graph_full = None

        self.current_lat = None
        self.current_lon = None
        self.current_fix_type = 0
        self.current_h_acc = 0.0
        self.current_hdop = 0.0
        self.current_num_sats = 0
        self.last_gps_time = 0.0        # FIX 2: track when GPS was last received

        self.goal_lat = None
        self.goal_lon = None
        self.current_path_latlon = None
        self._last_plan_time = 0.0

        # 4. Pub/Sub
        self.waypoint_pub = self.create_publisher(WaypointList, 'nav/waypoints', 10)
        self.path_geojson_pub = self.create_publisher(GeoJSON, 'nav/path_geojson', 1)
        self.gps_sub = self.create_subscription(GpsFix, 'can/gps', self.gps_callback, 10)
        self.goal_sub = self.create_subscription(NavGoal, 'nav/goal', self.goal_callback, 10)
        self.create_timer(2.0, self.check_replan)

        # 5. Load Map
        self.load_network(geojson_path)

    def load_network(self, geojson_path):
        self.get_logger().info(f"Loading map from: {geojson_path}")
        t0 = time.time()

        if not os.path.exists(geojson_path):
            self.get_logger().error(f"GeoJSON file not found at: {geojson_path}")
            return

        try:
            with open(geojson_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to parse GeoJSON: {e}")
            return

        if self.walk_only:
            self.graph_full = RoadNetworkGraph()

        features = data.get('features', [])
        used = 0

        for feat in features:
            geom = feat.get('geometry', {})
            props = feat.get('properties', {})

            if geom.get('type') == 'LineString':
                coords = geom.get('coordinates', [])

                if self.walk_only:
                    self.graph_full.add_feature(coords)
                    if props.get('Walk_Path', '') != 'Yes':
                        continue

                self.graph.add_feature(coords)
                used += 1

        self.get_logger().info(
            f"  ✓ Built in {time.time()-t0:.2f}s: {self.graph.stats()} "
            f"({used} features used)"
            + (f" | fallback full graph: {self.graph_full.stats()}" if self.graph_full else ""))

    def gps_callback(self, msg: GpsFix):
        self.current_fix_type = msg.fix_type
        self.current_h_acc = msg.h_acc
        self.current_hdop = msg.hdop
        self.current_num_sats = msg.num_sats

        if msg.h_acc > self.max_h_acc_m and msg.h_acc > 0.0:
            self.get_logger().warn(
                f"GPS rejected: h_acc={msg.h_acc:.1f}m > {self.max_h_acc_m:.0f}m",
                throttle_duration_sec=5.0)
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_time = time.time()    # FIX 2: stamp every accepted fix

    def goal_callback(self, msg: NavGoal):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude
        self.get_logger().info(f"Goal received: ({self.goal_lat:.6f}, {self.goal_lon:.6f})")
        self.plan_path()

    def _gps_is_fresh(self):
        """FIX 2: Returns True only if a valid GPS fix arrived within the timeout window."""
        if self.last_gps_time == 0.0:
            return False
        return (time.time() - self.last_gps_time) < self.gps_timeout_s

    def plan_path(self):
        if not self.graph.adj:
            self.get_logger().error("No network loaded. Cannot plan."); return
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().warn("No GPS fix yet. Cannot plan."); return
        if not self._gps_is_fresh():                             # FIX 2
            self.get_logger().warn("GPS data is stale. Cannot plan."); return
        if self.goal_lat is None or self.goal_lon is None:
            self.get_logger().warn("No goal set."); return
        if self.current_fix_type < 2:
            self.get_logger().warn(f"GPS fix type {self.current_fix_type} too low"); return

        self.get_logger().info(f"  GPS: fix={self.current_fix_type} sats={self.current_num_sats} h_acc={self.current_h_acc:.1f}m")

        start_node, sd = self.graph.snap_to_nearest(self.current_lon, self.current_lat, self.max_snap_m)
        goal_node, gd = self.graph.snap_to_nearest(self.goal_lon, self.goal_lat, self.max_snap_m)

        if start_node is None:
            self.get_logger().error(f"Start too far from network ({sd:.0f}m)"); return
        if goal_node is None:
            self.get_logger().error(f"Goal too far from network ({gd:.0f}m)"); return

        self.get_logger().info(f"  Snapped to map: start_dist={sd:.1f}m, goal_dist={gd:.1f}m")

        t0 = time.time()
        active_graph = self.graph
        path_keys = self.graph.dijkstra(start_node, goal_node)

        if path_keys is None and self.graph_full is not None:
            self.get_logger().warn("Walk-only path not found — retrying with full network")
            start_node_f, _ = self.graph_full.snap_to_nearest(self.current_lon, self.current_lat, self.max_snap_m)
            goal_node_f, _ = self.graph_full.snap_to_nearest(self.goal_lon, self.goal_lat, self.max_snap_m)
            if start_node_f is not None and goal_node_f is not None:
                path_keys = self.graph_full.dijkstra(start_node_f, goal_node_f)
                if path_keys is not None:
                    active_graph = self.graph_full

        if path_keys is None:
            self.get_logger().error("No path found — network may be disconnected"); return

        raw_lats, raw_lons = [], []
        for key in path_keys:
            lon, lat = active_graph.node_coords[key]
            raw_lats.append(lat)
            raw_lons.append(lon)

        # FIX 4: Downsample dense waypoints before publishing
        lats, lons = downsample_path(raw_lats, raw_lons, self.waypoint_spacing_m)

        self.get_logger().info(
            f"  ✓ {len(path_keys)} raw → {len(lats)} waypoints in {time.time()-t0:.4f}s"
            + (" [full network fallback]" if active_graph is self.graph_full else ""))

        # 1. Publish Waypoints
        wp_msg = WaypointList()
        wp_msg.latitudes = lats
        wp_msg.longitudes = lons
        wp_msg.total_waypoints = len(lats)
        wp_msg.current_index = 0
        self.waypoint_pub.publish(wp_msg)

        # 2. Publish GeoJSON for Foxglove
        coords = [[lon, lat] for lat, lon in zip(lats, lons)]
        geo_msg = GeoJSON()
        geo_msg.geojson = json.dumps({
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": coords},
                "properties": {"name": "planned_path"}
            }]
        })
        self.path_geojson_pub.publish(geo_msg)

        self.current_path_latlon = list(zip(lats, lons))
        self._last_plan_time = time.time()

        total_m = sum(
            haversine_m(lats[i], lons[i], lats[i+1], lons[i+1])
            for i in range(len(lats)-1)
        )
        self.get_logger().info(f"  Total Path Distance: {total_m:.0f} meters")

    def check_replan(self):
        """Check if rover has deviated from the planned path."""
        if not self.current_path_latlon or self.current_lat is None or len(self.current_path_latlon) < 2:
            return
        if not self._gps_is_fresh():                            # FIX 2
            return
        if time.time() - self._last_plan_time < self.replan_cooldown:
            return

        min_segment_dist = float('inf')
        for i in range(len(self.current_path_latlon) - 1):
            lat1, lon1 = self.current_path_latlon[i]
            lat2, lon2 = self.current_path_latlon[i + 1]
            dist = self._distance_to_segment(self.current_lat, self.current_lon, lat1, lon1, lat2, lon2)
            min_segment_dist = min(min_segment_dist, dist)

        if min_segment_dist > self.replan_dev:
            self.get_logger().warn(f"Deviated {min_segment_dist:.1f}m from path — replanning")
            self._last_plan_time = time.time()  # FIX 3: reset cooldown on deviation to prevent thrash
            self.plan_path()

    def _distance_to_segment(self, p_lat, p_lon, lat1, lon1, lat2, lon2):
        """
        Approximate perpendicular distance in meters from point (p_lat, p_lon)
        to line segment (lat1,lon1)-(lat2,lon2).

        FIX 1: corrected axis labeling and cos_lat applied to longitude (x-axis) only.
        """
        deg_to_m = 111320.0
        cos_lat = math.cos(math.radians(p_lat))   # FIX 1: use p_lat (latitude) for cos correction

        # Convert to local flat meters: lat -> y, lon -> x (scaled by cos_lat)
        px_m = p_lon * deg_to_m * cos_lat
        py_m = p_lat * deg_to_m
        x1_m = lon1 * deg_to_m * cos_lat
        y1_m = lat1 * deg_to_m
        x2_m = lon2 * deg_to_m * cos_lat
        y2_m = lat2 * deg_to_m

        dx = x2_m - x1_m
        dy = y2_m - y1_m

        if dx == 0 and dy == 0:
            return haversine_m(p_lat, p_lon, lat1, lon1)

        t = max(0.0, min(1.0, ((px_m - x1_m) * dx + (py_m - y1_m) * dy) / (dx * dx + dy * dy)))

        nearest_x = x1_m + t * dx
        nearest_y = y1_m + t * dy

        return math.sqrt((px_m - nearest_x)**2 + (py_m - nearest_y)**2)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
