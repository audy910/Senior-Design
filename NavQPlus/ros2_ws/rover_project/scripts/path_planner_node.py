#!/usr/bin/env python3
"""
path_planner_node.py

Builds a walkable road/path network graph from UCR_Centerlines shapefile
and computes shortest paths using Dijkstra's algorithm.

YOUR SHAPEFILE DATA:
  - 766 PolyLine features (road/sidewalk centerlines at UCR)
  - CRS: NAD 1983 StatePlane California VI (units: US feet)
  - Attributes: Walk_Path (Yes/empty), Paved, Notes
  - 187 features marked as Walk_Path=Yes
  - Notes include: 'Street Crossing', 'Ramp', etc.

APPROACH:
  Instead of a visibility graph around buildings, we build a graph
  directly from the centerline segments. Each line segment becomes
  edges in the graph, and intersections become nodes. Then we snap
  the rover's GPS position and the goal to the nearest point on the
  network and run Dijkstra.

NO EXTERNAL GEO DEPENDENCIES — uses pure stdlib shapefile parsing
and a manual State Plane ↔ WGS84 converter.

Subscribes:
  can/gps    (rover_project/GpsFix)    — current rover position
  nav/goal   (rover_project/NavGoal)   — target destination

Publishes:
  nav/waypoints (rover_project/WaypointList) — planned GPS waypoints
"""

import rclpy
from rclpy.node import Node
from rover_project.msg import GpsFix, NavGoal, WaypointList
from foxglove_msgs.msg import GeoJSON

import struct
import math
import heapq
import os
import time
import json


#  COORDINATE CONVERSION
#  NAD83 StatePlane California VI (US feet) ↔ WGS84 (lat/lon)
#
#  Your shapefile uses Lambert Conformal Conic projection:
#    False Easting:   6561664.5267 ft
#    False Northing:  1640417.5167 ft
#    Central Meridian: -116.25 deg
#    Std Parallel 1:   32.7833 deg
#    Std Parallel 2:   33.8833 deg
#    Latitude Origin:  32.1667 deg (standard for CA Zone VI)

class StatePlaneConverter:
    """Convert between NAD83 StatePlane California Zone VI (US feet) and WGS84 lat/lon."""

    def __init__(self):
        # GRS80 ellipsoid (NAD83)
        self.a = 6378137.0
        self.f = 1 / 298.257222101
        self.e2 = 2 * self.f - self.f ** 2
        self.e = math.sqrt(self.e2)

        # Lambert Conformal Conic parameters for CA Zone VI
        self.lat0 = math.radians(32.166666666667)
        self.lon0 = math.radians(-116.25)
        self.lat1 = math.radians(32.783333333333)
        self.lat2 = math.radians(33.883333333333)

        # US survey foot
        self.ft_to_m = 0.3048006096012192
        self.m_to_ft = 1.0 / self.ft_to_m

        self.FE = 6561664.52666667 * self.ft_to_m   # false easting in meters
        self.FN = 1640417.51666667 * self.ft_to_m   # false northing in meters

        self.n, self.F, self.rho0 = self._lambert_constants()

    def _m_func(self, lat):
        sin_lat = math.sin(lat)
        return math.cos(lat) / math.sqrt(1 - self.e2 * sin_lat ** 2)

    def _t_func(self, lat):
        sin_lat = math.sin(lat)
        return math.tan(math.pi / 4 - lat / 2) / (
            ((1 - self.e * sin_lat) / (1 + self.e * sin_lat)) ** (self.e / 2))

    def _lambert_constants(self):
        m1 = self._m_func(self.lat1)
        m2 = self._m_func(self.lat2)
        t0 = self._t_func(self.lat0)
        t1 = self._t_func(self.lat1)
        t2 = self._t_func(self.lat2)

        # Check for degenerate case (identical standard parallels)
        log_diff = math.log(t1) - math.log(t2)
        if abs(log_diff) < 1e-10:
            raise ValueError("Standard parallels are too close or identical")

        n = (math.log(m1) - math.log(m2)) / log_diff
        F = m1 / (n * t1 ** n)
        rho0 = self.a * F * t0 ** n
        return n, F, rho0

    def to_latlon(self, easting_ft, northing_ft):
        """State Plane (US feet) -> (lat_deg, lon_deg)."""
        x = easting_ft * self.ft_to_m - self.FE
        y = northing_ft * self.ft_to_m - self.FN

        rho = math.sqrt(x ** 2 + (self.rho0 - y) ** 2)
        if self.n < 0:
            rho = -rho

        # Check for zero rho (would cause division by zero)
        if abs(rho) < 1e-10:
            raise ValueError("Invalid coordinate: rho is zero")

        theta = math.atan2(x, self.rho0 - y)
        t = (rho / (self.a * self.F)) ** (1 / self.n)
        lon = theta / self.n + self.lon0

        lat = math.pi / 2 - 2 * math.atan(t)
        for _ in range(10):
            sin_lat = math.sin(lat)
            lat_new = math.pi / 2 - 2 * math.atan(
                t * ((1 - self.e * sin_lat) / (1 + self.e * sin_lat)) ** (self.e / 2))
            if abs(lat_new - lat) < 1e-12:
                break
            lat = lat_new

        lat_deg = math.degrees(lat)
        lon_deg = math.degrees(lon)

        # Validate output is within reasonable bounds (California region)
        if not (32.0 <= lat_deg <= 34.5):
            raise ValueError(f"Invalid latitude: {lat_deg} (expected 32-34.5)")
        if not (-118.0 <= lon_deg <= -116.0):
            raise ValueError(f"Invalid longitude: {lon_deg} (expected -118 to -116)")

        return lat_deg, lon_deg

    def to_stateplane(self, lat_deg, lon_deg):
        """(lat_deg, lon_deg) -> State Plane (easting_ft, northing_ft)."""
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        t = self._t_func(lat)
        rho = self.a * self.F * t ** self.n
        theta = self.n * (lon - self.lon0)
        x = rho * math.sin(theta) + self.FE
        y = self.rho0 - rho * math.cos(theta) + self.FN
        return x * self.m_to_ft, y * self.m_to_ft


#  SHAPEFILE PARSER

def read_shapefile(shp_path, dbf_path):
    """Read PolyLine shapefile + DBF. Returns list of feature dicts."""

    # Read DBF
    with open(dbf_path, "rb") as f:
        f.read(1); f.read(3)
        num_records = struct.unpack("<I", f.read(4))[0]
        header_size = struct.unpack("<H", f.read(2))[0]
        f.read(2); f.read(20)

        fields = []
        while True:
            fd = f.read(1)
            if fd == b'\r':
                break
            fd += f.read(31)
            name = fd[0:11].split(b'\x00')[0].decode('ascii')
            flen = fd[16]
            fields.append((name, flen))

        f.seek(header_size)
        attributes = []
        for _ in range(num_records):
            f.read(1)
            rec = {}
            for name, flen in fields:
                rec[name] = f.read(flen).decode('utf-8', errors='replace').strip()
            attributes.append(rec)

    # Read SHP
    geometries = []
    with open(shp_path, "rb") as f:
        f.seek(24)
        file_length = struct.unpack(">I", f.read(4))[0] * 2
        f.seek(100)

        while f.tell() < file_length:
            try:
                f.read(4)  # rec num
                content_len = struct.unpack(">I", f.read(4))[0] * 2
                rec_start = f.tell()
                shape_type = struct.unpack("<I", f.read(4))[0]

                if shape_type in (3, 13, 23):
                    f.read(32)  # bbox
                    num_parts = struct.unpack("<I", f.read(4))[0]
                    num_points = struct.unpack("<I", f.read(4))[0]
                    f.read(4 * num_parts)  # part indices
                    points = []
                    for _ in range(num_points):
                        x = struct.unpack("<d", f.read(8))[0]
                        y = struct.unpack("<d", f.read(8))[0]
                        points.append((x, y))
                    geometries.append(points)
                else:
                    geometries.append(None)

                f.seek(rec_start + content_len)
            except struct.error:
                break

    features = []
    for i in range(min(len(geometries), len(attributes))):
        if geometries[i] is not None:
            features.append({'attrs': attributes[i], 'points': geometries[i]})
    return features

#  ROAD NETWORK GRAPH

class RoadNetworkGraph:
    """
    Nodes = unique coordinate points (snapped to merge intersections).
    Edges = line segments weighted by distance in feet.
    """

    def __init__(self, snap_tolerance_ft=2.0):
        self.snap = snap_tolerance_ft
        self.adj = {}           # node_key -> [(neighbor_key, distance_ft)]
        self.node_coords = {}   # node_key -> (x_ft, y_ft)

    def _snap_key(self, x, y):
        sx = round(x / self.snap) * self.snap
        sy = round(y / self.snap) * self.snap
        return (sx, sy)

    def add_feature(self, points):
        if len(points) < 2:
            return
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            k1 = self._snap_key(x1, y1)
            k2 = self._snap_key(x2, y2)
            if k1 == k2:
                continue
            self.node_coords[k1] = (x1, y1)
            self.node_coords[k2] = (x2, y2)
            dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            self.adj.setdefault(k1, []).append((k2, dist))
            self.adj.setdefault(k2, []).append((k1, dist))

    def snap_to_nearest(self, x, y, max_dist_ft=500.0):
        best_key = None
        best_dist = float('inf')
        for key, (nx, ny) in self.node_coords.items():
            d = math.sqrt((x - nx) ** 2 + (y - ny) ** 2)
            if d < best_dist:
                best_dist = d
                best_key = key
        if best_dist > max_dist_ft:
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


#  ROS2 NODE

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter('shapefile_path', '')
        self.declare_parameter('walk_paths_only', False)
        self.declare_parameter('snap_tolerance_ft', 2.0)
        self.declare_parameter('max_snap_distance_ft', 500.0)
        self.declare_parameter('replan_deviation_m', 5.0)
        self.declare_parameter('replan_cooldown_s', 20.0)
        self.declare_parameter('max_h_acc_m', 10.0)

        shapefile_path = self.get_parameter('shapefile_path').value
        self.walk_only = self.get_parameter('walk_paths_only').value
        snap_tol = self.get_parameter('snap_tolerance_ft').value
        self._snap_tol = snap_tol
        self.max_snap_ft = self.get_parameter('max_snap_distance_ft').value
        self.replan_dev = self.get_parameter('replan_deviation_m').value
        self.replan_cooldown = self.get_parameter('replan_cooldown_s').value
        self.max_h_acc_m = self.get_parameter('max_h_acc_m').value

        # State
        self.converter = StatePlaneConverter()
        self.graph = RoadNetworkGraph(snap_tolerance_ft=snap_tol)
        self.current_lat = None
        self.current_lon = None
        self.current_fix_type = 0
        self.current_h_acc = 0.0
        self.current_hdop = 0.0
        self.current_num_sats = 0
        self.goal_lat = None
        self.goal_lon = None
        self.current_path_latlon = None
        self._last_plan_time = 0.0
        self.graph_full = None   # fallback when walk_only network is disconnected

        # Pub/Sub
        self.waypoint_pub = self.create_publisher(WaypointList, 'nav/waypoints', 10)
        self.path_geojson_pub = self.create_publisher(GeoJSON, 'nav/path_geojson', 1)
        self.gps_sub = self.create_subscription(GpsFix, 'can/gps', self.gps_callback, 10)
        self.goal_sub = self.create_subscription(NavGoal, 'nav/goal', self.goal_callback, 10)
        self.create_timer(2.0, self.check_replan)

        # Load
        if shapefile_path:
            shp = shapefile_path
            dbf = shapefile_path.replace('.shp', '.dbf')
            if os.path.exists(shp) and os.path.exists(dbf):
                self.load_network(shp, dbf)
            else:
                self.get_logger().error(f"Shapefile not found: {shp}")
        else:
            self.get_logger().error("No shapefile_path parameter set!")

    def load_network(self, shp_path, dbf_path):
        self.get_logger().info(f"Loading centerlines: {shp_path}")
        t0 = time.time()

        features = read_shapefile(shp_path, dbf_path)
        self.get_logger().info(f"  {len(features)} features loaded")

        if self.walk_only:
            self.graph_full = RoadNetworkGraph(snap_tolerance_ft=self._snap_tol)

        used = 0
        for feat in features:
            if self.walk_only:
                self.graph_full.add_feature(feat['points'])
                if feat['attrs'].get('Walk_Path', '') != 'Yes':
                    continue
            self.graph.add_feature(feat['points'])
            used += 1

        self.get_logger().info(
            f"  ✓ Built in {time.time()-t0:.2f}s: {self.graph.stats()} "
            f"({used} features used)"
            + (f" | fallback full graph: {self.graph_full.stats()}" if self.graph_full else ""))

        # Sanity check: convert a centroid point
        test_lat, test_lon = self.converter.to_latlon(6235000, 2300000)
        self.get_logger().info(
            f"  Coord test: SP(6235000,2300000) → ({test_lat:.5f}, {test_lon:.5f})")

    def gps_callback(self, msg: GpsFix):
        self.current_fix_type = msg.fix_type
        self.current_h_acc = msg.h_acc
        self.current_hdop = msg.hdop
        self.current_num_sats = msg.num_sats
        if msg.h_acc > self.max_h_acc_m and msg.h_acc > 0.0:
            self.get_logger().warn(
                f"GPS rejected: h_acc={msg.h_acc:.1f}m > {self.max_h_acc_m:.0f}m "
                f"(sats={msg.num_sats}, HDOP={msg.hdop:.1f})", throttle_duration_sec=5.0)
            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def goal_callback(self, msg: NavGoal):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude
        self.get_logger().info(f"Goal: ({self.goal_lat:.6f}, {self.goal_lon:.6f})")
        self.plan_path()

    def plan_path(self):
        if not self.graph.adj:
            self.get_logger().error("No network loaded"); return
        if self.current_lat is None:
            self.get_logger().warn("No GPS fix"); return
        if self.goal_lat is None:
            self.get_logger().warn("No goal"); return
        if self.current_fix_type < 2:
            self.get_logger().warn(f"GPS fix type {self.current_fix_type} too low"); return

        self.get_logger().info(
            f"  GPS: fix={self.current_fix_type} sats={self.current_num_sats} "
            f"HDOP={self.current_hdop:.1f} h_acc={self.current_h_acc:.1f}m")

        start_ft = self.converter.to_stateplane(self.current_lat, self.current_lon)
        goal_ft = self.converter.to_stateplane(self.goal_lat, self.goal_lon)

        start_node, sd = self.graph.snap_to_nearest(start_ft[0], start_ft[1], self.max_snap_ft)
        goal_node, gd = self.graph.snap_to_nearest(goal_ft[0], goal_ft[1], self.max_snap_ft)

        if start_node is None:
            self.get_logger().error(f"Start too far from network ({sd:.0f}ft)"); return
        if goal_node is None:
            self.get_logger().error(f"Goal too far from network ({gd:.0f}ft)"); return

        self.get_logger().info(f"  Snapped start={sd:.1f}ft, goal={gd:.1f}ft")

        t0 = time.time()
        active_graph = self.graph
        path_keys = self.graph.dijkstra(start_node, goal_node)

        if path_keys is None and self.graph_full is not None:
            self.get_logger().warn(
                "Walk-only path not found — retrying with full network")
            start_node_f, _ = self.graph_full.snap_to_nearest(
                start_ft[0], start_ft[1], self.max_snap_ft)
            goal_node_f, _ = self.graph_full.snap_to_nearest(
                goal_ft[0], goal_ft[1], self.max_snap_ft)
            if start_node_f is not None and goal_node_f is not None:
                path_keys = self.graph_full.dijkstra(start_node_f, goal_node_f)
                if path_keys is not None:
                    active_graph = self.graph_full

        if path_keys is None:
            self.get_logger().error("No path found — network may be disconnected"); return

        self.get_logger().info(
            f"  ✓ {len(path_keys)} waypoints in {time.time()-t0:.4f}s"
            + (" [full network fallback]" if active_graph is self.graph_full else ""))

        lats, lons = [], []
        for key in path_keys:
            x, y = active_graph.node_coords[key]
            lat, lon = self.converter.to_latlon(x, y)
            lats.append(lat)
            lons.append(lon)

        wp_msg = WaypointList()
        wp_msg.latitudes = lats
        wp_msg.longitudes = lons
        wp_msg.total_waypoints = len(lats)
        wp_msg.current_index = 0
        self.waypoint_pub.publish(wp_msg)

        # Publish path as GeoJSON for Foxglove Map panel
        # GeoJSON coordinates are [longitude, latitude]
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

        total_ft = sum(
            math.sqrt((active_graph.node_coords[path_keys[i+1]][0] - active_graph.node_coords[path_keys[i]][0])**2 +
                       (active_graph.node_coords[path_keys[i+1]][1] - active_graph.node_coords[path_keys[i]][1])**2)
            for i in range(len(path_keys)-1))
        self.get_logger().info(f"  Distance: {total_ft*0.3048:.0f}m ({total_ft:.0f}ft)")

        for i, (lat, lon) in enumerate(self.current_path_latlon):
            self.get_logger().info(f"  WP{i}: ({lat:.6f}, {lon:.6f})")

    def check_replan(self):
        """Check if rover has deviated from the planned path."""
        if not self.current_path_latlon or self.current_lat is None or len(self.current_path_latlon) < 2:
            return
        if time.time() - self._last_plan_time < self.replan_cooldown:
            return

        # Calculate distance to nearest path segment (not just nearest point)
        min_segment_dist = float('inf')
        for i in range(len(self.current_path_latlon) - 1):
            lat1, lon1 = self.current_path_latlon[i]
            lat2, lon2 = self.current_path_latlon[i + 1]
            dist = self._distance_to_segment(self.current_lat, self.current_lon, lat1, lon1, lat2, lon2)
            min_segment_dist = min(min_segment_dist, dist)

        if min_segment_dist > self.replan_dev:
            self.get_logger().warn(f"Deviated {min_segment_dist:.1f}m from path — replanning")
            self.plan_path()

    def _distance_to_segment(self, px, py, x1, y1, x2, y2):
        """Calculate perpendicular distance from point (px, py) to line segment (x1,y1)-(x2,y2)."""
        # Convert to simplified 2D (this is approximate but works for small distances)
        # Project point onto line segment and compute distance
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            # Segment is a point
            return self._hav(px, py, x1, y1)

        # Parameter t for point projection onto line
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))

        # Nearest point on segment
        nearest_x = x1 + t * dx
        nearest_y = y1 + t * dy

        return self._hav(px, py, nearest_x, nearest_y)

    @staticmethod
    def _hav(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()