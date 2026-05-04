#!/usr/bin/env python3
"""Drone Control Dashboard — tkinter GUI with ROS 2 backend.

Shows each drone's position, status, and a Takeoff/Land toggle button.

Usage:
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 ~/PX4-Autopilot/scripts/drone_dashboard.py
"""
import json
import math
import os
import random
import re
import time
import threading
import tkinter as tk
from heapq import heappush, heappop

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# Drone instance IDs to control (no instance 0 — only x500_1, x500_2, x500_3)
DRONE_IDS = [1, 2, 3]

NAV_STATE_NAMES = {
    0: 'Manual', 1: 'AltCtl', 2: 'PosCtl', 3: 'Mission',
    4: 'Loiter', 5: 'RTL', 10: 'Acro', 14: 'Offboard',
    15: 'Stabilized', 17: 'Takeoff', 18: 'Land', 20: 'PrecLand',
}

CONNECTION_TIMEOUT = 2.0  # seconds without a message = disconnected

# Path to the buildings model (for the 2D map)
BUILDINGS_SDF = os.path.join(
    os.path.dirname(__file__), '..', 'Tools', 'simulation', 'gz', 'models',
    'city_buildings', 'model.sdf')

DRONE_COLORS = {1: '#00ccff', 2: '#ff6644', 3: '#44ff66'}
DRONE_LABELS = {1: 'D1', 2: 'D2', 3: 'D3'}

# No-fly zones JSON file
NO_FLY_ZONES_FILE = os.path.join(os.path.dirname(__file__), 'no_fly_zones.json')

# Battery replacement stations JSON file
BRS_FILE = os.path.join(os.path.dirname(__file__), 'battery_stations.json')

# Storage buildings JSON file
STORAGE_FILE = os.path.join(os.path.dirname(__file__), 'storage_buildings.json')

SAFETY_RADIUS = 5.0  # metres — expansion radius for obstacle corners / zones

# Spawn positions in WORLD NED (north, east) — derived from city.sdf Gazebo ENU poses.
# Gazebo ENU (east, north) → NED (north, east):
#   x500_1: ENU (3, -3) → NED (-3, 3)
#   x500_2: ENU (-3, -3) → NED (-3, -3)
#   x500_3: ENU (0, 4) → NED (4, 0)
DRONE_SPAWN_NED = {
    1: (-3.0,  3.0),
    2: (-3.0, -3.0),
    3: ( 4.0,  0.0),
}


def _load_brs(json_path):
    """Load battery replacement stations from JSON.
    Each station: {"name": str, "position": [East, North]}.
    Returns list of dicts with NED keys added: 'north', 'east'.
    """
    try:
        with open(json_path) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []
    stations = []
    for s in data.get('stations', []):
        east, north = s['position']
        stations.append({
            'name': s.get('name', 'BRS'),
            'north': north,
            'east': east,
        })
    return stations


def _load_storage(json_path):
    """Load storage buildings from JSON.
    Each building: {"name": str, "position": [East, North]}.
    Returns list of dicts with NED keys: 'name', 'north', 'east'.
    """
    try:
        with open(json_path) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []
    result = []
    for b in data.get('buildings', []):
        east, north = b['position']
        result.append({
            'name': b.get('name', 'Storage'),
            'north': north,
            'east': east,
        })
    return result


def _parse_buildings(sdf_path):
    """Return list of (x, y, size_x, size_y, height) in NED frame.

    The SDF uses Gazebo ENU (x=East, y=North).  We convert to PX4 NED
    (x=North, y=East) so that building positions match drone positions.
    """
    try:
        with open(sdf_path) as f:
            content = f.read()
    except FileNotFoundError:
        return []
    buildings = []
    for m in re.finditer(
            r'<model name="b\d+">.*?<pose>([-\d.]+) ([-\d.]+) ([-\d.]+)'
            r'.*?<size>(\d+) (\d+) (\d+)</size>', content, re.DOTALL):
        enu_x, enu_y = float(m.group(1)), float(m.group(2))
        enu_w, enu_d, h = int(m.group(4)), int(m.group(5)), int(m.group(6))
        # ENU→NED: swap position (x,y) and size (w,d)
        buildings.append((enu_y, enu_x, enu_d, enu_w, h))
    return buildings


def _load_no_fly_zones(json_path):
    """Load no-fly zones from JSON. Returns list of zone dicts."""
    try:
        with open(json_path) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []
    return data.get('zones', [])


# ── Visibility Graph A* Path Planner ──────────────────────────────────────────
# Routes through polygon-corner vertices of expanded obstacles for shortest
# collision-free paths that naturally hug the 5m safety boundary.

class _Polygon:
    """A convex polygon obstacle with AABB cache."""
    __slots__ = ('verts', 'xmin', 'xmax', 'ymin', 'ymax')
    def __init__(self, verts):
        self.verts = verts
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        self.xmin, self.xmax = min(xs), max(xs)
        self.ymin, self.ymax = min(ys), max(ys)


def _build_obstacles(buildings, zones, storage=None, margin=SAFETY_RADIUS):
    """Build obstacle polygons expanded by safety margin.

    Returns (polygons, building_rects):
      polygons       — list of _Polygon (expanded obstacle boundaries)
      building_rects  — list of (cx, cy, hw, hh) original building rectangles
    """
    polygons = []

    # Buildings → axis-aligned rectangles expanded by margin.
    # Used only as bounding polygons for fast pre-filtering.
    for bx, by, bw, bd, _bh in buildings:
        hw = bw / 2 + margin
        hh = bd / 2 + margin
        verts = [
            (bx - hw, by - hh),
            (bx + hw, by - hh),
            (bx + hw, by + hh),
            (bx - hw, by + hh),
        ]
        polygons.append(_Polygon(verts))

    n_building_polys = len(polygons)

    # No-fly zones
    for z in zones:
        if z['type'] == 'rectangle':
            c_list = z['corners']  # [East, North]
            es = [c[0] for c in c_list]
            ns = [c[1] for c in c_list]
            cx_ned = (min(ns) + max(ns)) / 2
            cy_ned = (min(es) + max(es)) / 2
            hw = (max(ns) - min(ns)) / 2 + margin
            hh = (max(es) - min(es)) / 2 + margin
            verts = [
                (cx_ned - hw, cy_ned - hh),
                (cx_ned + hw, cy_ned - hh),
                (cx_ned + hw, cy_ned + hh),
                (cx_ned - hw, cy_ned + hh),
            ]
            polygons.append(_Polygon(verts))
        elif z['type'] == 'circle':
            east, north = z['center']
            r = z['diameter'] / 2 + margin
            n_sides = 16
            verts = []
            for k in range(n_sides):
                angle = 2 * math.pi * k / n_sides
                verts.append((north + r * math.cos(angle),
                              east + r * math.sin(angle)))
            polygons.append(_Polygon(verts))

    building_rects = [(bx, by, bw / 2, bd / 2) for bx, by, bw, bd, _bh in buildings]
    return polygons, building_rects


def _point_in_polygon(px, py, poly):
    """Ray-casting point-in-polygon test."""
    verts = poly.verts
    n = len(verts)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _seg_intersects_polygon(ax, ay, bx, by, poly):
    """Test if segment AB intersects polygon (AABB early-out)."""
    seg_xmin, seg_xmax = (ax, bx) if ax < bx else (bx, ax)
    seg_ymin, seg_ymax = (ay, by) if ay < by else (by, ay)
    if seg_xmax < poly.xmin or seg_xmin > poly.xmax:
        return False
    if seg_ymax < poly.ymin or seg_ymin > poly.ymax:
        return False
    if _point_in_polygon(ax, ay, poly) or _point_in_polygon(bx, by, poly):
        return True
    verts = poly.verts
    n = len(verts)
    for i in range(n):
        j = (i + 1) % n
        d1 = (verts[j][0] - verts[i][0]) * (ay - verts[i][1]) - \
             (verts[j][1] - verts[i][1]) * (ax - verts[i][0])
        d2 = (verts[j][0] - verts[i][0]) * (by - verts[i][1]) - \
             (verts[j][1] - verts[i][1]) * (bx - verts[i][0])
        d3 = (bx - ax) * (verts[i][1] - ay) - (by - ay) * (verts[i][0] - ax)
        d4 = (bx - ax) * (verts[j][1] - ay) - (by - ay) * (verts[j][0] - ax)
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True
    return False


def _segment_blocked_skip(ax, ay, bx, by, polygons, skip_set):
    """Check if segment is blocked by any polygon not in skip_set."""
    for i, poly in enumerate(polygons):
        if i in skip_set:
            continue
        if _seg_intersects_polygon(ax, ay, bx, by, poly):
            return True
    return False


def _segment_blocked(ax, ay, bx, by, polygons):
    """Check if segment is blocked by any polygon."""
    for poly in polygons:
        if _seg_intersects_polygon(ax, ay, bx, by, poly):
            return True
    return False


def _edges_crossed(ax, ay, bx, by, poly):
    """Check if segment AB properly crosses any edge of polygon."""
    verts = poly.verts
    n = len(verts)
    for i in range(n):
        j = (i + 1) % n
        ex, ey = verts[i]
        fx, fy = verts[j]
        d1 = (fx - ex) * (ay - ey) - (fy - ey) * (ax - ex)
        d2 = (fx - ex) * (by - ey) - (fy - ey) * (bx - ex)
        d3 = (bx - ax) * (ey - ay) - (by - ay) * (ex - ax)
        d4 = (bx - ax) * (fy - ay) - (by - ay) * (fx - ax)
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
           ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True
    return False


def _is_visible(ax, ay, bx, by, polygons):
    """Robust visibility check for the visibility graph.

    Returns True if segment AB does not pass through the interior of any polygon.
    Uses two tests:
      1. Segment does not properly cross any polygon edge
      2. Midpoint of segment is not inside any polygon
    This correctly handles endpoints on polygon boundaries (vertices).
    """
    mx = (ax + bx) / 2
    my = (ay + by) / 2

    for poly in polygons:
        # AABB early-out
        seg_xmin, seg_xmax = (ax, bx) if ax < bx else (bx, ax)
        seg_ymin, seg_ymax = (ay, by) if ay < by else (by, ay)
        if seg_xmax < poly.xmin or seg_xmin > poly.xmax:
            continue
        if seg_ymax < poly.ymin or seg_ymin > poly.ymax:
            continue
        # Test 1: does segment cross any polygon edge?
        if _edges_crossed(ax, ay, bx, by, poly):
            return False
        # Test 2: is midpoint inside polygon?
        if _point_in_polygon(mx, my, poly):
            return False
    return True


def _segment_clearance(ax, ay, bx, by, rects):
    """Minimum distance from any point on segment AB to any building rectangle."""
    # AABB of segment expanded by SAFETY_RADIUS
    seg_xmin = min(ax, bx) - SAFETY_RADIUS
    seg_xmax = max(ax, bx) + SAFETY_RADIUS
    seg_ymin = min(ay, by) - SAFETY_RADIUS
    seg_ymax = max(ay, by) + SAFETY_RADIUS

    # Pre-filter: only check buildings near the segment
    nearby = []
    for cx, cy, hw, hh in rects:
        if cx + hw < seg_xmin or cx - hw > seg_xmax:
            continue
        if cy + hh < seg_ymin or cy - hh > seg_ymax:
            continue
        nearby.append((cx, cy, hw, hh))

    if not nearby:
        return float('inf')

    length = math.hypot(bx - ax, by - ay)
    N = max(int(length / 1.0), 8)
    min_d = float('inf')
    inv_N = 1.0 / N
    dx_s = bx - ax
    dy_s = by - ay
    for k in range(N + 1):
        t = k * inv_N
        px = ax + t * dx_s
        py = ay + t * dy_s
        for cx, cy, hw, hh in nearby:
            dx = abs(px - cx) - hw
            dy = abs(py - cy) - hh
            if dx < 0:
                dx = 0.0
            if dy < 0:
                dy = 0.0
            d = dx * dx + dy * dy  # squared distance
            if d < min_d:
                min_d = d
                if d < 1.0:
                    return math.sqrt(d)
    return math.sqrt(min_d)


def _is_safe_segment(ax, ay, bx, by, building_rects, zone_polys, min_dist,
                     bldg_polys=None):
    """Check if segment AB is safe: >=min_dist from buildings, outside no-fly zones.

    Uses polygon check as fast pre-filter. Only runs expensive distance
    sampling when the segment enters a building's expanded polygon.
    """
    # Fast check: if segment is outside ALL building expanded polygons,
    # it's at least SAFETY_RADIUS from all buildings — skip distance check.
    if bldg_polys is not None:
        need_dist_check = not _is_visible(ax, ay, bx, by, bldg_polys)
    else:
        need_dist_check = True

    if need_dist_check:
        if _segment_clearance(ax, ay, bx, by, building_rects) < min_dist:
            return False

    # Check no-fly zones
    if zone_polys and not _is_visible(ax, ay, bx, by, zone_polys):
        return False
    return True


def _plan_path(sx, sy, gx, gy, obstacles):
    """Plan shortest path using visibility-graph A*.

    Uses actual building distance for safety (not expanded polygon containment).
    This allows paths to cut building corners tightly while maintaining the
    SAFETY_RADIUS distance from all buildings.

    obstacles = (polygons, building_rects) from _build_obstacles.
    Returns list of (x, y) waypoints (excluding start).
    """
    polygons, building_rects = obstacles
    n_bldg = len(building_rects)
    bldg_polys = polygons[:n_bldg]
    zone_polys = polygons[n_bldg:]

    # Direct line of sight → straight path
    if _is_safe_segment(sx, sy, gx, gy, building_rects, zone_polys, SAFETY_RADIUS,
                        bldg_polys):
        return [(gx, gy)]

    # Try with expanding corridor to limit vertex count
    for expand in (60.0, 150.0, 400.0):
        result = _visgraph_astar(sx, sy, gx, gy, polygons,
                                 building_rects, zone_polys, expand)
        if result:
            return result

    return [(gx, gy)]  # fallback


def _visgraph_astar(sx, sy, gx, gy, polygons, building_rects, zone_polys, expand):
    """Visibility-graph A* within a corridor of ±expand metres.

    Uses actual building distance for safety checks (not polygon containment).
    This allows paths to cut building corners tightly at exactly SAFETY_RADIUS.
    """
    xmin = min(sx, gx) - expand
    xmax = max(sx, gx) + expand
    ymin = min(sy, gy) - expand
    ymax = max(sy, gy) + expand

    n_bldg = len(building_rects)

    # Filter relevant building polygons in corridor (for vertex generation)
    rel_bldg_idx = []
    for i in range(n_bldg):
        p = polygons[i]
        if p.xmax >= xmin and p.xmin <= xmax and p.ymax >= ymin and p.ymin <= ymax:
            rel_bldg_idx.append(i)

    # Filter relevant zone polygons in corridor
    rel_zone_polys = [p for p in zone_polys
                      if p.xmax >= xmin and p.xmin <= xmax and
                         p.ymax >= ymin and p.ymin <= ymax]

    # Filter relevant building rects in corridor
    rel_brects = []
    for i in rel_bldg_idx:
        rel_brects.append(building_rects[i])
    if not rel_brects:
        rel_brects = building_rects  # safety fallback

    # Direct path check
    rel_bldg_polys = [polygons[i] for i in rel_bldg_idx]
    if _is_safe_segment(sx, sy, gx, gy, rel_brects, rel_zone_polys, SAFETY_RADIUS,
                        rel_bldg_polys):
        return [(gx, gy)]

    # Generate candidate waypoints for routing around buildings.
    # Two types:
    #   1. Corner bypass points: at the 4 expanded rectangle corners
    #   2. Edge bypass points: along building sides at SAFETY_RADIUS distance,
    #      spaced every ~15m. These let paths go alongside a building's edge
    #      rather than detouring to a far corner.
    # Plus a sparse open-space grid to find direct routes through gaps.
    _R = SAFETY_RADIUS
    all_verts = set()
    for i in rel_bldg_idx:
        cx, cy, hw, hh = building_rects[i]
        ehw = hw + _R  # expanded half-width
        ehh = hh + _R  # expanded half-height
        # 4 corner vertices (expanded rectangle corners)
        all_verts.add((cx - ehw, cy - ehh))
        all_verts.add((cx + ehw, cy - ehh))
        all_verts.add((cx + ehw, cy + ehh))
        all_verts.add((cx - ehw, cy + ehh))
        # Edge waypoints along each side at 5m distance
        spacing = 15.0
        # East and West edges
        n_ey = max(1, int(2 * hh / spacing))
        for k in range(1, n_ey):
            ey = cy - hh + k * (2 * hh) / n_ey
            all_verts.add((cx + ehw, ey))
            all_verts.add((cx - ehw, ey))
        # North and South edges
        n_ex = max(1, int(2 * hw / spacing))
        for k in range(1, n_ex):
            ex = cx - hw + k * (2 * hw) / n_ex
            all_verts.add((ex, cy + ehh))
            all_verts.add((ex, cy - ehh))

    # Generate routing vertices around no-fly zones (same approach as buildings)
    for zp in rel_zone_polys:
        # Use polygon bounding box as a rectangle approximation
        zcx = (zp.xmin + zp.xmax) / 2
        zcy = (zp.ymin + zp.ymax) / 2
        zhw = (zp.xmax - zp.xmin) / 2
        zhh = (zp.ymax - zp.ymin) / 2
        # Corner vertices (already expanded by SAFETY_RADIUS in _build_obstacles)
        all_verts.add((zp.xmin, zp.ymin))
        all_verts.add((zp.xmax, zp.ymin))
        all_verts.add((zp.xmax, zp.ymax))
        all_verts.add((zp.xmin, zp.ymax))
        # Edge waypoints along zone sides
        spacing_z = 15.0
        n_zy = max(1, int((zp.ymax - zp.ymin) / spacing_z))
        for k in range(1, n_zy):
            ey = zp.ymin + k * (zp.ymax - zp.ymin) / n_zy
            all_verts.add((zp.xmax, ey))
            all_verts.add((zp.xmin, ey))
        n_zx = max(1, int((zp.xmax - zp.xmin) / spacing_z))
        for k in range(1, n_zx):
            ex = zp.xmin + k * (zp.xmax - zp.xmin) / n_zx
            all_verts.add((ex, zp.ymax))
            all_verts.add((ex, zp.ymin))

    # Open-space grid: 2D grid of candidate waypoints in the corridor.
    # Enables finding direct routes through building gaps rather than
    # routing around building corners.
    grid_spacing = 35.0
    gx_start = math.floor(xmin / grid_spacing) * grid_spacing
    gy_start = math.floor(ymin / grid_spacing) * grid_spacing
    gx_v = gx_start
    while gx_v <= xmax:
        gy_v = gy_start
        while gy_v <= ymax:
            all_verts.add((gx_v, gy_v))
            gy_v += grid_spacing
        gx_v += grid_spacing

    # Filter to corridor bounds
    all_verts = {v for v in all_verts
                 if xmin <= v[0] <= xmax and ymin <= v[1] <= ymax}

    # Remove vertices that are too close to any building or inside no-fly zones
    valid_verts = []
    for v in all_verts:
        inside = False
        # Check if inside a no-fly zone polygon
        for zp in rel_zone_polys:
            if _point_in_polygon(v[0], v[1], zp):
                inside = True
                break
        if inside:
            continue
        # Check if too close to any building (< SAFETY_RADIUS)
        for cx, cy, hw, hh in rel_brects:
            dx = abs(v[0] - cx) - hw
            dy = abs(v[1] - cy) - hh
            if dx < 0:
                dx = 0.0
            if dy < 0:
                dy = 0.0
            d = math.hypot(dx, dy)
            if d < SAFETY_RADIUS - 0.1:
                inside = True
                break
        if not inside:
            valid_verts.append(v)

    # Build node list: [0]=start, [1]=goal, [2..]=vertices
    nodes = [(sx, sy), (gx, gy)] + valid_verts
    n = len(nodes)

    # Pre-compute clearance for each node (distance to nearest building)
    node_clearance = [0.0] * n
    for i in range(n):
        nx, ny = nodes[i]
        min_d = float('inf')
        for cx, cy, hw, hh in rel_brects:
            dx = abs(nx - cx) - hw
            dy = abs(ny - cy) - hh
            if dx < 0:
                dx = 0.0
            if dy < 0:
                dy = 0.0
            d = math.hypot(dx, dy)
            if d < min_d:
                min_d = d
        node_clearance[i] = min_d

    # A* with lazy evaluation using actual building distance
    dist = [float('inf')] * n
    dist[0] = 0.0
    prev = [-1] * n
    heap = [(math.hypot(sx - gx, sy - gy), 0, 0)]
    counter = 1
    visited = [False] * n

    while heap:
        f, _, u = heappop(heap)
        if visited[u]:
            continue
        visited[u] = True
        if u == 1:
            break
        ux, uy = nodes[u]
        # Limit neighbor search radius based on remaining distance to goal
        max_edge = math.hypot(ux - gx, uy - gy) + 50.0
        max_edge_sq = max_edge * max_edge
        for v in range(n):
            if visited[v]:
                continue
            if dist[v] <= dist[u]:
                continue
            vx, vy = nodes[v]
            dx_nv = ux - vx
            dy_nv = uy - vy
            dsq = dx_nv * dx_nv + dy_nv * dy_nv
            if dsq > max_edge_sq:
                continue
            if _is_safe_segment(ux, uy, vx, vy,
                                rel_brects, rel_zone_polys, SAFETY_RADIUS,
                                rel_bldg_polys):
                w = math.sqrt(dsq)
                # Proximity penalty: prefer paths through open space.
                min_clr = min(node_clearance[u], node_clearance[v])
                if min_clr < 12.0:
                    penalty = 0.10 * (1.0 - (min_clr - SAFETY_RADIUS) / 7.0)
                    if penalty > 0:
                        w *= (1.0 + penalty)
                nd = dist[u] + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    h = math.hypot(vx - gx, vy - gy)
                    heappush(heap, (nd + h, counter, v))
                    counter += 1

    if dist[1] == float('inf'):
        return []

    # Reconstruct path
    path_idx = []
    cur = 1
    while cur != -1:
        path_idx.append(cur)
        cur = prev[cur]
    path_idx.reverse()

    return [nodes[i] for i in path_idx[1:]]

# ── Delivery Order ───────────────────────────────────────────────────────────

class DeliveryOrder:
    """One delivery: pick up at a storage building, deliver to a target point."""
    _counter = 0

    def __init__(self, storage_name, pickup_n, pickup_e,
                 target_n, target_e):
        DeliveryOrder._counter += 1
        self.id = DeliveryOrder._counter
        self.storage_name = storage_name
        self.pickup_n = pickup_n   # NED north
        self.pickup_e = pickup_e   # NED east
        self.target_n = target_n
        self.target_e = target_e
        self.drone_id = None       # assigned drone
        self.status = 'pending'    # pending | pickup | delivering | done


class DroneState:
    """Holds the live state for one drone."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading_deg = 0.0
        self.armed = False
        self.nav_state = 0
        self.last_msg_time = 0.0
        # Control state machine
        self.phase = 'idle'  # idle | warmup | arming | flying | hovering | descending | landing
        self.counter = 0
        # Target position (NED: z negative = up)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -5.0
        # Waypoint path (list of (x, y) tuples); drone follows first one
        self.waypoints: list[tuple[float, float]] = []
        self.final_target_x = 0.0
        self.final_target_y = 0.0
        # Battery
        self.battery_pct = 100.0  # 0-100 %
        self.battery_warning = 0
        self.heading_to_brs = False  # flying to a BRS/storage for landing
        self.landing_alt = 0.0      # ground level for BRS, -20.0 for storage rooftop (NED)
        self.heading_to_storage = False  # flying to storage for pickup
        self.carrying_package = False    # currently carrying a package
        self.active_order_id = None      # id of current delivery order

    @property
    def connected(self):
        return (time.monotonic() - self.last_msg_time) < CONNECTION_TIMEOUT


class DashboardNode(Node):
    """ROS 2 node that manages subscriptions and publishers for all drones."""

    def __init__(self):
        super().__init__('drone_dashboard')

        # Publishers use VOLATILE (PX4 input topics expect it)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Subscribers use TRANSIENT_LOCAL (PX4 output topics publish with it)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.drones: dict[int, DroneState] = {}
        self._offboard_pubs = {}
        self._setpoint_pubs = {}
        self._command_pubs = {}

        for drone_id in DRONE_IDS:
            prefix = f'/px4_{drone_id}/fmu'
            ds = DroneState()
            self.drones[drone_id] = ds

            # Publishers
            self._offboard_pubs[drone_id] = self.create_publisher(
                OffboardControlMode, f'{prefix}/in/offboard_control_mode', pub_qos)
            self._setpoint_pubs[drone_id] = self.create_publisher(
                TrajectorySetpoint, f'{prefix}/in/trajectory_setpoint', pub_qos)
            self._command_pubs[drone_id] = self.create_publisher(
                VehicleCommand, f'{prefix}/in/vehicle_command', pub_qos)

            # Subscribers — try both versioned and unversioned topic names
            for suffix in ['', '_v1']:
                self.create_subscription(
                    VehicleLocalPosition, f'{prefix}/out/vehicle_local_position{suffix}',
                    lambda msg, did=drone_id: self._on_local_pos(did, msg), sub_qos)
            for suffix in ['', '_v4', '_v3', '_v2', '_v1']:
                self.create_subscription(
                    VehicleStatus, f'{prefix}/out/vehicle_status{suffix}',
                    lambda msg, did=drone_id: self._on_status(did, msg), sub_qos)

        # 10 Hz control timer
        self.create_timer(0.1, self._control_loop)

        # Load obstacles once
        buildings = _parse_buildings(BUILDINGS_SDF)
        self._no_fly_zones = _load_no_fly_zones(NO_FLY_ZONES_FILE)
        self._brs = _load_brs(BRS_FILE)
        self._storage = _load_storage(STORAGE_FILE)
        self._obstacles = _build_obstacles(buildings, self._no_fly_zones, self._storage)

        # Delivery order queue
        self.orders: list[DeliveryOrder] = []
        # Check for pending orders every 2 minutes (120 s)
        self.create_timer(120.0, self.dispatch_pending_orders)

    @property
    def brs(self):
        return self._brs

    @property
    def storage(self):
        return self._storage

    @property
    def no_fly_zones(self):
        return self._no_fly_zones

    @property
    def buildings(self):
        return _parse_buildings(BUILDINGS_SDF)

    def _on_local_pos(self, drone_id, msg):
        ds = self.drones[drone_id]
        # Clear connection-lost flag on reconnect
        if getattr(ds, '_conn_lost_logged', False):
            self.get_logger().info(f'Drone {drone_id} connection restored')
            ds._conn_lost_logged = False
        # Convert PX4 local NED (origin = spawn point) → world NED
        spawn_n, spawn_e = DRONE_SPAWN_NED[drone_id]
        ds.x = msg.x + spawn_n
        ds.y = msg.y + spawn_e
        ds.z = msg.z
        ds.heading_deg = math.degrees(msg.heading)
        ds.last_msg_time = time.monotonic()

    def _on_status(self, drone_id, msg):
        ds = self.drones[drone_id]
        was_armed = ds.armed
        ds.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        ds.nav_state = msg.nav_state
        ds.last_msg_time = time.monotonic()

        # Detect landing complete: disarmed while in landing phase
        if not ds.armed and ds.phase == 'landing':
            # Check if landed near a BRS — auto-refuel
            if ds.heading_to_brs:
                for s in self._brs:
                    d = math.hypot(ds.x - s['north'], ds.y - s['east'])
                    if d < 8.0:  # within 8 m of station
                        ds.battery_pct = 100.0
                        self.get_logger().info(
                            f'Drone {drone_id} refuelled at {s["name"]}')
                        break
                ds.heading_to_brs = False
            if ds.heading_to_storage:
                ds.heading_to_storage = False
                ds.carrying_package = True
                ds.battery_pct = 100.0
                self.get_logger().info(
                    f'Drone {drone_id} picked up package + refuelled at storage')
                # Phase 2 of delivery: auto-takeoff and fly to delivery target
                order = self._find_order(ds.active_order_id)
                if order:
                    order.status = 'delivering'
                    self._auto_deliver(drone_id, order)
                    return  # don't set idle
            # Check if just completed a delivery landing (has order, carrying package, not heading to storage)
            elif ds.active_order_id and ds.carrying_package:
                order = self._find_order(ds.active_order_id)
                if order:
                    order.status = 'done'
                    self.get_logger().info(
                        f'Drone {drone_id} delivered order #{order.id}')
                ds.carrying_package = False
                ds.active_order_id = None
                ds.landing_alt = 0.0
                # Immediately pick up next pending order (FIFO)
                next_order = next(
                    (o for o in self.orders if o.status == 'pending'), None)
                if next_order:
                    ds.phase = 'idle'
                    next_order.drone_id = drone_id
                    next_order.status = 'pickup'
                    ds.active_order_id = next_order.id
                    self._send_to_storage(drone_id, next_order)
                    self.get_logger().info(
                        f'Order #{next_order.id} assigned to Drone {drone_id}')
                    return
                else:
                    ds.phase = 'idle'
                    self._auto_takeoff_to_brs(drone_id)
                    return
            ds.phase = 'idle'

    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, drone_id, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.target_system = drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._ts()
        self._command_pubs[drone_id].publish(msg)

    def _send_arm_force(self, drone_id):
        """Send force-arm as internal command to bypass preflight checks in SITL."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 21196.0
        msg.target_system = drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = False
        msg.timestamp = self._ts()
        self._command_pubs[drone_id].publish(msg)

    def _send_offboard_mode(self, drone_id):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self._ts()
        self._offboard_pubs[drone_id].publish(msg)

    def _send_setpoint(self, drone_id, x, y, z):
        # x, y are in world NED — convert to PX4 local NED (origin = spawn)
        spawn_n, spawn_e = DRONE_SPAWN_NED[drone_id]
        msg = TrajectorySetpoint()
        msg.position = [x - spawn_n, y - spawn_e, z]
        msg.yaw = 1.57
        msg.timestamp = self._ts()
        self._setpoint_pubs[drone_id].publish(msg)

    def _control_loop(self):
        """Runs the offboard warmup / flight state machine for each drone.
        When flying with waypoints, advances to next waypoint when close.
        Also simulates battery drain."""
        for drone_id, ds in self.drones.items():
            # ── Simulated battery drain (called at 10 Hz) ──
            if ds.phase == 'flying':
                ds.battery_pct = max(0.0, ds.battery_pct - 0.05)   # ~50 s per 1%
            # Battery drain for descending/hovering
            elif ds.phase in ('warmup', 'arming', 'landing', 'hovering', 'descending'):
                ds.battery_pct = max(0.0, ds.battery_pct - 0.02)   # slower drain

            # ── Connection loss → hold position ──
            # Only trigger if we had a connection before (last_msg_time > 0)
            if ds.last_msg_time > 0 and not ds.connected and \
                    ds.phase in ('flying', 'hovering', 'descending'):
                # PX4 will auto-hold via COM_OBL_RC_ACT=5 (Hold mode)
                # Just freeze dashboard state so it resumes on reconnect
                if not getattr(ds, '_conn_lost_logged', False):
                    self.get_logger().warn(
                        f'Drone {drone_id} connection lost — PX4 holding position')
                    ds._conn_lost_logged = True
                continue

            if ds.phase == 'warmup':
                self._send_offboard_mode(drone_id)
                self._send_setpoint(drone_id, ds.target_x, ds.target_y, ds.target_z)
                ds.counter += 1
                if ds.counter >= 10:  # 1 s warmup done
                    self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                      param1=1.0, param2=6.0)
                    self._send_arm_force(drone_id)
                    ds.phase = 'arming'
            elif ds.phase == 'arming':
                # Keep sending offboard + setpoint and re-try arm until armed
                self._send_offboard_mode(drone_id)
                self._send_setpoint(drone_id, ds.target_x, ds.target_y, ds.target_z)
                ds.counter += 1
                if ds.armed:
                    ds.phase = 'flying'
                elif ds.counter % 20 == 0:  # re-send every 2 s
                    self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                      param1=1.0, param2=6.0)
                    self._send_arm_force(drone_id)
            elif ds.phase == 'flying':
                # Low battery auto-land
                if ds.battery_pct < 5.0:
                    self.get_logger().warn(
                        f'Drone {drone_id} battery critical ({ds.battery_pct:.1f}%%) — auto-landing')
                    self.request_land(drone_id)
                    continue
                # Advance waypoints when close enough
                if ds.waypoints:
                    wx, wy = ds.waypoints[0]
                    dist = math.hypot(ds.x - wx, ds.y - wy)
                    # Use tighter radius for the last waypoint when heading to BRS
                    accept_r = 1.0 if (ds.heading_to_brs and len(ds.waypoints) == 1) else 3.0
                    if dist < accept_r:
                        ds.waypoints.pop(0)
                        if ds.waypoints:
                            ds.target_x, ds.target_y = ds.waypoints[0]
                        elif ds.heading_to_brs:
                            # Arrived near BRS — lock setpoint to exact BRS center
                            # Find the actual BRS station we're heading to
                            brs_x, brs_y = ds.final_target_x, ds.final_target_y
                            self.get_logger().info(
                                f'Drone {drone_id} near BRS — '
                                f'drone=({ds.x:.1f},{ds.y:.1f}) '
                                f'BRS=({brs_x:.1f},{brs_y:.1f}) '
                                f'err={math.hypot(ds.x-brs_x, ds.y-brs_y):.1f}m')
                            ds.target_x = brs_x
                            ds.target_y = brs_y
                            ds.target_z = ds.landing_alt - 2.0  # hold 2 m above landing surface
                            ds.phase = 'hovering'
                            ds.counter = 0
                            continue
                self._send_offboard_mode(drone_id)
                self._send_setpoint(drone_id, ds.target_x, ds.target_y, ds.target_z)
            elif ds.phase == 'hovering':
                # Keep setpoint locked to BRS center, wait for drone to settle
                # Continuously correct target to exact BRS position
                brs_x = ds.final_target_x
                brs_y = ds.final_target_y
                ds.target_x = brs_x
                ds.target_y = brs_y
                self._send_offboard_mode(drone_id)
                self._send_setpoint(drone_id, ds.target_x, ds.target_y, ds.target_z)
                ds.counter += 1
                hdist = math.hypot(ds.x - brs_x, ds.y - brs_y)
                # Need to be within 0.5 m AND hold for at least 2 s
                if ds.counter >= 20 and hdist < 0.5:
                    self.get_logger().info(
                        f'Drone {drone_id} locked above target — '
                        f'err={hdist:.2f}m — descending')
                    ds.phase = 'descending'
                    ds.target_z = ds.landing_alt - 1.0  # 1 m above landing surface
                    ds.counter = 0
                elif ds.counter >= 40:
                    # Timeout: accept even if not perfect after 4 s
                    self.get_logger().warn(
                        f'Drone {drone_id} hover timeout — err={hdist:.2f}m')
                    ds.phase = 'descending'
                    ds.target_z = ds.landing_alt - 1.0
                    ds.counter = 0
            elif ds.phase == 'descending':
                # Offboard-controlled descent — keep XY locked to target center
                if ds.heading_to_brs:
                    ds.target_x = ds.final_target_x
                    ds.target_y = ds.final_target_y
                self._send_offboard_mode(drone_id)
                self._send_setpoint(drone_id, ds.target_x, ds.target_y, ds.target_z)
                ds.counter += 1
                hdist = math.hypot(ds.x - ds.target_x, ds.y - ds.target_y)
                land_z = ds.landing_alt - 1.0  # expected hold-z (1 m above surface)
                # Stage 1: hold at 1 m above surface until settled
                if abs(ds.target_z - land_z) < 0.2:
                    if ds.z < (land_z - 0.5) or ds.counter < 15:
                        pass  # still descending or waiting
                    elif hdist < 0.5:
                        self.get_logger().info(
                            f'Drone {drone_id} settled — err={hdist:.2f}m — final landing')
                        self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        ds.phase = 'landing'
                    elif ds.counter >= 40:  # 4s timeout
                        self.get_logger().warn(
                            f'Drone {drone_id} timeout — err={hdist:.2f}m — landing anyway')
                        self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        ds.phase = 'landing'
                else:
                    # Manual land (not BRS/storage) — land immediately near ground
                    if ds.z > -0.5:
                        self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        ds.phase = 'landing'


    def request_takeoff(self, drone_id):
        ds = self.drones[drone_id]
        if ds.battery_pct < 5.0:
            self.get_logger().warn(
                f'Drone {drone_id} battery too low ({ds.battery_pct:.1f}%%) — refuel first')
            return
        if ds.phase in ('idle', 'landing'):
            ds.target_x = ds.x
            ds.target_y = ds.y
            ds.target_z = -95.0
            ds.waypoints = []
            ds.final_target_x = ds.x
            ds.final_target_y = ds.y
            ds.phase = 'warmup'
            ds.counter = 0

    def request_land(self, drone_id):
        ds = self.drones[drone_id]
        if ds.phase in ('warmup', 'flying', 'arming', 'hovering', 'descending'):
            ds.waypoints = []
            ds.heading_to_brs = False
            # Hold current XY, descend via offboard
            ds.target_x = ds.x
            ds.target_y = ds.y
            ds.target_z = -0.15
            ds.phase = 'descending'
            ds.counter = 0

    def refuel(self, drone_id):
        """Reset simulated battery to 100 %."""
        ds = self.drones[drone_id]
        ds.battery_pct = 100.0
        ds.battery_warning = 0
        self.get_logger().info(f'Drone {drone_id} refuelled to 100 %%')

    def request_random_location(self, drone_id):
        ds = self.drones[drone_id]
        if ds.phase != 'flying':
            return
        # Find a clear random target
        for _ in range(200):
            rx = random.uniform(-200, 200)
            ry = random.uniform(-200, 200)
            clear = True
            polys, _corners = self._obstacles
            for poly in polys:
                if _point_in_polygon(rx, ry, poly):
                    clear = False
                    break
            if clear:
                break
        alt = random.uniform(-95, -100)  # NED: 95-100 m above ground (above 90 m buildings)
        # Plan path around obstacles
        waypoints = _plan_path(ds.x, ds.y, rx, ry, self._obstacles)
        if not waypoints:
            self.get_logger().warn(
                f'Drone {drone_id}: no safe path to ({rx:.0f}, {ry:.0f}) — skipping')
            return
        ds.waypoints = list(waypoints)
        ds.final_target_x = rx
        ds.final_target_y = ry
        ds.target_z = alt
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        self.get_logger().info(
            f'Drone {drone_id} -> ({rx:.0f}, {ry:.0f}, {-alt:.0f}m) '
            f'via {len(ds.waypoints)} waypoints')

    def request_nearest_brs(self, drone_id):
        """Fly to the nearest Battery Replacement Station and land there."""
        ds = self.drones[drone_id]
        if ds.phase != 'flying':
            return
        if not self._brs:
            self.get_logger().warn('No battery stations defined')
            return
        # Find nearest BRS
        best = None
        best_dist = float('inf')
        for s in self._brs:
            d = math.hypot(ds.x - s['north'], ds.y - s['east'])
            if d < best_dist:
                best_dist = d
                best = s
        bx, by = best['north'], best['east']
        # Plan path
        waypoints = _plan_path(ds.x, ds.y, bx, by, self._obstacles)
        # Replace final waypoint with exact BRS position
        if waypoints:
            waypoints[-1] = (bx, by)
        ds.waypoints = list(waypoints)
        ds.final_target_x = bx
        ds.final_target_y = by
        ds.target_z = -95.0
        ds.heading_to_brs = True
        ds.landing_alt = 0.0  # BRS is on the ground
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        self.get_logger().info(
            f'Drone {drone_id} -> BRS "{best["name"]}" ({bx:.0f}, {by:.0f}) '
            f'via {len(ds.waypoints)} waypoints')

    def request_random_storage(self, drone_id):
        """Fly to a random storage building and land on its roof to pick up a package."""
        ds = self.drones[drone_id]
        if ds.phase != 'flying':
            return
        if not self._storage:
            self.get_logger().warn('No storage buildings defined')
            return
        # Remove existing package if going for another pickup
        ds.carrying_package = False
        target = random.choice(self._storage)
        sx, sy = target['north'], target['east']
        # Plan path to the storage building
        waypoints = _plan_path(ds.x, ds.y, sx, sy, self._obstacles)
        if not waypoints:
            self.get_logger().warn(
                f'Drone {drone_id}: no safe path to {target["name"]} — skipping')
            return
        # Replace final waypoint with exact storage position
        waypoints[-1] = (sx, sy)
        ds.waypoints = list(waypoints)
        ds.final_target_x = sx
        ds.final_target_y = sy
        ds.target_z = -95.0
        ds.heading_to_brs = True
        ds.heading_to_storage = True
        ds.landing_alt = -20.0
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        self.get_logger().info(
            f'Drone {drone_id} -> {target["name"]} ({sx:.0f}, {sy:.0f}) '
            f'via {len(ds.waypoints)} waypoints')

    # ── Delivery system helpers ──────────────────────────────────────────────

    def _find_order(self, order_id):
        """Find an order by id."""
        if order_id is None:
            return None
        for o in self.orders:
            if o.id == order_id:
                return o
        return None

    def free_drones(self):
        """Return list of drone_ids that are idle and connected."""
        free = []
        for did, ds in self.drones.items():
            if ds.phase == 'idle' and ds.connected and ds.active_order_id is None:
                free.append(did)
        return free

    def add_random_order(self):
        """Create a random delivery order and dispatch it to the nearest free drone."""
        if not self._storage:
            self.get_logger().warn('No storage buildings defined')
            return None
        storage = random.choice(self._storage)
        # Generate random clear delivery target
        for _ in range(200):
            tx = random.uniform(-300, 300)
            ty = random.uniform(-300, 300)
            clear = True
            polys, _corners = self._obstacles
            for poly in polys:
                if _point_in_polygon(tx, ty, poly):
                    clear = False
                    break
            if clear:
                break
        order = DeliveryOrder(
            storage['name'], storage['north'], storage['east'],
            tx, ty)
        self.orders.append(order)
        self.get_logger().info(
            f'Order #{order.id}: pickup={storage["name"]} '
            f'({storage["north"]:.0f},{storage["east"]:.0f}) '
            f'-> deliver ({tx:.0f},{ty:.0f})')
        self._try_dispatch(order)
        return order

    def _try_dispatch(self, order):
        """Assign the nearest free drone to an order and send it to pickup."""
        free = self.free_drones()
        if not free:
            self.get_logger().info(
                f'Order #{order.id} queued — no free drones')
            return False
        # Find closest free drone to the storage building
        best_did = None
        best_dist = float('inf')
        for did in free:
            ds = self.drones[did]
            d = math.hypot(ds.x - order.pickup_n, ds.y - order.pickup_e)
            if d < best_dist:
                best_dist = d
                best_did = did
        if best_did is None:
            return False
        order.drone_id = best_did
        order.status = 'pickup'
        ds = self.drones[best_did]
        ds.active_order_id = order.id
        # Takeoff first, then fly to storage
        self._send_to_storage(best_did, order)
        self.get_logger().info(
            f'Order #{order.id} assigned to Drone {best_did}')
        return True

    def _send_to_storage(self, drone_id, order):
        """Takeoff and fly drone to the storage building for pickup."""
        ds = self.drones[drone_id]
        sx, sy = order.pickup_n, order.pickup_e
        # Plan path
        waypoints = _plan_path(ds.x, ds.y, sx, sy, self._obstacles)
        if not waypoints:
            self.get_logger().warn(
                f'Drone {drone_id}: no path to storage — order #{order.id} failed')
            order.status = 'done'
            ds.active_order_id = None
            return
        waypoints[-1] = (sx, sy)
        ds.waypoints = list(waypoints)
        ds.final_target_x = sx
        ds.final_target_y = sy
        ds.target_z = -95.0
        ds.heading_to_brs = True
        ds.heading_to_storage = True
        ds.landing_alt = -20.0
        ds.carrying_package = False
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        # Auto-takeoff if idle
        if ds.phase == 'idle':
            ds.phase = 'warmup'
            ds.counter = 0

    def _auto_deliver(self, drone_id, order):
        """After picking up at storage, takeoff and fly to delivery target."""
        ds = self.drones[drone_id]
        tx, ty = order.target_n, order.target_e
        waypoints = _plan_path(ds.x, ds.y, tx, ty, self._obstacles)
        if not waypoints:
            self.get_logger().warn(
                f'Drone {drone_id}: no path to delivery target — landing')
            order.status = 'done'
            ds.carrying_package = False
            ds.active_order_id = None
            ds.phase = 'idle'
            return
        ds.waypoints = list(waypoints)
        ds.final_target_x = tx
        ds.final_target_y = ty
        ds.target_z = -95.0
        ds.heading_to_brs = True
        ds.heading_to_storage = False
        ds.landing_alt = 0.0  # deliver to ground level
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        ds.phase = 'warmup'
        ds.counter = 0
        self.get_logger().info(
            f'Drone {drone_id} taking off to deliver order #{order.id} '
            f'-> ({tx:.0f}, {ty:.0f})')

    def _auto_takeoff_to_brs(self, drone_id):
        """After delivery, auto-takeoff and fly to nearest BRS."""
        ds = self.drones[drone_id]
        if not self._brs or ds.battery_pct < 5.0:
            return
        # Find nearest BRS
        best = None
        best_dist = float('inf')
        for s in self._brs:
            d = math.hypot(ds.x - s['north'], ds.y - s['east'])
            if d < best_dist:
                best_dist = d
                best = s
        bx, by = best['north'], best['east']
        waypoints = _plan_path(ds.x, ds.y, bx, by, self._obstacles)
        if waypoints:
            waypoints[-1] = (bx, by)
        ds.waypoints = list(waypoints)
        ds.final_target_x = bx
        ds.final_target_y = by
        ds.target_z = -95.0
        ds.heading_to_brs = True
        ds.heading_to_storage = False
        ds.landing_alt = 0.0
        if ds.waypoints:
            ds.target_x, ds.target_y = ds.waypoints[0]
        ds.phase = 'warmup'
        ds.counter = 0
        self.get_logger().info(
            f'Drone {drone_id} heading to BRS "{best["name"]}" after delivery')

    def dispatch_pending_orders(self):
        """Try to dispatch pending orders to free drones (called every 2 min, FIFO)."""
        free = self.free_drones()
        if not free:
            return
        for order in self.orders:  # FIFO: oldest first
            if order.status == 'pending':
                if self._try_dispatch(order):
                    free = self.free_drones()
                    if not free:
                        break

    def clear_completed_orders(self):
        """Remove all completed orders from the list."""
        self.orders = [o for o in self.orders if o.status != 'done']


# ── GUI ──────────────────────────────────────────────────────────────────────

class DashboardGUI:
    REFRESH_MS = 100  # GUI refresh rate

    def __init__(self, node: DashboardNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('PX4 Drone Dashboard')
        self.root.configure(bg='#1e1e1e')
        self.root.resizable(False, False)

        self.panels: dict[int, dict] = {}

        for col, drone_id in enumerate(DRONE_IDS):
            self._create_drone_panel(drone_id, col)

        self._refresh()

    def _create_drone_panel(self, drone_id, col):
        colors = {
            'bg': '#2d2d2d', 'fg': '#e0e0e0', 'accent': '#569cd6',
            'green': '#4ec9b0', 'red': '#f44747', 'dim': '#808080',
        }

        frame = tk.LabelFrame(
            self.root, text=f'  Drone {drone_id}  (x500_{drone_id})  ',
            font=('Consolas', 12, 'bold'), fg=colors['accent'], bg=colors['bg'],
            labelanchor='n', padx=12, pady=8, bd=2, relief='groove',
        )
        frame.grid(row=0, column=col, padx=8, pady=8, sticky='nsew')

        row = 0

        # Connection indicator
        conn_label = tk.Label(frame, text='● Disconnected', font=('Consolas', 10),
                              fg=colors['red'], bg=colors['bg'], anchor='w')
        conn_label.grid(row=row, column=0, columnspan=2, sticky='w', pady=(0, 6))
        row += 1

        # Position
        tk.Label(frame, text='Position', font=('Consolas', 10, 'bold'),
                 fg=colors['dim'], bg=colors['bg'], anchor='w').grid(
            row=row, column=0, columnspan=2, sticky='w')
        row += 1

        pos_labels = {}
        for axis in ('X', 'Y', 'Z', 'Hdg'):
            tk.Label(frame, text=f'{axis}:', font=('Consolas', 10),
                     fg=colors['dim'], bg=colors['bg'], width=5, anchor='e').grid(
                row=row, column=0, sticky='e')
            val = tk.Label(frame, text='—', font=('Consolas', 11),
                           fg=colors['fg'], bg=colors['bg'], width=10, anchor='w')
            val.grid(row=row, column=1, sticky='w')
            pos_labels[axis] = val
            row += 1

        row += 1  # spacer

        # Status
        tk.Label(frame, text='Status', font=('Consolas', 10, 'bold'),
                 fg=colors['dim'], bg=colors['bg'], anchor='w').grid(
            row=row, column=0, columnspan=2, sticky='w')
        row += 1

        armed_label = tk.Label(frame, text='Disarmed', font=('Consolas', 10),
                               fg=colors['dim'], bg=colors['bg'], anchor='w')
        armed_label.grid(row=row, column=0, columnspan=2, sticky='w')
        row += 1

        nav_label = tk.Label(frame, text='—', font=('Consolas', 10),
                             fg=colors['fg'], bg=colors['bg'], anchor='w')
        nav_label.grid(row=row, column=0, columnspan=2, sticky='w')
        row += 1

        phase_label = tk.Label(frame, text='Idle', font=('Consolas', 10),
                               fg=colors['dim'], bg=colors['bg'], anchor='w')
        phase_label.grid(row=row, column=0, columnspan=2, sticky='w')
        row += 1

        # Battery bar
        tk.Label(frame, text='Battery', font=('Consolas', 10, 'bold'),
                 fg=colors['dim'], bg=colors['bg'], anchor='w').grid(
            row=row, column=0, columnspan=2, sticky='w', pady=(4, 0))
        row += 1

        batt_frame = tk.Frame(frame, bg='#444444', height=18, width=160)
        batt_frame.grid(row=row, column=0, columnspan=2, sticky='w', pady=(1, 0))
        batt_frame.grid_propagate(False)

        batt_bar = tk.Frame(batt_frame, bg=colors['green'], height=18, width=160)
        batt_bar.place(x=0, y=0, relheight=1.0, width=160)

        batt_pct_label = tk.Label(batt_frame, text='100 %', font=('Consolas', 9, 'bold'),
                                   fg='#1e1e1e', bg=colors['green'])
        batt_pct_label.place(relx=0.5, rely=0.5, anchor='center')
        row += 1

        row += 1  # spacer

        # Takeoff / Land button
        btn = tk.Button(
            frame, text='Takeoff', font=('Consolas', 12, 'bold'),
            bg=colors['green'], fg='#1e1e1e', activebackground='#3da88a',
            width=14, height=2, relief='flat', cursor='hand2',
            command=lambda did=drone_id: self._on_button(did),
        )
        btn.grid(row=row, column=0, columnspan=2, pady=(8, 0))
        row += 1

        # Random Location button
        rand_btn = tk.Button(
            frame, text='Random Location', font=('Consolas', 11, 'bold'),
            bg='#3a3a3a', fg='#808080', activebackground='#505050',
            width=14, height=1, relief='flat', cursor='hand2',
            state='disabled',
            command=lambda did=drone_id: self._on_random(did),
        )
        rand_btn.grid(row=row, column=0, columnspan=2, pady=(4, 0))
        row += 1

        # Refuel button
        refuel_btn = tk.Button(
            frame, text='⛽ Refuel', font=('Consolas', 11, 'bold'),
            bg='#daa520', fg='#1e1e1e', activebackground='#c4961a',
            width=14, height=1, relief='flat', cursor='hand2',
            command=lambda did=drone_id: self._on_refuel(did),
        )
        refuel_btn.grid(row=row, column=0, columnspan=2, pady=(4, 0))
        row += 1

        # Nearest BRS button
        brs_btn = tk.Button(
            frame, text='🔋 Nearest BRS', font=('Consolas', 11, 'bold'),
            bg='#3a3a3a', fg='#808080', activebackground='#505050',
            width=14, height=1, relief='flat', cursor='hand2',
            state='disabled',
            command=lambda did=drone_id: self._on_brs(did),
        )
        brs_btn.grid(row=row, column=0, columnspan=2, pady=(4, 0))
        row += 1

        # Random Storage button
        storage_btn = tk.Button(
            frame, text='🏢 Rnd Storage', font=('Consolas', 11, 'bold'),
            bg='#3a3a3a', fg='#808080', activebackground='#505050',
            width=14, height=1, relief='flat', cursor='hand2',
            state='disabled',
            command=lambda did=drone_id: self._on_storage(did),
        )
        storage_btn.grid(row=row, column=0, columnspan=2, pady=(4, 0))
        row += 1

        # Cargo status indicator
        cargo_label = tk.Label(frame, text='', font=('Consolas', 10, 'bold'),
                               fg='#cc7722', bg=colors['bg'], anchor='w')
        cargo_label.grid(row=row, column=0, columnspan=2, sticky='w', pady=(4, 0))
        row += 1

        # Target label
        target_label = tk.Label(frame, text='', font=('Consolas', 9),
                                fg=colors['dim'], bg=colors['bg'], anchor='w')
        target_label.grid(row=row, column=0, columnspan=2, sticky='w', pady=(2, 0))

        self.panels[drone_id] = {
            'frame': frame, 'conn': conn_label, 'pos': pos_labels,
            'armed': armed_label, 'nav': nav_label, 'phase': phase_label,
            'batt_bar': batt_bar, 'batt_pct': batt_pct_label,
            'batt_frame': batt_frame,
            'btn': btn, 'rand_btn': rand_btn, 'refuel_btn': refuel_btn,
            'brs_btn': brs_btn, 'storage_btn': storage_btn,
            'cargo_label': cargo_label,
            'target': target_label,
            'colors': colors,
        }

    def _on_button(self, drone_id):
        ds = self.node.drones[drone_id]
        if ds.phase in ('idle', 'landing'):
            self.node.request_takeoff(drone_id)
        else:
            self.node.request_land(drone_id)

    def _on_random(self, drone_id):
        self.node.request_random_location(drone_id)

    def _on_refuel(self, drone_id):
        self.node.refuel(drone_id)

    def _on_brs(self, drone_id):
        self.node.request_nearest_brs(drone_id)

    def _on_storage(self, drone_id):
        self.node.request_random_storage(drone_id)

    def _refresh(self):
        for drone_id in DRONE_IDS:
            panel = self.panels[drone_id]
            ds = self.node.drones[drone_id]
            c = panel['colors']

            # Connection
            if ds.connected:
                panel['conn'].configure(text='● Connected', fg=c['green'])
            else:
                panel['conn'].configure(text='● Disconnected', fg=c['red'])

            # Position
            panel['pos']['X'].configure(text=f'{ds.x:8.2f} m')
            panel['pos']['Y'].configure(text=f'{ds.y:8.2f} m')
            panel['pos']['Z'].configure(text=f'{-ds.z:8.2f} m')  # NED → display positive up
            panel['pos']['Hdg'].configure(text=f'{ds.heading_deg:8.1f}°')

            # Armed state
            if ds.armed:
                panel['armed'].configure(text='Armed', fg=c['red'])
            else:
                panel['armed'].configure(text='Disarmed', fg=c['dim'])

            # Nav state
            nav_name = NAV_STATE_NAMES.get(ds.nav_state, f'State {ds.nav_state}')
            panel['nav'].configure(text=nav_name)

            # Phase
            phase_text = {'idle': 'Idle', 'warmup': 'Warming up…',
                          'arming': 'Arming…', 'flying': 'Flying',
                          'hovering': 'Hovering…', 'descending': 'Descending…',
                          'landing': 'Landing…'}
            panel['phase'].configure(text=phase_text.get(ds.phase, ds.phase))

            # Button
            if ds.phase in ('idle', 'landing', 'descending'):
                panel['btn'].configure(text='Takeoff', bg=c['green'])
            else:
                panel['btn'].configure(text='Land', bg=c['red'])

            # Battery bar
            pct = ds.battery_pct
            bar_w = int(160 * pct / 100)
            if pct > 25:
                batt_color = c['green']
            elif pct > 10:
                batt_color = '#daa520'  # amber
            else:
                batt_color = c['red']
            panel['batt_bar'].configure(bg=batt_color, width=max(bar_w, 1))
            panel['batt_pct'].configure(text=f'{pct:.0f} %', bg=batt_color)

            # Refuel button — always enabled
            panel['refuel_btn'].configure(state='normal', bg='#daa520', fg='#1e1e1e')

            # Random Location / BRS / Storage buttons — only when flying above 20 m
            safe_alt = ds.phase == 'flying' and ds.z < -20.0
            if safe_alt:
                panel['rand_btn'].configure(state='normal', bg='#569cd6', fg='#1e1e1e')
            else:
                panel['rand_btn'].configure(state='disabled', bg='#3a3a3a', fg='#808080')

            if safe_alt:
                panel['brs_btn'].configure(state='normal', bg='#e8b619', fg='#1e1e1e')
            else:
                panel['brs_btn'].configure(state='disabled', bg='#3a3a3a', fg='#808080')

            if safe_alt and not ds.carrying_package:
                panel['storage_btn'].configure(state='normal', bg='#4488dd', fg='#1e1e1e')
            else:
                panel['storage_btn'].configure(state='disabled', bg='#3a3a3a', fg='#808080')

            # Cargo indicator
            if ds.carrying_package:
                panel['cargo_label'].configure(text='📦 CARGO ATTACHED')
            else:
                panel['cargo_label'].configure(text='')

            # Target display
            if ds.phase == 'flying' and (ds.final_target_x != 0 or ds.final_target_y != 0):
                wp_info = f' ({len(ds.waypoints)} wp)' if ds.waypoints else ''
                panel['target'].configure(
                    text=f'Target: ({ds.final_target_x:.0f}, {ds.final_target_y:.0f}, {-ds.target_z:.0f}m){wp_info}')
            else:
                panel['target'].configure(text='')

        self.root.after(self.REFRESH_MS, self._refresh)

    def run(self):
        self.root.mainloop()


# ── 2D Map Window ────────────────────────────────────────────────────────────

class MapWindow:
    """Top-down 2D view of the city with live drone positions."""

    MAP_SIZE = 900          # canvas pixels (initial)
    WORLD_RANGE = 400       # meters from centre shown (±400 m) at default zoom
    REFRESH_MS = 100
    DRONE_RADIUS = 8        # pixels
    ZOOM_MIN = 50           # min world range (max zoom in)
    ZOOM_MAX = 800          # max world range (max zoom out)
    ZOOM_STEP = 1.2         # zoom factor per scroll tick

    def __init__(self, node: DashboardNode, master):
        self.node = node
        self.win = tk.Toplevel(master)
        self.win.title('City Map — Top View')
        self.win.configure(bg='#1a1a1a')
        self.win.resizable(True, True)
        self.win.geometry('920x960')

        # Current zoom level (world metres visible from centre)
        self._world_range = self.WORLD_RANGE
        # Pan offset in world coordinates (north, east)
        self._pan_north = 0.0
        self._pan_east = 0.0
        # Drag state
        self._drag_start = None

        # Toolbar with zoom buttons
        toolbar = tk.Frame(self.win, bg='#1a1a1a')
        toolbar.pack(fill='x', padx=6, pady=(6, 0))
        tk.Button(toolbar, text='＋ Zoom In', font=('Consolas', 10, 'bold'),
                  bg='#333', fg='white', activebackground='#555',
                  relief='flat', cursor='hand2', padx=8, pady=2,
                  command=self._zoom_in).pack(side='left', padx=(0, 4))
        tk.Button(toolbar, text='－ Zoom Out', font=('Consolas', 10, 'bold'),
                  bg='#333', fg='white', activebackground='#555',
                  relief='flat', cursor='hand2', padx=8, pady=2,
                  command=self._zoom_out).pack(side='left', padx=(0, 4))
        tk.Button(toolbar, text='⊙ Reset', font=('Consolas', 10, 'bold'),
                  bg='#333', fg='white', activebackground='#555',
                  relief='flat', cursor='hand2', padx=8, pady=2,
                  command=self._zoom_reset).pack(side='left', padx=(0, 8))
        self._zoom_label = tk.Label(toolbar, text='', font=('Consolas', 9),
                                    fg='#888', bg='#1a1a1a')
        self._zoom_label.pack(side='left')
        self._update_zoom_label()

        sz = self.MAP_SIZE
        self.canvas = tk.Canvas(self.win, width=sz, height=sz,
                                bg='#1a1a1a', highlightthickness=0)
        self.canvas.pack(padx=6, pady=6, fill='both', expand=True)

        # Bind mouse wheel for zoom
        self.canvas.bind('<MouseWheel>', self._on_mousewheel)      # Windows/macOS
        self.canvas.bind('<Button-4>', self._on_scroll_up)          # Linux scroll up
        self.canvas.bind('<Button-5>', self._on_scroll_down)        # Linux scroll down
        # Bind mouse drag for panning
        self.canvas.bind('<ButtonPress-1>', self._on_drag_start)
        self.canvas.bind('<B1-Motion>', self._on_drag_motion)
        self.canvas.bind('<ButtonRelease-1>', self._on_drag_end)
        # Bind resize
        self.canvas.bind('<Configure>', self._on_resize)

        # Legend
        legend = tk.Frame(self.win, bg='#1a1a1a')
        legend.pack(pady=(0, 6))
        for did in DRONE_IDS:
            tk.Label(legend, text=f'  ● {DRONE_LABELS[did]} (x500_{did})  ',
                     font=('Consolas', 10, 'bold'),
                     fg=DRONE_COLORS[did], bg='#1a1a1a').pack(side='left')
        tk.Label(legend, text='  ■ No-fly zone  ',
                 font=('Consolas', 10, 'bold'),
                 fg='#cc3333', bg='#1a1a1a').pack(side='left')
        tk.Label(legend, text='  ◆ BRS  ',
                 font=('Consolas', 10, 'bold'),
                 fg='#e8b619', bg='#1a1a1a').pack(side='left')
        tk.Label(legend, text='  ■ Storage  ',
                 font=('Consolas', 10, 'bold'),
                 fg='#4488dd', bg='#1a1a1a').pack(side='left')

        # Parse buildings + zones once
        self.buildings = _parse_buildings(BUILDINGS_SDF)
        self.no_fly_zones = node.no_fly_zones
        self.brs = node.brs
        self.storage = node.storage

        # Draw static elements
        self._draw_grid()
        self._draw_buildings()
        self._draw_no_fly_zones()
        self._draw_brs()
        self._draw_storage()

        # Drone canvas items (created once, moved each refresh)
        self._drone_items: dict[int, tuple] = {}  # drone_id → (oval, label)
        self._target_items: dict[int, tuple] = {}  # drone_id → (cross, ring, line)
        self._trails: dict[int, list] = {}  # drone_id → list of (north, east) world positions
        self._trail_items: dict[int, list] = {}  # drone_id → canvas line IDs

        for did in DRONE_IDS:
            color = DRONE_COLORS[did]
            r = self.DRONE_RADIUS

            # Target marker: crosshair + ring + dashed line from drone
            cross_h = self.canvas.create_line(0, 0, 0, 0, fill=color, width=2,
                                               dash=(4, 4), state='hidden')
            cross_v = self.canvas.create_line(0, 0, 0, 0, fill=color, width=2,
                                               dash=(4, 4), state='hidden')
            ring = self.canvas.create_oval(0, 0, 0, 0, outline=color, width=2,
                                            dash=(3, 3), state='hidden')
            line = self.canvas.create_line(0, 0, 0, 0, fill=color, width=1,
                                            dash=(6, 4), state='hidden')
            self._target_items[did] = (cross_h, cross_v, ring, line)

            # Drone dot + label
            oval = self.canvas.create_oval(-r, -r, r, r, fill=color,
                                           outline='white', width=2)
            label = self.canvas.create_text(0, 0, text=DRONE_LABELS[did],
                                            fill='white',
                                            font=('Consolas', 8, 'bold'))
            self._drone_items[did] = (oval, label)
            self._trails[did] = []  # list of (north, east) world positions
            self._trail_items[did] = []  # canvas line item IDs

        # Waypoint path lines (redrawn each frame)
        self._waypoint_line_items: dict[int, list] = {did: [] for did in DRONE_IDS}
        self._pkg_items: dict[int, list] = {did: [] for did in DRONE_IDS}

        self._refresh()

    def _w2c(self, north, east):
        """NED world metres → canvas pixels.

        Horizontal axis = East (+right), vertical axis = North (+up).
        Uses current canvas size, zoom level, and pan offset.
        """
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 2:
            w = self.MAP_SIZE
        if h < 2:
            h = self.MAP_SIZE
        sz = min(w, h)
        s = sz / (2 * self._world_range)
        cx = w / 2 + (east - self._pan_east) * s   # East → right
        cy = h / 2 - (north - self._pan_north) * s  # North → up
        return cx, cy

    def _on_resize(self, event):
        """Redraw static elements when canvas is resized."""
        self._redraw_static()

    def _zoom_in(self):
        self._world_range = max(self.ZOOM_MIN, self._world_range / self.ZOOM_STEP)
        self._update_zoom_label()
        self._redraw_static()

    def _zoom_out(self):
        self._world_range = min(self.ZOOM_MAX, self._world_range * self.ZOOM_STEP)
        self._update_zoom_label()
        self._redraw_static()

    def _zoom_reset(self):
        self._world_range = self.WORLD_RANGE
        self._pan_north = 0.0
        self._pan_east = 0.0
        self._update_zoom_label()
        self._redraw_static()

    def _on_mousewheel(self, event):
        if event.delta > 0:
            self._zoom_in()
        else:
            self._zoom_out()

    def _on_scroll_up(self, event):
        self._zoom_in()

    def _on_scroll_down(self, event):
        self._zoom_out()

    def _on_drag_start(self, event):
        self._drag_start = (event.x, event.y)
        self.canvas.config(cursor='fleur')

    def _on_drag_motion(self, event):
        if self._drag_start is None:
            return
        dx = event.x - self._drag_start[0]
        dy = event.y - self._drag_start[1]
        self._drag_start = (event.x, event.y)
        # Convert pixel delta to world delta
        w = self.canvas.winfo_width() or self.MAP_SIZE
        h = self.canvas.winfo_height() or self.MAP_SIZE
        sz = min(w, h)
        s = sz / (2 * self._world_range)
        # dx positive = dragged right = view moves west (pan_east decreases)
        self._pan_east -= dx / s
        # dy positive = dragged down = view moves south (pan_north decreases)
        self._pan_north += dy / s
        self._redraw_static()

    def _on_drag_end(self, event):
        self._drag_start = None
        self.canvas.config(cursor='')

    def _update_zoom_label(self):
        self._zoom_label.config(
            text=f'  Range: ±{self._world_range:.0f}m')

    def _redraw_static(self):
        """Clear and redraw all static map elements (grid, buildings, zones)."""
        # Delete all static items (tag them on creation)
        self.canvas.delete('static')
        self._draw_grid()
        self._draw_buildings()
        self._draw_no_fly_zones()
        self._draw_brs()
        self._draw_storage()

    def _draw_grid(self):
        """Draw faint coordinate grid every 100 m."""
        step = 100
        wr = int(self._world_range)
        for v in range(-wr, wr + 1, step):
            x0, y0 = self._w2c(v, -wr)
            x1, y1 = self._w2c(v, wr)
            self.canvas.create_line(x0, y0, x1, y1, fill='#2a2a2a', tags=('static',))
            x0, y0 = self._w2c(-wr, v)
            x1, y1 = self._w2c(wr, v)
            self.canvas.create_line(x0, y0, x1, y1, fill='#2a2a2a', tags=('static',))
        # Origin crosshair
        cx, cy = self._w2c(0, 0)
        self.canvas.create_line(cx - 8, cy, cx + 8, cy, fill='#555', width=1, tags=('static',))
        self.canvas.create_line(cx, cy - 8, cx, cy + 8, fill='#555', width=1, tags=('static',))
        # Axis labels
        w = self.canvas.winfo_width() or self.MAP_SIZE
        h = self.canvas.winfo_height() or self.MAP_SIZE
        self.canvas.create_text(w - 20, h // 2 + 12,
                                text='E+', fill='#555', font=('Consolas', 8), tags=('static',))
        self.canvas.create_text(w // 2 + 12, 10,
                                text='N+', fill='#555', font=('Consolas', 8), tags=('static',))

    def _draw_buildings(self):
        """Draw building footprints and 5m safety buffer borders."""
        margin = SAFETY_RADIUS  # 5 m
        arc_steps = 12  # segments per quarter-circle arc
        for (bx, by, bw, bd, bh) in self.buildings:
            # bx=north, by=east, bw=size_north, bd=size_east
            x0, y0 = self._w2c(bx - bw / 2, by - bd / 2)
            x1, y1 = self._w2c(bx + bw / 2, by + bd / 2)
            # Building footprint (solid)
            self.canvas.create_rectangle(x0, y0, x1, y1,
                                         fill='#3a1111', outline='#cc3333',
                                         width=1, tags=('static',))

            # ── Safety buffer border (rounded rectangle) ──
            # Building half-sizes
            hw = bw / 2
            hd = bd / 2
            # Corner centers (north, east) — the building corners
            corners = [
                (bx + hw, by + hd),  # top-right (NE corner)
                (bx + hw, by - hd),  # top-left  (NW corner)
                (bx - hw, by - hd),  # bot-left  (SW corner)
                (bx - hw, by + hd),  # bot-right (SE corner)
            ]
            # Start angle for each corner's quarter arc (in radians, CCW from east)
            start_angles = [
                0,                 # NE: arc from east (0) to north (π/2)
                math.pi / 2,      # NW: arc from north (π/2) to west (π)
                math.pi,          # SW: arc from west (π) to south (3π/2)
                3 * math.pi / 2,  # SE: arc from south (3π/2) to east (2π)
            ]

            buf_color = '#ff6633'
            # Draw 4 quarter-circle arcs + 4 tangent lines
            for i in range(4):
                cn, ce = corners[i]
                a_start = start_angles[i]
                # Draw quarter-circle arc as line segments
                arc_pts = []
                for k in range(arc_steps + 1):
                    angle = a_start + (math.pi / 2) * k / arc_steps
                    pn = cn + margin * math.sin(angle)
                    pe = ce + margin * math.cos(angle)
                    arc_pts.append(self._w2c(pn, pe))
                for k in range(len(arc_pts) - 1):
                    self.canvas.create_line(
                        arc_pts[k][0], arc_pts[k][1],
                        arc_pts[k + 1][0], arc_pts[k + 1][1],
                        fill=buf_color, width=1, dash=(3, 3),
                        tags=('static',))

                # Tangent line: end of this arc → start of next arc
                j = (i + 1) % 4
                cn2, ce2 = corners[j]
                a2_start = start_angles[j]
                # End point of current arc
                a_end = a_start + math.pi / 2
                ep_n = cn + margin * math.sin(a_end)
                ep_e = ce + margin * math.cos(a_end)
                # Start point of next arc
                sp_n = cn2 + margin * math.sin(a2_start)
                sp_e = ce2 + margin * math.cos(a2_start)
                ex, ey = self._w2c(ep_n, ep_e)
                sx, sy = self._w2c(sp_n, sp_e)
                self.canvas.create_line(
                    ex, ey, sx, sy,
                    fill=buf_color, width=1, dash=(3, 3),
                    tags=('static',))

    def _draw_no_fly_zones(self):
        """Draw JSON-defined no-fly zones in red with rounded-corner 5m safety buffer."""
        margin = SAFETY_RADIUS  # 5 m
        arc_steps = 12
        buf_color = '#ff6633'
        for z in self.no_fly_zones:
            if z['type'] == 'rectangle':
                corners = z['corners']  # each [East, North]
                es = [c[0] for c in corners]
                ns = [c[1] for c in corners]
                n_min, n_max = min(ns), max(ns)
                e_min, e_max = min(es), max(es)
                x0, y0 = self._w2c(n_min, e_min)
                x1, y1 = self._w2c(n_max, e_max)
                # Zone footprint
                self.canvas.create_rectangle(x0, y0, x1, y1,
                                             fill='#3a1111', outline='#ff4444',
                                             width=2, dash=(6, 3), tags=('static',))
                # Rounded-corner safety buffer (same style as buildings)
                hw = (n_max - n_min) / 2
                hd = (e_max - e_min) / 2
                cx_n = (n_min + n_max) / 2
                cx_e = (e_min + e_max) / 2
                rect_corners = [
                    (cx_n + hw, cx_e + hd),
                    (cx_n + hw, cx_e - hd),
                    (cx_n - hw, cx_e - hd),
                    (cx_n - hw, cx_e + hd),
                ]
                start_angles = [0, math.pi / 2, math.pi, 3 * math.pi / 2]
                for i in range(4):
                    cn, ce = rect_corners[i]
                    a_start = start_angles[i]
                    arc_pts = []
                    for k in range(arc_steps + 1):
                        angle = a_start + (math.pi / 2) * k / arc_steps
                        pn = cn + margin * math.sin(angle)
                        pe = ce + margin * math.cos(angle)
                        arc_pts.append(self._w2c(pn, pe))
                    for k in range(len(arc_pts) - 1):
                        self.canvas.create_line(
                            arc_pts[k][0], arc_pts[k][1],
                            arc_pts[k + 1][0], arc_pts[k + 1][1],
                            fill=buf_color, width=1, dash=(3, 3),
                            tags=('static',))
                    j = (i + 1) % 4
                    cn2, ce2 = rect_corners[j]
                    a2_start = start_angles[j]
                    a_end = a_start + math.pi / 2
                    ep_n = cn + margin * math.sin(a_end)
                    ep_e = ce + margin * math.cos(a_end)
                    sp_n = cn2 + margin * math.sin(a2_start)
                    sp_e = ce2 + margin * math.cos(a2_start)
                    ex, ey = self._w2c(ep_n, ep_e)
                    sx, sy = self._w2c(sp_n, sp_e)
                    self.canvas.create_line(
                        ex, ey, sx, sy,
                        fill=buf_color, width=1, dash=(3, 3),
                        tags=('static',))
                # Label
                mx, my = self._w2c(cx_n, cx_e)
                self.canvas.create_text(mx, my, text=z.get('name', ''),
                                        fill='#ff6666',
                                        font=('Consolas', 7), tags=('static',))
            elif z['type'] == 'circle':
                east, north = z['center']
                r_w = z['diameter'] / 2
                x0, y0 = self._w2c(north - r_w, east - r_w)
                x1, y1 = self._w2c(north + r_w, east + r_w)
                self.canvas.create_oval(x0, y0, x1, y1,
                                        fill='#3a1111', outline='#ff4444',
                                        width=2, dash=(6, 3), tags=('static',))
                # 5m safety buffer circle
                r_buf = r_w + margin
                bx0, by0 = self._w2c(north - r_buf, east - r_buf)
                bx1, by1 = self._w2c(north + r_buf, east + r_buf)
                self.canvas.create_oval(bx0, by0, bx1, by1,
                                        fill='', outline=buf_color,
                                        width=1, dash=(3, 3), tags=('static',))
                mx, my = self._w2c(north, east)
                self.canvas.create_text(mx, my, text=z.get('name', ''),
                                        fill='#ff6666',
                                        font=('Consolas', 7), tags=('static',))

    def _draw_brs(self):
        """Draw Battery Replacement Stations as yellow diamonds on the map."""
        brs_color = '#e8b619'
        r = 8  # diamond radius in pixels
        for s in self.brs:
            cx, cy = self._w2c(s['north'], s['east'])
            # Diamond shape
            self.canvas.create_polygon(
                cx, cy - r,  cx + r, cy,  cx, cy + r,  cx - r, cy,
                fill=brs_color, outline='#ffe066', width=2, tags=('static',))
            # Label
            self.canvas.create_text(cx, cy + r + 8, text=s['name'],
                                    fill=brs_color,
                                    font=('Consolas', 7, 'bold'), tags=('static',))

    def _draw_storage(self):
        """Draw Storage Buildings as blue squares on the map."""
        color = '#4488dd'
        outline = '#6ab0ff'
        r = 9  # half-size in pixels
        for s in self.storage:
            cx, cy = self._w2c(s['north'], s['east'])
            # Blue square
            self.canvas.create_rectangle(
                cx - r, cy - r, cx + r, cy + r,
                fill=color, outline=outline, width=2, tags=('static',))
            # White X landing mark
            m = 4
            self.canvas.create_line(cx - m, cy - m, cx + m, cy + m,
                                    fill='white', width=1, tags=('static',))
            self.canvas.create_line(cx - m, cy + m, cx + m, cy - m,
                                    fill='white', width=1, tags=('static',))
            # Label
            self.canvas.create_text(cx, cy + r + 8, text=s['name'],
                                    fill=color,
                                    font=('Consolas', 7, 'bold'), tags=('static',))

    def _refresh(self):
        r = self.DRONE_RADIUS
        tr = 10  # target ring radius
        for did in DRONE_IDS:
            ds = self.node.drones[did]
            cx, cy = self._w2c(ds.x, ds.y)
            color = DRONE_COLORS[did]

            # ── Trajectory trail ──
            # Store trail as world positions, redraw each frame for zoom/pan
            if ds.phase in ('flying', 'warmup'):
                trail = self._trails[did]
                # Add new point if drone moved enough in world coords
                if trail:
                    ln, le = trail[-1]
                    if math.hypot(ds.x - ln, ds.y - le) > 1.0:
                        trail.append((ds.x, ds.y))
                else:
                    trail.append((ds.x, ds.y))
                # Keep max 500 points
                if len(trail) > 500:
                    self._trails[did] = trail[-500:]
            elif ds.phase in ('idle', 'landing'):
                self._trails[did] = []

            # Redraw trail lines from world coords
            for item in self._trail_items[did]:
                self.canvas.delete(item)
            self._trail_items[did].clear()
            trail = self._trails[did]
            if len(trail) >= 2:
                for i in range(len(trail) - 1):
                    px0, py0 = self._w2c(trail[i][0], trail[i][1])
                    px1, py1 = self._w2c(trail[i + 1][0], trail[i + 1][1])
                    seg = self.canvas.create_line(
                        px0, py0, px1, py1, fill=color, width=2,
                        stipple='gray50')
                    self._trail_items[did].append(seg)

            # ── Target marker ──
            cross_h, cross_v, ring, line = self._target_items[did]
            has_target = (ds.phase == 'flying' and
                          (ds.final_target_x != 0 or ds.final_target_y != 0))
            if has_target:
                tx, ty = self._w2c(ds.final_target_x, ds.final_target_y)
                # Crosshair
                self.canvas.coords(cross_h, tx - tr, ty, tx + tr, ty)
                self.canvas.coords(cross_v, tx, ty - tr, tx, ty + tr)
                # Ring
                self.canvas.coords(ring, tx - tr, ty - tr, tx + tr, ty + tr)
                # Dashed line from drone to target
                self.canvas.coords(line, cx, cy, tx, ty)
                for item in (cross_h, cross_v, ring, line):
                    self.canvas.itemconfigure(item, state='normal')
            else:
                for item in (cross_h, cross_v, ring, line):
                    self.canvas.itemconfigure(item, state='hidden')

            # ── Drone dot + label ──
            oval, label = self._drone_items[did]
            self.canvas.coords(oval, cx - r, cy - r, cx + r, cy + r)
            self.canvas.coords(label, cx, cy - r - 8)

            # ── Waypoint path ──
            # Remove old waypoint lines
            for item in self._waypoint_line_items[did]:
                self.canvas.delete(item)
            self._waypoint_line_items[did].clear()
            # Remove old package indicators
            for item in self._pkg_items[did]:
                self.canvas.delete(item)
            self._pkg_items[did].clear()
            # Draw new waypoint path (current leg)
            if ds.waypoints and ds.phase in ('flying', 'warmup', 'arming'):
                pts = [(ds.x, ds.y)] + list(ds.waypoints)
                for i in range(len(pts) - 1):
                    px0, py0 = self._w2c(pts[i][0], pts[i][1])
                    px1, py1 = self._w2c(pts[i + 1][0], pts[i + 1][1])
                    seg = self.canvas.create_line(
                        px0, py0, px1, py1, fill=color, width=2,
                        dash=(3, 5), arrow='last')
                    self._waypoint_line_items[did].append(seg)

            # ── Package indicator ──
            if ds.carrying_package:
                ps = 5
                pkg = self.canvas.create_rectangle(
                    cx - ps, cy + r + 1, cx + ps, cy + r + 1 + ps * 2,
                    fill='#cc7722', outline='#ffaa44', width=1)
                self._pkg_items[did].append(pkg)

            # Lift drones + targets above buildings and trails
            for item in (cross_h, cross_v, ring, line):
                self.canvas.tag_raise(item)
            for item in self._waypoint_line_items[did]:
                self.canvas.tag_raise(item)
            for item in self._pkg_items[did]:
                self.canvas.tag_raise(item)
            self.canvas.tag_raise(oval)
            self.canvas.tag_raise(label)

        self.win.after(self.REFRESH_MS, self._refresh)


# ── Delivery Management Panel ────────────────────────────────────────────────

class DeliveryPanel:
    """Window showing delivery orders with Add / Clear buttons."""
    REFRESH_MS = 200

    def __init__(self, node: DashboardNode, master):
        self.node = node
        self.win = tk.Toplevel(master)
        self.win.title('Delivery Management')
        self.win.configure(bg='#1e1e1e')
        self.win.resizable(False, False)
        self.win.geometry('520x460')

        # Button bar
        btn_frame = tk.Frame(self.win, bg='#1e1e1e')
        btn_frame.pack(fill='x', padx=10, pady=(10, 4))

        self.add_btn = tk.Button(
            btn_frame, text='📦 Add Random Order', font=('Consolas', 11, 'bold'),
            bg='#4ec9b0', fg='#1e1e1e', activebackground='#3da88a',
            relief='flat', cursor='hand2', padx=10, pady=4,
            command=self._on_add)
        self.add_btn.pack(side='left', padx=(0, 8))

        tk.Button(
            btn_frame, text='🗑 Clear Done', font=('Consolas', 11, 'bold'),
            bg='#cc5555', fg='#1e1e1e', activebackground='#aa4444',
            relief='flat', cursor='hand2', padx=10, pady=4,
            command=self._on_clear).pack(side='left')

        # Status summary
        self.summary_label = tk.Label(
            self.win, text='', font=('Consolas', 10),
            fg='#808080', bg='#1e1e1e', anchor='w')
        self.summary_label.pack(fill='x', padx=10, pady=(2, 4))

        # Header
        hdr = tk.Frame(self.win, bg='#2d2d2d')
        hdr.pack(fill='x', padx=10)
        cols = [('#', 4), ('Status', 11), ('Drone', 6), ('Storage', 12), ('Delivery To', 16)]
        for text, w in cols:
            tk.Label(hdr, text=text, font=('Consolas', 9, 'bold'),
                     fg='#569cd6', bg='#2d2d2d', width=w, anchor='w').pack(side='left')

        # Scrollable order list
        list_frame = tk.Frame(self.win, bg='#1e1e1e')
        list_frame.pack(fill='both', expand=True, padx=10, pady=(0, 10))

        self.canvas = tk.Canvas(list_frame, bg='#1e1e1e', highlightthickness=0)
        scrollbar = tk.Scrollbar(list_frame, orient='vertical', command=self.canvas.yview)
        self.scroll_frame = tk.Frame(self.canvas, bg='#1e1e1e')
        self.scroll_frame.bind('<Configure>',
                               lambda e: self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.canvas.create_window((0, 0), window=self.scroll_frame, anchor='nw')
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')

        self._order_rows: dict[int, dict] = {}  # order_id -> {frame, labels}
        self._last_order_snapshot = None  # cache to skip no-op refreshes
        self._refresh()

    def _on_add(self):
        self.node.add_random_order()

    def _on_clear(self):
        self.node.clear_completed_orders()
        # Force rebuild on next refresh
        for row_data in self._order_rows.values():
            row_data['frame'].destroy()
        self._order_rows.clear()
        self._last_order_snapshot = None

    def _refresh(self):
        # Update Add button — always enabled (orders queue as pending)
        self.add_btn.configure(state='normal', bg='#4ec9b0')
        free_count = len(self.node.free_drones())

        # Summary
        total = len(self.node.orders)
        active = sum(1 for o in self.node.orders if o.status in ('pickup', 'delivering'))
        done = sum(1 for o in self.node.orders if o.status == 'done')
        pend = sum(1 for o in self.node.orders if o.status == 'pending')
        self.summary_label.configure(
            text=f'Total: {total}  |  Pending: {pend}  |  Active: {active}  |  Done: {done}  |  Free drones: {free_count}')

        # Build a snapshot to detect changes
        snapshot = tuple(
            (o.id, o.status, o.drone_id) for o in self.node.orders)
        if snapshot == self._last_order_snapshot:
            self.win.after(self.REFRESH_MS, self._refresh)
            return
        self._last_order_snapshot = snapshot

        STATUS_COLORS = {
            'pending': '#808080', 'pickup': '#e8b619',
            'delivering': '#569cd6', 'done': '#4ec9b0',
        }
        STATUS_LABELS = {
            'pending': '⏳ Pending', 'pickup': '🔄 Pickup',
            'delivering': '🚁 Delivering', 'done': '✅ Done',
        }

        # Determine which order IDs currently exist
        current_ids = {o.id for o in self.node.orders}
        # Remove rows for deleted orders
        for oid in list(self._order_rows):
            if oid not in current_ids:
                self._order_rows[oid]['frame'].destroy()
                del self._order_rows[oid]

        # Update or create rows (newest first)
        for idx, order in enumerate(reversed(self.node.orders)):
            row_bg = '#2d2d2d' if order.id % 2 == 0 else '#252525'
            fg = STATUS_COLORS.get(order.status, '#e0e0e0')
            drone_text = f'D{order.drone_id}' if order.drone_id else '—'
            d_color = DRONE_COLORS.get(order.drone_id, '#808080') if order.drone_id else '#808080'
            status_text = STATUS_LABELS.get(order.status, order.status)

            if order.id in self._order_rows:
                # Update existing labels
                labels = self._order_rows[order.id]
                labels['status'].configure(text=status_text, fg=fg)
                labels['id_lbl'].configure(fg=fg)
                labels['drone'].configure(text=drone_text, fg=d_color)
            else:
                # Create new row
                row = tk.Frame(self.scroll_frame, bg=row_bg)
                id_lbl = tk.Label(row, text=f'#{order.id}', font=('Consolas', 9),
                                  fg=fg, bg=row_bg, width=4, anchor='w')
                id_lbl.pack(side='left')
                st_lbl = tk.Label(row, text=status_text, font=('Consolas', 9),
                                  fg=fg, bg=row_bg, width=11, anchor='w')
                st_lbl.pack(side='left')
                dr_lbl = tk.Label(row, text=drone_text, font=('Consolas', 9, 'bold'),
                                  fg=d_color, bg=row_bg, width=6, anchor='w')
                dr_lbl.pack(side='left')
                tk.Label(row, text=order.storage_name, font=('Consolas', 9),
                         fg='#c0c0c0', bg=row_bg, width=12, anchor='w').pack(side='left')
                tk.Label(row, text=f'({order.target_n:.0f}, {order.target_e:.0f})',
                         font=('Consolas', 9), fg='#c0c0c0', bg=row_bg, width=16,
                         anchor='w').pack(side='left')
                self._order_rows[order.id] = {
                    'frame': row, 'id_lbl': id_lbl,
                    'status': st_lbl, 'drone': dr_lbl,
                }

            # Ensure correct display order (newest at top)
            self._order_rows[order.id]['frame'].pack_configure(before=None)

        # Repack in correct order (newest first)
        for order in reversed(self.node.orders):
            if order.id in self._order_rows:
                self._order_rows[order.id]['frame'].pack(fill='x', pady=1)

        self.win.after(self.REFRESH_MS, self._refresh)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = DashboardNode()

    # Spin ROS 2 in a background thread so tkinter owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = DashboardGUI(node)
    map_win = MapWindow(node, gui.root)
    delivery_panel = DeliveryPanel(node, gui.root)
    gui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
