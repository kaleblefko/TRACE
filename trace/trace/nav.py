#!/usr/bin/env python3
"""
slam_nav.py
===========
SLAM-aware 2-D navigation for a PX4 drone in ROS2 Humble.

Handles arming, mode switch, takeoff to 1 m, then A* path planning to
/nav_goal targets while holding altitude.

Subscriptions
─────────────
  /slam/occupancy_grid             nav_msgs/OccupancyGrid     (live, 1 Hz)
  /nav_goal                        geometry_msgs/PoseStamped  (operator goal)
  /fmu/out/vehicle_local_position  px4_msgs/VehicleLocalPosition
  /fmu/out/vehicle_status          px4_msgs/VehicleStatus
  /nav/yaw_cmd                     std_msgs/Float32           (radians, NED)
                                     — when present and in HOLD, the held
                                       position is published with this yaw
                                       instead of NaN, so the drone rotates
                                       in place. Set to NaN to release.

Publications
────────────
  /fmu/in/offboard_control_mode    px4_msgs/OffboardControlMode  (10 Hz always)
  /fmu/in/trajectory_setpoint      px4_msgs/TrajectorySetpoint   (10 Hz always)
  /fmu/in/vehicle_command          px4_msgs/VehicleCommand
  /slam/nav_path                   nav_msgs/Path                 (planned path)
  /nav/status                      std_msgs/String               (1 Hz)
                                     — one of: PREFLIGHT, ARM, OFFBOARD,
                                       TAKEOFF, HOLD, NAV, BLOCKED.

Nav goal format
───────────────
  geometry_msgs/PoseStamped in the 'map' frame.
  pose.position.x = North [m]
  pose.position.y = East  [m]
  pose.position.z = ignored — drone holds current altitude throughout.
"""

import heapq
import math
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float32, String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ── Grid constants (keep in sync with occupancy_grid_mapper.py) ───────────────
GRID_RES    = 0.05
GRID_CELLS  = 500
GRID_ORIGIN = 250

# ── Planner settings ──────────────────────────────────────────────────────────
OCC_THRESHOLD   = 60
INFLATE_CELLS   = 4
WAYPOINT_RADIUS = 0.30
REPLAN_INTERVAL = 2.0

# ── Takeoff ───────────────────────────────────────────────────────────────────
TAKEOFF_ALT_NED  = -1.0
TAKEOFF_RADIUS   = 0.15
PREFLIGHT_TICKS  = 20
VEHICLE_CMD_ARM  = 400
VEHICLE_CMD_MODE = 176
PX4_MODE_OFFBOARD = 6
NAV_STATE_OFFBOARD = 14

# ── PX4 QoS ───────────────────────────────────────────────────────────────────
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers
# ─────────────────────────────────────────────────────────────────────────────

def world_to_cell(north: float, east: float) -> tuple[int, int]:
    row = int(round(north / GRID_RES)) + GRID_ORIGIN
    col = int(round(east  / GRID_RES)) + GRID_ORIGIN
    return row, col


def cell_to_world(row: int, col: int) -> tuple[float, float]:
    return (row - GRID_ORIGIN) * GRID_RES, (col - GRID_ORIGIN) * GRID_RES


def in_grid(row: int, col: int) -> bool:
    return 0 <= row < GRID_CELLS and 0 <= col < GRID_CELLS


def inflate_obstacles(occ: np.ndarray, radius: int) -> np.ndarray:
    blocked = (occ >= OCC_THRESHOLD).astype(np.uint8)
    ps = np.zeros((GRID_CELLS + 1, GRID_CELLS + 1), dtype=np.int32)
    ps[1:, 1:] = np.cumsum(np.cumsum(blocked, axis=0), axis=1)
    r1 = np.arange(GRID_CELLS)
    c1 = np.arange(GRID_CELLS)
    rlo = np.clip(r1 - radius,     0, GRID_CELLS)
    rhi = np.clip(r1 + radius + 1, 0, GRID_CELLS)
    clo = np.clip(c1 - radius,     0, GRID_CELLS)
    chi = np.clip(c1 + radius + 1, 0, GRID_CELLS)
    R0 = rlo[:, None]; R1 = rhi[:, None]
    C0 = clo[None, :]; C1 = chi[None, :]
    window = ps[R1, C1] - ps[R0, C1] - ps[R1, C0] + ps[R0, C0]
    return window > 0


def astar(
    inflated: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> Optional[list[tuple[int, int]]]:
    if inflated[goal[0], goal[1]]:
        return None

    sr, sc = start
    gr, gc = goal

    def h(r: int, c: int) -> float:
        return math.hypot(r - gr, c - gc)

    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (h(sr, sc), 0.0, sr, sc))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {(sr, sc): 0.0}

    NEIGHBOURS = [
        (-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414),
        ( 0, -1, 1.0),                 ( 0, 1, 1.0),
        ( 1, -1, 1.414), ( 1, 0, 1.0), ( 1, 1, 1.414),
    ]

    while open_heap:
        _, g, r, c = heapq.heappop(open_heap)
        if (r, c) == goal:
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            path.reverse()
            return path
        if g > g_score.get((r, c), float('inf')) + 1e-6:
            continue
        for dr, dc, cost in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if not in_grid(nr, nc) or inflated[nr, nc]:
                continue
            ng = g + cost
            if ng < g_score.get((nr, nc), float('inf')):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))
    return None


def string_pull(
    path: list[tuple[int, int]],
    inflated: np.ndarray,
) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    pruned = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _line_clear(path[i], path[j], inflated):
                break
            j -= 1
        pruned.append(path[j])
        i = j
    return pruned


def _line_clear(
    a: tuple[int, int],
    b: tuple[int, int],
    inflated: np.ndarray,
) -> bool:
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0); dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while (r, c) != (r1, c1):
        if inflated[r, c]:
            return False
        e2 = 2 * err
        if e2 > -dc:
            err -= dc; r += sr
        if e2 < dr:
            err += dr; c += sc
    return True


def path_still_clear(
    path: list[tuple[int, int]],
    inflated: np.ndarray,
) -> bool:
    return all(not inflated[r, c] for r, c in path)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class SlamNav(Node):

    def __init__(self) -> None:
        super().__init__('slam_nav')

        # ── Grid state ────────────────────────────────────────────────────────
        self._grid_lock = threading.Lock()
        self._occ_grid:  Optional[np.ndarray] = None
        self._inflated:  Optional[np.ndarray] = None

        # ── Drone position (NED metres from VehicleLocalPosition) ─────────────
        self._pos: Optional[tuple[float, float, float]] = None

        # ── Navigation state ─────────────────────────────────────────────────
        self._path:        list[tuple[int, int]] = []
        self._wp_idx:      int = 0
        self._goal_world:  Optional[tuple[float, float]] = None
        self._last_replan: float = 0.0
        self._arming_state: int = -1
        self._nav_state:    int = -1
        self._tick:         int = 0
        # States: PREFLIGHT -> ARM -> OFFBOARD -> TAKEOFF -> HOLD -> NAV
        # plus transient BLOCKED reported when planning fails.
        self._state: str = 'PREFLIGHT'

        # Externally-commanded yaw for HOLD state. NaN means "free yaw"
        # (PX4 holds whatever heading it has).
        self._yaw_cmd: float = float('nan')

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_ocm  = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_sp   = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   PX4_QOS)
        self._pub_cmd  = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self._pub_path = self.create_publisher(
            Path, '/slam/nav_path', 10)
        self._pub_status = self.create_publisher(
            String, '/nav/status', 10)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, '/slam/occupancy_grid', self._grid_cb, 10)
        self.create_subscription(
            PoseStamped, '/nav_goal', self._goal_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, PX4_QOS)
        self.create_subscription(
            Float32, '/nav/yaw_cmd', self._yaw_cmd_cb, 10)

        # ── 10 Hz control loop ───────────────────────────────────────────────
        self.create_timer(0.1, self._loop)
        # ── 1 Hz status publisher ────────────────────────────────────────────
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('slam_nav ready — will arm, take off to 1 m, then await nav goals.')

    # ─────────────────────  Callbacks  ───────────────────────────────────────

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        data = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        inflated = inflate_obstacles(data, INFLATE_CELLS)
        with self._grid_lock:
            self._occ_grid = data
            self._inflated = inflated

    def _goal_cb(self, msg: PoseStamped) -> None:
        goal_n = float(msg.pose.position.x)
        goal_e = float(msg.pose.position.y)
        self._goal_world = (goal_n, goal_e)
        self.get_logger().info(
            f'Nav goal received: N={goal_n:.2f} m  E={goal_e:.2f} m')
        self._replan()

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (float(msg.x), float(msg.y), float(msg.z))

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._arming_state = int(msg.arming_state)
        self._nav_state    = int(msg.nav_state)

    def _yaw_cmd_cb(self, msg: Float32) -> None:
        """External yaw command for HOLD state."""
        self._yaw_cmd = float(msg.data)

    # ─────────────────────  Status publisher  ────────────────────────────────

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._state
        self._pub_status.publish(msg)

    # ─────────────────────  10 Hz loop  ──────────────────────────────────────

    def _loop(self) -> None:
        self._publish_heartbeat()
        self._tick += 1

        if self._state == 'PREFLIGHT':
            self._publish_setpoint(0.0, 0.0, TAKEOFF_ALT_NED, yaw=float("nan"))
            if self._tick >= PREFLIGHT_TICKS:
                self.get_logger().info('Preflight done — arming.')
                self._send_command(VEHICLE_CMD_ARM, param1=1.0)
                self._state = 'ARM'

        elif self._state == 'ARM':
            self._publish_setpoint(0.0, 0.0, TAKEOFF_ALT_NED, yaw=float("nan"))
            if self._arming_state == 2:
                self.get_logger().info('Armed — switching to Offboard.')
                self._send_command(VEHICLE_CMD_MODE, param1=1.0,
                                   param2=float(PX4_MODE_OFFBOARD))
                self._state = 'OFFBOARD'

        elif self._state == 'OFFBOARD':
            self._publish_setpoint(0.0, 0.0, TAKEOFF_ALT_NED, yaw=float("nan"))
            if self._nav_state == NAV_STATE_OFFBOARD:
                self.get_logger().info('Offboard active — climbing to 1 m.')
                self._state = 'TAKEOFF'

        elif self._state == 'TAKEOFF':
            self._publish_setpoint(0.0, 0.0, TAKEOFF_ALT_NED, yaw=float("nan"))
            if self._pos is not None:
                if abs(self._pos[2] - TAKEOFF_ALT_NED) < TAKEOFF_RADIUS:
                    self.get_logger().info(
                        'Takeoff complete — ready for nav goals on /nav_goal.')
                    self._state = 'HOLD'

        elif self._state == 'HOLD':
            # Re-send current position with externally commanded yaw so
            # the state machine can rotate the drone in place via /nav/yaw_cmd.
            if self._pos is not None:
                self._publish_setpoint(*self._pos, yaw=self._yaw_cmd)

        elif self._state == 'NAV':
            self._nav_step()

        elif self._state == 'BLOCKED':
            # Planning failed — hold position until next goal arrives.
            if self._pos is not None:
                self._publish_setpoint(*self._pos, yaw=self._yaw_cmd)

    # ─────────────────────  Navigation  ──────────────────────────────────────

    def _nav_step(self) -> None:
        if self._pos is None or not self._path:
            self._state = 'HOLD'
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_replan > REPLAN_INTERVAL:
            self._last_replan = now
            with self._grid_lock:
                inflated = self._inflated
            if inflated is not None and not path_still_clear(
                    self._path[self._wp_idx:], inflated):
                self.get_logger().warn('Obstacle on path — replanning.')
                self._replan()
                return

        wp_r, wp_c = self._path[self._wp_idx]
        wp_n, wp_e = cell_to_world(wp_r, wp_c)
        wp_z       = self._pos[2]
        if self._wp_idx == 0:
            prev_n, prev_e = self._pos[0], self._pos[1]
        else:
            pr, pc = self._path[self._wp_idx - 1]
            prev_n, prev_e = cell_to_world(pr, pc)
        yaw = math.atan2(wp_e - prev_e, wp_n - prev_n)
        self._publish_setpoint(wp_n, wp_e, wp_z, yaw)

        dist = math.hypot(wp_n - self._pos[0], wp_e - self._pos[1])
        if dist < WAYPOINT_RADIUS:
            self._wp_idx += 1
            if self._wp_idx >= len(self._path):
                self.get_logger().info('Goal reached — holding position.')
                self._state = 'HOLD'
                self._path  = []
            else:
                nr, nc     = self._path[self._wp_idx]
                nn, ne     = cell_to_world(nr, nc)
                self.get_logger().info(
                    f'Waypoint reached — next: N={nn:.2f} m  E={ne:.2f} m')

    # ─────────────────────  Path planning  ───────────────────────────────────

    def _replan(self) -> None:
        if self._pos is None:
            self.get_logger().warn('No position yet — cannot plan.')
            self._state = 'BLOCKED'
            return
        if self._goal_world is None:
            return

        with self._grid_lock:
            inflated = self._inflated

        if inflated is None:
            self.get_logger().warn('No map yet — cannot plan.')
            self._state = 'BLOCKED'
            return

        start = world_to_cell(self._pos[0], self._pos[1])
        goal  = world_to_cell(*self._goal_world)

        if not in_grid(*start) or not in_grid(*goal):
            self.get_logger().error('Start or goal is outside grid bounds.')
            self._state = 'BLOCKED'
            return

        start_blocked = bool(inflated[start[0], start[1]])
        goal_blocked  = bool(inflated[goal[0],  goal[1]])
        self.get_logger().info(
            f'Planning: cell {start} (blocked={start_blocked}) -> '
            f'cell {goal} (blocked={goal_blocked})')

        if goal_blocked:
            self.get_logger().error(
                'Goal cell is inside an inflated obstacle — pick a free space.')
            self._state = 'BLOCKED'
            return

        raw = astar(inflated, start, goal)
        if raw is None:
            self.get_logger().error('A* found no path to goal.')
            self._state = 'BLOCKED'
            return

        smooth = string_pull(raw, inflated)
        self.get_logger().info(
            f'Path found: {len(raw)} raw cells -> '
            f'{len(smooth)} waypoints after string-pull.')

        self._path        = smooth
        self._wp_idx      = 0
        self._last_replan = self.get_clock().now().nanoseconds * 1e-9
        self._state       = 'NAV'
        self._publish_path_msg(smooth)

    # ─────────────────────  PX4 publishers  ──────────────────────────────────

    def _publish_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp    = self._now_us()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._pub_ocm.publish(msg)

    def _publish_setpoint(
        self, x: float, y: float, z: float, yaw: float = 0.0
    ) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp    = self._now_us()
        msg.position     = [x, y, z]
        msg.yaw          = yaw
        msg.velocity     = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        self._pub_sp.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp        = self._now_us()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    def _publish_path_msg(self, path: list[tuple[int, int]]) -> None:
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for row, col in path:
            n, e = cell_to_world(row, col)
            ps   = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = n
            ps.pose.position.y = e
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()