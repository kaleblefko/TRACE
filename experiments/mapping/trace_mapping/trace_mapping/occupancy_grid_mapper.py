#!/usr/bin/env python3
"""
occupancy_grid_mapper.py
========================
Real-time 2D probabilistic occupancy grid mapping for a PX4 drone in Gazebo
simulation, using an OAK-D Lite depth camera and NED vehicle odometry.

Subscriptions
─────────────
  /depth_camera               gz-transport (GzImage, float32 metres per pixel)
  /fmu/out/vehicle_odometry   px4_msgs/VehicleOdometry  (NED frame)

Publications  (1 Hz, sim-time-driven timer)
───────────────────────────────────────────
  /slam/occupancy_grid        nav_msgs/OccupancyGrid
  /slam/map_image             sensor_msgs/Image  (bgr8)

Design notes
────────────
* All timestamps come from self.get_clock() (sim time) — wall clock is never used.
* Occupancy is maintained as a float32 log-odds grid and converted to int8 on publish.
* Depth processing runs on the gz-transport thread the moment a frame arrives,
  using the pose snapshot taken at arrival.  This eliminates the motion-induced
  pose/depth lag that the previous timer-driven version exhibited.
* Each pixel is back-projected through the full pinhole model and rotated
  through body and world (NED) frames using the drone's quaternion.  This
  correctly handles pitch/roll and treats depth as Z-depth (gz-sim convention),
  not slant range.
* Map image colours age over 30 s to give a visual sense of map staleness.
"""

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

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage

from px4_msgs.msg import VehicleOdometry

# gz-transport / gz-msgs Python bindings
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage


# ──────────────────────────────────────────────────────────────────────────────
# Camera constants  (OAK-D Lite, PX4 Gazebo model)
# ──────────────────────────────────────────────────────────────────────────────
CAM_FX: float = 429.3          # focal length x  [pixels]
CAM_FY: float = 429.3          # focal length y  [pixels]
CAM_CX: float = 320.0          # principal point x  [pixels]
CAM_CY: float = 240.0          # principal point y  [pixels]
CAM_W:  int   = 640            # full image width
CAM_H:  int   = 480            # full image height
CAM_NEAR: float = 0.2          # near clip  [m]
CAM_FAR:  float = 19.1         # far clip   [m]

# Vertical band: ±10 % of image height around the centre row.
# Even with full 3-D projection + altitude filtering, the band crop is a
# cheap pre-filter that throws away the bulk of pixels that could never
# fall inside the altitude band anyway (the sky, the floor right under
# the drone), saving work in the hot path.
BAND_FRAC: float = 0.10

# ──────────────────────────────────────────────────────────────────────────────
# Occupancy grid constants
# ──────────────────────────────────────────────────────────────────────────────
GRID_RES:    float = 0.05      # metres per cell
GRID_CELLS:  int   = 500       # cells per side  →  25 m × 25 m world coverage
GRID_ORIGIN: int   = 250       # world (0, 0)  ↔  cell (250, 250)

# Log-odds update increments
LO_HIT:  float =  0.85         # single hit
LO_MISS: float = -0.40         # single miss
LO_MIN:  float = -3.5          # saturation floor
LO_MAX:  float =  3.5          # saturation ceiling

# int8 conversion:  int8 = clamp( round(lo × scale), 0, 100 )
LO_SCALE: float = 14.28

# Altitude band [m] around the drone's current Down value: only points
# whose world altitude lies within ±BAND_HALF of the drone are committed
# to the 2-D map.  Tunes a trade-off between (a) accepting walls of
# reasonable height in front of the drone and (b) rejecting ground/
# ceiling returns when the drone pitches.
BAND_HALF: float = 0.50

# ──────────────────────────────────────────────────────────────────────────────
# Visualisation constants
# ──────────────────────────────────────────────────────────────────────────────
AGE_MAX_SECS: float = 30.0     # colour decay window [s]


# ──────────────────────────────────────────────────────────────────────────────
class OccupancyGridMapper(Node):
    """
    ROS2 node that fuses depth-camera measurements into a 2-D log-odds
    occupancy grid and publishes it as a nav_msgs/OccupancyGrid plus a
    colourised debug image every second.
    """

    # ─────────────────────────────  init  ────────────────────────────────────

    def __init__(self) -> None:
        super().__init__('occupancy_grid_mapper')

        # ── Sim-time parameter ───────────────────────────────────────────────
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        # ── Internal grid state ──────────────────────────────────────────────
        self.log_odds  = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.float32)
        self.last_seen = np.full( (GRID_CELLS, GRID_CELLS), np.nan, dtype=np.float64)

        # The depth callback runs on a gz-transport thread; the publish timer
        # runs on the rclpy executor.  Both touch self.log_odds / self.last_seen.
        # Wrap every read/write batch in this lock — the critical sections are
        # short (vectorised numpy) so contention is negligible.
        self._grid_lock = threading.Lock()

        # ── Drone pose snapshot (atomic single-reference swap) ───────────────
        # Stored as a single tuple (n, e, d, R, ts_sec).  The odom callback
        # rebuilds the tuple and rebinds self._pose in one assignment, which
        # is atomic under the GIL.  The depth callback reads self._pose once
        # at frame arrival and is guaranteed to see a fully-formed snapshot.
        self._pose: Optional[tuple] = None

        # ── ROS2 publishers ──────────────────────────────────────────────────
        self._pub_grid = self.create_publisher(
            OccupancyGrid, '/slam/occupancy_grid', 10)
        self._pub_img  = self.create_publisher(
            RosImage, '/slam/map_image', 10)

        # ── ROS2 subscriber — vehicle odometry ───────────────────────────────
        # PX4 micro-XRCE-DDS: BEST_EFFORT + TRANSIENT_LOCAL, depth=1.
        odom_qos = QoSProfile(depth=1)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        odom_qos.history     = HistoryPolicy.KEEP_LAST

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            odom_qos,
        )

        # ── gz-transport subscriber — depth camera ───────────────────────────
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, '/depth_camera', self._depth_cb)

        # ── Pre-computed pixel coordinate grids for the vertical band ────────
        # These never change, so build them once.  Used to vectorise the
        # camera-frame back-projection per depth frame.
        h_band = max(1, int(CAM_H * BAND_FRAC))
        v_lo = CAM_H // 2 - h_band
        v_hi = CAM_H // 2 + h_band
        u_grid, v_grid = np.meshgrid(
            np.arange(CAM_W, dtype=np.float32),
            np.arange(v_lo, v_hi, dtype=np.float32),
        )
        # Normalised image-plane coords: (u-cx)/fx, (v-cy)/fy.
        # 3-D camera-frame point for a pixel with depth Z is then simply
        #   x_cam = x_norm * Z   (right)
        #   y_cam = y_norm * Z   (down)
        #   z_cam = Z            (forward)
        self._x_norm = (u_grid - CAM_CX) / CAM_FX     # (band_h, W)
        self._y_norm = (v_grid - CAM_CY) / CAM_FY     # (band_h, W)
        self._v_lo, self._v_hi = v_lo, v_hi

        # ── 1 Hz publish timer (driven by sim clock) ─────────────────────────
        # The timer no longer drives integration — only output.  Increase the
        # rate freely if downstream consumers want fresher maps.
        self.create_timer(1.0, self._publish_cb)

        self.get_logger().info(
            'OccupancyGridMapper ready — '
            f'{GRID_CELLS}×{GRID_CELLS} grid @ {GRID_RES} m/cell '
            f'({GRID_CELLS * GRID_RES:.1f} m × {GRID_CELLS * GRID_RES:.1f} m)'
        )

    # ─────────────────────  odometry callback  ───────────────────────────────

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        """
        Build a pose snapshot tuple and atomically publish it for the
        depth callback to consume.

        Frame conventions
        ─────────────────
        PX4 NED:
          msg.position[0]  = North  [m]
          msg.position[1]  = East   [m]
          msg.position[2]  = Down   [m]
          msg.q            = [w, x, y, z]  body→world rotation (Hamilton).

        We build the full 3×3 rotation matrix here (instead of just yaw)
        so the depth callback can rotate body-frame points all the way
        into world NED in a single matmul, including pitch and roll.
        """
        n = float(msg.position[0])
        e = float(msg.position[1])
        d = float(msg.position[2])

        qw = float(msg.q[0])
        qx = float(msg.q[1])
        qy = float(msg.q[2])
        qz = float(msg.q[3])

        R = _quat_to_rot(qw, qx, qy, qz)

        # PX4 timestamp_sample is the sample-acquisition time (µs since boot).
        ts_us  = float(msg.timestamp_sample) if msg.timestamp_sample else float(msg.timestamp)
        ts_sec = ts_us * 1e-6

        # Single-reference swap — atomic under the GIL.
        self._pose = (n, e, d, R, ts_sec)

    # ─────────────────────  depth image callback  ────────────────────────────

    def _depth_cb(self, msg: GzImage) -> None:
        """
        Project a depth frame into the world and update the grid.

        Crucially, this runs on the gz-transport thread the instant a frame
        arrives and snapshots whichever pose was most recently published by
        odometry.  At PX4's typical 250 Hz odom rate, the pose used here is
        ≤4 ms old — small enough that a 5 m/s drone has moved <2 cm, well
        below the 5 cm grid resolution.
        """
        pose = self._pose
        if pose is None:
            return

        expected = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size != expected:
            self.get_logger().warn(
                f'Depth frame size mismatch: got {raw.size}, '
                f'expected {msg.width}×{msg.height}={expected}'
            )
            return
        depth = raw.reshape(msg.height, msg.width)

        n, e, d, R, _ = pose
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        with self._grid_lock:
            self._integrate(depth, n, e, d, R, now_sec)

    # ─────────────────────  depth integration  ───────────────────────────────

    def _integrate(
        self,
        depth: np.ndarray,
        drone_n: float,
        drone_e: float,
        drone_d: float,
        R: np.ndarray,
        now_sec: float,
    ) -> None:
        """
        Convert one depth frame into log-odds grid updates.

        Pipeline
        ────────
        1. Crop to the central vertical band (cheap pre-filter).
        2. Discard non-finite / out-of-range pixels.
        3. Back-project each surviving pixel through the pinhole model into
           a 3-D camera-frame point.
        4. Rotate camera→body (FRD) by the fixed mounting transform.
        5. Rotate body→world (NED) by R from the drone's quaternion.
        6. Filter to a thin altitude slab around the drone's Down value.
        7. Convert (N, E) endpoints to grid cells, dedupe, and ray-cast
           free-space + occupied updates from the drone cell to each
           unique endpoint.

        Why the proper 3-D path matters
        ───────────────────────────────
        The previous version assumed the camera was perfectly horizontal
        and treated depth as a horizontal slant range.  Two errors followed:
        (a) Gazebo reports Z-depth, not slant range, so an edge column
            at horizontal angle α was placed at distance Z instead of
            Z/cos(α) — a ~12 % under-estimate at the band edges.
        (b) PX4 quadcopters pitch forward to translate forward.  A "level"
            assumption maps tilted measurements to wrong world coordinates,
            and the central band starts catching ground returns in front
            of the drone.
        Both vanish once we project through the actual quaternion.
        """
        # ── 1 + 2. Band crop and validity mask ──────────────────────────────
        band  = depth[self._v_lo:self._v_hi, :]
        valid = np.isfinite(band) & (band > CAM_NEAR) & (band < CAM_FAR)
        if not valid.any():
            return

        z_cam  = band[valid].astype(np.float32)            # (N,)
        x_norm = self._x_norm[valid]                       # (N,)
        y_norm = self._y_norm[valid]                       # (N,)

        # ── 3. Pinhole back-projection: pixel + Z-depth → 3-D camera point ──
        # Camera frame: x right, y down, z forward (OpenCV / OAK convention).
        x_cam = x_norm * z_cam
        y_cam = y_norm * z_cam
        # z_cam already correct.

        # ── 4. Camera → body (FRD) via the fixed mounting transform ─────────
        # Mounting: optical axis along body +X (forward), camera-right along
        # body +Y, camera-down along body +Z.  So:
        #     body_x = cam_z   (forward)
        #     body_y = cam_x   (right)
        #     body_z = cam_y   (down)
        body_pts = np.vstack((z_cam, x_cam, y_cam))        # (3, N)

        # ── 5. Body → world NED by the drone quaternion ─────────────────────
        # R is body→world.  world_off has shape (3, N).
        world_off = R @ body_pts
        world_n = world_off[0] + drone_n
        world_e = world_off[1] + drone_e
        world_d = world_off[2] + drone_d

        # ── 6. Altitude-band filter ─────────────────────────────────────────
        in_band = np.abs(world_d - drone_d) < BAND_HALF
        if not in_band.any():
            return
        end_n = world_n[in_band]
        end_e = world_e[in_band]

        # ── 7. World → grid cell, in-grid mask, dedupe ──────────────────────
        end_rows = np.round(end_n / GRID_RES).astype(np.int32) + GRID_ORIGIN
        end_cols = np.round(end_e / GRID_RES).astype(np.int32) + GRID_ORIGIN

        in_grid = (
            (end_rows >= 0) & (end_rows < GRID_CELLS) &
            (end_cols >= 0) & (end_cols < GRID_CELLS)
        )
        end_rows = end_rows[in_grid]
        end_cols = end_cols[in_grid]
        if end_rows.size == 0:
            return

        # Dedupe endpoints: pack (row, col) into a single int key, take unique.
        # Many band rows often hit the same wall column, so this can cut the
        # ray-cast loop by an order of magnitude with no loss of fidelity.
        keys         = end_rows.astype(np.int64) * GRID_CELLS + end_cols.astype(np.int64)
        unique_keys  = np.unique(keys)
        unique_rows  = (unique_keys // GRID_CELLS).astype(np.int32)
        unique_cols  = (unique_keys %  GRID_CELLS).astype(np.int32)

        drone_row, drone_col = self._world_to_cell(drone_n, drone_e)

        # ── 8. Ray-cast each unique endpoint ────────────────────────────────
        for er, ec in zip(unique_rows, unique_cols):
            rs, cs = _line_cells(drone_row, drone_col, int(er), int(ec))

            # All cells along the line, except the terminal cell, are FREE.
            if rs.size > 1:
                self._apply_delta(rs[:-1], cs[:-1], LO_MISS, now_sec)
            # Terminal cell is OCCUPIED.
            self._apply_delta(rs[-1:], cs[-1:], LO_HIT, now_sec)

    # ─────────────────────  vectorised log-odds update  ──────────────────────

    def _apply_delta(
        self,
        rows: np.ndarray,
        cols: np.ndarray,
        delta: float,
        now_sec: float,
    ) -> None:
        """
        Vectorised log-odds update with clamp + last_seen stamp.
        Caller must hold self._grid_lock.
        """
        if rows.size == 0:
            return
        new = self.log_odds[rows, cols] + delta
        np.clip(new, LO_MIN, LO_MAX, out=new)
        self.log_odds[rows, cols]  = new
        self.last_seen[rows, cols] = now_sec

    # ─────────────────────  grid helpers  ────────────────────────────────────

    @staticmethod
    def _world_to_cell(north: float, east: float) -> tuple[int, int]:
        """
        Convert NED world coordinates (metres) to integer grid (row, col).

        Layout: row increases with North; col increases with East;
        cell (250, 250) ↔ world (north=0, east=0).
        """
        row = int(round(north / GRID_RES)) + GRID_ORIGIN
        col = int(round(east  / GRID_RES)) + GRID_ORIGIN
        return row, col

    # ─────────────────────  1 Hz publish timer  ──────────────────────────────

    def _publish_cb(self) -> None:
        """
        Publish the current grid state.  Integration happens elsewhere; this
        method only serialises the grid into a nav_msgs/OccupancyGrid and a
        debug image.
        """
        if self._pose is None:
            return
        now_ros = self.get_clock().now()
        now_sec = now_ros.nanoseconds * 1e-9

        with self._grid_lock:
            grid_msg = self._build_grid_msg(now_ros)
            img_msg  = self._build_image_msg(now_ros, now_sec)

        self._pub_grid.publish(grid_msg)
        if img_msg is not None:
            self._pub_img.publish(img_msg)

    # ─────────────────────  OccupancyGrid builder  ───────────────────────────

    def _build_grid_msg(self, now_ros) -> OccupancyGrid:
        """
        Convert the internal float32 log-odds grid to a nav_msgs/OccupancyGrid.

        Encoding
        ────────
          Never-observed cell (last_seen == NaN) → -1  (unknown)
          Observed cell                          → clamp(round(lo·14.28), 0, 100)
        """
        msg = OccupancyGrid()
        msg.header.stamp    = now_ros.to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = GRID_RES
        msg.info.width      = GRID_CELLS
        msg.info.height     = GRID_CELLS
        half_extent = GRID_CELLS * GRID_RES / 2.0
        msg.info.origin.position.x = -half_extent
        msg.info.origin.position.y = -half_extent
        msg.info.origin.position.z =  0.0
        msg.info.origin.orientation.w = 1.0

        lo_flat  = self.log_odds.ravel()
        observed = np.isfinite(self.last_seen.ravel())

        int8_vals = np.where(
            observed,
            np.clip(
                np.round(lo_flat * LO_SCALE).astype(np.int32),
                0,
                100,
            ),
            -1,
        ).astype(np.int8)

        msg.data = int8_vals.tolist()
        return msg

    # ─────────────────────  map image renderer  ──────────────────────────────

    def _build_image_msg(self, now_ros, now_sec: float) -> Optional[RosImage]:
        """
        Render a BGR8 visualisation image of the occupancy grid.
        Same colour scheme as before: unknown grey, free decays bright→dim,
        occupied ages red→orange, drone position is a 3×3 green square.
        """
        lo = self.log_odds
        ls = self.last_seen

        observed = np.isfinite(ls)
        age      = np.where(observed,
                            np.clip(now_sec - ls, 0.0, AGE_MAX_SECS),
                            np.nan)
        age_frac      = age / AGE_MAX_SECS
        age_frac_safe = np.where(observed, age_frac, 0.0)

        is_occupied = observed & (lo >  0.0)
        is_free     = observed & (lo <= 0.0)

        img = np.full((GRID_CELLS, GRID_CELLS, 3), 40, dtype=np.uint8)

        # Free cells: equal-channel grey, brightness 200 → 120 with age.
        bright = np.where(is_free,
                          (200 - 80.0 * age_frac_safe).astype(np.uint8),
                          img[:, :, 0])
        for ch in range(3):
            img[:, :, ch] = np.where(is_free, bright, img[:, :, ch])

        # Occupied cells: BGR (0, 0, 200) → (0, 120, 200) — red drifting to orange.
        g_channel = np.where(is_occupied,
                              (120.0 * age_frac_safe).astype(np.uint8),
                              img[:, :, 1])
        img[:, :, 0] = np.where(is_occupied,   0,         img[:, :, 0])
        img[:, :, 1] = np.where(is_occupied, g_channel,   img[:, :, 1])
        img[:, :, 2] = np.where(is_occupied, 200,         img[:, :, 2])

        # Drone marker: 3×3 bright green square.
        if self._pose is not None:
            n, e, _, _, _ = self._pose
            dr, dc = self._world_to_cell(n, e)
            if _in_grid(dr, dc):
                r0 = max(dr - 1, 0); r1 = min(dr + 2, GRID_CELLS)
                c0 = max(dc - 1, 0); c1 = min(dc + 2, GRID_CELLS)
                img[r0:r1, c0:c1] = (0, 255, 0)

        # Flip so North is at image top.
        img = np.flipud(img)

        ros_img                  = RosImage()
        ros_img.header.stamp     = now_ros.to_msg()
        ros_img.header.frame_id  = 'map'
        ros_img.height           = img.shape[0]
        ros_img.width            = img.shape[1]
        ros_img.encoding         = 'bgr8'
        ros_img.is_bigendian     = 0
        ros_img.step             = img.shape[1] * 3
        ros_img.data             = img.tobytes()
        return ros_img


# ──────────────────────────────────────────────────────────────────────────────
# Module-level helpers (pure functions — no self dependency)
# ──────────────────────────────────────────────────────────────────────────────

def _quat_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """
    Body→world rotation matrix from a Hamilton unit quaternion (w, x, y, z).
    PX4 publishes q as body→world (NED), so this matrix maps a body-frame
    vector to its world-frame (NED) representation.
    """
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz),  2.0 * (xy - wz),       2.0 * (xz + wy)      ],
        [2.0 * (xy + wz),        1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)      ],
        [2.0 * (xz - wy),        2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


def _line_cells(r0: int, c0: int, r1: int, c1: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Return (rows, cols) for all cells from (r0, c0) to (r1, c1) inclusive,
    using a linspace-based supercover line.  Length is Chebyshev(r,c) + 1.
    """
    dr = r1 - r0
    dc = c1 - c0
    n  = max(abs(dr), abs(dc)) + 1
    if n == 1:
        return np.array([r0], dtype=np.int32), np.array([c0], dtype=np.int32)
    ts = np.linspace(0.0, 1.0, n)
    rs = np.round(r0 + ts * dr).astype(np.int32)
    cs = np.round(c0 + ts * dc).astype(np.int32)
    return rs, cs


def _in_grid(row: int, col: int) -> bool:
    """Return True if (row, col) is a valid cell index."""
    return 0 <= row < GRID_CELLS and 0 <= col < GRID_CELLS


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()