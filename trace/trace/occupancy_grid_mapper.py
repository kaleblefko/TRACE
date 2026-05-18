#!/usr/bin/env python3
"""
occupancy_grid_mapper.py
========================
Real-time 2-D probabilistic occupancy grid mapping for a PX4 drone in Gazebo
simulation, using an OAK-D Lite depth camera and NED vehicle odometry.

Subscriptions
─────────────
  /depth_camera               gz-transport (GzImage, float32 metres per pixel)
  /fmu/out/vehicle_odometry   px4_msgs/VehicleOdometry  (NED frame)

Publications
────────────
  /slam/occupancy_grid        nav_msgs/OccupancyGrid    (1 Hz)
  /slam/map_image             sensor_msgs/Image  bgr8   (1 Hz)

Pipeline
────────
On every depth frame (gz-transport thread):
  1. Crop a vertical band around the image centre row.
  2. Reject non-finite / out-of-range pixels.
  3. Back-project surviving pixels through the pinhole model into the
     camera frame, then rotate camera→body→world(NED) using the
     drone's quaternion.
  4. Keep only points whose world altitude is within ±BAND_HALF of the
     drone (rejects floor / ceiling returns).
  5. Round to grid cells, dedupe endpoints with hit counts.
  6. Vectorised ray-cast: for each unique endpoint, walk a supercover
     line from the drone cell.  Cells touched along the way get
     LO_MISS (×N for N grazing rays); endpoints get LO_HIT (×K for K
     converging rays).  Within a single frame, an endpoint cell is
     never decremented by a ray passing through it — this is what
     keeps thin walls crisp instead of letting grazing rays nibble
     them away.
  7. Clip the grid to [LO_MIN, LO_MAX].

On publish (1 Hz):
  • OccupancyGrid: log-odds → probability via sigmoid → int8 (0..100).
    Saturated walls map to ~97 and saturated free space to ~3, so
    RViz renders them as crisp black / white instead of mid-grey.
  • Debug image: free=grey, occupied=red→orange (ageing), drone=green.
"""

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

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage


# ──────────────────────────────────────────────────────────────────────────────
# Camera (OAK-D Lite, PX4 Gazebo)
# ──────────────────────────────────────────────────────────────────────────────
CAM_FX, CAM_FY = 429.3, 429.3      # focal lengths [px]
CAM_CX, CAM_CY = 320.0, 240.0      # principal point [px]
CAM_W,  CAM_H  = 640,   480        # image size [px]
CAM_NEAR, CAM_FAR = 0.2, 19.1      # depth clip [m]

# Vertical band kept before world rotation: ±15 % of image height around the
# centre row.  Wider than the previous 10 % so walls stay in the band when
# the drone pitches by a few degrees; the altitude filter is the semantic
# cut, this is just a cheap pre-filter.
BAND_FRAC = 0.15

# ──────────────────────────────────────────────────────────────────────────────
# Grid
# ──────────────────────────────────────────────────────────────────────────────
GRID_RES    = 0.05                 # [m / cell]
GRID_CELLS  = 500                  # 25 m × 25 m world coverage
GRID_ORIGIN = 250                  # world (0, 0) ↔ cell (250, 250)

# Log-odds increments per ray.  log(p/(1-p)) for p_hit≈0.72, p_miss≈0.40.
LO_HIT  =  0.95
LO_MISS = -0.40
LO_MIN, LO_MAX = -3.5, 3.5         # saturation, ±~3.5 ↔ ±0.97 probability

# Altitude slab around the drone's Down value that contributes to the 2-D
# map.  At 1 m AGL with BAND_HALF=0.5 this samples wall heights [0.5, 1.5] m
# and ignores both floor and ceiling.
BAND_HALF = 0.50

# Visualisation: occupied/free cells fade from "fresh" to "stale" colour
# over this window.
AGE_MAX_SECS = 30.0


# ──────────────────────────────────────────────────────────────────────────────
class OccupancyGridMapper(Node):
    """ROS2 node fusing depth measurements into a 2-D log-odds grid."""

    # ─────────────────────────────  init  ────────────────────────────────────

    def __init__(self) -> None:
        super().__init__('occupancy_grid_mapper')

        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        # Grid state.  Float32 log-odds + last-seen timestamps (sim seconds).
        self.log_odds  = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.float32)
        self.last_seen = np.full( (GRID_CELLS, GRID_CELLS), np.nan, dtype=np.float64)

        # The depth callback runs on a gz-transport thread; the publish timer
        # runs on the rclpy executor.  Critical sections are short numpy ops.
        self._grid_lock = threading.Lock()

        # Pose snapshot tuple (n, e, d, R_body→world, ts_sec).  Rebuilt
        # whole and rebound atomically (single reference assignment under
        # the GIL) by the odometry callback so the depth callback always
        # reads a fully-formed snapshot.
        self._pose: Optional[tuple] = None

        # Publishers
        self._pub_grid = self.create_publisher(
            OccupancyGrid, '/slam/occupancy_grid', 10)
        self._pub_img  = self.create_publisher(
            RosImage, '/slam/map_image', 10)

        # PX4 odometry — micro-XRCE-DDS QoS: BEST_EFFORT + TRANSIENT_LOCAL.
        odom_qos = QoSProfile(depth=1)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        odom_qos.history     = HistoryPolicy.KEEP_LAST
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self._odom_cb, odom_qos)

        # gz-transport depth subscriber.
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, '/depth_camera', self._depth_cb)

        # Pre-compute normalised pixel coordinates once for the band.
        h_band = max(1, int(CAM_H * BAND_FRAC))
        v_lo   = CAM_H // 2 - h_band
        v_hi   = CAM_H // 2 + h_band
        u, v = np.meshgrid(
            np.arange(CAM_W, dtype=np.float32),
            np.arange(v_lo, v_hi, dtype=np.float32),
        )
        self._x_norm = (u - CAM_CX) / CAM_FX        # (band_h, W)
        self._y_norm = (v - CAM_CY) / CAM_FY        # (band_h, W)
        self._v_lo, self._v_hi = v_lo, v_hi

        self.create_timer(1.0, self._publish_cb)

        self.get_logger().info(
            f'OccupancyGridMapper ready — {GRID_CELLS}×{GRID_CELLS} grid '
            f'@ {GRID_RES} m/cell '
            f'({GRID_CELLS * GRID_RES:.1f} m × {GRID_CELLS * GRID_RES:.1f} m)'
        )

    # ─────────────────────  odometry callback  ───────────────────────────────

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        """Snapshot drone pose for the depth callback to consume."""
        n = float(msg.position[0])
        e = float(msg.position[1])
        d = float(msg.position[2])
        R = _quat_to_rot(
            float(msg.q[0]), float(msg.q[1]),
            float(msg.q[2]), float(msg.q[3]),
        )
        ts_us  = float(msg.timestamp_sample) if msg.timestamp_sample else float(msg.timestamp)
        self._pose = (n, e, d, R, ts_us * 1e-6)   # atomic rebind

    # ─────────────────────  depth image callback  ────────────────────────────

    def _depth_cb(self, msg: GzImage) -> None:
        """Project a depth frame and update the grid."""
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
        # 1. Band crop + validity mask
        band  = depth[self._v_lo:self._v_hi, :]
        valid = np.isfinite(band) & (band > CAM_NEAR) & (band < CAM_FAR)
        if not valid.any():
            return

        z_cam  = band[valid].astype(np.float32)
        x_norm = self._x_norm[valid]
        y_norm = self._y_norm[valid]

        # 2. Pinhole back-projection (camera frame: x-right, y-down, z-forward)
        x_cam = x_norm * z_cam
        y_cam = y_norm * z_cam

        # 3. Camera (OpenCV) → body (FRD): forward=cam_z, right=cam_x, down=cam_y
        body_pts = np.vstack((z_cam, x_cam, y_cam))   # (3, N)

        # 4. Body → world NED
        world = R @ body_pts
        world_n = world[0] + drone_n
        world_e = world[1] + drone_e
        world_d = world[2] + drone_d

        # 5. Altitude band: keep only points within ±BAND_HALF of drone's Down.
        in_band = np.abs(world_d - drone_d) < BAND_HALF
        if not in_band.any():
            return
        end_n = world_n[in_band]
        end_e = world_e[in_band]

        # 6. World → grid cells, in-grid mask
        end_rows = np.round(end_n / GRID_RES).astype(np.int32) + GRID_ORIGIN
        end_cols = np.round(end_e / GRID_RES).astype(np.int32) + GRID_ORIGIN
        ok = ((end_rows >= 0) & (end_rows < GRID_CELLS) &
              (end_cols >= 0) & (end_cols < GRID_CELLS))
        end_rows = end_rows[ok]
        end_cols = end_cols[ok]
        if end_rows.size == 0:
            return

        drone_row, drone_col = self._world_to_cell(drone_n, drone_e)
        if not _in_grid(drone_row, drone_col):
            return

        # 7. Dedupe endpoints, keep hit counts.  This is the key fix vs.
        #    the previous version: K rays converging on the same wall cell
        #    contribute K × LO_HIT, not 1 × LO_HIT.
        keys = end_rows.astype(np.int64) * GRID_CELLS + end_cols.astype(np.int64)
        unique_keys, hit_counts = np.unique(keys, return_counts=True)
        unique_rows = (unique_keys // GRID_CELLS).astype(np.int32)
        unique_cols = (unique_keys %  GRID_CELLS).astype(np.int32)

        # 8. Build all ray cells in one vectorised pass.
        line_rows, line_cols, endpoint_mask = _vectorised_lines(
            drone_row, drone_col, unique_rows, unique_cols)

        # 9. MISS pass — every cell strictly between drone and an endpoint,
        #    EXCLUDING any cell that is itself a hit endpoint in this same
        #    frame (endpoint precedence).  Repeated misses on the same cell
        #    aggregate so that a cell missed by N grazing rays gets N×LO_MISS.
        miss_sel = ~endpoint_mask
        if miss_sel.any():
            mr = line_rows[miss_sel]
            mc = line_cols[miss_sel]
            mk = mr.astype(np.int64) * GRID_CELLS + mc.astype(np.int64)
            mk = mk[~np.isin(mk, unique_keys, assume_unique=False)]
            if mk.size > 0:
                u_mk, mc_counts = np.unique(mk, return_counts=True)
                rr = (u_mk // GRID_CELLS).astype(np.int32)
                cc = (u_mk %  GRID_CELLS).astype(np.int32)
                self.log_odds[rr, cc] += LO_MISS * mc_counts.astype(np.float32)
                self.last_seen[rr, cc] = now_sec

        # 10. HIT pass — each unique endpoint, weighted by its ray count.
        self.log_odds[unique_rows, unique_cols] += (
            LO_HIT * hit_counts.astype(np.float32)
        )
        self.last_seen[unique_rows, unique_cols] = now_sec

        # 11. Saturation clamp.
        np.clip(self.log_odds, LO_MIN, LO_MAX, out=self.log_odds)

    # ─────────────────────  grid helpers  ────────────────────────────────────

    @staticmethod
    def _world_to_cell(north: float, east: float) -> tuple[int, int]:
        """NED metres → integer (row, col).  Row grows north, col grows east."""
        row = int(round(north / GRID_RES)) + GRID_ORIGIN
        col = int(round(east  / GRID_RES)) + GRID_ORIGIN
        return row, col

    # ─────────────────────  1 Hz publish timer  ──────────────────────────────

    def _publish_cb(self) -> None:
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
        Convert log-odds → int8 occupancy via the proper sigmoid:
            prob = 1 / (1 + exp(-lo))
            int8 = round(prob × 100)

        With LO_MAX=3.5 this maps:
            saturated occupied (lo=+3.5) → ~97   (black in RViz)
            uncertain          (lo= 0.0) →  50   (mid grey)
            saturated free     (lo=-3.5) →   3   (white in RViz)

        Cells that have never been observed (last_seen NaN) → -1 (unknown).
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

        lo_flat = self.log_odds.ravel()
        prob    = 1.0 / (1.0 + np.exp(-lo_flat))
        int8    = np.round(prob * 100.0).astype(np.int32)

        observed = np.isfinite(self.last_seen.ravel())
        msg.data = np.where(observed, int8, -1).astype(np.int8).tolist()
        return msg

    # ─────────────────────  map image renderer  ──────────────────────────────

    def _build_image_msg(self, now_ros, now_sec: float) -> Optional[RosImage]:
        """Render a BGR8 visualisation of the current grid."""
        lo = self.log_odds
        ls = self.last_seen
        observed = np.isfinite(ls)
        age      = np.where(observed,
                            np.clip(now_sec - ls, 0.0, AGE_MAX_SECS),
                            0.0)
        age_frac = age / AGE_MAX_SECS

        is_occupied = observed & (lo >  0.0)
        is_free     = observed & (lo <= 0.0)

        img = np.full((GRID_CELLS, GRID_CELLS, 3), 40, dtype=np.uint8)

        # Free: equal-channel grey, brightness 200 → 120 with age.
        bright = (200 - 80.0 * age_frac).astype(np.uint8)
        for ch in range(3):
            img[:, :, ch] = np.where(is_free, bright, img[:, :, ch])

        # Occupied: BGR red (0, 0, 200) → orange (0, 120, 200) with age.
        g_chan = (120.0 * age_frac).astype(np.uint8)
        img[:, :, 0] = np.where(is_occupied,   0,    img[:, :, 0])
        img[:, :, 1] = np.where(is_occupied, g_chan, img[:, :, 1])
        img[:, :, 2] = np.where(is_occupied, 200,    img[:, :, 2])

        # Drone marker — 3×3 bright green square.
        if self._pose is not None:
            n, e, _, _, _ = self._pose
            dr, dc = self._world_to_cell(n, e)
            if _in_grid(dr, dc):
                r0 = max(dr - 1, 0); r1 = min(dr + 2, GRID_CELLS)
                c0 = max(dc - 1, 0); c1 = min(dc + 2, GRID_CELLS)
                img[r0:r1, c0:c1] = (0, 255, 0)

        # North up.
        img = np.flipud(img)

        ros_img = RosImage()
        ros_img.header.stamp    = now_ros.to_msg()
        ros_img.header.frame_id = 'map'
        ros_img.height          = img.shape[0]
        ros_img.width           = img.shape[1]
        ros_img.encoding        = 'bgr8'
        ros_img.is_bigendian    = 0
        ros_img.step            = img.shape[1] * 3
        ros_img.data            = img.tobytes()
        return ros_img


# ──────────────────────────────────────────────────────────────────────────────
# Module-level helpers (pure functions)
# ──────────────────────────────────────────────────────────────────────────────

def _quat_to_rot(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Body→world rotation matrix from a Hamilton unit quaternion."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    return np.array([
        [1 - 2 * (yy + zz),  2 * (xy - wz),       2 * (xz + wy)      ],
        [2 * (xy + wz),      1 - 2 * (xx + zz),   2 * (yz - wx)      ],
        [2 * (xz - wy),      2 * (yz + wx),       1 - 2 * (xx + yy)  ],
    ], dtype=np.float32)


def _vectorised_lines(
    r0: int,
    c0: int,
    end_rows: np.ndarray,
    end_cols: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate supercover lines from (r0, c0) to every (end_rows[i], end_cols[i])
    in one vectorised pass.

    Returns
    -------
    rows, cols : np.ndarray of shape (total_cells,)
        Concatenated cell coordinates of all lines.  For ray i with length
        n_i, its cells occupy a contiguous slice of length n_i.
    endpoint_mask : np.ndarray[bool] of shape (total_cells,)
        True at the last cell of each ray (the endpoint), False elsewhere.
    """
    drs = end_rows - r0                                          # (K,)
    dcs = end_cols - c0                                          # (K,)
    n_per = np.maximum(np.abs(drs), np.abs(dcs)).astype(np.int32) + 1
    total = int(n_per.sum())

    # Per-cell ray index and step-along-ray.
    ray_idx = np.repeat(np.arange(end_rows.size, dtype=np.int32), n_per)
    starts  = np.concatenate(([0], np.cumsum(n_per[:-1])))
    step    = np.arange(total, dtype=np.int32) - starts[ray_idx]

    # Parametric position along each ray.  n=1 lines (start==end) get t=0.
    denom = np.maximum(n_per[ray_idx] - 1, 1).astype(np.float32)
    t     = step.astype(np.float32) / denom

    rows = np.round(r0 + drs[ray_idx] * t).astype(np.int32)
    cols = np.round(c0 + dcs[ray_idx] * t).astype(np.int32)

    endpoint_mask = np.zeros(total, dtype=bool)
    endpoint_mask[starts + n_per - 1] = True
    return rows, cols, endpoint_mask


def _in_grid(row: int, col: int) -> bool:
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