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
* Depth processing: vertical band crop → horizontal edge crop → nanmedian collapse
  → per-column Bresenham-style ray cast with log-odds free/hit updates.
* Map image colours age over 30 s to give a visual sense of map staleness.
"""

import math
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

from builtin_interfaces.msg import Time          # noqa: F401  (used implicitly by to_msg)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Header

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
CAM_NEAR: float = 0.2          # near clip  [m]  — depths below this are noise
CAM_FAR:  float = 19.1         # far clip   [m]  — depths above this are unreliable

# ──────────────────────────────────────────────────────────────────────────────
# Occupancy grid constants
# ──────────────────────────────────────────────────────────────────────────────
GRID_RES:    float = 0.05      # metres per cell
GRID_CELLS:  int   = 500       # cells per side  →  25 m × 25 m world coverage
GRID_ORIGIN: int   = 250       # world (0, 0)  ↔  cell (250, 250)

# Log-odds update increments
LO_HIT:  float =  0.85         # single hit   ≈ P(occ | hit)  ≈ 0.70
LO_MISS: float = -0.40         # single miss  ≈ P(occ | free) ≈ 0.40
LO_MIN:  float = -3.5          # saturation floor
LO_MAX:  float =  3.5          # saturation ceiling

# int8 conversion:  int8 = clamp( round(lo × scale), 0, 100 )
# scale = 14.28 maps  lo ∈ [-3.5, 3.5]  →  int ∈ [-50, 50]
# Positive log-odds → occupied (> 0); the spec clamps to [0, 100].
LO_SCALE: float = 14.28

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
        # rclpy's Node base class declares use_sim_time itself on some builds;
        # guard against the duplicate-declaration exception so the node works
        # regardless.  The launch file / CLI still sets use_sim_time:=true
        # globally; this call just supplies the default when it is not overridden.
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass   # already declared by the rclpy Node base class — that's fine

        # ── Internal grid state ──────────────────────────────────────────────
        # log_odds[row, col] : float32, initialised to 0 = complete uncertainty
        # last_seen[row, col]: float64 sim-seconds, NaN = never observed
        self.log_odds  = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.float32)
        self.last_seen = np.full( (GRID_CELLS, GRID_CELLS), np.nan, dtype=np.float64)

        # ── Drone pose in NED frame ──────────────────────────────────────────
        self._north:     float = 0.0    # metres, positive = North
        self._east:      float = 0.0    # metres, positive = East
        self._yaw:       float = 0.0    # radians; 0 = North, +ve = clockwise (NED)
        self._pose_ready: bool = False

        # Latest depth frame as float32 ndarray (H×W), or None before first msg
        self._depth: Optional[np.ndarray] = None

        # ── ROS2 publishers ──────────────────────────────────────────────────
        self._pub_grid = self.create_publisher(
            OccupancyGrid, '/slam/occupancy_grid', 10)
        self._pub_img  = self.create_publisher(
            RosImage, '/slam/map_image', 10)

        # ── ROS2 subscriber — vehicle odometry ───────────────────────────────
        # PX4 micro-XRCE-DDS uses BEST_EFFORT + TRANSIENT_LOCAL, depth=1.
        odom_qos = QoSProfile(
            reliability=DurabilityPolicy.TRANSIENT_LOCAL,  # type: ignore[arg-type]
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Rebuild correctly — QoSProfile fields differ from above shortcut
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
        # gz-transport manages its own thread pool; the callback runs there.
        # We only buffer the latest frame (no locking needed per spec).
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, '/depth_camera', self._depth_cb)

        # ── 1 Hz publish timer (driven by sim clock) ─────────────────────────
        self.create_timer(1.0, self._publish_cb)

        self.get_logger().info(
            'OccupancyGridMapper ready — '
            f'{GRID_CELLS}×{GRID_CELLS} grid @ {GRID_RES} m/cell '
            f'({GRID_CELLS * GRID_RES:.1f} m × {GRID_CELLS * GRID_RES:.1f} m)'
        )

    # ─────────────────────  odometry callback  ───────────────────────────────

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        """
        Extract NED position and yaw from PX4 VehicleOdometry.

        Frame convention
        ────────────────
        PX4 uses NED (North-East-Down):
          msg.position[0]  = North  [m]
          msg.position[1]  = East   [m]
          msg.position[2]  = Down   [m]  (unused here)

        Quaternion field: msg.q = [w, x, y, z]  (Hamilton, body-from-world).

        Yaw extraction (ZYX Euler, NED frame)
        ──────────────────────────────────────
          ψ = atan2( 2(w·z + x·y),  1 − 2(y² + z²) )

        This gives yaw measured from North, positive clockwise — the standard
        NED heading convention used throughout this node's bearing math.
        """
        self._north = float(msg.position[0])
        self._east  = float(msg.position[1])

        qw = float(msg.q[0])
        qx = float(msg.q[1])
        qy = float(msg.q[2])
        qz = float(msg.q[3])

        # ZYX Euler yaw from a NED quaternion; equivalent to heading from North.
        self._yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self._pose_ready = True

    # ─────────────────────  depth image callback  ────────────────────────────

    def _depth_cb(self, msg: GzImage) -> None:
        """
        Receive a Gazebo float32 depth image and store the latest frame.
        Each pixel value is the range to that scene point in metres.
        NaN / ±Inf = invalid / no return.
        """
        expected = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size != expected:
            self.get_logger().warn(
                f'Depth frame size mismatch: got {raw.size}, '
                f'expected {msg.width}×{msg.height}={expected}'
            )
            return
        # Copy because the protobuf buffer may be reused by gz-transport.
        self._depth = raw.reshape(msg.height, msg.width).copy()

    # ─────────────────────  depth processing  ────────────────────────────────

    def _process_depth(self, depth: np.ndarray, now_sec: float) -> None:
        """
        Convert one depth frame into log-odds grid updates.

        Processing pipeline
        ───────────────────
        1. Vertical crop  : keep a ±10 % band around the horizontal centre row.
           Rationale — the drone flies approximately level; rows near the centre
           correspond to roughly horizontal rays, giving reliable ground-plane
           obstacle detection without needing pitch correction.

        2. Horizontal crop: discard the outermost 15 % of columns on each side.
           Rationale — wide-angle lenses exhibit barrel distortion at the edges;
           these pixels have unreliable angular correspondence to the nominal fx.

        3. Collapse       : nanmedian per column across the band rows.
           Result is a 1-D array of one representative range per column.
           nanmedian is robust to a mix of valid and invalid pixels in the band.

        4. Ray cast       : for each valid column depth, cast a ray in the NED
           world frame, mark free cells along it (log-odds miss), and mark the
           terminal cell occupied (log-odds hit).

        Bearing maths (NED)
        ────────────────────
        The camera boresight points forward along the drone's body +X axis,
        which is aligned with North when yaw = 0.

        Horizontal angle of column c relative to boresight:
            α_rel = atan2( c_orig − cx,  fx )
            (positive α_rel = rightward in image = clockwise in NED = East)

        Absolute NED bearing of that column's ray:
            β = yaw + α_rel
            (yaw = 0 → North;  yaw > 0 → clockwise → East)

        NED displacement to range r along bearing β:
            Δnorth = r · cos(β)
            Δeast  = r · sin(β)
        """
        h, w = depth.shape     # should be 480 × 640

        # ── 1. Vertical crop: ±10 % band about centre row ──────────────────
        half_band = max(1, int(h * 0.10))
        row_lo = h // 2 - half_band
        row_hi = h // 2 + half_band          # Python slice: exclusive upper bound
        band = depth[row_lo:row_hi, :]       # shape: (2*half_band, w)

        # ── 2. Horizontal edge crop: remove 15 % from each side ────────────
        margin   = int(w * 0.15)             # e.g. 96 pixels for w=640
        band     = band[:, margin : w - margin]   # shape: (band_rows, crop_w)
        crop_w   = band.shape[1]

        # Original (full-frame) column index for each surviving column.
        # This is critical: bearing is computed from cx and fx which reference
        # the *original* full-width frame.  If we used 0..crop_w instead, every
        # bearing would be biased leftward by `margin` pixels.
        orig_cols = np.arange(margin, w - margin, dtype=np.float64)   # (crop_w,)

        # ── 3. Collapse band to 1-D range per column ───────────────────────
        col_depths = np.nanmedian(band, axis=0)    # (crop_w,) float32 / float64

        # ── 4. Ray cast ─────────────────────────────────────────────────────
        drone_row, drone_col = self._world_to_cell(self._north, self._east)

        for i in range(crop_w):
            r = float(col_depths[i])

            # Skip invalid or out-of-sensor-range depths
            if not math.isfinite(r) or r < CAM_NEAR or r > CAM_FAR:
                continue

            # Bearing of this column's ray in NED world frame [radians]
            alpha_rel = math.atan2(orig_cols[i] - CAM_CX, CAM_FX)
            bearing   = self._yaw + alpha_rel

            # World coordinates of the ray endpoint
            end_north = self._north + r * math.cos(bearing)
            end_east  = self._east  + r * math.sin(bearing)

            end_row, end_col = self._world_to_cell(end_north, end_east)

            # Discard rays whose endpoints fall outside the mapped area
            if not _in_grid(end_row, end_col):
                continue

            # ── Free cells (log-odds miss) ──────────────────────────────────
            # Sample the ray uniformly from the drone to just *before* the
            # endpoint (endpoint=False).  Step count ≈ Chebyshev distance so
            # that we visit approximately one cell per step.
            n_steps = max(
                int(max(abs(end_row - drone_row), abs(end_col - drone_col))),
                1,
            )
            ts = np.linspace(0.0, 1.0, n_steps, endpoint=False)
            for t in ts:
                rr = int(round(drone_row + t * (end_row - drone_row)))
                rc = int(round(drone_col + t * (end_col - drone_col)))
                if _in_grid(rr, rc):
                    self._update_cell(rr, rc, LO_MISS, now_sec)

            # ── Terminal cell (log-odds hit) ─────────────────────────────────
            # Clamp inside-grid check already done above for end_row/col.
            self._update_cell(end_row, end_col, LO_HIT, now_sec)

    # ─────────────────────  grid helpers  ────────────────────────────────────

    @staticmethod
    def _world_to_cell(north: float, east: float) -> tuple[int, int]:
        """
        Convert NED world coordinates (metres) to integer grid (row, col).

        Grid layout
        ───────────
          row increases with North  (row 0 = southernmost edge)
          col increases with East   (col 0 = westernmost  edge)
          cell (250, 250) ↔ world (north=0, east=0)

        So for a point P:
          row = round( north / res ) + 250
          col = round( east  / res ) + 250
        """
        row = int(round(north / GRID_RES)) + GRID_ORIGIN
        col = int(round(east  / GRID_RES)) + GRID_ORIGIN
        return row, col

    def _update_cell(
        self, row: int, col: int, delta: float, now_sec: float
    ) -> None:
        """
        Apply a log-odds delta to one cell, clamp to [LO_MIN, LO_MAX], and
        record the current sim time as the last-observation timestamp.

        Clamping prevents the map from becoming too "confident" about any cell
        and retains the ability to un-occupy cells that were previously hit
        (e.g. a moving obstacle or a spurious sensor return).
        """
        new_lo = self.log_odds[row, col] + delta
        self.log_odds[row, col] = max(LO_MIN, min(LO_MAX, new_lo))
        self.last_seen[row, col] = now_sec

    # ─────────────────────  1 Hz publish timer  ──────────────────────────────

    def _publish_cb(self) -> None:
        """
        Process the latest buffered depth frame and publish both map products.

        All timestamps come exclusively from self.get_clock() to honour the
        use_sim_time parameter — never from time.time() or the wall clock.
        """
        now_ros = self.get_clock().now()
        now_sec = now_ros.nanoseconds * 1e-9   # float64 sim seconds

        # Integrate the most recent depth frame (if pose and depth are available)
        if self._depth is not None and self._pose_ready:
            self._process_depth(self._depth, now_sec)

        # Publish occupancy grid
        grid_msg = self._build_grid_msg(now_ros)
        self._pub_grid.publish(grid_msg)

        # Publish visualisation image
        img_msg = self._build_image_msg(now_ros, now_sec)
        if img_msg is not None:
            self._pub_img.publish(img_msg)

    # ─────────────────────  OccupancyGrid builder  ───────────────────────────

    def _build_grid_msg(self, now_ros) -> OccupancyGrid:
        """
        Convert the internal float32 log-odds grid to a nav_msgs/OccupancyGrid.

        Encoding
        ────────
          Never-observed cell (last_seen == NaN) → data value = -1  (unknown)
          Observed cell                           → clamp( round(lo × 14.28), 0, 100 )

        At lo = 0 → int8 = 0 (free boundary / no information after being seen).
        At lo = +3.5 → int8 = 50 (strongly occupied, per the 14.28 scale).
        The positive clamp [0, 100] means free cells (lo < 0) map to 0.

        OccupancyGrid origin
        ────────────────────
        The info.origin describes the bottom-left (south-west) corner in the
        ROS map frame (ENU: x = East, y = North).
          origin.x = −12.5 m  (west edge)
          origin.y = −12.5 m  (south edge)
        data is row-major: index = row × width + col,
          where row 0 is at y = origin.y (south), col 0 is at x = origin.x (west).
        This matches the _world_to_cell convention exactly.
        """
        msg = OccupancyGrid()

        # Header — sim time, standard map frame
        msg.header.stamp    = now_ros.to_msg()
        msg.header.frame_id = 'map'

        # Grid metadata
        msg.info.resolution = GRID_RES
        msg.info.width      = GRID_CELLS
        msg.info.height     = GRID_CELLS
        half_extent = GRID_CELLS * GRID_RES / 2.0   # 12.5 m
        msg.info.origin.position.x = -half_extent
        msg.info.origin.position.y = -half_extent
        msg.info.origin.position.z =  0.0
        msg.info.origin.orientation.w = 1.0          # identity rotation

        # ── Log-odds → int8 conversion ──────────────────────────────────────
        lo_flat = self.log_odds.ravel()              # (250000,) float32

        # Cells where last_seen is finite have been observed at least once.
        observed = np.isfinite(self.last_seen.ravel())

        int8_vals = np.where(
            observed,
            np.clip(
                np.round(lo_flat * LO_SCALE).astype(np.int32),
                0,       # free cells (lo < 0) map to 0, not negative
                100,
            ),
            -1,          # never-observed → unknown
        ).astype(np.int8)

        msg.data = int8_vals.tolist()
        return msg

    # ─────────────────────  map image renderer  ──────────────────────────────

    def _build_image_msg(self, now_ros, now_sec: float) -> Optional[RosImage]:
        """
        Render a BGR8 visualisation image of the occupancy grid.

        Colour scheme
        ─────────────
          Unknown   (never observed)  : dark grey  BGR = (40, 40, 40)
          Free      (lo ≤ 0, observed): grey, brightness decays
                                        200 → 120 over AGE_MAX_SECS seconds
          Occupied  (lo > 0, observed): red → orange over AGE_MAX_SECS
                                        fresh:  BGR = (  0,   0, 200)  ← red
                                        old:    BGR = (  0, 120, 200)  ← orange
                                        Note: OpenCV BGR means ch0=Blue, ch2=Red.
                                        The orange shift comes from increasing
                                        the Green channel (ch1) while Red stays.
          Drone pos                   : 3×3 bright green  BGR = (0, 255, 0)

        Display orientation
        ───────────────────
        The internal grid has row 0 at the south edge (small North values).
        np.flipud is applied before encoding so North appears at the image top,
        matching the conventional map-viewer orientation.  The drone marker is
        painted before the flip; after flipping, a drone at row dr appears at
        display row (GRID_CELLS − 1 − dr), which is correctly in the north-up image.
        """
        lo  = self.log_odds
        ls  = self.last_seen

        # Age of each cell [0, AGE_MAX_SECS], NaN for unobserved
        observed = np.isfinite(ls)
        age      = np.where(observed,
                            np.clip(now_sec - ls, 0.0, AGE_MAX_SECS),
                            np.nan)
        age_frac = age / AGE_MAX_SECS        # 0.0 = fresh, 1.0 = old, NaN = unseen

        # NaN-safe version for arithmetic that feeds into .astype(np.uint8).
        # np.where evaluates BOTH branches before selecting, so a NaN in the
        # discarded branch still triggers "invalid value in cast" warnings even
        # though those cells are never written to the output image.
        # Replacing NaN with 0.0 before the cast eliminates this entirely.
        age_frac_safe = np.where(observed, age_frac, 0.0)

        # Cell classification masks
        is_occupied = observed & (lo >  0.0)
        is_free     = observed & (lo <= 0.0)

        # ── Allocate image: initialise everything to unknown grey ───────────
        img = np.full((GRID_CELLS, GRID_CELLS, 3), 40, dtype=np.uint8)

        # ── Free cells: equal-channel grey, brightness 200 → 120 ───────────
        # age_frac is finite wherever is_free is True (both require observed).
        bright = np.where(is_free,
                          (200 - 80.0 * age_frac_safe).astype(np.uint8),
                          img[:, :, 0])
        # Assign the same value to all three channels (B = G = R → grey)
        for ch in range(3):
            img[:, :, ch] = np.where(is_free, bright, img[:, :, ch])

        # ── Occupied cells: BGR (0, 0, 200) → (0, 120, 200) with age ───────
        # ch0 (Blue)  = 0     always
        # ch1 (Green) = 0 → 120 as age_frac goes 0 → 1  (creates orange shift)
        # ch2 (Red)   = 200   always
        #
        # Why this looks red then orange:
        #   BGR (0, 0, 200) = pure red (Red=200, no Blue, no Green)
        #   BGR (0, 120, 200) = Red+Green mix → orange
        g_channel = np.where(is_occupied,
                              (120.0 * age_frac_safe).astype(np.uint8),
                              img[:, :, 1])
        img[:, :, 0] = np.where(is_occupied,   0,         img[:, :, 0])  # B = 0
        img[:, :, 1] = np.where(is_occupied, g_channel,   img[:, :, 1])  # G grows
        img[:, :, 2] = np.where(is_occupied, 200,         img[:, :, 2])  # R = 200

        # ── Drone marker: 3×3 bright green square ───────────────────────────
        if self._pose_ready:
            dr, dc = self._world_to_cell(self._north, self._east)
            if _in_grid(dr, dc):
                r0 = max(dr - 1, 0);          r1 = min(dr + 2, GRID_CELLS)
                c0 = max(dc - 1, 0);          c1 = min(dc + 2, GRID_CELLS)
                img[r0:r1, c0:c1] = (0, 255, 0)    # BGR green

        # ── Flip so North is at image top ────────────────────────────────────
        img = np.flipud(img)

        # ── Build sensor_msgs/Image directly from the numpy array ───────────
        # We bypass cv_bridge entirely because it depends on a compiled NumPy
        # 1.x extension that crashes under NumPy 2.x.  A bgr8 RosImage is just
        # a flat byte buffer with a few header fields — trivial to fill manually.
        ros_img                  = RosImage()
        ros_img.header.stamp     = now_ros.to_msg()
        ros_img.header.frame_id  = 'map'
        ros_img.height           = img.shape[0]
        ros_img.width            = img.shape[1]
        ros_img.encoding         = 'bgr8'
        ros_img.is_bigendian     = 0
        ros_img.step             = img.shape[1] * 3    # bytes per row  (3 ch × uint8)
        ros_img.data             = img.tobytes()        # C-contiguous row-major bytes
        return ros_img


# ──────────────────────────────────────────────────────────────────────────────
# Module-level helpers (no self dependency — keep them pure functions)
# ──────────────────────────────────────────────────────────────────────────────

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