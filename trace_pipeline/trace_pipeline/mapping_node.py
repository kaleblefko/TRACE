#!/usr/bin/env python3
"""
mapping_node.py

Single bridge node feeding RTAB-Map everything it needs:
  - PX4 pose → nav_msgs/Odometry + TF tree
  - Gazebo depth image → /camera/depth/image_raw + CameraInfo
  - Gazebo RGB image   → /camera/rgb/image_raw   + CameraInfo

Subscribes (PX4):
  /fmu/out/vehicle_local_position_v1  (px4_msgs/VehicleLocalPosition)
  /fmu/out/vehicle_attitude           (px4_msgs/VehicleAttitude)

Subscribes (Gazebo transport):
  /depth_camera
  /world/small_house/model/trace_drone_0/model/mono_cam/...

Publishes:
  /rtabmap/odom                 (nav_msgs/Odometry)
  /camera/depth/image_raw       (sensor_msgs/Image,      32FC1)
  /camera/depth/camera_info     (sensor_msgs/CameraInfo)
  /camera/rgb/image_raw         (sensor_msgs/Image,      rgb8)
  /camera/rgb/camera_info       (sensor_msgs/CameraInfo)
  TF: map → odom → base_link → camera_link
"""

<<<<<<< Updated upstream
import cv2
=======
>>>>>>> Stashed changes
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

<<<<<<< Updated upstream
<<<<<<< Updated upstream
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from px4_msgs.msg import TrajectorySetpoint
=======
=======
>>>>>>> Stashed changes
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


# ---------------------------------------------------------------------------
# Camera intrinsics (OakD-Lite, from model.sdf)
# ---------------------------------------------------------------------------

def _make_camera_info(width: int, height: int, hfov_rad: float,
                      frame_id: str) -> CameraInfo:
    """Build CameraInfo from image dimensions and horizontal FOV."""
    fx = (width / 2.0) / np.tan(hfov_rad / 2.0)
    fy = fx
    cx = width  / 2.0
    cy = height / 2.0

    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width  = width
    info.height = height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx,  0.0, cx,
              0.0, fy,  cy,
              0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [fx,  0.0, cx,  0.0,
              0.0, fy,  cy,  0.0,
              0.0, 0.0, 1.0, 0.0]
    return info
<<<<<<< Updated upstream


DEPTH_INFO = _make_camera_info(640,  480,  1.274, 'camera_link')
RGB_INFO   = _make_camera_info(1920, 1080, 1.204, 'camera_link')
>>>>>>> Stashed changes


=======


DEPTH_INFO = _make_camera_info(640,  480,  1.274, 'camera_link')
RGB_INFO   = _make_camera_info(1920, 1080, 1.204, 'camera_link')


>>>>>>> Stashed changes
# ---------------------------------------------------------------------------
# Bridge node
# ---------------------------------------------------------------------------

class MappingNode(Node):
    def __init__(self):
        super().__init__("trace_mapping_node")

<<<<<<< Updated upstream
        # ------------------------------------------------------------------
        # PX4 QoS (must match publisher side)
        # ------------------------------------------------------------------
=======
        # =======================
        # PX4 QoS
        # =======================
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

<<<<<<< Updated upstream
        # ------------------------------------------------------------------
        # Drone pose — sourced from PX4 actual local position
        # ------------------------------------------------------------------
        self._pos_x: float = 0.0   # NED North (metres)
        self._pos_y: float = 0.0   # NED East  (metres)
        self._yaw:   float = 0.0   # heading   (radians, 0 = North, +CCW)
=======
        # =======================
        # Pose state
        # =======================
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q = [1.0, 0.0, 0.0, 0.0]   # [w, x, y, z]
        self.have_position = False
        self.have_attitude = False
<<<<<<< Updated upstream
>>>>>>> Stashed changes

        # =======================
        # PX4 subscribers
        # =======================
        self.create_subscription(
<<<<<<< Updated upstream
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self._trajectory_setpoint_callback,
            px4_qos,
        )

        # ------------------------------------------------------------------
        # Depth-camera parameters
        # OakD-Lite — https://github.com/PX4/PX4-gazebo-models
        # ------------------------------------------------------------------
        self._cam_hfov   = 1.274   # radians
        self._cam_width  = 640
        self._cam_height = 480
        self._cam_near   = 0.2     # metres
        self._cam_far    = 19.1    # metres

        # ------------------------------------------------------------------
        # Occupancy-map parameters
        # ------------------------------------------------------------------
        self._map_resolution = 0.05   # metres / cell
        self._map_cols       = 500    # 25 m wide
        self._map_rows       = 500    # 25 m tall

        # World origin (0, 0) maps to this cell
        self._map_origin_col = self._map_cols // 2
        self._map_origin_row = self._map_rows // 2

        # -1 = unknown, 0 = free, 100 = occupied
        self._occ_grid  = np.full(
            (self._map_rows, self._map_cols), -1, dtype=np.int8
        )
        # Seconds since node start when a cell was last observed
        self._map_stamp = np.zeros(
            (self._map_rows, self._map_cols), dtype=np.float64
        )

        # ------------------------------------------------------------------
        # ROS publishers
        # ------------------------------------------------------------------
        self._map_pub = self.create_publisher(
            OccupancyGrid, "/slam/occupancy_grid", 10
        )
        self._map_img_pub = self.create_publisher(
            RosImage, "/slam/map_image", 10
        )

        # ------------------------------------------------------------------
        # Gazebo depth-camera subscriber
        # ------------------------------------------------------------------
        self._depth_gz_node = GzNode()
        self._depth_gz_node.subscribe(
            GzImage, "/depth_camera", self._gz_depth_callback
        )

        # ------------------------------------------------------------------
        # Publish map at ~1 Hz
        # ------------------------------------------------------------------
        self._map_pub_timer = self.create_timer(1.0, self._publish_map)

        self.get_logger().info("MappingNode started.")
        self.get_logger().info(
            f"  Map: {self._map_cols}×{self._map_rows} cells @ "
            f"{self._map_resolution} m/cell  "
            f"({self._map_cols * self._map_resolution:.1f} m × "
            f"{self._map_rows * self._map_resolution:.1f} m)"
        )

    # ------------------------------------------------------------------
    # PX4 pose callback
    # ------------------------------------------------------------------
    def _trajectory_setpoint_callback(self, msg: TrajectorySetpoint) -> None:
        """Mirror the commanded pose published by whichever node is flying the drone."""
        self._pos_x = float(msg.position[0])
        self._pos_y = float(msg.position[1])
        self._yaw   = float(msg.yaw)

    # ------------------------------------------------------------------
    # Gazebo depth-camera callback
    # ------------------------------------------------------------------
    def _gz_depth_callback(self, msg: GzImage) -> None:
        """Process each incoming depth frame and update the occupancy map."""
        try:
            depth_data = np.frombuffer(msg.data, dtype=np.float32)
            depth_img  = depth_data.reshape((msg.height, msg.width))

            h, w = depth_img.shape

            # Use a narrow horizontal band around the centre 
            crop_half = max(1, int(h * 0.10))
            row_start = h // 2 - crop_half
            row_end   = h // 2 + crop_half
            band = depth_img[row_start:row_end, :]

            # Crop horizontal FOV edges 
            h_crop = int(band.shape[1] * 0.15)
            band = band[:, h_crop : band.shape[1] - h_crop]

            self._update_mapping(band)

        except Exception as exc:
            self.get_logger().error(f"Error processing depth image: {exc}")

    # ------------------------------------------------------------------
    # Mapping helpers
    # ------------------------------------------------------------------
    def _world_to_grid(self, wx: float, wy: float):
        """
        Convert a world (NED) position to (col, row) grid indices.
        Returns None if the position is outside the map bounds.
        """
        col = int(round(wy / self._map_resolution)) + self._map_origin_col
        row = int(round(wx / self._map_resolution)) + self._map_origin_row
        if 0 <= col < self._map_cols and 0 <= row < self._map_rows:
            return col, row
        return None

    def _update_mapping(self, cropped_depth: np.ndarray) -> None:
        """
        Raycast each depth column into the occupancy grid.

        For every pixel column in the cropped depth image:
          - Project the minimum depth along that column into world space.
          - Raycast all cells along the ray as *free*.
          - Mark the terminal cell as *occupied*.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Collapse band to 1-D depth array — median is robust to hot pixels
        # and lens-edge artifacts that would pull nanmin to a false close reading
        col_min = np.nanmedian(cropped_depth, axis=0)

        W = cropped_depth.shape[1]  # actual width after horizontal crop
        # Bearing for each column, offset so the cropped array still maps to
        # the correct sub-arc of the full HFOV (centre column → drone yaw)
        col_indices = np.arange(W, dtype=np.float64)
        h_crop      = (self._cam_width - W) // 2          # pixels removed each side
        full_col    = col_indices + h_crop                 # index in the original frame
        bearings    = self._yaw + (full_col - self._cam_width / 2.0) * (self._cam_hfov / self._cam_width)

        drone_cell = self._world_to_grid(self._pos_x, self._pos_y)
        if drone_cell is None:
            return
        d_col, d_row = drone_cell

        for c in range(W):
            depth = float(col_min[c])

            if not np.isfinite(depth) or depth < self._cam_near or depth > self._cam_far:
                continue

            obs_x = self._pos_x + depth * math.cos(bearings[c])
            obs_y = self._pos_y + depth * math.sin(bearings[c])
=======
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            px4_qos
        )
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            px4_qos
        )

        # =======================
        # ROS publishers
        # =======================
        self.odom_pub       = self.create_publisher(Odometry,    '/rtabmap/odom',               10)
        self.depth_pub      = self.create_publisher(Image,       '/camera/depth/image_raw',     10)
        self.depth_info_pub = self.create_publisher(CameraInfo,  '/camera/depth/camera_info',   10)
        self.rgb_pub        = self.create_publisher(Image,       '/camera/rgb/image_raw',       10)
        self.rgb_info_pub   = self.create_publisher(CameraInfo,  '/camera/rgb/camera_info',     10)

        # =======================
        # TF
        # =======================
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # =======================
        # Gazebo transport
        # =======================
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, '/depth_camera', self._depth_callback)
        self.gz_node.subscribe(
            GzImage,
            '/world/small_house/model/trace_drone_0/model/mono_cam'
            '/link/camera_link/sensor/camera/image',
            self._rgb_callback
        )

        # Odometry published at 20 Hz
        self.create_timer(0.05, self._publish_odom)

        self.get_logger().info('MappingNode started.')

    # =======================
    # PX4 callbacks
    # =======================

    def _position_callback(self, msg: VehicleLocalPosition):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.z = -float(msg.z)   # NED → ENU: negate z
        self.have_position = True

    def _attitude_callback(self, msg: VehicleAttitude):
        # NED → ENU quaternion conversion: swap x↔y, negate z
        self.q = [
             float(msg.q[0]),   # w unchanged
             float(msg.q[2]),   # NED y → ENU x
             float(msg.q[1]),   # NED x → ENU y
            -float(msg.q[3]),   # negate z
        ]
        self.have_attitude = True

    # =======================
    # Gazebo callbacks
    # =======================

    def _depth_callback(self, msg: GzImage):
        try:
            now = self.get_clock().now().to_msg()

            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                (msg.height, msg.width)
            )

            ros_img = Image()
            ros_img.header.stamp    = now
            ros_img.header.frame_id = 'camera_link'
            ros_img.height   = msg.height
            ros_img.width    = msg.width
            ros_img.encoding = '32FC1'
            ros_img.step     = msg.width * 4
            ros_img.data     = depth.tobytes()
            self.depth_pub.publish(ros_img)
>>>>>>> Stashed changes

            DEPTH_INFO.header.stamp = now
            self.depth_info_pub.publish(DEPTH_INFO)

<<<<<<< Updated upstream
            # Raycast: mark all cells before the obstacle as free
            ray_len = max(abs(obs_col - d_col), abs(obs_row - d_row))
            if ray_len < 1:
                continue

            sample_cols = np.round(
                np.linspace(d_col, obs_col, ray_len, endpoint=False)
            ).astype(int)
            sample_rows = np.round(
                np.linspace(d_row, obs_row, ray_len, endpoint=False)
            ).astype(int)

            valid = (
                (sample_cols >= 0) & (sample_cols < self._map_cols) &
                (sample_rows >= 0) & (sample_rows < self._map_rows)
=======
        except Exception as e:
            self.get_logger().error(f'Depth bridge error: {e}')

    def _rgb_callback(self, msg: GzImage):
        try:
            now = self.get_clock().now().to_msg()

            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
>>>>>>> Stashed changes
            )

<<<<<<< Updated upstream
            # Only mark free if not already confirmed occupied
            free_mask = self._occ_grid[fr, fc] != 100
            self._occ_grid[fr[free_mask], fc[free_mask]] = 0
            self._map_stamp[fr[free_mask], fc[free_mask]] = now_sec

            # Mark terminal cell as occupied
            self._occ_grid[obs_row, obs_col] = 100
            self._map_stamp[obs_row, obs_col] = now_sec

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_map(self) -> None:
        """Publish the occupancy grid and a colorized map image."""
        now = self.get_clock().now().to_msg()

        # ---- nav_msgs/OccupancyGrid ----------------------------------------
        og = OccupancyGrid()
        og.header.stamp    = now
        og.header.frame_id = "map"
        og.info.resolution = self._map_resolution
        og.info.width      = self._map_cols
        og.info.height     = self._map_rows
        og.info.origin.position.x = -(self._map_origin_col * self._map_resolution)
        og.info.origin.position.y = -(self._map_origin_row * self._map_resolution)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0
        og.data = self._occ_grid.flatten().tolist()
        self._map_pub.publish(og)

        # ---- sensor_msgs/Image (bgr8 rendered map) -------------------------
        render = self._render_map()
        img_msg = RosImage()
        img_msg.header.stamp    = now
        img_msg.header.frame_id = "map"
        img_msg.height   = render.shape[0]
        img_msg.width    = render.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.step     = render.shape[1] * 3
        img_msg.data     = render.tobytes()
        self._map_img_pub.publish(img_msg)

    def _render_map(self) -> np.ndarray:
        """
        Colorized top-down map rendering.

        Color scheme
        -------------
        Unknown  — dark grey  (40, 40, 40)
        Free     — grey, fades darker with age
        Occupied — red/orange, fades orange with age
        Drone    — bright green 3×3 square
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        age_max = 30.0  # seconds until a cell reaches its "oldest" color

        img = np.zeros((self._map_rows, self._map_cols, 3), dtype=np.uint8)

        # Unknown cells
        img[self._occ_grid == -1] = (40, 40, 40)

        # Free cells (grey, brightness decays with age)
        free_mask = self._occ_grid == 0
        if free_mask.any():
            age        = np.clip(
                (now_sec - self._map_stamp[free_mask]) / age_max, 0, 1
            )
            brightness = (200 - age * 80).astype(np.uint8)
            img[free_mask] = np.stack(
                [brightness, brightness, brightness], axis=-1
            )

        # Occupied cells (red, shifts toward orange with age)
        occ_mask = self._occ_grid == 100
        if occ_mask.any():
            age = np.clip(
                (now_sec - self._map_stamp[occ_mask]) / age_max, 0, 1
            )
            r = np.zeros(occ_mask.sum(), dtype=np.uint8)
            g = (age * 120).astype(np.uint8)
            b = np.full(occ_mask.sum(), 200, dtype=np.uint8)
            img[occ_mask] = np.stack([r, g, b], axis=-1)

        # Drone position (3×3 green square)
        dc = self._world_to_grid(self._pos_x, self._pos_y)
        if dc is not None:
            col, row = dc
            r0 = max(0, row - 1);  r1 = min(self._map_rows, row + 2)
            c0 = max(0, col - 1);  c1 = min(self._map_cols, col + 2)
            img[r0:r1, c0:c1] = (0, 255, 0)
=======
            ros_img = Image()
            ros_img.header.stamp    = now
            ros_img.header.frame_id = 'camera_link'
            ros_img.height   = msg.height
            ros_img.width    = msg.width
            ros_img.encoding = 'rgb8'
            ros_img.step     = msg.width * 3
            ros_img.data     = rgb.tobytes()
            self.rgb_pub.publish(ros_img)

            RGB_INFO.header.stamp = now
            self.rgb_info_pub.publish(RGB_INFO)

        except Exception as e:
            self.get_logger().error(f'RGB bridge error: {e}')

    # =======================
    # Odometry + TF
    # =======================
=======

        # =======================
        # PX4 subscribers
        # =======================
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            px4_qos
        )
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            px4_qos
        )

        # =======================
        # ROS publishers
        # =======================
        self.odom_pub       = self.create_publisher(Odometry,    '/rtabmap/odom',               10)
        self.depth_pub      = self.create_publisher(Image,       '/camera/depth/image_raw',     10)
        self.depth_info_pub = self.create_publisher(CameraInfo,  '/camera/depth/camera_info',   10)
        self.rgb_pub        = self.create_publisher(Image,       '/camera/rgb/image_raw',       10)
        self.rgb_info_pub   = self.create_publisher(CameraInfo,  '/camera/rgb/camera_info',     10)

        # =======================
        # TF
        # =======================
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # =======================
        # Gazebo transport
        # =======================
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, '/depth_camera', self._depth_callback)
        self.gz_node.subscribe(
            GzImage,
            '/world/small_house/model/trace_drone_0/model/mono_cam'
            '/link/camera_link/sensor/camera/image',
            self._rgb_callback
        )

        # Odometry published at 20 Hz
        self.create_timer(0.05, self._publish_odom)

        self.get_logger().info('MappingNode started.')

    # =======================
    # PX4 callbacks
    # =======================

    def _position_callback(self, msg: VehicleLocalPosition):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.z = -float(msg.z)   # NED → ENU: negate z
        self.have_position = True

    def _attitude_callback(self, msg: VehicleAttitude):
        # NED → ENU quaternion conversion: swap x↔y, negate z
        self.q = [
             float(msg.q[0]),   # w unchanged
             float(msg.q[2]),   # NED y → ENU x
             float(msg.q[1]),   # NED x → ENU y
            -float(msg.q[3]),   # negate z
        ]
        self.have_attitude = True

    # =======================
    # Gazebo callbacks
    # =======================

    def _depth_callback(self, msg: GzImage):
        try:
            now = self.get_clock().now().to_msg()

            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                (msg.height, msg.width)
            )

            ros_img = Image()
            ros_img.header.stamp    = now
            ros_img.header.frame_id = 'camera_link'
            ros_img.height   = msg.height
            ros_img.width    = msg.width
            ros_img.encoding = '32FC1'
            ros_img.step     = msg.width * 4
            ros_img.data     = depth.tobytes()
            self.depth_pub.publish(ros_img)

            DEPTH_INFO.header.stamp = now
            self.depth_info_pub.publish(DEPTH_INFO)

        except Exception as e:
            self.get_logger().error(f'Depth bridge error: {e}')

    def _rgb_callback(self, msg: GzImage):
        try:
            now = self.get_clock().now().to_msg()

            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )

            ros_img = Image()
            ros_img.header.stamp    = now
            ros_img.header.frame_id = 'camera_link'
            ros_img.height   = msg.height
            ros_img.width    = msg.width
            ros_img.encoding = 'rgb8'
            ros_img.step     = msg.width * 3
            ros_img.data     = rgb.tobytes()
            self.rgb_pub.publish(ros_img)

            RGB_INFO.header.stamp = now
            self.rgb_info_pub.publish(RGB_INFO)

        except Exception as e:
            self.get_logger().error(f'RGB bridge error: {e}')

    # =======================
    # Odometry + TF
    # =======================

    def _publish_static_tfs(self):
        """
        Publish once-only transforms:
          map → odom       : identity (no loop closure correction yet)
          base_link → camera_link : camera mount from model.sdf
                                    SDF pose .12 .03 .242 (NED x y z)
                                    → ENU: x=.03 y=.12 z=.242
        """
        now = self.get_clock().now().to_msg()
        tfs = []

        # map → odom
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # base_link → camera_link
        tc = TransformStamped()
        tc.header.stamp    = now
        tc.header.frame_id = 'base_link'
        tc.child_frame_id  = 'camera_link'
        tc.transform.translation.x = 0.03    # ENU x (NED y)
        tc.transform.translation.y = 0.12    # ENU y (NED x)
        tc.transform.translation.z = 0.242   # ENU z
        tc.transform.rotation.w = 1.0
        tfs.append(tc)

        self.static_tf_broadcaster.sendTransform(tfs)

    def _publish_odom(self):
        if not (self.have_position and self.have_attitude):
            return

        now = self.get_clock().now().to_msg()
        w, x, y, z = self.q

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = self.z
        odom.pose.pose.orientation.w = w
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        self.odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.w = w
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        self.tf_broadcaster.sendTransform(t)
>>>>>>> Stashed changes

    def _publish_static_tfs(self):
        """
        Publish once-only transforms:
          map → odom       : identity (no loop closure correction yet)
          base_link → camera_link : camera mount from model.sdf
                                    SDF pose .12 .03 .242 (NED x y z)
                                    → ENU: x=.03 y=.12 z=.242
        """
        now = self.get_clock().now().to_msg()
        tfs = []

        # map → odom
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # base_link → camera_link
        tc = TransformStamped()
        tc.header.stamp    = now
        tc.header.frame_id = 'base_link'
        tc.child_frame_id  = 'camera_link'
        tc.transform.translation.x = 0.03    # ENU x (NED y)
        tc.transform.translation.y = 0.12    # ENU y (NED x)
        tc.transform.translation.z = 0.242   # ENU z
        tc.transform.rotation.w = 1.0
        tfs.append(tc)

        self.static_tf_broadcaster.sendTransform(tfs)

    def _publish_odom(self):
        if not (self.have_position and self.have_attitude):
            return
>>>>>>> Stashed changes

        now = self.get_clock().now().to_msg()
        w, x, y, z = self.q

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = self.z
        odom.pose.pose.orientation.w = w
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        self.odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.w = w
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        self.tf_broadcaster.sendTransform(t)


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
<<<<<<< Updated upstream
<<<<<<< Updated upstream
        node.get_logger().info("MappingNode shutting down.")
=======
        node.get_logger().info('Keyboard interrupt — shutting down MappingNode.')
>>>>>>> Stashed changes
=======
        node.get_logger().info('Keyboard interrupt — shutting down MappingNode.')
>>>>>>> Stashed changes
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
