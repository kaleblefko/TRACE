#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from px4_msgs.msg import VehicleLocalPosition


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        # =======================
        # PX4 QoS
        # =======================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =======================
        # Pose
        # =======================
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_position_callback,
            qos_profile
        )

        # =======================
        # Depth camera
        # =======================
        # Depth camera params 
        self.cam_hfov  = 1.274   # radians
        self.cam_width = 640
        self.cam_near  = 0.2     # metres
        self.cam_far   = 19.1    # metres

        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, "/depth_camera", self._depth_callback)

        # =======================
        # Occupancy map
        # =======================
        self.map_resolution = 0.05   # metres per cell
        self.map_cols       = 500    # 25 m wide
        self.map_rows       = 500    # 25 m tall
        self.map_origin_col = self.map_cols // 2
        self.map_origin_row = self.map_rows // 2

        # -1 = unknown, 0 = free, 100 = occupied
        self.occ_grid  = np.full((self.map_rows, self.map_cols), -1, dtype=np.int8)
        self.map_stamp = np.zeros((self.map_rows, self.map_cols), dtype=np.float64)

        # =======================
        # Publishers
        # =======================
        self.map_pub     = self.create_publisher(OccupancyGrid, "/slam/occupancy_grid", 10)
        self.map_img_pub = self.create_publisher(RosImage, "/slam/map_image", 10)

        # Publish at 1 Hz independently of depth frame rate
        self.create_timer(1.0, self._publish_map)

        self.get_logger().info("MappingNode started.")

    # =======================
    # Pose callback
    # =======================
    def _local_position_callback(self, msg: VehicleLocalPosition):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.yaw   = msg.heading   # radians, NED convention

    # =======================
    # Depth callback
    # =======================
    def _depth_callback(self, msg: GzImage):
        try:
            depth_data = np.frombuffer(msg.data, dtype=np.float32)
            depth_img  = depth_data.reshape((msg.height, msg.width))

            # Crop to middle third vertically to exclude sky and ground
            h = depth_img.shape[0]
            band = depth_img[h // 3 : 2 * h // 3, :]

            self._update_mapping(band)
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    # =======================
    # Mapping helpers
    # =======================

    def _world_to_grid(self, wx, wy):
        """Convert NED world position to (col, row). Returns None if out of bounds."""
        col = int(round(wy / self.map_resolution)) + self.map_origin_col
        row = int(round(wx / self.map_resolution)) + self.map_origin_row
        if 0 <= col < self.map_cols and 0 <= row < self.map_rows:
            return col, row
        return None

    def _update_mapping(self, cropped_depth: np.ndarray):
        """Column-minimum projection + ray-cast update of the occupancy grid."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Collapse cropped image to 1-D: closest depth per bearing
        col_min = np.nanmin(cropped_depth, axis=0)   # shape: (cam_width,)

        W = self.cam_width
        col_indices = np.arange(W, dtype=np.float64)
        bearings = self.yaw + (col_indices - W / 2.0) * (self.cam_hfov / W)

        drone_cell = self._world_to_grid(self.pos_x, self.pos_y)
        if drone_cell is None:
            return
        d_col, d_row = drone_cell

        for c in range(W):
            depth = float(col_min[c])

            if not np.isfinite(depth) or depth < self.cam_near or depth > self.cam_far:
                continue

            obs_x = self.pos_x + depth * math.cos(bearings[c])
            obs_y = self.pos_y + depth * math.sin(bearings[c])

            obs_cell = self._world_to_grid(obs_x, obs_y)
            if obs_cell is None:
                continue
            obs_col, obs_row = obs_cell

            # Ray-cast cells along the ray as FREE (excluding endpoint)
            ray_len = max(abs(obs_col - d_col), abs(obs_row - d_row))
            if ray_len < 1:
                continue

            sample_cols = np.round(np.linspace(d_col, obs_col, ray_len, endpoint=False)).astype(int)
            sample_rows = np.round(np.linspace(d_row, obs_row, ray_len, endpoint=False)).astype(int)

            valid = (
                (sample_cols >= 0) & (sample_cols < self.map_cols) &
                (sample_rows >= 0) & (sample_rows < self.map_rows)
            )
            fc, fr = sample_cols[valid], sample_rows[valid]

            # Don't overwrite confirmed occupied cells with free
            free_mask = self.occ_grid[fr, fc] != 100
            self.occ_grid[fr[free_mask], fc[free_mask]] = 0
            self.map_stamp[fr[free_mask], fc[free_mask]] = now_sec

            # Mark obstacle
            self.occ_grid[obs_row, obs_col] = 100
            self.map_stamp[obs_row, obs_col] = now_sec

    def _publish_map(self):
        """Publish occupancy grid and rendered map image at 1 Hz."""
        now = self.get_clock().now().to_msg()

        # nav_msgs/OccupancyGrid → /slam/occupancy_grid
        og = OccupancyGrid()
        og.header.stamp    = now
        og.header.frame_id = "map"
        og.info.resolution = self.map_resolution
        og.info.width      = self.map_cols
        og.info.height     = self.map_rows
        og.info.origin.position.x = -(self.map_origin_col * self.map_resolution)
        og.info.origin.position.y = -(self.map_origin_row * self.map_resolution)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0
        og.data = self.occ_grid.flatten().tolist()
        self.map_pub.publish(og)

        # sensor_msgs/Image → /slam/map_image
        render = self._render_map()
        img_msg = RosImage()
        img_msg.header.stamp    = now
        img_msg.header.frame_id = "map"
        img_msg.height   = render.shape[0]
        img_msg.width    = render.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.step     = render.shape[1] * 3
        img_msg.data     = render.tobytes()
        self.map_img_pub.publish(img_msg)

    def _render_map(self) -> np.ndarray:
        """
        Renders a colourised top-down map image.

        unknown  → dark grey
        free     → grey, fades darker as reading ages (up to 30 s)
        occupied → red, fades toward orange as reading ages
        drone    → green dot
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        age_max  = 30.0

        img = np.zeros((self.map_rows, self.map_cols, 3), dtype=np.uint8)
        img[self.occ_grid == -1] = (40, 40, 40)

        free_mask = self.occ_grid == 0
        if free_mask.any():
            age = np.clip((now_sec - self.map_stamp[free_mask]) / age_max, 0, 1)
            brightness = (200 - age * 80).astype(np.uint8)
            img[free_mask] = np.stack([brightness, brightness, brightness], axis=-1)

        occ_mask = self.occ_grid == 100
        if occ_mask.any():
            age = np.clip((now_sec - self.map_stamp[occ_mask]) / age_max, 0, 1)
            r = np.zeros(occ_mask.sum(), dtype=np.uint8)
            g = (age * 120).astype(np.uint8)
            b = np.full(occ_mask.sum(), 200, dtype=np.uint8)
            img[occ_mask] = np.stack([r, g, b], axis=-1)

        dc = self._world_to_grid(self.pos_x, self.pos_y)
        if dc is not None:
            col, row = dc
            r0, r1 = max(0, row - 1), min(self.map_rows, row + 2)
            c0, c1 = max(0, col - 1), min(self.map_cols, col + 2)
            img[r0:r1, c0:c1] = (0, 255, 0)

        return img


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down MappingNode.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()