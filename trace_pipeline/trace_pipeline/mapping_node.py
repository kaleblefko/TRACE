#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as RosImage
from px4_msgs.msg import VehicleLocalPosition


class MappingNode(Node):
    def __init__(self):
        super().__init__('trace_mapping_node')

        # =======================
        # Map Parameters
        # =======================
        self.map_resolution = 0.05
        self.map_cols = 500
        self.map_rows = 500

        self.map_origin_col = self.map_cols // 2
        self.map_origin_row = self.map_rows // 2

        # -1 unknown, 0 free, 100 occupied
        self.occ_grid = np.full((self.map_rows, self.map_cols), -1, dtype=np.int8)

        # =======================
        # Depth Camera Parameters
        # =======================
        self.cam_hfov = 1.274
        self.cam_width = 640
        self.cam_near = 0.2
        self.cam_far = 19.1

        # =======================
        # Vehicle Pose (PX4 NED)
        # =======================
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.pose_valid = False

        # =======================
        # Publishers
        # =======================
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "/mapping/occupancy_grid",
            10
        )

        self.map_img_pub = self.create_publisher(
            RosImage,
            "/mapping/map_image",
            10
        )

        # =======================
        # PX4 Local Position
        # =======================
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.local_position_callback,
            10
        )

        # =======================
        # Depth Camera (Gazebo)
        # =======================
        self.depth_node = GzNode()
        self.depth_node.subscribe(
            GzImage,
            "/depth_camera",
            self.gz_depth_callback
        )

        # Publish map at 1 Hz
        self.timer = self.create_timer(1.0, self._publish_map)

        self.get_logger().info("SLAM node with raycasting started.")

    # ==========================================================
    # PX4 Pose Callback
    # ==========================================================
    def local_position_callback(self, msg: VehicleLocalPosition):

        if not msg.xy_valid:
            return

        if not (
            math.isfinite(msg.x) and
            math.isfinite(msg.y) and
            math.isfinite(msg.heading)
        ):
            return

        self.pos_x = float(msg.x)
        self.pos_y = float(msg.y)
        self.yaw = float(msg.heading)
        self.pose_valid = True

    # ==========================================================
    # Depth Callback
    # ==========================================================
    def gz_depth_callback(self, msg: GzImage):

        if not self.pose_valid:
            return

        depth_data = np.frombuffer(msg.data, dtype=np.float32)
        depth_img = depth_data.reshape((msg.height, msg.width))

        row_start = msg.height // 3
        row_end = 2 * msg.height // 3
        band = depth_img[row_start:row_end, :]

        self._update_mapping(band)

    # ==========================================================
    # Mapping Utilities
    # ==========================================================
    def _world_to_grid(self, wx, wy):

        col = int(round(wy / self.map_resolution)) + self.map_origin_col
        row = int(round(wx / self.map_resolution)) + self.map_origin_row

        if 0 <= col < self.map_cols and 0 <= row < self.map_rows:
            return col, row
        return None

    def _update_mapping(self, cropped_depth):

        col_min = np.nanmin(cropped_depth, axis=0)
        W = self.cam_width

        col_indices = np.arange(W, dtype=np.float64)
        bearings = self.yaw + (col_indices - W / 2.0) * (self.cam_hfov / W)

        drone_cell = self._world_to_grid(self.pos_x, self.pos_y)
        if drone_cell is None:
            return

        d_col, d_row = drone_cell

        for c in range(W):

            depth = float(col_min[c])

            if not np.isfinite(depth):
                continue
            if depth < self.cam_near or depth > self.cam_far:
                continue

            obs_x = self.pos_x + depth * math.cos(bearings[c])
            obs_y = self.pos_y + depth * math.sin(bearings[c])

            obs_cell = self._world_to_grid(obs_x, obs_y)
            if obs_cell is None:
                continue

            obs_col, obs_row = obs_cell

            # --- Raycast from drone to obstacle ---
            ray_length = max(abs(obs_col - d_col), abs(obs_row - d_row))

            if ray_length < 1:
                continue

            cols = np.round(
                np.linspace(d_col, obs_col, ray_length, endpoint=False)
            ).astype(int)

            rows = np.round(
                np.linspace(d_row, obs_row, ray_length, endpoint=False)
            ).astype(int)

            valid = (
                (cols >= 0) & (cols < self.map_cols) &
                (rows >= 0) & (rows < self.map_rows)
            )

            free_cols = cols[valid]
            free_rows = rows[valid]

            # mark free cells
            free_mask = self.occ_grid[free_rows, free_cols] != 100
            self.occ_grid[free_rows[free_mask], free_cols[free_mask]] = 0

            # mark obstacle cell
            self.occ_grid[obs_row, obs_col] = 100

    # ==========================================================
    # Publish Map
    # ==========================================================
    def _publish_map(self):

        now = self.get_clock().now().to_msg()

        og = OccupancyGrid()
        og.header.stamp = now
        og.header.frame_id = "map"

        og.info.resolution = self.map_resolution
        og.info.width = self.map_cols
        og.info.height = self.map_rows

        og.info.origin.position.x = -(self.map_origin_col * self.map_resolution)
        og.info.origin.position.y = -(self.map_origin_row * self.map_resolution)
        og.info.origin.orientation.w = 1.0

        og.data = self.occ_grid.flatten().tolist()

        self.map_pub.publish(og)

        render = self._render_map()

        img_msg = RosImage()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "map"
        img_msg.height = render.shape[0]
        img_msg.width = render.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.step = render.shape[1] * 3
        img_msg.data = render.tobytes()

        self.map_img_pub.publish(img_msg)

    # ==========================================================
    # Debug Rendering
    # ==========================================================
    def _render_map(self):

        img = np.zeros((self.map_rows, self.map_cols, 3), dtype=np.uint8)

        img[self.occ_grid == -1] = (40, 40, 40)
        img[self.occ_grid == 0] = (180, 180, 180)
        img[self.occ_grid == 100] = (0, 0, 255)

        drone_cell = self._world_to_grid(self.pos_x, self.pos_y)
        if drone_cell is not None:
            col, row = drone_cell
            img[row-1:row+2, col-1:col+2] = (0, 255, 0)

        return img


def main(args=None):

    rclpy.init(args=args)
    node = MappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()