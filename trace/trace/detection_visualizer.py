#!/usr/bin/env python3
"""
detection_visualizer.py
=======================
ROS 2 node that displays the RGB camera feed with bounding boxes overlaid,
synced to inference response timings from the object_detector node.

Subscriptions
─────────────
  /world/small_house/model/trace_drone_0/model/depth_cam/link/camera_link/sensor/IMX214/image
                           gz-transport (GzImage, RGB_INT8) — live display feed
  /detection/bbox          std_msgs/String — JSON bbox from object_detector

Design notes
────────────
* RGB frames are displayed continuously at camera rate, giving a live feed
  regardless of inference speed.
* The latest bbox is held and re-drawn on every frame until a new inference
  result arrives — the box never flickers off between slow inference calls.
* A status bar shows: detection state, depth at centre, body-frame x/y,
  and time since last inference result.
* Press Q or ESC in the OpenCV window to close.
"""

import json
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode


# ──────────────────────────────────────────────────────────────────────────────
# Camera / display constants
# ──────────────────────────────────────────────────────────────────────────────
CAM_W:  int = 640
CAM_H:  int = 480

RGB_TOPIC = (
    '/world/small_house/model/trace_drone_0'
    '/model/depth_cam/link/camera_link/sensor/IMX214/image'
)

DEFAULT_TARGET_LABEL: str = "blue ball"

WIN_NAME       = "TRACE — RGB Detection"
BOX_COLOR      = (0, 255, 80)
BOX_THICKNESS  = 2
LABEL_BG_COLOR = (0, 255, 80)
LABEL_FG_COLOR = (0, 0, 0)
NO_DET_COLOR   = (0, 60, 255)
STATUS_BG      = (15, 15, 15)
STATUS_HEIGHT  = 56
FONT           = cv2.FONT_HERSHEY_SIMPLEX
STALE_THRESH   = 5.0    # seconds before bbox is considered stale


# ──────────────────────────────────────────────────────────────────────────────
class DetectionVisualizerNode(Node):

    def __init__(self) -> None:
        super().__init__('detection_visualizer')

        # ── Shared state ─────────────────────────────────────────────────────
        self._latest_rgb: Optional[np.ndarray] = None   # (H, W, 3) BGR uint8
        self._rgb_lock = threading.Lock()

        self._latest_bbox: Optional[dict] = None
        self._bbox_lock = threading.Lock()
        self._last_inference_time: float = 0.0

        self._target_label: str = DEFAULT_TARGET_LABEL

        # ── gz-transport RGB subscriber ───────────────────────────────────────
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, RGB_TOPIC, self._rgb_cb)

        # ── ROS 2 subscribers ─────────────────────────────────────────────────
        self.create_subscription(String, '/detection/bbox',    self._bbox_cb,       10)
        self.create_subscription(String, '/mission/new_target', self._new_target_cb, 10)

        # ── OpenCV window ─────────────────────────────────────────────────────
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, CAM_W, CAM_H + STATUS_HEIGHT)

        # ── 30 Hz display timer ───────────────────────────────────────────────
        self.create_timer(1.0 / 30.0, self._display_cb)

        self.get_logger().info(
            'DetectionVisualizerNode ready — press Q or ESC to close'
        )

    # ─────────────────────  target update  ──────────────────────────────────

    def _new_target_cb(self, msg: String) -> None:
        label = msg.data.strip()
        if label:
            self._target_label = label

    # ─────────────────────  gz RGB callback  ─────────────────────────────────

    def _rgb_cb(self, msg: GzImage) -> None:
        expected = msg.width * msg.height * 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != expected:
            return
        rgb = raw.reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        bgr = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
        with self._rgb_lock:
            self._latest_rgb = bgr

    # ─────────────────────  bbox callback  ───────────────────────────────────

    def _bbox_cb(self, msg: String) -> None:
        try:
            bbox = json.loads(msg.data)
            with self._bbox_lock:
                self._latest_bbox = bbox
                self._last_inference_time = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad bbox JSON: {e}')

    # ─────────────────────  overlay helpers  ─────────────────────────────────

    def _draw_bbox(self, canvas: np.ndarray, bbox: dict, stale: bool) -> None:
        # bbox['label'] is always stamped with the detector's current target.
        label = bbox.get('label', self._target_label)
        if not bbox.get('detected', False):
            cv2.putText(canvas, f'No {label} detected',
                        (12, 30), FONT, 0.65, NO_DET_COLOR, 2, cv2.LINE_AA)
            return

        x1 = int(bbox.get('x1', 0))
        y1 = int(bbox.get('y1', 0))
        x2 = int(bbox.get('x2', 0))
        y2 = int(bbox.get('y2', 0))
        cx = int(bbox.get('cx_px', (x1 + x2) // 2))
        cy = int(bbox.get('cy_px', (y1 + y2) // 2))

        color = (100, 100, 100) if stale else BOX_COLOR

        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, BOX_THICKNESS)
        cv2.drawMarker(canvas, (cx, cy), color, cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)

        depth_m = bbox.get('depth_m')
        depth_str = f' {depth_m:.2f}m' if depth_m is not None else ''
        stale_str = ' [STALE]' if stale else ''
        label_text = f'{label}{depth_str}{stale_str}'

        (tw, th), _ = cv2.getTextSize(label_text, FONT, 0.55, 1)
        lx1, ly1 = x1, max(y1 - th - 8, 0)
        lx2, ly2 = x1 + tw + 6, max(y1, th + 8)

        if not stale:
            cv2.rectangle(canvas, (lx1, ly1), (lx2, ly2), LABEL_BG_COLOR, -1)
            cv2.putText(canvas, label_text, (lx1 + 3, ly2 - 4),
                        FONT, 0.55, LABEL_FG_COLOR, 1, cv2.LINE_AA)
        else:
            cv2.putText(canvas, label_text, (lx1 + 3, ly2 - 4),
                        FONT, 0.55, color, 1, cv2.LINE_AA)

    def _draw_status_bar(
        self,
        canvas: np.ndarray,
        bbox: Optional[dict],
        since_inference: float,
    ) -> np.ndarray:
        frame_w = canvas.shape[1]
        bar = np.full((STATUS_HEIGHT, frame_w, 3), STATUS_BG, dtype=np.uint8)

        if bbox is not None and bbox.get('detected'):
            depth_m = bbox.get('depth_m')
            x_m     = bbox.get('x_m')
            z_m     = bbox.get('z_m')
            body_fwd   = f'{z_m:.2f}m'   if z_m     is not None else 'N/A'
            body_right = f'{x_m:.2f}m'   if x_m     is not None else 'N/A'
            depth_str  = f'{depth_m:.2f}m' if depth_m is not None else 'N/A'

            cv2.putText(bar, 'DETECTED', (10, 20), FONT, 0.55, BOX_COLOR, 1, cv2.LINE_AA)
            cv2.putText(bar,
                        f'depth:{depth_str}  fwd:{body_fwd}  right:{body_right}',
                        (10, 42), FONT, 0.48, (200, 200, 200), 1, cv2.LINE_AA)
        else:
            searching = bbox.get('label', self._target_label) if bbox is not None else self._target_label
            cv2.putText(bar, 'NOT DETECTED', (10, 20), FONT, 0.55, NO_DET_COLOR, 1, cv2.LINE_AA)
            cv2.putText(bar, f'searching: {searching}',
                        (10, 42), FONT, 0.48, (140, 140, 140), 1, cv2.LINE_AA)

        if since_inference > 0:
            timing_str = f'last inference: {since_inference:.1f}s ago'
            (tw, _), _ = cv2.getTextSize(timing_str, FONT, 0.45, 1)
            cv2.putText(bar, timing_str, (frame_w - tw - 10, 20),
                        FONT, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        return np.vstack([canvas, bar])

    # ─────────────────────  30 Hz display timer  ─────────────────────────────

    def _display_cb(self) -> None:
        with self._rgb_lock:
            if self._latest_rgb is None:
                return
            canvas = self._latest_rgb.copy()

        with self._bbox_lock:
            bbox  = self._latest_bbox
            since = (time.time() - self._last_inference_time
                     if self._last_inference_time > 0 else 0.0)

        if bbox is not None:
            self._draw_bbox(canvas, bbox, stale=(since > STALE_THRESH))

        frame = self._draw_status_bar(canvas, bbox, since)

        cv2.imshow(WIN_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            self.get_logger().info('Visualizer closed by user')
            cv2.destroyAllWindows()
            rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()