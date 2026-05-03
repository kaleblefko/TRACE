#!/usr/bin/env python3
"""
detection_visualizer.py
=======================
ROS 2 node that displays the depth camera feed with bounding boxes overlaid,
synced to inference response timings from the object_detector node.

Subscriptions
─────────────
  /depth_camera        gz-transport (GzImage, float32 metres per pixel)
  /detection/bbox      std_msgs/String  — JSON bbox from object_detector

Design notes
────────────
* Depth frames are displayed continuously at camera rate using a JET colormap,
  giving a live feed regardless of inference speed.
* The latest bbox is held and re-drawn on every frame until a new inference
  result arrives — so the box is always visible and never flickers off between
  slow inference calls.
* A status bar shows: detection state, depth at centre, body-frame x/y,
  and time since last inference result.
* OpenCV window is driven from the gz-transport callback thread directly —
  no ROS timer needed for display.
* Press Q or ESC in the OpenCV window to close.
"""

import json
import math
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
# Camera intrinsics (must match object_detector.py)
# ──────────────────────────────────────────────────────────────────────────────
CAM_W:   int   = 640
CAM_H:   int   = 480
VIS_NEAR: float = 0.2
VIS_FAR:  float = 10.0

TARGET_LABEL: str = "blue ball"

# ──────────────────────────────────────────────────────────────────────────────
# Display constants
# ──────────────────────────────────────────────────────────────────────────────
WIN_NAME        = "TRACE — Depth Detection"
BOX_COLOR       = (0, 255, 80)       # bright green
BOX_THICKNESS   = 2
LABEL_BG_COLOR  = (0, 255, 80)
LABEL_FG_COLOR  = (0, 0, 0)
NO_DET_COLOR    = (0, 60, 255)       # red
CROSS_COLOR     = (0, 255, 80)
STATUS_BG       = (15, 15, 15)
STATUS_HEIGHT   = 56                 # pixels for status bar at bottom
FONT            = cv2.FONT_HERSHEY_SIMPLEX
STALE_THRESH    = 5.0                # seconds before bbox is considered stale


# ──────────────────────────────────────────────────────────────────────────────
class DetectionVisualizerNode(Node):
    """
    Subscribes to the raw depth camera and the bbox topic from object_detector,
    and renders a live OpenCV window with bounding boxes synced to inference
    response timings.
    """

    def __init__(self) -> None:
        super().__init__('detection_visualizer')

        # ── Shared state ─────────────────────────────────────────────────────
        self._latest_depth: Optional[np.ndarray] = None
        self._depth_lock = threading.Lock()

        self._latest_bbox: Optional[dict] = None
        self._bbox_lock   = threading.Lock()
        self._last_inference_time: float = 0.0   # wall time of last bbox msg

        # ── gz-transport depth subscriber ────────────────────────────────────
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, '/depth_camera', self._depth_cb)

        # ── ROS 2 bbox subscriber ─────────────────────────────────────────────
        self.create_subscription(String, '/detection/bbox', self._bbox_cb, 10)

        # ── OpenCV window ─────────────────────────────────────────────────────
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_NAME, CAM_W, CAM_H + STATUS_HEIGHT)

        # ── Display timer at ~30 Hz ───────────────────────────────────────────
        self.create_timer(1.0 / 30.0, self._display_cb)

        self.get_logger().info(
            f'DetectionVisualizerNode ready — '
            f'press Q or ESC in the OpenCV window to close'
        )

    # ─────────────────────  gz depth callback  ───────────────────────────────

    def _depth_cb(self, msg: GzImage) -> None:
        expected = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size != expected:
            return
        frame = raw.reshape(msg.height, msg.width).copy()
        with self._depth_lock:
            self._latest_depth = frame

    # ─────────────────────  bbox callback  ───────────────────────────────────

    def _bbox_cb(self, msg: String) -> None:
        """
        Called whenever object_detector publishes a new inference result.
        Parse and store; timestamp the arrival for staleness tracking.
        """
        try:
            bbox = json.loads(msg.data)
            with self._bbox_lock:
                self._latest_bbox = bbox
                self._last_inference_time = time.time()
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad bbox JSON: {e}')

    # ─────────────────────  depth → BGR  ─────────────────────────────────────

    @staticmethod
    def _depth_to_bgr(depth: np.ndarray) -> np.ndarray:
        safe = np.where(np.isfinite(depth), depth, VIS_FAR)
        norm = np.clip((safe - VIS_NEAR) / (VIS_FAR - VIS_NEAR), 0.0, 1.0)
        grey = (norm * 255).astype(np.uint8)
        return cv2.applyColorMap(grey, cv2.COLORMAP_JET)

    # ─────────────────────  overlay helpers  ─────────────────────────────────

    def _draw_bbox(self, canvas: np.ndarray, bbox: dict, stale: bool) -> None:
        """Draw bounding box, label, crosshair on canvas in-place."""
        if not bbox.get('detected', False):
            cv2.putText(
                canvas,
                f'No {TARGET_LABEL} detected',
                (12, 30), FONT, 0.65, NO_DET_COLOR, 2, cv2.LINE_AA,
            )
            return

        x1 = int(bbox.get('x1', 0))
        y1 = int(bbox.get('y1', 0))
        x2 = int(bbox.get('x2', 0))
        y2 = int(bbox.get('y2', 0))
        cx = int(bbox.get('cx_px', (x1 + x2) // 2))
        cy = int(bbox.get('cy_px', (y1 + y2) // 2))

        color = (100, 100, 100) if stale else BOX_COLOR

        # Bounding box
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, BOX_THICKNESS)

        # Crosshair at centre
        cv2.drawMarker(canvas, (cx, cy), color,
                       cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)

        # Label
        depth_m = bbox.get('depth_m')
        depth_str = f' {depth_m:.2f}m' if depth_m is not None else ''
        stale_str = ' [STALE]' if stale else ''
        label_text = f'{TARGET_LABEL}{depth_str}{stale_str}'

        (tw, th), baseline = cv2.getTextSize(label_text, FONT, 0.55, 1)
        lx1 = x1
        ly1 = max(y1 - th - 8, 0)
        lx2 = x1 + tw + 6
        ly2 = max(y1, th + 8)

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
        """
        Append a dark status bar below the camera image showing:
          - Detected / Not detected
          - depth, body-frame x (forward), y (right)
          - seconds since last inference result
        """
        bar = np.full((STATUS_HEIGHT, CAM_W, 3), STATUS_BG, dtype=np.uint8)

        # Left side — detection info
        if bbox is not None and bbox.get('detected'):
            depth_m  = bbox.get('depth_m')
            x_m      = bbox.get('x_m')    # cam right
            z_m      = bbox.get('z_m')    # cam forward = body forward

            body_fwd   = f'{z_m:.2f}m'   if z_m      is not None else 'N/A'
            body_right = f'{x_m:.2f}m'   if x_m      is not None else 'N/A'
            depth_str  = f'{depth_m:.2f}m' if depth_m is not None else 'N/A'

            cv2.putText(bar, 'DETECTED', (10, 20),
                        FONT, 0.55, BOX_COLOR, 1, cv2.LINE_AA)
            cv2.putText(
                bar,
                f'depth:{depth_str}  fwd:{body_fwd}  right:{body_right}',
                (10, 42), FONT, 0.48, (200, 200, 200), 1, cv2.LINE_AA,
            )
        else:
            cv2.putText(bar, 'NOT DETECTED', (10, 20),
                        FONT, 0.55, NO_DET_COLOR, 1, cv2.LINE_AA)
            cv2.putText(bar, f'target: {TARGET_LABEL}',
                        (10, 42), FONT, 0.48, (140, 140, 140), 1, cv2.LINE_AA)

        # Right side — inference timing
        if since_inference > 0:
            timing_str = f'last inference: {since_inference:.1f}s ago'
            (tw, _), _ = cv2.getTextSize(timing_str, FONT, 0.45, 1)
            cv2.putText(bar, timing_str, (CAM_W - tw - 10, 20),
                        FONT, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        return np.vstack([canvas, bar])

    # ─────────────────────  30 Hz display timer  ─────────────────────────────

    def _display_cb(self) -> None:
        """
        Render one frame into the OpenCV window.
        Runs at 30 Hz from the ROS timer — completely independent of inference
        rate, so the depth feed is always live.
        """
        with self._depth_lock:
            if self._latest_depth is None:
                return
            depth = self._latest_depth.copy()

        with self._bbox_lock:
            bbox  = self._latest_bbox
            since = time.time() - self._last_inference_time if self._last_inference_time > 0 else 0.0

        # Convert depth to false-colour BGR
        canvas = self._depth_to_bgr(depth)

        # Overlay bbox (greyed out if stale)
        if bbox is not None:
            stale = since > STALE_THRESH
            self._draw_bbox(canvas, bbox, stale)

        # Append status bar
        frame = self._draw_status_bar(canvas, bbox, since)

        cv2.imshow(WIN_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):   # Q or ESC
            self.get_logger().info('Visualizer window closed by user')
            cv2.destroyAllWindows()
            rclpy.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
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