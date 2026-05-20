#!/usr/bin/env python3
"""
object_detector.py
==================
ROS 2 node that subscribes to the Gazebo RGB camera and depth camera,
sends RGB frames to Qwen3-VL via Ollama for object detection, and publishes
the bounding-box result together with the 3-D metric coordinates of the
detected object's centre back-projected from the depth frame.

Subscriptions
─────────────
  /world/small_house/model/trace_drone_0/model/depth_cam/link/camera_link/sensor/IMX214/image
                           gz-transport (GzImage, RGB_INT8) — sent to VLM
  /depth_camera            gz-transport (GzImage, float32 metres per pixel) — back-projection only

Publications
────────────
  /detection/image         sensor_msgs/Image (bgr8) — annotated RGB frame
  /detection/bbox          std_msgs/String   — JSON {
                                                  "detected": bool,
                                                  "label":    str,
                                                  "x1": int, "y1": int,
                                                  "x2": int, "y2": int,
                                                  "cx_px": int, "cy_px": int,
                                                  "depth_m": float,
                                                  "x_m": float,   # cam-frame right
                                                  "y_m": float,   # cam-frame down
                                                  "z_m": float    # cam-frame forward
                                               }
"""

import base64
import json
import math
import os
import re
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests
import rclpy
from dotenv import load_dotenv
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode


# ──────────────────────────────────────────────────────────────────────────────
# Camera intrinsics (must match occupancy_grid_mapper.py)
# ──────────────────────────────────────────────────────────────────────────────
CAM_FX: float = 429.3
CAM_FY: float = 429.3
CAM_CX: float = 320.0
CAM_CY: float = 240.0
CAM_W:  int   = 640
CAM_H:  int   = 480
CAM_NEAR: float = 0.2
CAM_FAR:  float = 19.1

# Depth normalisation range (for visualiser fallback only)
VIS_NEAR: float = 0.2
VIS_FAR:  float = 10.0

# gz-transport topic for the IMX214 RGB sensor
RGB_TOPIC = (
    '/world/small_house/model/trace_drone_0'
    '/model/depth_cam/link/camera_link/sensor/IMX214/image'
)
DEPTH_TOPIC = '/depth_camera'

DEFAULT_TARGET_LABEL: str = "blue ball"


def _build_system_prompt(label: str) -> str:
    return f"""\
You are a visual object-detection assistant for an autonomous drone.
You will receive a single RGB camera image.
Your task is to locate the {label} in the image.

Respond with ONLY a single JSON object — no markdown, no prose, no fences.
All coordinates are NORMALIZED between 0.0 and 1.0 as a fraction of image width/height.
0.0 is left/top edge, 1.0 is right/bottom edge.

Schema:
{{
  "detected": <true|false>,
  "label": "{label}",
  "x1": <float 0.0-1.0, left edge>,
  "y1": <float 0.0-1.0, top edge>,
  "x2": <float 0.0-1.0, right edge>,
  "y2": <float 0.0-1.0, bottom edge>
}}

If the {label} is NOT visible set detected=false and all coordinates to 0.0.
Do NOT return pixel values. Return ONLY normalized 0.0-1.0 fractions.
"""


# ──────────────────────────────────────────────────────────────────────────────
class ObjectDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__('object_detector')

        # ── Load environment ─────────────────────────────────────────────────
        env_path = os.path.expanduser('~/TRACE/.env')
        load_dotenv(dotenv_path=env_path)

        self._ollama_endpoint: str = os.getenv('OLLAMA_ENDPOINT', 'localhost:11434')
        self._ollama_model:    str = os.getenv('OLLAMA_MODEL',    'gemma3:4b')
        self._api_url: str = f"http://{self._ollama_endpoint}/api/chat"

        # ── Shared state ─────────────────────────────────────────────────────
        self._latest_rgb:   Optional[np.ndarray] = None   # (H, W, 3) uint8 BGR
        self._latest_depth: Optional[np.ndarray] = None   # (H, W)    float32

        self._rgb_lock   = threading.Lock()
        self._depth_lock = threading.Lock()

        self._is_processing = False
        self._last_result: Optional[dict] = None

        # ── Target label (updated via /mission/new_target) ───────────────────
        self._target_label: str = DEFAULT_TARGET_LABEL
        self._target_lock = threading.Lock()

        # ── ROS 2 publishers ─────────────────────────────────────────────────
        self._pub_image = self.create_publisher(RosImage, '/detection/image', 10)
        self._pub_bbox  = self.create_publisher(String,   '/detection/bbox',  10)

        # ── ROS 2 subscriptions ───────────────────────────────────────────────
        self.create_subscription(String, '/mission/new_target', self._new_target_cb, 10)

        # ── gz-transport subscriptions ────────────────────────────────────────
        self.declare_parameter('gz_world_name', 'small_house')
        gz_world = self.get_parameter('gz_world_name').get_parameter_value().string_value
        rgb_topic = (
            f'/world/{gz_world}/model/trace_drone_0'
            f'/model/depth_cam/link/camera_link/sensor/IMX214/image'
        )
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, rgb_topic,   self._rgb_cb)
        self._gz_node.subscribe(GzImage, DEPTH_TOPIC, self._depth_cb)

        # ── 5 Hz inference trigger ───────────────────────────────────────────
        self.create_timer(0.2, self._inference_trigger_cb)

        self.get_logger().info(
            f'ObjectDetectorNode ready — target: "{self._target_label}" | '
            f'model: {self._ollama_model} | endpoint: {self._ollama_endpoint}'
        )

    # ─────────────────────  target update  ──────────────────────────────────

    def _new_target_cb(self, msg: String) -> None:
        label = msg.data.strip()
        if label:
            with self._target_lock:
                self._target_label = label
            self.get_logger().info(f'Detection target updated to: "{label}"')

    # ─────────────────────  gz callbacks  ────────────────────────────────────

    def _rgb_cb(self, msg: GzImage) -> None:
        """Decode RGB_INT8 gz image → BGR numpy array."""
        expected = msg.width * msg.height * 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != expected:
            return
        # Gazebo publishes RGB; OpenCV uses BGR
        rgb = raw.reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        with self._rgb_lock:
            self._latest_rgb = bgr

    def _depth_cb(self, msg: GzImage) -> None:
        """Buffer float32 depth frame for back-projection."""
        expected = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size != expected:
            return
        with self._depth_lock:
            self._latest_depth = raw.reshape(msg.height, msg.width).copy()

    # ─────────────────────  inference trigger  ───────────────────────────────

    def _inference_trigger_cb(self) -> None:
        if self._is_processing:
            return

        with self._rgb_lock:
            if self._latest_rgb is None:
                return
            rgb_snapshot = self._latest_rgb.copy()

        with self._depth_lock:
            depth_snapshot = self._latest_depth.copy() if self._latest_depth is not None else None

        self._is_processing = True
        threading.Thread(
            target=self._run_inference,
            args=(rgb_snapshot, depth_snapshot),
            daemon=True,
        ).start()

    # ─────────────────────  inference thread  ────────────────────────────────

    def _run_inference(self, bgr: np.ndarray, depth: Optional[np.ndarray]) -> None:
        try:
            with self._target_lock:
                target = self._target_label

            # Resize full-res frame (e.g. 1920x1080) down to CAM_W x CAM_H.
            # All bbox coords are in this space. Visualizer also displays at this size.
            bgr = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)

            success, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not success:
                self.get_logger().error('JPEG encode failed')
                return
            img_b64 = base64.b64encode(buf).decode('utf-8')

            # ── 2. Call Ollama ───────────────────────────────────────────────
            payload = {
                'model': self._ollama_model,
                'stream': True,
                'messages': [
                    {'role': 'system', 'content': _build_system_prompt(target)},
                    {
                        'role': 'user',
                        'content': f'Detect the {target} in this image. Return normalized 0.0-1.0 coordinates, NOT pixels.',
                        'images': [img_b64],
                    },
                ],
            }

            self.get_logger().info(f'Sending RGB frame to {self._ollama_model}...')
            t_inf_start = time.perf_counter()
            response = requests.post(
                self._api_url, json=payload, stream=True, timeout=30
            )
            response.raise_for_status()

            # ── 3. Accumulate streamed response ─────────────────────────────
            full_reply = ''
            for line in response.iter_lines():
                if not line:
                    continue
                try:
                    chunk = json.loads(line)
                    full_reply += chunk.get('message', {}).get('content', '')
                    if chunk.get('done', False):
                        break
                except json.JSONDecodeError:
                    continue
            inference_ms = (time.perf_counter() - t_inf_start) * 1000.0

            self.get_logger().info(f'VLM raw reply: {full_reply[:200]}')

            # ── 4. Parse bounding box ────────────────────────────────────────
            bbox = self._parse_bbox(full_reply, target)

            # Convert normalized 0.0-1.0 coords to pixel space
            if bbox['detected']:
                bbox['x1'] = int(bbox['x1'] * CAM_W)
                bbox['y1'] = int(bbox['y1'] * CAM_H)
                bbox['x2'] = int(bbox['x2'] * CAM_W)
                bbox['y2'] = int(bbox['y2'] * CAM_H)

            # Reject degenerate/zero bbox
            if bbox['detected'] and (bbox['x2'] - bbox['x1'] < 4 or bbox['y2'] - bbox['y1'] < 4):
                self.get_logger().warn(
                    f'VLM returned detected=true but degenerate bbox '
                    f'({bbox["x1"]},{bbox["y1"]})-({bbox["x2"]},{bbox["y2"]}) — treating as not detected'
                )
                bbox = _not_detected(target)


            # ── 5. Back-project to 3-D using depth frame ─────────────────────
            if bbox['detected']:
                cx_px = int(np.clip((bbox['x1'] + bbox['x2']) // 2, 0, CAM_W - 1))
                cy_px = int(np.clip((bbox['y1'] + bbox['y2']) // 2, 0, CAM_H - 1))

                depth_m = float('nan')
                if depth is not None:
                    raw_d = float(depth[cy_px, cx_px])
                    if math.isfinite(raw_d) and CAM_NEAR <= raw_d <= CAM_FAR:
                        depth_m = raw_d

                if math.isfinite(depth_m):
                    x_m = (cx_px - CAM_CX) * depth_m / CAM_FX   # right
                    y_m = (cy_px - CAM_CY) * depth_m / CAM_FY   # down
                    z_m = depth_m                                  # forward
                else:
                    x_m = y_m = z_m = float('nan')

                bbox.update({
                    'cx_px':   cx_px,
                    'cy_px':   cy_px,
                    'depth_m': round(depth_m, 3) if math.isfinite(depth_m) else None,
                    'x_m':     round(x_m, 3)     if math.isfinite(x_m)     else None,
                    'y_m':     round(y_m, 3)      if math.isfinite(y_m)     else None,
                    'z_m':     round(z_m, 3)      if math.isfinite(z_m)     else None,
                })

                self.get_logger().info(
                    f'Detected "{target}" | '
                    f'bbox=({bbox["x1"]},{bbox["y1"]})-({bbox["x2"]},{bbox["y2"]}) | '
                    f'depth={bbox["depth_m"]} m | '
                    f'forward={bbox["z_m"]} m | right={bbox["x_m"]} m'
                )
            else:
                bbox.update({
                    'cx_px': 0, 'cy_px': 0,
                    'depth_m': None, 'x_m': None, 'y_m': None, 'z_m': None,
                })
                self.get_logger().info(f'"{target}" not detected in frame')

            bbox['inference_ms'] = round(inference_ms, 1)
            self._last_result = bbox

            # ── 6. Publish ───────────────────────────────────────────────────
            self._publish_bbox(bbox)
            self._publish_image(bgr, bbox)

        except requests.exceptions.Timeout:
            self.get_logger().warn('Ollama request timed out — skipping frame')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Ollama request error: {e}')
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}', throttle_duration_sec=5.0)
        finally:
            self._is_processing = False

    # ─────────────────────  bbox parser  ─────────────────────────────────────

    @staticmethod
    def _parse_bbox(text: str, target_label: str) -> dict:
        text = re.sub(r'```[a-zA-Z]*', '', text).strip('`').strip()
        match = re.search(r'\{.*?\}', text, re.DOTALL)
        if not match:
            return _not_detected(target_label)
        try:
            obj = json.loads(match.group())
        except json.JSONDecodeError:
            return _not_detected(target_label)

        if not {'detected', 'x1', 'y1', 'x2', 'y2'}.issubset(obj.keys()):
            return _not_detected(target_label)

        try:
            return {
                'detected': bool(obj['detected']),
                'label':    str(obj.get('label', target_label)),
                'x1': float(obj['x1']),
                'y1': float(obj['y1']),
                'x2': float(obj['x2']),
                'y2': float(obj['y2']),
            }
        except (ValueError, TypeError):
            return _not_detected(target_label)

    # ─────────────────────  publishers  ──────────────────────────────────────

    def _publish_bbox(self, bbox: dict) -> None:
        msg = String()
        msg.data = json.dumps(bbox)
        self._pub_bbox.publish(msg)

    def _publish_image(self, bgr: np.ndarray, bbox: dict) -> None:
        vis = bgr.copy()
        if bbox['detected']:
            x1, y1, x2, y2 = bbox['x1'], bbox['y1'], bbox['x2'], bbox['y2']
            depth_str = f" {bbox['depth_m']:.2f}m" if bbox.get('depth_m') is not None else ''
            text = f'{bbox.get("label", DEFAULT_TARGET_LABEL)}{depth_str}'
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(vis, text, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
            cx, cy = bbox.get('cx_px', (x1 + x2) // 2), bbox.get('cy_px', (y1 + y2) // 2)
            cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)
        else:
            label_str = bbox.get('label', DEFAULT_TARGET_LABEL)
            cv2.putText(vis, f'No {label_str} detected',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

        ros_img = RosImage()
        ros_img.header.stamp    = self.get_clock().now().to_msg()
        ros_img.header.frame_id = 'camera'
        ros_img.height          = vis.shape[0]
        ros_img.width           = vis.shape[1]
        ros_img.encoding        = 'bgr8'
        ros_img.is_bigendian    = 0
        ros_img.step            = vis.shape[1] * 3
        ros_img.data            = vis.tobytes()
        self._pub_image.publish(ros_img)


# ──────────────────────────────────────────────────────────────────────────────

def _not_detected(label: str = DEFAULT_TARGET_LABEL) -> dict:
    return {'detected': False, 'label': label,
            'x1': 0, 'y1': 0, 'x2': 0, 'y2': 0}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()