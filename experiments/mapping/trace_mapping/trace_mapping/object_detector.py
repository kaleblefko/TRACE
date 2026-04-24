#!/usr/bin/env python3
"""
object_detector.py
==================
ROS 2 node that subscribes to the Gazebo depth camera, converts the float32
depth frame into a false-colour RGB image, sends it to Qwen3-VL via Ollama,
and publishes the bounding-box result together with the 3-D metric coordinates
of the detected object's centre.

Subscriptions
─────────────
  /depth_camera          gz-transport (GzImage, float32 metres per pixel)

Publications
────────────
  /detection/image       sensor_msgs/Image  (bgr8) — annotated visualisation
  /detection/bbox        std_msgs/String    — JSON  {
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

Design notes
────────────
* Inference is *non-blocking* w.r.t. the ROS spin thread.
  A threading.Thread is spawned for each VLM call; `_is_processing` prevents
  stacking.  The 20 Hz depth callback just keeps the latest frame fresh.
* The depth frame is converted to a JET-colourmap BGR image before being
  base64-encoded and sent to the model.  This gives Qwen3-VL visible colour
  contrast between near/far surfaces, which helps localise the ball.
* Bounding-box parsing uses a simple regex that tolerates the model wrapping
  coordinates in markdown fences or extra prose.
* 3-D back-projection uses the same camera intrinsics as occupancy_grid_mapper.
"""

import base64
import json
import math
import os
import re
import threading
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
# Camera intrinsics  (must match occupancy_grid_mapper.py)
# ──────────────────────────────────────────────────────────────────────────────
CAM_FX: float = 429.3
CAM_FY: float = 429.3
CAM_CX: float = 320.0
CAM_CY: float = 240.0
CAM_W:  int   = 640
CAM_H:  int   = 480
CAM_NEAR: float = 0.2
CAM_FAR:  float = 19.1

# Depth normalisation range for false-colour encoding sent to the VLM
VIS_NEAR: float = 0.2    # metres — mapped to 0
VIS_FAR:  float = 10.0   # metres — mapped to 255

# Target object description sent to the model
TARGET_LABEL: str = "blue ball"

# ──────────────────────────────────────────────────────────────────────────────
# Prompt
# ──────────────────────────────────────────────────────────────────────────────
SYSTEM_PROMPT = f"""\
You are a visual object-detection assistant for an autonomous drone.
You will receive a single false-colour depth image (JET colourmap: blue=far,
red/yellow=close).  Your task is to locate the {TARGET_LABEL} in the image.

Respond with ONLY a single JSON object — no markdown, no prose, no fences.
Schema:
{{
  "detected": <true|false>,
  "label": "{TARGET_LABEL}",
  "x1": <int, left pixel>,
  "y1": <int, top pixel>,
  "x2": <int, right pixel>,
  "y2": <int, bottom pixel>
}}

If the {TARGET_LABEL} is NOT visible set detected=false and all coordinates to 0.
Image resolution is {CAM_W}×{CAM_H}.
"""


# ──────────────────────────────────────────────────────────────────────────────
class ObjectDetectorNode(Node):
    """
    Publishes bounding-box detections for a hardcoded target object by
    sending false-colour depth frames to Qwen3-VL via an Ollama endpoint.
    """

    def __init__(self) -> None:
        super().__init__('object_detector')

        # ── Load environment ─────────────────────────────────────────────────
        env_path = os.path.expanduser('~/TRACE/.env')
        load_dotenv(dotenv_path=env_path)

        self._ollama_endpoint: str = os.getenv('OLLAMA_ENDPOINT', 'localhost:11434')
        self._ollama_model:    str = os.getenv('OLLAMA_MODEL',    'qwen3-vl')
        self._api_url: str = f"http://{self._ollama_endpoint}/api/chat"

        # ── Latest depth frame (written by gz callback, read by infer thread) ─
        self._latest_depth: Optional[np.ndarray] = None
        self._depth_lock = threading.Lock()

        # ── Inference state ──────────────────────────────────────────────────
        self._is_processing = False   # True while a VLM call is in-flight
        self._last_result: Optional[dict] = None

        # ── ROS 2 publishers ─────────────────────────────────────────────────
        self._pub_image = self.create_publisher(RosImage, '/detection/image', 10)
        self._pub_bbox  = self.create_publisher(String,   '/detection/bbox',  10)

        # ── gz-transport depth subscriber ────────────────────────────────────
        self._gz_node = GzNode()
        self._gz_node.subscribe(GzImage, '/depth_camera', self._depth_cb)

        # ── 5 Hz timer: trigger inference when idle ──────────────────────────
        # We do NOT run inference inside the timer callback — we just decide
        # whether to spawn an inference thread.  This keeps the ROS executor
        # non-blocking regardless of how long Ollama takes.
        self.create_timer(0.2, self._inference_trigger_cb)

        self.get_logger().info(
            f'ObjectDetectorNode ready — target: "{TARGET_LABEL}" | '
            f'model: {self._ollama_model} | endpoint: {self._ollama_endpoint}'
        )

    # ─────────────────────  gz depth callback  ───────────────────────────────

    def _depth_cb(self, msg: GzImage) -> None:
        """
        Buffer the latest float32 depth frame.
        Called from the gz-transport thread — keep it fast.
        """
        expected = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size != expected:
            return
        frame = raw.reshape(msg.height, msg.width).copy()
        with self._depth_lock:
            self._latest_depth = frame

    # ─────────────────────  inference trigger  ───────────────────────────────

    def _inference_trigger_cb(self) -> None:
        """
        Called at 5 Hz by the ROS timer.
        Spawns an inference thread only when:
          1. No inference is currently running.
          2. A depth frame is available.
        """
        if self._is_processing:
            return

        with self._depth_lock:
            if self._latest_depth is None:
                return
            depth_snapshot = self._latest_depth.copy()

        self._is_processing = True
        t = threading.Thread(
            target=self._run_inference,
            args=(depth_snapshot,),
            daemon=True,
        )
        t.start()

    # ─────────────────────  depth → BGR visualisation  ───────────────────────

    @staticmethod
    def _depth_to_bgr(depth: np.ndarray) -> np.ndarray:
        """
        Convert a float32 depth array (metres) to a uint8 JET-colourmap BGR
        image suitable for sending to the VLM.

        Clamps to [VIS_NEAR, VIS_FAR], scales to [0, 255], applies COLORMAP_JET.
        NaN / Inf pixels are treated as VIS_FAR (distant / no return).
        """
        safe = np.where(np.isfinite(depth), depth, VIS_FAR)
        norm = np.clip((safe - VIS_NEAR) / (VIS_FAR - VIS_NEAR), 0.0, 1.0)
        grey = (norm * 255).astype(np.uint8)
        return cv2.applyColorMap(grey, cv2.COLORMAP_JET)

    # ─────────────────────  inference thread  ────────────────────────────────

    def _run_inference(self, depth: np.ndarray) -> None:
        """
        Full inference pipeline (runs in a daemon thread):
          1. Convert depth frame to false-colour BGR.
          2. JPEG-encode and base64-encode.
          3. POST to Ollama /api/chat.
          4. Parse the JSON bounding box from the response.
          5. Back-project centre pixel to 3-D camera-frame coordinates.
          6. Publish annotated image + bbox JSON to ROS topics.
        """
        try:
            bgr = self._depth_to_bgr(depth)

            # ── 1. Encode image ──────────────────────────────────────────────
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
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {
                        'role': 'user',
                        'content': f'Detect the {TARGET_LABEL} in this depth image.',
                        'images': [img_b64],
                    },
                ],
            }

            self.get_logger().info(f'Sending frame to {self._ollama_model}...')
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

            self.get_logger().info(f'VLM raw reply: {full_reply[:200]}')

            # ── 4. Parse bounding box ────────────────────────────────────────
            bbox = self._parse_bbox(full_reply)

            # ── 5. Back-project to 3-D if detected ──────────────────────────
            if bbox['detected']:
                cx_px = (bbox['x1'] + bbox['x2']) // 2
                cy_px = (bbox['y1'] + bbox['y2']) // 2

                # Clamp to valid pixel range
                cx_px = int(np.clip(cx_px, 0, CAM_W - 1))
                cy_px = int(np.clip(cy_px, 0, CAM_H - 1))

                raw_depth = float(depth[cy_px, cx_px])
                depth_m = raw_depth if (
                    math.isfinite(raw_depth) and CAM_NEAR <= raw_depth <= CAM_FAR
                ) else float('nan')

                # Camera-frame 3-D coordinates (right-down-forward convention)
                if math.isfinite(depth_m):
                    x_m = (cx_px - CAM_CX) * depth_m / CAM_FX   # right
                    y_m = (cy_px - CAM_CY) * depth_m / CAM_FY   # down
                    z_m = depth_m                                  # forward
                else:
                    x_m = y_m = z_m = float('nan')

                bbox.update({
                    'cx_px': cx_px,
                    'cy_px': cy_px,
                    'depth_m': round(depth_m, 3) if math.isfinite(depth_m) else None,
                    'x_m': round(x_m, 3) if math.isfinite(x_m) else None,
                    'y_m': round(y_m, 3) if math.isfinite(y_m) else None,
                    'z_m': round(z_m, 3) if math.isfinite(z_m) else None,
                })

                self.get_logger().info(
                    f'Detected "{TARGET_LABEL}" | '
                    f'bbox=({bbox["x1"]},{bbox["y1"]})-({bbox["x2"]},{bbox["y2"]}) | '
                    f'depth={bbox["depth_m"]} m | '
                    f'cam xyz=({bbox["x_m"]}, {bbox["y_m"]}, {bbox["z_m"]}) m'
                )
            else:
                bbox.update({'cx_px': 0, 'cy_px': 0,
                             'depth_m': None, 'x_m': None, 'y_m': None, 'z_m': None})
                self.get_logger().info(f'"{TARGET_LABEL}" not detected in frame')

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
            # Always clear the flag so the next trigger can fire
            self._is_processing = False

    # ─────────────────────  bbox parser  ─────────────────────────────────────

    @staticmethod
    def _parse_bbox(text: str) -> dict:
        """
        Extract a bounding-box JSON object from the model's reply.

        Strategy:
          1. Strip any markdown fences.
          2. Find the first '{...}' block and JSON-parse it.
          3. Validate the required keys; fall back to not-detected on failure.
        """
        # Remove ```json ... ``` fences if present
        text = re.sub(r'```[a-zA-Z]*', '', text).strip('`').strip()

        # Find the first JSON object
        match = re.search(r'\{.*?\}', text, re.DOTALL)
        if not match:
            return _not_detected()

        try:
            obj = json.loads(match.group())
        except json.JSONDecodeError:
            return _not_detected()

        required = {'detected', 'x1', 'y1', 'x2', 'y2'}
        if not required.issubset(obj.keys()):
            return _not_detected()

        # Normalise types
        try:
            return {
                'detected': bool(obj['detected']),
                'label':    str(obj.get('label', TARGET_LABEL)),
                'x1': int(obj['x1']),
                'y1': int(obj['y1']),
                'x2': int(obj['x2']),
                'y2': int(obj['y2']),
            }
        except (ValueError, TypeError):
            return _not_detected()

    # ─────────────────────  publishers  ──────────────────────────────────────

    def _publish_bbox(self, bbox: dict) -> None:
        msg = String()
        msg.data = json.dumps(bbox)
        self._pub_bbox.publish(msg)

    def _publish_image(self, bgr: np.ndarray, bbox: dict) -> None:
        """
        Overlay the bounding box and label on the false-colour image and
        publish as a sensor_msgs/Image (bgr8) for RViz.
        """
        vis = bgr.copy()

        if bbox['detected']:
            x1, y1, x2, y2 = bbox['x1'], bbox['y1'], bbox['x2'], bbox['y2']
            label = bbox.get('label', TARGET_LABEL)
            depth_str = (
                f" {bbox['depth_m']:.2f}m" if bbox.get('depth_m') is not None else ''
            )

            # Bounding box rectangle — bright green
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label background + text
            text = f'{label}{depth_str}'
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(vis, text, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

            # Centre crosshair
            cx, cy = bbox.get('cx_px', (x1 + x2) // 2), bbox.get('cy_px', (y1 + y2) // 2)
            cv2.drawMarker(vis, (cx, cy), (0, 255, 0),
                           cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA)
        else:
            cv2.putText(vis, f'No {TARGET_LABEL} detected',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

        # Build RosImage without cv_bridge (same approach as occupancy_grid_mapper)
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
# Module-level helper
# ──────────────────────────────────────────────────────────────────────────────

def _not_detected() -> dict:
    return {
        'detected': False,
        'label': TARGET_LABEL,
        'x1': 0, 'y1': 0, 'x2': 0, 'y2': 0,
    }


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

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