#!/usr/bin/env python3
"""
eval_pipeline.py
================
Testbed evaluation node — cycles through every target object in sequence,
sweeps the full 360° scan at each one, and collects COLLECT_FRAMES positive
inference frames per confirmed detection WITHOUT flying to the object.

State flow
──────────
  WAIT_NAV_READY  — wait for slam_nav HOLD (takeoff complete).
  TARGET_SWITCH   — publish new target label; pause so the detector updates.
  SCAN            — 8-quadrant yaw sweep (identical Phase A/B logic to
                    state_machine.py). On 3 consecutive positives → COLLECT.
                    On 3 consecutive negatives → next quadrant.
                    Full 360° without detection → log "not found", next target.
  COLLECT         — freeze at current yaw; record every new positive inference
                    frame's world NED coordinates until COLLECT_FRAMES gathered,
                    then advance to the next target.
  DONE            — all targets tested; log written to disk and stdout.

Log format (appended to LOG_PATH)
──────────────────────────────────
  <model-name>:
  <mean_ms>ms ± <std_ms>ms
  blue ball (GT: N=-4.222, E=-6.946):
    frame 1: N=X.XXX  E=Y.YYY
    frame 2: ...
  ...

Subscriptions
─────────────
  /detection/bbox                      std_msgs/String
  /nav/status                          std_msgs/String
  /fmu/out/vehicle_local_position_v1   px4_msgs/VehicleLocalPosition
  /fmu/out/vehicle_attitude_v1         px4_msgs/VehicleAttitude

Publications
────────────
  /nav/yaw_cmd         std_msgs/Float32
  /mission/new_target  std_msgs/String
  /mission/state       std_msgs/String
"""

import datetime
import json
import math
import os
import re
import statistics
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Float32, String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


# ── Ground-truth object list ──────────────────────────────────────────────────
# (label sent to detector, (GT_north_m, GT_east_m) in nav NED frame)
# GT converted from Gazebo (x=East, y=North) → NED (x=North, y=East).

# Canonical full set used for merged log output (always 5 objects in this order).
ALL_OBJECTS_CANONICAL: list[tuple[str, tuple[float, float]]] = [
    ("blue ball",       (-4.222, -6.946)),
    ("dark blue chair", ( 1.681,  6.067)),
    ("orange chair",    ( 4.104, -1.381)),
    ("refrigerator",    (-1.032,  8.703)),
    ("barbell",         (-0.033, -4.647)),
]

# Objects to sweep in this eval run.  Set to a subset to retest specific items.
OBJECTS: list[tuple[str, tuple[float, float]]] = [
    ("refrigerator",    (-1.032,  8.703)),
]

# ── Evaluation parameters ─────────────────────────────────────────────────────
COLLECT_FRAMES     = 5      # positive frames to record per confirmed detection
TARGET_SWITCH_SECS = 4.0   # seconds to wait after publishing new target label

# ── Scan / lock parameters (match state_machine.py) ──────────────────────────
SCAN_STEPS          = 8
SCAN_YAW_STEP       = 2.0 * math.pi / SCAN_STEPS
# Relaxed thresholds — Gazebo sim yaw controller has more residual oscillation
# than the physical drone; the drone still physically settles quickly.
YAW_LOCK_TOLERANCE  = math.radians(10.0)
YAW_RATE_THRESH     = math.radians(20.0)
YAW_STILL_TICKS     = 3
# Fast timeout: 45° rotation + settle takes <3 s; force-lock at 3.5 s so each
# quadrant doesn't waste 15 s if the ticks mechanism fails.
YAW_LOCK_TIMEOUT    = 3.5
REQUIRED_DETECTIONS = 3
REQUIRED_NEGATIVES  = 3
MIN_LOCK_DWELL_SECS = 2.0
INFERENCE_FRAME_BUDGET = 20
MIN_DEPTH_M         = 0.4
MAX_DEPTH_M         = 15.0

# ── Output ────────────────────────────────────────────────────────────────────
_LOG_DIR   = '/home/kimjammy/SrDesign/TRACE/test logs'
MODEL_NAME = os.getenv('OLLAMA_MODEL', 'unknown-model')
LOG_PATH   = os.path.join(_LOG_DIR, f'{MODEL_NAME}_eval_log.txt')

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def cam_to_body(x_m: float, z_m: float) -> tuple[float, float]:
    """Camera (right, fwd) → body FRD (fwd, right)."""
    return z_m, x_m


def _parse_existing_log(path: str) -> tuple[str, str, dict]:
    """Read an existing log file and extract previous results for merging.

    Returns (run_start_str, inference_line_str, {label: (gt_n, gt_e, frames_or_None)}).
    frames_or_None is None for FAILED, a list of (N,E) tuples for detected, or
    omitted from the dict if the entry was only (pending).
    """
    try:
        with open(path) as f:
            lines = f.readlines()
    except OSError:
        return ('', '', {})

    run_start = ''
    inf_line  = ''
    sections: dict = {}
    cur_label: str | None = None
    cur_gt: tuple[float, float] | None = None
    cur_frames: list[tuple[float, float]] = []
    cur_failed = False

    for raw in lines:
        line = raw.rstrip('\n')
        if line.startswith('run started:'):
            run_start = line[len('run started: '):]
            continue
        if line.startswith('inference time:'):
            inf_line = line
            continue
        m = re.match(r'^(.+?) \(GT: N=(-?\d+\.\d+), E=(-?\d+\.\d+)\):$', line)
        if m:
            if cur_label is not None:
                sections[cur_label] = (
                    cur_gt[0], cur_gt[1],
                    None if cur_failed else (cur_frames or None)
                )
            cur_label  = m.group(1)
            cur_gt     = (float(m.group(2)), float(m.group(3)))
            cur_frames = []
            cur_failed = False
            continue
        if cur_label is not None:
            s = line.strip()
            if 'FAILED' in s:
                cur_failed = True
            elif s.startswith('frame'):
                fm = re.match(r'frame \d+: N=(-?\d+\.\d+)\s+E=(-?\d+\.\d+)', s)
                if fm:
                    cur_frames.append((float(fm.group(1)), float(fm.group(2))))
            # '(pending)' → leave out of dict so merge treats it as missing

    if cur_label is not None:
        sections[cur_label] = (
            cur_gt[0], cur_gt[1],
            None if cur_failed else (cur_frames or None)
        )

    return run_start, inf_line, sections


# ─────────────────────────────────────────────────────────────────────────────

class EvalPipeline(Node):

    def __init__(self) -> None:
        super().__init__('eval_pipeline')

        # ── Sensor state ─────────────────────────────────────────────────────
        self._bbox: Optional[dict] = None
        self._bbox_stamp: float = 0.0
        self._bbox_lock = threading.Lock()

        self._nav_status: str = ''
        self._pos: Optional[tuple[float, float, float]] = None
        self._yaw: float = 0.0
        self._prev_yaw: Optional[float] = None
        self._prev_yaw_stamp: float = 0.0
        self._yaw_rate: float = 0.0

        # ── Scan bookkeeping ─────────────────────────────────────────────────
        self._scan_yaw_start: float = 0.0
        self._scan_idx: int = 0
        self._scan_target_yaw: float = 0.0
        self._scan_stop_entered: float = 0.0

        self._quadrant_locked: bool = False
        self._lock_time: float = 0.0
        self._still_ticks: int = 0
        self._detection_count: int = 0
        self._negative_count: int = 0
        self._frames_since_lock: int = 0
        self._last_detection_stamp: float = 0.0

        # ── Eval state ───────────────────────────────────────────────────────
        self._state: str = 'WAIT_NAV_READY'
        self._obj_idx: int = 0
        self._target_switch_time: float = 0.0

        # COLLECT state
        self._collect_yaw: float = 0.0
        self._collected_frames: list[tuple[float, float]] = []

        # Accumulated results: list of (label, gt_tuple, frames_or_None)
        # frames = list of (N, E) tuples when detected; None when FAILED.
        self._results: list[tuple[str, tuple[float, float], list | None]] = []
        self._all_inf_ms: list[float] = []
        self._run_start: str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_yaw    = self.create_publisher(Float32, '/nav/yaw_cmd',        10)
        self._pub_target = self.create_publisher(String,  '/mission/new_target', 10)
        self._pub_state  = self.create_publisher(String,  '/mission/state',      10)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String, '/detection/bbox', self._bbox_cb, 10)
        self.create_subscription(String, '/nav/status',     self._nav_cb,  10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude_v1',
            self._att_cb, PX4_QOS)

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._echo_state)

        self.get_logger().info(
            f'eval_pipeline ready — {len(OBJECTS)} objects to test. '
            f'Waiting for slam_nav HOLD.')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _bbox_cb(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._bbox_lock:
            self._bbox = obj
            self._bbox_stamp = time.time()
        if obj.get('inference_ms') is not None:
            self._all_inf_ms.append(float(obj['inference_ms']))

    def _nav_cb(self, msg: String) -> None:
        self._nav_status = msg.data

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (float(msg.x), float(msg.y), float(msg.z))

    def _att_cb(self, msg: VehicleAttitude) -> None:
        new_yaw = quat_to_yaw(
            float(msg.q[0]), float(msg.q[1]),
            float(msg.q[2]), float(msg.q[3]),
        )
        now = time.time()
        if self._prev_yaw is not None and now > self._prev_yaw_stamp:
            dt = now - self._prev_yaw_stamp
            if dt > 0.002:
                d = math.atan2(
                    math.sin(new_yaw - self._prev_yaw),
                    math.cos(new_yaw - self._prev_yaw),
                )
                alpha = min(1.0, dt / 0.3)
                self._yaw_rate = alpha * (d / dt) + (1.0 - alpha) * self._yaw_rate
        self._prev_yaw = new_yaw
        self._prev_yaw_stamp = now
        self._yaw = new_yaw

    def _echo_state(self) -> None:
        m = String()
        m.data = self._state
        self._pub_state.publish(m)

    # ── Publisher helpers ─────────────────────────────────────────────────────

    def _publish_yaw(self, yaw: float) -> None:
        m = Float32()
        m.data = float(yaw)
        self._pub_yaw.publish(m)

    def _release_yaw(self) -> None:
        m = Float32()
        m.data = float('nan')
        self._pub_yaw.publish(m)

    def _publish_target(self, label: str) -> None:
        m = String()
        m.data = label
        self._pub_target.publish(m)

    # ── Detection helpers ─────────────────────────────────────────────────────

    def _is_confident(self, bbox: dict) -> bool:
        if not bbox.get('detected', False):
            return False
        depth_m = bbox.get('depth_m')
        if depth_m is None or not (MIN_DEPTH_M <= float(depth_m) <= MAX_DEPTH_M):
            return False
        return bbox.get('x_m') is not None and bbox.get('z_m') is not None

    def _project_to_world(self, bbox: dict, yaw: float) -> Optional[tuple[float, float]]:
        if self._pos is None:
            return None
        body_fwd, body_right = cam_to_body(float(bbox['x_m']), float(bbox['z_m']))
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        d_n = body_fwd * cos_y - body_right * sin_y
        d_e = body_fwd * sin_y + body_right * cos_y
        return (self._pos[0] + d_n, self._pos[1] + d_e)

    # ── Scan helpers ──────────────────────────────────────────────────────────

    def _enter_scan_stop(self) -> None:
        target = self._scan_yaw_start + self._scan_idx * SCAN_YAW_STEP
        wrapped = math.atan2(math.sin(target), math.cos(target))
        self._scan_target_yaw = wrapped
        self._publish_yaw(wrapped)
        self._scan_stop_entered = time.time()
        self._quadrant_locked = False
        self._lock_time = 0.0
        self._still_ticks = 0
        self._detection_count = 0
        self._negative_count = 0
        self._frames_since_lock = 0
        self._last_detection_stamp = 0.0
        with self._bbox_lock:
            self._bbox = None
            self._bbox_stamp = 0.0
        self.get_logger().info(
            f'[{self._current_label()}] Quadrant {self._scan_idx + 1}/{SCAN_STEPS} — '
            f'rotating to {math.degrees(wrapped):+.1f}°')

    def _advance_quadrant(self) -> None:
        self.get_logger().info(
            f'[{self._current_label()}] Quadrant {self._scan_idx + 1} — '
            f'not found, advancing.')
        self._scan_idx += 1
        if self._scan_idx >= SCAN_STEPS:
            label, gt = OBJECTS[self._obj_idx]
            self.get_logger().warn(
                f'[{label}] Full 360° sweep — FAILED (no confirmed detection).')
            self._results.append((label, gt, None))   # None = explicit FAIL
            self._advance_target()
            return
        self._enter_scan_stop()

    def _current_label(self) -> str:
        if self._obj_idx < len(OBJECTS):
            return OBJECTS[self._obj_idx][0]
        return '(done)'

    # ── Target advancement ────────────────────────────────────────────────────

    def _advance_target(self) -> None:
        # Flush after every object so partial runs are saved on disk.
        self._write_log()
        self._obj_idx += 1
        if self._obj_idx >= len(OBJECTS):
            self.get_logger().info('All objects tested — final log written.')
            self._release_yaw()
            self._state = 'DONE'
            return
        label, _ = OBJECTS[self._obj_idx]
        self.get_logger().info(f'─── Next target: "{label}" ───')
        self._publish_target(label)
        self._target_switch_time = time.time()
        self._collected_frames = []
        self._state = 'TARGET_SWITCH'

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._state == 'WAIT_NAV_READY':
            self._do_wait()
        elif self._state == 'TARGET_SWITCH':
            self._do_target_switch()
        elif self._state == 'SCAN':
            self._do_scan()
        elif self._state == 'COLLECT':
            self._do_collect()

    # ── WAIT_NAV_READY ────────────────────────────────────────────────────────

    def _do_wait(self) -> None:
        if self._nav_status != 'HOLD' or self._pos is None:
            return
        label, _ = OBJECTS[0]
        self.get_logger().info(
            f'Nav ready — publishing first target "{label}" and starting scan.')
        self._publish_target(label)
        self._scan_yaw_start = self._yaw
        self._scan_idx = 0
        self._collected_frames = []
        self._enter_scan_stop()
        self._state = 'SCAN'

    # ── TARGET_SWITCH ─────────────────────────────────────────────────────────

    def _do_target_switch(self) -> None:
        # Keep republishing the label so the detector picks it up even if the
        # first message was dropped.
        label, _ = OBJECTS[self._obj_idx]
        self._publish_target(label)
        if time.time() - self._target_switch_time >= TARGET_SWITCH_SECS:
            self._scan_yaw_start = self._yaw
            self._scan_idx = 0
            self._enter_scan_stop()
            self._state = 'SCAN'

    # ── SCAN ──────────────────────────────────────────────────────────────────

    def _do_scan(self) -> None:
        now = time.time()
        self._publish_yaw(self._scan_target_yaw)

        # ── Phase A: wait for yaw lock ────────────────────────────────────────
        if not self._quadrant_locked:
            err = math.atan2(
                math.sin(self._scan_target_yaw - self._yaw),
                math.cos(self._scan_target_yaw - self._yaw),
            )
            settled = (abs(err) < YAW_LOCK_TOLERANCE
                       and abs(self._yaw_rate) < YAW_RATE_THRESH)
            if settled:
                self._still_ticks += 1
            else:
                self._still_ticks = 0

            # Diagnostic — log once per second so we can see why lock fails.
            elapsed = now - self._scan_stop_entered
            if int(elapsed) != int(elapsed - 0.1):
                self.get_logger().info(
                    f'  Phase A Q{self._scan_idx + 1}: '
                    f'err={math.degrees(err):+.1f}°  '
                    f'rate={math.degrees(self._yaw_rate):+.1f}°/s  '
                    f'ticks={self._still_ticks}/{YAW_STILL_TICKS}  '
                    f't={elapsed:.1f}s')

            if self._still_ticks >= YAW_STILL_TICKS:
                self._quadrant_locked = True
                self._lock_time = now
                with self._bbox_lock:
                    self._last_detection_stamp = self._bbox_stamp
                self.get_logger().info(
                    f'Quadrant {self._scan_idx + 1} LOCKED — collecting inferences.')
                return

            if elapsed > YAW_LOCK_TIMEOUT:
                self.get_logger().warn(
                    f'Quadrant {self._scan_idx + 1} — yaw lock timed out '
                    f'(err={math.degrees(err):+.1f}°  '
                    f'rate={math.degrees(self._yaw_rate):+.1f}°/s); force-locking.')
                self._quadrant_locked = True
                self._lock_time = now
                with self._bbox_lock:
                    self._last_detection_stamp = self._bbox_stamp
            return

        # ── Phase B: track detection streak ──────────────────────────────────
        with self._bbox_lock:
            bbox  = self._bbox
            stamp = self._bbox_stamp

        is_new = bbox is not None and stamp > self._last_detection_stamp
        if not is_new:
            return

        self._last_detection_stamp = stamp
        self._frames_since_lock += 1

        # Discard first post-lock frame (captured while rotating).
        if self._frames_since_lock == 1:
            return

        if self._is_confident(bbox):
            self._detection_count += 1
            self._negative_count = 0
            self.get_logger().info(
                f'Quadrant {self._scan_idx + 1} — positive '
                f'{self._detection_count}/{REQUIRED_DETECTIONS}')
            if self._detection_count >= REQUIRED_DETECTIONS:
                self.get_logger().info(
                    f'[{self._current_label()}] Confirmed — switching to COLLECT.')
                self._collect_yaw = self._scan_target_yaw
                self._collected_frames = []
                self._last_detection_stamp = stamp  # COLLECT picks up from here
                self._state = 'COLLECT'
        else:
            if now - self._lock_time < MIN_LOCK_DWELL_SECS:
                return
            self._negative_count += 1
            self._detection_count = 0
            self.get_logger().info(
                f'Quadrant {self._scan_idx + 1} — negative '
                f'{self._negative_count}/{REQUIRED_NEGATIVES}')
            if self._negative_count >= REQUIRED_NEGATIVES:
                self._advance_quadrant()
                return

        if self._frames_since_lock >= INFERENCE_FRAME_BUDGET:
            self._advance_quadrant()

    # ── COLLECT ───────────────────────────────────────────────────────────────

    def _do_collect(self) -> None:
        self._publish_yaw(self._collect_yaw)

        with self._bbox_lock:
            bbox  = self._bbox
            stamp = self._bbox_stamp

        is_new = bbox is not None and stamp > self._last_detection_stamp
        if not is_new:
            return
        self._last_detection_stamp = stamp

        if not self._is_confident(bbox):
            return  # skip negative frames during collection

        coords = self._project_to_world(bbox, self._collect_yaw)
        if coords is None:
            return

        n, e = coords
        self._collected_frames.append((n, e))
        idx = len(self._collected_frames)
        self.get_logger().info(
            f'  [{self._current_label()}] collected frame {idx}/{COLLECT_FRAMES}: '
            f'N={n:.3f}  E={e:.3f}')

        if idx >= COLLECT_FRAMES:
            label, gt = OBJECTS[self._obj_idx]
            self._results.append((label, gt, list(self._collected_frames)))
            self.get_logger().info(
                f'[{label}] {COLLECT_FRAMES} frames collected.')
            self._advance_target()

    # ── Log writer ────────────────────────────────────────────────────────────

    def _write_log(self) -> None:
        """Merge current results with any existing log and write the combined file.

        Uses ALL_OBJECTS_CANONICAL as the authoritative object order so partial
        retests (e.g. refrigerator-only) are folded into a complete log.
        """
        # Load any pre-existing results for this model.
        existing_run_start, existing_inf_line, existing = _parse_existing_log(LOG_PATH)

        # Current run overrides existing entries for the same label.
        merged: dict = dict(existing)
        for label, (gt_n, gt_e), frames in self._results:
            merged[label] = (gt_n, gt_e, frames)

        lines: list[str] = []
        lines.append(f'{MODEL_NAME}:')
        lines.append(f'run started: {existing_run_start or self._run_start}')

        total     = len(ALL_OBJECTS_CANONICAL)
        completed = sum(1 for lbl, _ in ALL_OBJECTS_CANONICAL if lbl in merged)
        passed    = sum(1 for lbl, _ in ALL_OBJECTS_CANONICAL
                        if merged.get(lbl, (0, 0, None))[2])
        failed    = sum(1 for lbl, _ in ALL_OBJECTS_CANONICAL
                        if lbl in merged and merged[lbl][2] is None)
        lines.append(f'progress: {completed}/{total} objects  |  '
                     f'detected: {passed}  failed: {failed}')

        # Inference time: keep existing (more samples); append current if new.
        if existing_inf_line:
            lines.append(existing_inf_line)
        if self._all_inf_ms:
            mean_ms = statistics.mean(self._all_inf_ms)
            std_ms  = (statistics.stdev(self._all_inf_ms)
                       if len(self._all_inf_ms) > 1 else 0.0)
            tag = ' (retest)' if existing_inf_line else ''
            lines.append(f'inference time{tag}: {mean_ms:.0f}ms ± {std_ms:.0f}ms '
                         f'(n={len(self._all_inf_ms)})')
        elif not existing_inf_line:
            lines.append('inference time: not recorded')

        lines.append('')

        for label, (gt_n, gt_e) in ALL_OBJECTS_CANONICAL:
            lines.append(f'{label} (GT: N={gt_n:.3f}, E={gt_e:.3f}):')
            if label not in merged:
                lines.append('  (pending)')
            else:
                frames = merged[label][2]
                if frames is None:
                    lines.append('  FAILED — no confirmed detection in full 360° sweep')
                else:
                    for i, (n, e) in enumerate(frames, 1):
                        lines.append(f'  frame {i}: N={n:.3f}  E={e:.3f}')
            lines.append('')

        log_text = '\n'.join(lines)

        separator = '=' * 60
        print(f'\n{separator}')
        print(log_text)
        print(separator)

        try:
            with open(LOG_PATH, 'w') as f:
                f.write(log_text + '\n')
            self.get_logger().info(
                f'Log saved to {LOG_PATH} '
                f'({completed}/{total} objects, {passed} detected, {failed} failed)')
        except Exception as exc:
            self.get_logger().error(f'Failed to write log: {exc}')


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvalPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
