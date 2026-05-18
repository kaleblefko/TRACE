#!/usr/bin/env python3
"""
mission_state_machine.py
========================
Orchestrates a "find the blue ball" mission on top of slam_nav.py and
object_detector.py.

State flow
──────────
  WAIT_NAV_READY  — wait until slam_nav reports HOLD (takeoff complete).
  SCAN            — rotate through 8 yaw stops (45° each). At each stop:
                      1. Rotate to target yaw, wait for the drone to settle
                         (heading + yaw rate both stable for several ticks).
                      2. Once locked, watch /detection/bbox frame-by-frame.
                         3 consecutive positives → commit (use last frame's
                         coords). 3 consecutive negatives → next quadrant.
  APPROACH        — project the camera-frame detection (x_m right, z_m fwd)
                    into world NED using the drone's pose+yaw snapshotted
                    at the moment of detection, publish /nav_goal, then
                    wait for slam_nav to return to HOLD.
  ARRIVED         — hover above the object. Terminal for this v1.
  SCAN_FAILED     — full 360° produced no detection. Terminal.

Subscriptions
─────────────
  /detection/bbox                  std_msgs/String (JSON from object_detector)
  /nav/status                      std_msgs/String (from slam_nav)
  /fmu/out/vehicle_local_position_v1   px4_msgs/VehicleLocalPosition
  /fmu/out/vehicle_attitude_v1         px4_msgs/VehicleAttitude

Publications
────────────
  /nav/yaw_cmd                     std_msgs/Float32 (yaw radians, NED)
  /nav_goal                        geometry_msgs/PoseStamped (N/E target)
  /mission/state                   std_msgs/String (this node's state)
"""

import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition


# ─────────────────────────────────────────────────────────────────────────────
# Tunables
# ─────────────────────────────────────────────────────────────────────────────

SCAN_STEPS         = 8                   # 8 × 45° = full 360°
SCAN_YAW_STEP      = 2.0 * math.pi / SCAN_STEPS

# Quadrant lock parameters
YAW_LOCK_TOLERANCE  = math.radians(5.0)  # heading within this of target = locked
YAW_RATE_THRESH     = math.radians(8.0)  # rad/s — below this counts as "still"
YAW_STILL_TICKS     = 5                  # consecutive 10Hz ticks below thresh
YAW_LOCK_TIMEOUT    = 15.0               # s, give up waiting for clean lock

# Inference confirmation parameters
INFERENCE_FRAME_BUDGET = 20             # max frames per quadrant after lock (backstop)
REQUIRED_DETECTIONS    = 3              # consecutive detected=true to commit
REQUIRED_NEGATIVES     = 3             # consecutive detected=false to advance
MIN_LOCK_DWELL_SECS    = 2.0           # seconds after locking before negatives count

# Return-to-origin timing
RETURN_NAV_TIMEOUT     = 15.0          # s, wait for slam_nav to enter NAV after goal sent
RETURN_SETTLE_SECS     = 8.0           # s, physical settle time at origin before yaw rotation
RETURN_YAW_STILL_TICKS = 20            # 10 Hz ticks of yaw stability required (2 s)
RETURN_YAW_TIMEOUT     = 20.0          # s, give up on yaw alignment and start scan anyway
MIN_DEPTH_M            = 0.4             # closer than this is unreliable
MAX_DEPTH_M            = 15.0            # farther than this is unreliable
NAV_ARRIVED_RADIUS     = 0.40            # m, arrival tolerance around the standoff goal
APPROACH_STANDOFF      = 1.5            # m, stop this far short of the detected object

# Camera mount: detector reports camera-frame x=right, y=down, z=forward.
# The drone body is FRD (forward-right-down). For a forward-facing camera
# the mappings are identity: forward=z_m, right=x_m. If the camera is ever
# remounted, change here.
def cam_to_body(x_m: float, z_m: float) -> tuple[float, float]:
    """Camera (right, fwd) → body (fwd, right)."""
    body_fwd   = z_m
    body_right = x_m
    return body_fwd, body_right


# PX4 QoS for fmu/out topics.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ─────────────────────────────────────────────────────────────────────────────
# Quaternion → yaw helper (PX4 q is [w, x, y, z], NED, body→world Hamilton)
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    """Extract yaw (heading) from a body→world NED quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class MissionStateMachine(Node):

    def __init__(self) -> None:
        super().__init__('mission_state_machine')

        # ── Inputs ───────────────────────────────────────────────────────────
        self._bbox: Optional[dict] = None
        self._bbox_stamp: float = 0.0
        self._bbox_lock = threading.Lock()

        self._nav_status: str = ''   # latest /nav/status
        self._pos: Optional[tuple[float, float, float]] = None  # NED
        self._yaw: float = 0.0       # current heading (rad, NED)
        self._prev_yaw: Optional[float] = None
        self._prev_yaw_stamp: float = 0.0
        self._yaw_rate: float = 0.0  # rad/s

        # ── Mission state ───────────────────────────────────────────────────
        self._state: str = 'WAIT_NAV_READY'
        # Scan bookkeeping
        self._scan_idx: int = 0
        self._scan_yaw_start: float = 0.0       # absolute yaw at scan idx=0
        self._scan_target_yaw: float = 0.0      # absolute target yaw for current quadrant
        self._scan_stop_entered: float = 0.0    # wall time we set this stop's yaw
        # Lock + confirmation bookkeeping (reset per quadrant)
        self._quadrant_locked: bool = False
        self._lock_time: float = 0.0
        self._still_ticks: int = 0
        self._detection_count: int = 0
        self._negative_count: int = 0
        self._frames_since_lock: int = 0
        self._last_detection: Optional[dict] = None
        self._last_detection_stamp: float = 0.0  # detection stamp we've already counted
        # Approach bookkeeping
        self._goal_world: Optional[tuple[float, float]] = None
        self._goal_sent_time: float = 0.0
        # We wait for slam_nav to transition NAV -> HOLD to detect arrival.
        # Track whether we've seen NAV since the goal was sent.
        self._saw_nav_after_goal: bool = False
        # Return-to-origin bookkeeping
        self._origin: tuple[float, float] = (0.0, 0.0)
        self._origin_yaw: float = 0.0          # heading at takeoff, restored on return
        self._saw_nav_after_return: bool = False
        self._return_sent_time: float = 0.0
        self._return_at_origin: bool = False   # True once position HOLD reached
        self._return_yaw_still_ticks: int = 0
        self._return_yaw_entered: float = 0.0  # wall time yaw alignment began
        # New target requested by the operator via /mission/new_target
        self._new_target: Optional[str] = None

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_yaw  = self.create_publisher(Float32, '/nav/yaw_cmd', 10)
        self._pub_goal = self.create_publisher(PoseStamped, '/nav_goal', 10)
        self._pub_state = self.create_publisher(String, '/mission/state', 10)

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(String, '/detection/bbox',      self._bbox_cb,       10)
        self.create_subscription(String, '/nav/status',           self._nav_cb,        10)
        self.create_subscription(String, '/mission/new_target',   self._new_target_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude_v1',
            self._att_cb, PX4_QOS)

        # ── 10 Hz tick ──────────────────────────────────────────────────────
        self.create_timer(0.1, self._tick)
        # ── 1 Hz state echo ─────────────────────────────────────────────────
        self.create_timer(1.0, self._echo_state)

        self.get_logger().info(
            'mission_state_machine ready — waiting for slam_nav to reach HOLD.')

    # ─────────────────────  Callbacks  ───────────────────────────────────────

    def _bbox_cb(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._bbox_lock:
            self._bbox = obj
            self._bbox_stamp = time.time()

    def _nav_cb(self, msg: String) -> None:
        self._nav_status = msg.data

    def _new_target_cb(self, msg: String) -> None:
        label = msg.data.strip()
        if label:
            self._new_target = label

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (float(msg.x), float(msg.y), float(msg.z))

    def _att_cb(self, msg: VehicleAttitude) -> None:
        # PX4 VehicleAttitude.q is [w, x, y, z]
        new_yaw = quat_to_yaw(
            float(msg.q[0]), float(msg.q[1]),
            float(msg.q[2]), float(msg.q[3]),
        )
        now = time.time()
        if self._prev_yaw is not None and now > self._prev_yaw_stamp:
            dt = now - self._prev_yaw_stamp
            # Shortest-arc yaw difference (handles ±π wrap)
            d = new_yaw - self._prev_yaw
            d = math.atan2(math.sin(d), math.cos(d))
            self._yaw_rate = d / dt
        self._prev_yaw = new_yaw
        self._prev_yaw_stamp = now
        self._yaw = new_yaw

    def _echo_state(self) -> None:
        m = String()
        m.data = self._state
        self._pub_state.publish(m)

    # ─────────────────────  Helpers  ─────────────────────────────────────────

    def _publish_yaw_cmd(self, yaw: float) -> None:
        m = Float32()
        m.data = float(yaw)
        self._pub_yaw.publish(m)

    def _release_yaw(self) -> None:
        m = Float32()
        m.data = float('nan')
        self._pub_yaw.publish(m)

    def _publish_nav_goal(self, north: float, east: float) -> None:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(north)
        ps.pose.position.y = float(east)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self._pub_goal.publish(ps)

    def _project_to_world(self, bbox: dict) -> Optional[tuple[float, float]]:
        """
        Project a camera-frame detection into world NED using the drone's
        current pose + yaw. Assumes level flight.
        """
        if self._pos is None:
            return None
        x_m = float(bbox['x_m'])
        z_m = float(bbox['z_m'])
        body_fwd, body_right = cam_to_body(x_m, z_m)

        yaw = self._yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        # body FRD -> world NED, ignoring roll/pitch:
        #   N = N_drone + body_fwd * cos(yaw) - body_right * sin(yaw)
        #   E = E_drone + body_fwd * sin(yaw) + body_right * cos(yaw)
        d_n = body_fwd * cos_y - body_right * sin_y
        d_e = body_fwd * sin_y + body_right * cos_y
        return (self._pos[0] + d_n, self._pos[1] + d_e)

    # ─────────────────────  State machine tick  ──────────────────────────────

    def _tick(self) -> None:
        if self._state == 'WAIT_NAV_READY':
            self._do_wait_nav_ready()
        elif self._state == 'SCAN':
            self._do_scan()
        elif self._state == 'APPROACH':
            self._do_approach()
        elif self._state == 'ARRIVED':
            self._release_yaw()
            if self._new_target is not None:
                self._begin_return_to_origin(self._new_target)
                self._new_target = None
        elif self._state == 'SCAN_FAILED':
            self._release_yaw()
            if self._new_target is not None:
                self._begin_return_to_origin(self._new_target)
                self._new_target = None
        elif self._state == 'RETURN_TO_ORIGIN':
            self._do_return_to_origin()

    # ── State: WAIT_NAV_READY ────────────────────────────────────────────────

    def _do_wait_nav_ready(self) -> None:
        if self._nav_status != 'HOLD':
            return
        if self._pos is None:
            return
        # slam_nav has finished its own takeoff. Record origin and heading for return trips.
        self._origin = (self._pos[0], self._pos[1])
        self._origin_yaw = self._yaw
        # Begin the scan from the drone's current heading, sweeping CCW (positive yaw).
        self._scan_yaw_start = self._yaw
        self._scan_idx = 0
        self._enter_scan_stop()
        self.get_logger().info(
            f'Nav ready — starting 360° scan from yaw={math.degrees(self._scan_yaw_start):.1f}°.')
        self._state = 'SCAN'

    # ── State: SCAN ──────────────────────────────────────────────────────────

    def _enter_scan_stop(self) -> None:
        """Begin a new quadrant: set yaw target, reset lock/confirmation state."""
        target_yaw = self._scan_yaw_start + self._scan_idx * SCAN_YAW_STEP
        wrapped = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
        self._scan_target_yaw = wrapped
        self._publish_yaw_cmd(wrapped)
        self._scan_stop_entered = time.time()
        # Reset lock + confirmation tracking for this quadrant.
        self._quadrant_locked = False
        self._lock_time = 0.0
        self._still_ticks = 0
        self._detection_count = 0
        self._negative_count = 0
        self._frames_since_lock = 0
        self._last_detection = None
        self._last_detection_stamp = 0.0
        # Clear any stale bbox so previous-mission data cannot trigger early advance.
        with self._bbox_lock:
            self._bbox = None
            self._bbox_stamp = 0.0
        self.get_logger().info(
            f'Quadrant {self._scan_idx + 1}/{SCAN_STEPS} — rotating to '
            f'yaw target {math.degrees(wrapped):+.1f}°.')

    def _advance_quadrant(self, reason: str) -> None:
        """Move to next quadrant or end the scan."""
        self.get_logger().info(
            f'Quadrant {self._scan_idx + 1} done ({reason}).')
        self._scan_idx += 1
        if self._scan_idx >= SCAN_STEPS:
            self.get_logger().warn(
                'Completed full 360° scan with no confirmed detection — SCAN_FAILED.')
            self._release_yaw()
            self._state = 'SCAN_FAILED'
            return
        self._enter_scan_stop()

    def _do_scan(self) -> None:
        now = time.time()

        # Always republish the yaw command (robust to dropped messages).
        self._publish_yaw_cmd(self._scan_target_yaw)

        # ── Phase A: rotating into lock ─────────────────────────────────────
        if not self._quadrant_locked:
            # Yaw error to target (shortest arc).
            err = math.atan2(
                math.sin(self._scan_target_yaw - self._yaw),
                math.cos(self._scan_target_yaw - self._yaw),
            )
            if (abs(err) < YAW_LOCK_TOLERANCE
                    and abs(self._yaw_rate) < YAW_RATE_THRESH):
                self._still_ticks += 1
            else:
                self._still_ticks = 0

            if self._still_ticks >= YAW_STILL_TICKS:
                self._quadrant_locked = True
                self._lock_time = now
                # Discard any inference message that arrived during rotation
                # (or before we got here) by advancing the cursor to the
                # current latest. Only messages received from THIS POINT
                # FORWARD will count toward the streak.
                with self._bbox_lock:
                    self._last_detection_stamp = self._bbox_stamp
                self.get_logger().info(
                    f'Quadrant {self._scan_idx + 1} LOCKED at '
                    f'yaw {math.degrees(self._yaw):+.1f}° — collecting inferences.')
                return

            # Lock timeout — drone didn't fully settle. Force-lock anyway so
            # Phase B can still run inference and detect/reject the object.
            # Advancing here instead would cascade (each mid-rotation command
            # makes the next quadrant harder to settle, skipping all of them).
            if now - self._scan_stop_entered > YAW_LOCK_TIMEOUT:
                self.get_logger().warn(
                    f'Quadrant {self._scan_idx + 1} — yaw lock timed out; '
                    f'force-locking and collecting inferences anyway.')
                self._quadrant_locked = True
                self._lock_time = now
                with self._bbox_lock:
                    self._last_detection_stamp = self._bbox_stamp
            return

        # ── Phase B: locked — track consecutive streaks ─────────────────────
        # Each new inference frame is classified as positive (confident
        # detection) or negative. Streaks reset on the opposite class:
        #   3 consecutive positives  → commit, use frame 3's coords
        #   3 consecutive negatives  → advance to next quadrant
        # All budgets are FRAME-based, not time-based, so this scales with
        # whatever inference rate Ollama happens to deliver.
        with self._bbox_lock:
            bbox = self._bbox
            stamp = self._bbox_stamp

        # A new frame is simply one we haven't counted yet — identified by
        # a stamp newer than the last we processed.
        is_new = bbox is not None and stamp > self._last_detection_stamp

        if is_new:
            self._last_detection_stamp = stamp
            self._frames_since_lock += 1

            # The first inference message received after lock was captured
            # while the drone was still rotating (inference takes several
            # seconds; whatever publishes first was started before we
            # locked). Skip it.
            if self._frames_since_lock == 1:
                self.get_logger().info(
                    f'Quadrant {self._scan_idx + 1} — discarding first '
                    f'post-lock frame (captured during rotation).')
                return

            if self._is_confident(bbox):
                self._detection_count += 1
                self._negative_count = 0
                self._last_detection = dict(bbox)
                self.get_logger().info(
                    f'Quadrant {self._scan_idx + 1} — positive '
                    f'{self._detection_count}/{REQUIRED_DETECTIONS} '
                    f'(frame {self._frames_since_lock}, depth={bbox["depth_m"]:.2f}m).')

                if self._detection_count >= REQUIRED_DETECTIONS:
                    self.get_logger().info(
                        f'Quadrant {self._scan_idx + 1} CONFIRMED — '
                        f'using final inference for nav goal.')
                    self._begin_approach(self._last_detection)
                    return
            else:
                # Don't count negatives until the camera has fully settled after
                # locking — prevents noisy post-rotation frames from advancing
                # the quadrant before the VLM has seen a clean view.
                if now - self._lock_time < MIN_LOCK_DWELL_SECS:
                    self.get_logger().info(
                        f'Quadrant {self._scan_idx + 1} — negative during dwell '
                        f'({now - self._lock_time:.1f}s < {MIN_LOCK_DWELL_SECS}s), not counting.')
                    return

                self._negative_count += 1
                if self._detection_count > 0:
                    self.get_logger().info(
                        f'Quadrant {self._scan_idx + 1} — negative '
                        f'(frame {self._frames_since_lock}, '
                        f'positive streak of {self._detection_count} broken).')
                    self._detection_count = 0
                else:
                    self.get_logger().info(
                        f'Quadrant {self._scan_idx + 1} — negative '
                        f'{self._negative_count}/{REQUIRED_NEGATIVES} '
                        f'(frame {self._frames_since_lock}).')

                if self._negative_count >= REQUIRED_NEGATIVES:
                    self._advance_quadrant(
                        reason=f'{REQUIRED_NEGATIVES} consecutive negatives')
                    return

        # Backstop: advance if we've burned through the frame budget without
        # either streak completing. Protects against a stuck VLM or a target
        # that keeps flickering pos/neg/pos/neg indefinitely.
        if self._frames_since_lock >= INFERENCE_FRAME_BUDGET:
            self._advance_quadrant(
                reason=f'frame budget exhausted '
                       f'(pos={self._detection_count}, neg={self._negative_count})')

    def _is_confident(self, bbox: dict) -> bool:
        """Detection has detected=true + valid depth in working range."""
        if not bbox.get('detected', False):
            return False
        depth_m = bbox.get('depth_m')
        if depth_m is None:
            return False
        if not (MIN_DEPTH_M <= float(depth_m) <= MAX_DEPTH_M):
            return False
        if bbox.get('x_m') is None or bbox.get('z_m') is None:
            return False
        return True

    # ── State: APPROACH ──────────────────────────────────────────────────────

    def _begin_approach(self, det: dict) -> None:
        world = self._project_to_world(det)
        if world is None:
            self.get_logger().error(
                'Cannot project detection — no position. Returning to SCAN.')
            return
        n_obj, e_obj = world

        # Back the goal off by APPROACH_STANDOFF metres along the drone→object
        # vector so the drone never aims at the object (or the wall behind it).
        dn = n_obj - self._pos[0]
        de = e_obj - self._pos[1]
        dist_to_obj = math.hypot(dn, de)

        if dist_to_obj <= APPROACH_STANDOFF:
            # Already within standoff — hover here rather than creeping forward.
            self.get_logger().info(
                f'Object within standoff ({dist_to_obj:.2f}m ≤ {APPROACH_STANDOFF}m) '
                f'— treating as arrived.')
            self._state = 'ARRIVED'
            return

        scale = (dist_to_obj - APPROACH_STANDOFF) / dist_to_obj
        n = self._pos[0] + dn * scale
        e = self._pos[1] + de * scale

        self._release_yaw()
        self._publish_nav_goal(n, e)
        self._goal_world = (n, e)
        self._goal_sent_time = time.time()
        self._saw_nav_after_goal = False
        self.get_logger().info(
            f'Nav goal sent: N={n:.2f}m  E={e:.2f}m '
            f'(object at N={n_obj:.2f}m E={e_obj:.2f}m, '
            f'standoff={APPROACH_STANDOFF}m, range={dist_to_obj:.2f}m)')
        self._state = 'APPROACH'

    def _do_approach(self) -> None:
        # Watch slam_nav: it goes HOLD -> NAV (after our goal) -> HOLD (on arrival).
        if self._nav_status == 'NAV':
            self._saw_nav_after_goal = True
        elif self._nav_status == 'BLOCKED':
            self.get_logger().warn(
                'slam_nav BLOCKED on approach — hovering at current position.')
            self._state = 'ARRIVED'
            return

        if self._saw_nav_after_goal and self._nav_status == 'HOLD':
            # Confirm we're actually near the goal (defense against
            # slam_nav going HOLD for other reasons).
            if self._pos is not None and self._goal_world is not None:
                dist = math.hypot(
                    self._pos[0] - self._goal_world[0],
                    self._pos[1] - self._goal_world[1],
                )
                if dist <= NAV_ARRIVED_RADIUS + 0.5:  # a bit of slack
                    self.get_logger().info(
                        f'Arrived above object (dist to goal {dist:.2f}m). '
                        f'Hovering. Mission complete.')
                    self._state = 'ARRIVED'
                    return
                else:
                    self.get_logger().warn(
                        f'slam_nav in HOLD but {dist:.2f}m from goal — '
                        f'probably stopped short. Treating as arrived.')
                    self._state = 'ARRIVED'
                    return

        # Safety: if we sent a goal but slam_nav never entered NAV after
        # several seconds, the goal was likely rejected (blocked cell).
        if (not self._saw_nav_after_goal
                and time.time() - self._goal_sent_time > 5.0):
            self.get_logger().error(
                'slam_nav never entered NAV after goal — goal likely rejected.')
            self._state = 'SCAN_FAILED'

    # ── State: RETURN_TO_ORIGIN ──────────────────────────────────────────────

    def _begin_return_to_origin(self, new_target: str) -> None:
        on, oe = self._origin
        self.get_logger().info(
            f'New target "{new_target}" received — returning to origin '
            f'N={on:.2f}m E={oe:.2f}m, then realigning to '
            f'yaw={math.degrees(self._origin_yaw):.1f}°.')
        self._release_yaw()
        self._publish_nav_goal(on, oe)
        self._return_sent_time = time.time()
        self._saw_nav_after_return = False
        self._return_at_origin = False
        self._return_yaw_still_ticks = 0
        self._state = 'RETURN_TO_ORIGIN'

    def _do_return_to_origin(self) -> None:
        now = time.time()

        # ── Phase 1: flying back to origin position ──────────────────────────
        if not self._return_at_origin:
            if self._nav_status == 'NAV':
                self._saw_nav_after_return = True
            elif self._nav_status == 'BLOCKED':
                self.get_logger().error(
                    'slam_nav BLOCKED while returning to origin — staying SCAN_FAILED.')
                self._state = 'SCAN_FAILED'
                return

            if self._saw_nav_after_return and self._nav_status == 'HOLD':
                dist = 0.0
                if self._pos is not None:
                    dist = math.hypot(
                        self._pos[0] - self._origin[0],
                        self._pos[1] - self._origin[1],
                    )
                self.get_logger().info(
                    f'At origin (dist={dist:.2f}m) — settling for '
                    f'{RETURN_SETTLE_SECS:.0f}s then rotating to '
                    f'heading {math.degrees(self._origin_yaw):.1f}°.')
                self._return_at_origin = True
                self._return_yaw_entered = now
                return

            if (not self._saw_nav_after_return
                    and now - self._return_sent_time > RETURN_NAV_TIMEOUT):
                self.get_logger().error(
                    'slam_nav never entered NAV returning to origin — giving up.')
                self._state = 'SCAN_FAILED'
            return

        # ── Phase 2a: physical settle — hold position, no yaw command yet ────
        settled_for = now - self._return_yaw_entered
        if settled_for < RETURN_SETTLE_SECS:
            self.get_logger().info(
                f'Settling at origin ({settled_for:.1f}s / {RETURN_SETTLE_SECS:.0f}s)…',
                throttle_duration_sec=2.0)
            return

        # ── Phase 2b: yaw realignment ─────────────────────────────────────────
        self._publish_yaw_cmd(self._origin_yaw)

        err = math.atan2(
            math.sin(self._origin_yaw - self._yaw),
            math.cos(self._origin_yaw - self._yaw),
        )
        if abs(err) < YAW_LOCK_TOLERANCE and abs(self._yaw_rate) < YAW_RATE_THRESH:
            self._return_yaw_still_ticks += 1
        else:
            self._return_yaw_still_ticks = 0

        if self._return_yaw_still_ticks >= RETURN_YAW_STILL_TICKS:
            self.get_logger().info(
                'Origin heading restored — restarting 360° scan.')
            self._scan_yaw_start = self._yaw
            self._scan_idx = 0
            self._enter_scan_stop()
            self._state = 'SCAN'
            return

        # Safety timeout: start scan from current heading if alignment takes too long.
        if now - self._return_yaw_entered > RETURN_SETTLE_SECS + RETURN_YAW_TIMEOUT:
            self.get_logger().warn(
                'Yaw realignment timed out — starting scan from current heading.')
            self._scan_yaw_start = self._yaw
            self._scan_idx = 0
            self._enter_scan_stop()
            self._state = 'SCAN'


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()