#!/usr/bin/env python3
"""
mission_state_machine.py
========================
Orchestrates a "find the blue ball" mission on top of slam_nav.py and
object_detector.py.

State flow
──────────
  WAIT_NAV_READY  — wait until slam_nav reports HOLD (takeoff complete).
  SCAN            — rotate through 8 yaw stops (45° each). At each stop,
                    hold for SCAN_DWELL_SEC and listen for /detection/bbox
                    with detected=true AND a valid depth. First confident
                    detection breaks out.
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
  /fmu/out/vehicle_attitude        px4_msgs/VehicleAttitude

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
SCAN_DWELL_SEC     = 3.0                 # time per stop (settle + ~15 frames)
SCAN_SETTLE_SEC    = 1.5                 # ignore detections until settled
DETECTION_FRESH_SEC = 1.0                # bbox older than this is stale
MIN_DEPTH_M        = 0.4                 # closer than this is unreliable
MAX_DEPTH_M        = 15.0                # farther than this is unreliable
NAV_ARRIVED_RADIUS = 0.40                # m, additional sanity check

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

        # ── Mission state ───────────────────────────────────────────────────
        self._state: str = 'WAIT_NAV_READY'
        # Scan bookkeeping
        self._scan_idx: int = 0
        self._scan_yaw_start: float = 0.0  # absolute yaw at scan idx=0
        self._scan_stop_entered: float = 0.0  # wall time we set this stop's yaw
        # Approach bookkeeping
        self._goal_world: Optional[tuple[float, float]] = None
        self._goal_sent_time: float = 0.0
        # We wait for slam_nav to transition NAV -> HOLD to detect arrival.
        # Track whether we've seen NAV since the goal was sent.
        self._saw_nav_after_goal: bool = False

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_yaw  = self.create_publisher(Float32, '/nav/yaw_cmd', 10)
        self._pub_goal = self.create_publisher(PoseStamped, '/nav_goal', 10)
        self._pub_state = self.create_publisher(String, '/mission/state', 10)

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(String, '/detection/bbox', self._bbox_cb, 10)
        self.create_subscription(String, '/nav/status',     self._nav_cb,  10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
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

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._pos = (float(msg.x), float(msg.y), float(msg.z))

    def _att_cb(self, msg: VehicleAttitude) -> None:
        # PX4 VehicleAttitude.q is [w, x, y, z]
        self._yaw = quat_to_yaw(
            float(msg.q[0]), float(msg.q[1]),
            float(msg.q[2]), float(msg.q[3]),
        )

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

    def _confident_detection(self) -> Optional[dict]:
        """
        Return a copy of the latest bbox iff it's fresh, detected=true,
        and has a usable depth. Otherwise None.
        """
        with self._bbox_lock:
            bbox = self._bbox
            stamp = self._bbox_stamp
        if bbox is None:
            return None
        if time.time() - stamp > DETECTION_FRESH_SEC:
            return None
        if not bbox.get('detected', False):
            return None
        depth_m = bbox.get('depth_m')
        if depth_m is None:
            return None
        if not (MIN_DEPTH_M <= float(depth_m) <= MAX_DEPTH_M):
            return None
        if bbox.get('x_m') is None or bbox.get('z_m') is None:
            return None
        return dict(bbox)

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
            # Terminal — keep releasing yaw so PX4 holds heading freely.
            self._release_yaw()
        elif self._state == 'SCAN_FAILED':
            self._release_yaw()

    # ── State: WAIT_NAV_READY ────────────────────────────────────────────────

    def _do_wait_nav_ready(self) -> None:
        if self._nav_status != 'HOLD':
            return
        if self._pos is None:
            return
        # slam_nav has finished its own takeoff. Begin the scan from the
        # drone's current heading, sweeping CCW (positive yaw).
        self._scan_yaw_start = self._yaw
        self._scan_idx = 0
        self._enter_scan_stop()
        self.get_logger().info(
            f'Nav ready — starting 360° scan from yaw={math.degrees(self._scan_yaw_start):.1f}°.')
        self._state = 'SCAN'

    # ── State: SCAN ──────────────────────────────────────────────────────────

    def _enter_scan_stop(self) -> None:
        target_yaw = self._scan_yaw_start + self._scan_idx * SCAN_YAW_STEP
        # Wrap to [-pi, pi] for tidy logging; PX4 accepts any value.
        wrapped = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
        self._publish_yaw_cmd(wrapped)
        self._scan_stop_entered = time.time()
        self.get_logger().info(
            f'Scan stop {self._scan_idx + 1}/{SCAN_STEPS} — yaw target {math.degrees(wrapped):+.1f}°.')

    def _do_scan(self) -> None:
        elapsed = time.time() - self._scan_stop_entered

        # Keep republishing the yaw command so slam_nav's HOLD setpoint
        # carries it (slam_nav stores the last value, but republishing
        # makes us robust to a dropped message).
        target_yaw = self._scan_yaw_start + self._scan_idx * SCAN_YAW_STEP
        self._publish_yaw_cmd(
            math.atan2(math.sin(target_yaw), math.cos(target_yaw)))

        # After settle, check for a confident detection.
        if elapsed >= SCAN_SETTLE_SEC:
            det = self._confident_detection()
            if det is not None:
                self.get_logger().info(
                    f'Detection at scan stop {self._scan_idx + 1}: '
                    f'depth={det["depth_m"]:.2f}m  cam_fwd={det["z_m"]:.2f}m  '
                    f'cam_right={det["x_m"]:.2f}m — transitioning to APPROACH.')
                self._begin_approach(det)
                return

        # Stop expired — advance.
        if elapsed >= SCAN_DWELL_SEC:
            self._scan_idx += 1
            if self._scan_idx >= SCAN_STEPS:
                self.get_logger().warn(
                    'Completed full 360° scan with no detection — SCAN_FAILED.')
                self._release_yaw()
                self._state = 'SCAN_FAILED'
                return
            self._enter_scan_stop()

    # ── State: APPROACH ──────────────────────────────────────────────────────

    def _begin_approach(self, det: dict) -> None:
        world = self._project_to_world(det)
        if world is None:
            self.get_logger().error(
                'Cannot project detection — no position. Returning to SCAN.')
            return
        n, e = world
        # Release yaw so slam_nav computes path-aligned yaw during NAV.
        self._release_yaw()
        self._publish_nav_goal(n, e)
        self._goal_world = (n, e)
        self._goal_sent_time = time.time()
        self._saw_nav_after_goal = False
        self.get_logger().info(
            f'Nav goal sent: N={n:.2f}m  E={e:.2f}m '
            f'(from drone N={self._pos[0]:.2f} E={self._pos[1]:.2f} yaw={math.degrees(self._yaw):.1f}°)')
        self._state = 'APPROACH'

    def _do_approach(self) -> None:
        # Watch slam_nav: it goes HOLD -> NAV (after our goal) -> HOLD (on arrival).
        if self._nav_status == 'NAV':
            self._saw_nav_after_goal = True
        elif self._nav_status == 'BLOCKED':
            self.get_logger().error(
                'slam_nav reports BLOCKED — cannot reach object. Stopping.')
            self._state = 'SCAN_FAILED'
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