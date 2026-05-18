#!/usr/bin/env python3
"""
text_input.py
=============
Terminal text interface for the TRACE mission pipeline.

Waits for the mission to reach a terminal state (ARRIVED or SCAN_FAILED),
then prompts the operator to type the next target object. Publishing on
/mission/new_target triggers the state machine to return to origin and
begin a fresh scan for the new object.

Subscriptions
─────────────
  /mission/state       std_msgs/String  — current mission state

Publications
────────────
  /mission/new_target  std_msgs/String  — operator-supplied target label
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TERMINAL_STATES = {'ARRIVED', 'SCAN_FAILED'}


class TextInputNode(Node):

    def __init__(self) -> None:
        super().__init__('text_input')

        self._mission_state: str = ''
        self._lock = threading.Lock()

        self._pub = self.create_publisher(String, '/mission/new_target', 10)
        self.create_subscription(String, '/mission/state', self._state_cb, 10)

        threading.Thread(target=self._input_loop, daemon=True).start()
        self.get_logger().info(
            'text_input ready — will prompt for next target when mission completes.')

    def _state_cb(self, msg: String) -> None:
        with self._lock:
            self._mission_state = msg.data

    def _input_loop(self) -> None:
        while rclpy.ok():
            # Block until the mission reaches a terminal state.
            while rclpy.ok():
                with self._lock:
                    state = self._mission_state
                if state in TERMINAL_STATES:
                    break
                time.sleep(0.5)

            status = (
                'Object found — drone is hovering above target.'
                if state == 'ARRIVED'
                else 'Scan failed — object not found in full 360° sweep.'
            )
            print(f'\n[TRACE] {status}', flush=True)
            print('[TRACE] Enter next target object: ', end='', flush=True)

            try:
                label = input().strip()
            except EOFError:
                break

            if not label:
                print('[TRACE] No input received — please try again.', flush=True)
                continue

            with self._lock:
                current = self._mission_state
            if current not in TERMINAL_STATES:
                print('[TRACE] Mission already restarted — input discarded.', flush=True)
                continue

            out = String()
            out.data = label
            self._pub.publish(out)
            print(f'[TRACE] New target set: "{label}" — drone returning to origin.', flush=True)

            # Wait until the mission leaves the terminal state before prompting again.
            while rclpy.ok():
                with self._lock:
                    if self._mission_state not in TERMINAL_STATES:
                        break
                time.sleep(0.5)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
