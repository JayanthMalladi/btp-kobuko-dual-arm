#!/usr/bin/env python3
"""
Q-B coupling monitor: feedforward + FK monitor/abort
(docs/bimanual/README.md Q-B, docs/bimanual/IMPLEMENTATION.md).

Subscribes to /joint_states, runs arm_kinematics.fk() on both arms, and
compares the measured left/right grasp-frame separation against the nominal
separation implied by BIMANUAL_OFFSETS via arm_kinematics.coupling_error().
Past a threshold it logs an abort-level message.

This is a passive diagnostic - it never publishes to /arm/*/cmd_pos - so it
is safe to run alongside anything, including the existing pose_cmd.py:

    ros2 launch dual_arm_kobuki spawn.launch.py
    ros2 run dual_arm_kobuki coupling_monitor.py            # terminal 2
    ros2 run dual_arm_kobuki pose_cmd.py pinch               # terminal 3

There is no synchronized bimanual trajectory sequencer yet (see
docs/bimanual/IMPLEMENTATION.md "not built"), so nothing currently drives the
arms along a BIMANUAL_OFFSETS-consistent path - pose_cmd.py's "pinch" pose
does not track it, so expect a nonzero, non-alarming number from that combo.
This node is the piece of Q-B infrastructure a future sequencer would run
next to; --threshold documents the number from that section's own "Verify"
line (FK-derived separation drift < 5 mm across the trajectory).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import arm_kinematics as K

JOINTS = {
    "left": ["left_j1", "left_j2", "left_j3", "left_j4"],
    "right": ["right_j1", "right_j2", "right_j3", "right_j4"],
}


class CouplingMonitor(Node):
    def __init__(self, offsets, threshold):
        super().__init__("coupling_monitor")
        self.offsets = offsets
        self.threshold = threshold
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.get_logger().info(
            f"watching /joint_states, abort threshold {threshold * 1000:.1f} mm "
            f"against handle separation {K.BIMANUAL_HANDLE_SEP * 100:.0f} cm"
        )

    def on_joint_states(self, msg):
        idx = {name: i for i, name in enumerate(msg.name)}
        try:
            qL = tuple(msg.position[idx[n]] for n in JOINTS["left"])
            qR = tuple(msg.position[idx[n]] for n in JOINTS["right"])
        except KeyError:
            return  # not all 8 arm joints have published yet

        err = K.coupling_error(qL, qR, self.offsets)
        if abs(err) > self.threshold:
            self.get_logger().error(
                f"COUPLING ABORT: {err * 1000:+.2f} mm exceeds "
                f"{self.threshold * 1000:.1f} mm - a synchronized sequencer "
                "would stop motion here"
            )
        else:
            self.get_logger().debug(f"coupling error {err * 1000:+.2f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--threshold", type=float, default=0.005,
        help="abort threshold in metres (default 0.005 = 5 mm, "
             "docs/bimanual/README.md Q-B)",
    )
    args = ap.parse_args()

    rclpy.init()
    node = CouplingMonitor(K.BIMANUAL_OFFSETS, args.threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
