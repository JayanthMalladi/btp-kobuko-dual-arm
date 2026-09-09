#!/usr/bin/env python3
"""
Drive the arms to a named pose and set the gripper opening.

    ros2 run dual_arm_kobuki pose_cmd.py home
    ros2 run dual_arm_kobuki pose_cmd.py zero
    ros2 run dual_arm_kobuki pose_cmd.py reach
    ros2 run dual_arm_kobuki pose_cmd.py open
    ros2 run dual_arm_kobuki pose_cmd.py close

This is a convenience tool for eyeballing the kinematics, not a controller.
There is no trajectory interpolation - it publishes a step setpoint and the
per-joint PID in Gazebo does the rest.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

ARM_JOINTS = ["j1", "j2", "j3", "j4"]
FINGERS = ["finger_a", "finger_b"]

# (j1, j2, j3, j4) radians, applied to both arms
POSES = {
    "zero":  (0.0,  0.0,   0.0,  0.0),
    "home":  (0.0, -0.55,  1.35, 0.50),
    # forward and down toward the starting base
    "reach": (0.0,  0.70,  0.55, 0.30),
    # arms yawed outward, ready to close on the large object from both sides
    "pinch": (0.0,  0.85,  0.45, 0.25),
}

GRIP = {
    "open":  0.035,   # 7.0 cm total opening - the mechanical maximum
    "close": 0.0,
}


class PoseCmd(Node):
    def __init__(self, target):
        super().__init__("pose_cmd")
        self.pubs = {}
        for side in ("left", "right"):
            for j in ARM_JOINTS + FINGERS:
                name = f"{side}_{j}"
                self.pubs[name] = self.create_publisher(
                    Float64, f"/arm/{name}/cmd_pos", 10
                )
        self.target = target
        # Publishers need a moment to match with the bridge subscription.
        self.timer = self.create_timer(0.5, self.send)
        self.sent = 0

    def send(self):
        if self.target in POSES:
            angles = POSES[self.target]
            for side in ("left", "right"):
                for j, q in zip(ARM_JOINTS, angles):
                    self.pubs[f"{side}_{j}"].publish(Float64(data=float(q)))
            label = f"pose '{self.target}' -> {angles}"
        else:
            q = GRIP[self.target]
            for side in ("left", "right"):
                for f in FINGERS:
                    self.pubs[f"{side}_{f}"].publish(Float64(data=float(q)))
            label = f"gripper '{self.target}' -> {q*2*100:.1f} cm opening"

        self.sent += 1
        if self.sent == 1:
            self.get_logger().info(f"published {label}")
        if self.sent >= 3:          # publish a few times, then exit
            raise SystemExit


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in list(POSES) + list(GRIP):
        print(f"usage: pose_cmd.py <{'|'.join(list(POSES) + list(GRIP))}>")
        return 1

    rclpy.init()
    node = PoseCmd(sys.argv[1])
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
