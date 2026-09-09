#!/usr/bin/env python3
"""
Pick and place the small object from the starting base to the destination base.

    ros2 run dual_arm_kobuki pick_and_place.py
    ros2 run dual_arm_kobuki pick_and_place.py --dry-run    # solve only, no ROS

Uses the left arm. Every Cartesian waypoint is solved with the analytic IK in
arm_kinematics.py, then interpolated in JOINT space with a minimum-jerk
profile at 50 Hz. Joint-space interpolation means the path between waypoints
is not a straight line in Cartesian space - fine here because the waypoints
are close together and the space is empty, but it is the reason for the
explicit lift and transit waypoints rather than a direct pick-to-place move.

The grasp is friction-only: stiff finger PIDs squeezing against mu = 1.6 pads.
No attach plugin. See README section 8 if the object slips.
"""

import argparse
import math
import sys
import time

import arm_kinematics as K

# ---------------- Task definition ----------------
ARM = "left"
PSI = 2.35                      # 135 deg diagonal approach - see arm_kinematics
APPROACH = 0.08                 # vertical standoff for pre-grasp / pre-place

PICK = (0.31, 0.09, 0.225)      # object_min centre, on the starting base
PLACE = (0.20, 0.27, 0.225)     # target on the destination base
TRANSIT = (0.26, 0.18, 0.32)    # waypoint between the two tables

HOME = (0.0, -0.55, 1.35, 0.50)

OBJECT_RADIUS = 0.025
GRIP_CLOSED = OBJECT_RADIUS - 0.004     # 4 mm of interference -> real squeeze
GRIP_OPEN = 0.033                       # 6.6 cm opening, just under the stop

RATE = 50.0                             # Hz


def minimum_jerk(t):
    """Smooth 0->1 with zero velocity and acceleration at both ends."""
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def build_plan():
    """Solve every waypoint up front. Raises before anything moves if the
    task is not kinematically feasible."""
    wp = [
        ("pre-grasp", (PICK[0], PICK[1], PICK[2] + APPROACH), GRIP_OPEN, 2.5),
        ("grasp",     PICK,                                    GRIP_OPEN, 1.5),
        ("close",     PICK,                                    GRIP_CLOSED, 1.0),
        ("lift",      (PICK[0], PICK[1], PICK[2] + APPROACH), GRIP_CLOSED, 1.5),
        ("transit",   TRANSIT,                                 GRIP_CLOSED, 2.0),
        ("pre-place", (PLACE[0], PLACE[1], PLACE[2] + APPROACH), GRIP_CLOSED, 2.0),
        ("place",     PLACE,                                    GRIP_CLOSED, 1.5),
        ("open",      PLACE,                                    GRIP_OPEN, 1.0),
        ("retreat",   (PLACE[0], PLACE[1], PLACE[2] + APPROACH), GRIP_OPEN, 1.5),
    ]

    plan = []
    for label, xyz, grip, dur in wp:
        q = K.ik(xyz, ARM, psi=PSI)
        frac = K.reach_fraction(xyz, ARM, PSI)
        plan.append((label, q, grip, dur, frac))
    return plan


def report(plan):
    print(f"{'step':<11} {'reach':>6}   j1      j2      j3      j4      grip")
    print("-" * 62)
    for label, q, grip, dur, frac in plan:
        flag = " <-- extended" if frac > 0.90 else ""
        print(f"{label:<11} {frac*100:5.1f}%  "
              + "  ".join(f"{v:+.3f}" for v in q)
              + f"  {grip*200:5.1f}cm{flag}")
    print("-" * 62)
    print(f"total motion time: {sum(p[3] for p in plan):.1f} s")


# ---------------- ROS execution ----------------

def run(plan):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64

    rclpy.init()
    node = Node("pick_and_place")
    log = node.get_logger()

    names = [f"{ARM}_j{i}" for i in (1, 2, 3, 4)]
    fingers = [f"{ARM}_finger_a", f"{ARM}_finger_b"]
    other = "right" if ARM == "left" else "left"

    pubs = {
        n: node.create_publisher(Float64, f"/arm/{n}/cmd_pos", 10)
        for n in names + fingers + [f"{other}_j{i}" for i in (1, 2, 3, 4)]
    }

    # let the bridge match subscriptions before the first setpoint
    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # park the idle arm out of the way and hold it there
    for n, v in zip([f"{other}_j{i}" for i in (1, 2, 3, 4)], HOME):
        pubs[n].publish(Float64(data=float(v)))

    def send(q, grip):
        for n, v in zip(names, q):
            pubs[n].publish(Float64(data=float(v)))
        for f in fingers:
            pubs[f].publish(Float64(data=float(grip)))

    q_now = list(HOME)
    grip_now = GRIP_OPEN
    dt = 1.0 / RATE

    for label, q_target, grip_target, dur, frac in plan:
        log.info(f"{label}  (reach {frac*100:.0f}%)")
        steps = max(1, int(dur * RATE))
        q_start, grip_start = list(q_now), grip_now
        for k in range(1, steps + 1):
            s = minimum_jerk(k / steps)
            q = [a + (b - a) * s for a, b in zip(q_start, q_target)]
            g = grip_start + (grip_target - grip_start) * s
            send(q, g)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(dt)
        q_now, grip_now = list(q_target), grip_target

    log.info("returning to home")
    steps = int(2.5 * RATE)
    q_start = list(q_now)
    for k in range(1, steps + 1):
        s = minimum_jerk(k / steps)
        send([a + (b - a) * s for a, b in zip(q_start, HOME)], GRIP_OPEN)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(dt)

    log.info("done")
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="solve and print the plan without touching ROS")
    args = ap.parse_args()

    try:
        plan = build_plan()
    except K.Unreachable as e:
        print(f"PLAN INFEASIBLE: {e}", file=sys.stderr)
        return 1

    report(plan)
    if args.dry_run:
        return 0
    run(plan)
    return 0


if __name__ == "__main__":
    sys.exit(main())
