#!/usr/bin/env python3
"""
Closed-form kinematics for the 4-DOF arm.

The arm is J1 yaw about Z, then J2/J3/J4 all pitching about the SAME axis.
That makes it a planar 3R chain living in one vertical plane, with J1 choosing
the heading of that plane. So the IK decomposes cleanly:

  1. J1 comes straight from the target's bearing off the shoulder.
  2. Inside the plane, pick the tool pitch psi (the cumulative angle of the
     gripper from vertical). Back off L3 along psi to get the wrist point.
  3. Solve a textbook 2R problem for J2 and J3.
  4. J4 = psi - J2 - J3.

4 DOF means 4 constraints: three for position and ONE for orientation, which
is psi. There is no free choice of tool roll or yaw - the gripper's approach
axis is locked into the J1 plane. That is a property of the mechanism, not a
limitation of this solver.

Constants below MUST match urdf/common.xacro.
"""

import math

# ---- Geometry (metres), mirroring urdf/common.xacro ----
MOUNT_X = 0.095      # arm_offset_fwd
MOUNT_Y = 0.08875    # arm_offset_lat  (+left, -right)
J2_Z = 0.190         # deck_top_z + 0.010 (J1) + riser_h0
L1 = 0.160           # upper_arm_L1   J2 -> J3
L2 = 0.160           # forearm_L2     J3 -> J4
L3 = 0.093           # wrist_len + palm_len + 0.6*finger_len : J4 -> grasp frame

JOINT_LIMITS = {
    "j1": (-1.5708, 1.5708),
    "j2": (-1.5708, 1.5708),
    "j3": (-2.3562, 2.3562),
    "j4": (-2.0000, 2.0000),
}

FINGER_STROKE = 0.035        # per finger; opening = 2 * q


class Unreachable(Exception):
    pass


def shoulder(arm):
    """(x, y, z) of the J2 axis for 'left' or 'right'."""
    sign = 1.0 if arm == "left" else -1.0
    return (MOUNT_X, sign * MOUNT_Y, J2_Z)


def fk(q, arm="left"):
    """Forward kinematics. q = (j1, j2, j3, j4). Returns grasp-frame xyz."""
    j1, j2, j3, j4 = q
    sx, sy, sz = shoulder(arm)

    # in-plane: s is radial distance from the J1 axis, h is height above J2
    s = h = 0.0
    cum = 0.0
    for angle, length in ((j2, L1), (j3, L2), (j4, L3)):
        cum += angle
        s += length * math.sin(cum)
        h += length * math.cos(cum)

    return (sx + s * math.cos(j1), sy + s * math.sin(j1), sz + h)


def ik(target, arm="left", psi=2.35, elbow="auto"):
    """
    Inverse kinematics.

    target : (x, y, z) of the grasp frame in the base frame
    psi    : tool pitch, cumulative angle of the gripper from straight up.
             0    = pointing up
             pi/2 = pointing horizontally outward
             pi   = pointing straight down
             ~2.35 (135 deg) = a 45 degree diagonal approach, which costs far
             less reach than a pure top-down grasp
    elbow  : 'up', 'down', or 'auto' (tries both, returns the first that
             satisfies every joint limit)

    Returns (j1, j2, j3, j4). Raises Unreachable.
    """
    x, y, z = target
    sx, sy, sz = shoulder(arm)

    dx, dy = x - sx, y - sy
    j1 = math.atan2(dy, dx)
    s_t = math.hypot(dx, dy)
    h_t = z - sz

    # back off along the tool axis to the wrist
    s_w = s_t - L3 * math.sin(psi)
    h_w = h_t - L3 * math.cos(psi)

    r2 = s_w * s_w + h_w * h_w
    r = math.sqrt(r2)
    if r > L1 + L2:
        raise Unreachable(
            f"wrist point {r*100:.1f} cm from shoulder, max {(L1+L2)*100:.1f} cm"
        )
    if r < abs(L1 - L2):
        raise Unreachable(f"wrist point {r*100:.1f} cm is inside the dead zone")

    c3 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c3 = max(-1.0, min(1.0, c3))
    base3 = math.acos(c3)

    orders = {"up": (1,), "down": (-1,), "auto": (1, -1)}[elbow]
    failures = []

    for sign in orders:
        j3 = sign * base3
        a1 = math.atan2(s_w, h_w) - math.atan2(
            L2 * math.sin(j3), L1 + L2 * math.cos(j3)
        )
        j2 = a1
        j4 = psi - a1 - j3
        q = (j1, j2, j3, j4)

        bad = [
            n
            for n, v in zip(("j1", "j2", "j3", "j4"), q)
            if not (JOINT_LIMITS[n][0] <= v <= JOINT_LIMITS[n][1])
        ]
        if not bad:
            return q
        failures.append(f"elbow {'up' if sign > 0 else 'down'}: {','.join(bad)} out of range")

    raise Unreachable("; ".join(failures))


def reach_fraction(target, arm="left", psi=2.35):
    """How much of the 2R envelope a target consumes. Above ~0.9 is poorly
    conditioned - the arm is near full extension and IK gets stiff."""
    sx, sy, sz = shoulder(arm)
    s_t = math.hypot(target[0] - sx, target[1] - sy)
    h_t = target[2] - sz
    s_w = s_t - L3 * math.sin(psi)
    h_w = h_t - L3 * math.cos(psi)
    return math.hypot(s_w, h_w) / (L1 + L2)


if __name__ == "__main__":
    import random

    print("=== FK/IK round-trip, 4000 random reachable poses ===")
    worst = 0.0
    tested = skipped = 0
    random.seed(0)
    for _ in range(4000):
        for arm in ("left", "right"):
            q = (
                random.uniform(-1.4, 1.4),
                random.uniform(-1.4, 1.4),
                random.uniform(-2.2, 2.2),
                random.uniform(-1.9, 1.9),
            )
            p = fk(q, arm)
            psi = q[1] + q[2] + q[3]
            try:
                q2 = ik(p, arm, psi=psi)
            except Unreachable:
                skipped += 1
                continue
            err = max(abs(a - b) for a, b in zip(p, fk(q2, arm)))
            worst = max(worst, err)
            tested += 1
    print(f"  solved {tested}, unreachable {skipped}")
    print(f"  worst position error: {worst*1e6:.3f} micrometres")
