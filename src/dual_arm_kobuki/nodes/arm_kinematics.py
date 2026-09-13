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


# ======================================================================
# Bimanual co-manipulation core (decision-independent, docs/bimanual/README.md
# section 5). These functions are pure extensions of the single-arm ik/fk
# above: an object pose plus per-arm handle offsets in, per-arm joint targets
# out. See docs/bimanual/IMPLEMENTATION.md for the full derivation and the
# architecture decisions (Q-A/Q-B/Q-C) that this module now assumes.
# ======================================================================

# Q-A(a) resolved: co-manipulation targets a handle-post interface, not a
# rigid squeeze on the object's own surface (docs/bimanual/README.md section
# 2a - a rigid squeeze cannot carry the object at all). Mirrors the
# object_bimanual model in worlds/manipulation_task.sdf and the nominal pose
# used for the workspace sweep in section 3 of the same doc.
BIMANUAL_OBJ_NOMINAL = (0.28, 0.0, 0.24, 0.0)      # (x, y, z, yaw), metres/rad
BIMANUAL_HANDLE_SEP = 0.20                          # metres, section 3 sweep
BIMANUAL_OFFSETS = {
    "left":  (0.0,  BIMANUAL_HANDLE_SEP / 2.0, 0.0),
    "right": (0.0, -BIMANUAL_HANDLE_SEP / 2.0, 0.0),
}
BIMANUAL_PSI = 2.35                                 # reuse the single-arm diagonal approach

# Link-to-link contact threshold used in the section 3 clearance sweep
# ("min link-to-link gap 9.3-17.7 cm against a 5.2 cm contact threshold").
CLEARANCE_THRESHOLD = 0.052


def grasp_targets(obj_pose, offsets):
    """
    Map an object pose to each arm's grasp-frame target.

    obj_pose : (x, y, z, yaw) of the object frame in the base frame. Yaw-only
               by design - neither arm can reach an object roll/pitch offset
               (docs/bimanual/README.md section 2a: gripper ŷ is always
               horizontal), so this function does not accept them.
    offsets  : {"left": (dx, dy, dz), "right": (dx, dy, dz)} handle positions
               in the object's own (yaw-rotated) frame, e.g. BIMANUAL_OFFSETS.

    Returns {"left": (x, y, z), "right": (x, y, z)} in the base frame.
    """
    x, y, z, yaw = obj_pose
    c, s = math.cos(yaw), math.sin(yaw)
    targets = {}
    for arm, (dx, dy, dz) in offsets.items():
        targets[arm] = (x + c * dx - s * dy, y + s * dx + c * dy, z + dz)
    return targets


def pair_ik(obj_pose, offsets, psi_l, psi_r, elbow="auto"):
    """ik() for both arms against one object pose. Raises Unreachable tagged
    with which arm failed - never returns a one-sided solution."""
    targets = grasp_targets(obj_pose, offsets)
    try:
        qL = ik(targets["left"], "left", psi=psi_l, elbow=elbow)
    except Unreachable as e:
        raise Unreachable(f"left arm: {e}")
    try:
        qR = ik(targets["right"], "right", psi=psi_r, elbow=elbow)
    except Unreachable as e:
        raise Unreachable(f"right arm: {e}")
    return qL, qR


def pair_reach(obj_pose, offsets, psi_l, psi_r):
    """reach_fraction() for both arms against one object pose."""
    targets = grasp_targets(obj_pose, offsets)
    return (
        reach_fraction(targets["left"], "left", psi_l),
        reach_fraction(targets["right"], "right", psi_r),
    )


def pair_feasible(obj_pose, offsets, psi_l, psi_r, elbow="auto",
                   reach_limit=0.9, clearance=CLEARANCE_THRESHOLD):
    """True iff both arms reach their handle inside joint limits, stay under
    reach_limit of the 2R envelope (docs/bimanual/README.md section 3: above
    ~0.9 the solve is poorly conditioned), and the resulting grasp frames
    stay further apart than the contact-clearance threshold."""
    try:
        qL, qR = pair_ik(obj_pose, offsets, psi_l, psi_r, elbow)
    except Unreachable:
        return False
    fracL, fracR = pair_reach(obj_pose, offsets, psi_l, psi_r)
    if fracL >= reach_limit or fracR >= reach_limit:
        return False
    gap = math.dist(fk(qL, "left"), fk(qR, "right"))
    return gap > clearance


def coupling_error(qL, qR, offsets):
    """
    Q-B feedforward + FK monitor: FK-derived measured handle separation minus
    the nominal separation implied by offsets. Zero means the two arms are
    exactly where a rigid grasp requires them to be; a growing magnitude means
    the arms are drifting apart (or "fighting", see docs/bimanual/README.md
    Q-C on welding) faster than intended. Sign is measured-minus-nominal, so
    positive means the grasp frames are further apart than the object allows.
    """
    pL, pR = fk(qL, "left"), fk(qR, "right")
    dx, dy, dz = (offsets["left"][i] - offsets["right"][i] for i in range(3))
    nominal = math.sqrt(dx * dx + dy * dy + dz * dz)
    measured = math.dist(pL, pR)
    return measured - nominal


def jacobian(q, arm="left", eps=1e-6):
    """
    6x4 grasp-frame twist Jacobian: d(x,y,z,wx,wy,wz)/d(j1,j2,j3,j4).

    Linear rows come from central-difference on fk() - the closed-form fk
    above has no separate analytic derivative, and 4 joints make this cheap.

    Angular rows are analytic, straight from the mechanism identity derived
    in docs/bimanual/README.md section 2a: the grasp frame's angular velocity
    is  omega = j1_dot * z_hat + psi_dot * y_EE_hat,  where
    y_EE_hat = Rz(j1) * y_hat = (-sin(j1), cos(j1), 0)  is always horizontal,
    and psi_dot = j2_dot + j3_dot + j4_dot since J2/J3/J4 share one axis. So
    d(omega)/d(j1) = z_hat and d(omega)/d(j2) = d(omega)/d(j3) = d(omega)/d(j4)
    = y_EE_hat - no finite-differencing needed for the angular half, and no
    risk of the eps-cancellation error a numeric rotation derivative would have.

    Returned as 6 rows (vx,vy,vz,wx,wy,wz) x 4 columns (j1,j2,j3,j4).
    """
    j1 = q[0]
    p0 = fk(q, arm)
    lin_cols = []
    for i in range(4):
        qp, qm = list(q), list(q)
        qp[i] += eps
        qm[i] -= eps
        fp, fm = fk(qp, arm), fk(qm, arm)
        lin_cols.append(tuple((fp[k] - fm[k]) / (2.0 * eps) for k in range(3)))

    z_hat = (0.0, 0.0, 1.0)
    y_ee = (-math.sin(j1), math.cos(j1), 0.0)
    ang_cols = [z_hat, y_ee, y_ee, y_ee]

    rows = []
    for k in range(3):
        rows.append([lin_cols[c][k] for c in range(4)])
    for k in range(3):
        rows.append([ang_cols[c][k] for c in range(4)])
    return rows


def _matrix_rank(m, tol=1e-9):
    """Rank via Gauss-Jordan elimination with partial pivoting. No numpy
    dependency - this whole module is intentionally dependency-free."""
    m = [row[:] for row in m]
    rows, cols = len(m), len(m[0]) if m else 0
    rank = 0
    for col in range(cols):
        pivot = None
        best = tol
        for r in range(rank, rows):
            if abs(m[r][col]) > best:
                best = abs(m[r][col])
                pivot = r
        if pivot is None:
            continue
        m[rank], m[pivot] = m[pivot], m[rank]
        pv = m[rank][col]
        m[rank] = [v / pv for v in m[rank]]
        for r in range(rows):
            if r != rank and abs(m[r][col]) > tol:
                f = m[r][col]
                m[r] = [a - f * b for a, b in zip(m[r], m[rank])]
        rank += 1
        if rank == rows:
            break
    return rank


def pair_mobility(qL, qR):
    """
    Numeric rank of [J_L | -J_R], the 6x8 combined twist Jacobian of the two
    grasp frames stacked as one constraint: J_L @ qL_dot - J_R @ qR_dot. This
    is the "equal grasp-frame twist" constraint model - it does not apply a
    lever-arm correction for the two handles being ~20 cm apart, so treat its
    output as a diagnostic rank check, not a literal object-DOF count (that
    count, with the lever arm included, is worked out by hand in
    docs/bimanual/README.md section 2a and IMPLEMENTATION.md using the
    standard Grubler mobility formula: object DOF = joints + 6 - constraints).

    Historically this is exactly the computation that would have caught the
    Q-C welding failure ahead of time: a rank-6 constraint against only 8
    joint DOF leaves 2 spare DOF for the two arms to disagree about (their
    individual redundancy inside a shared 2-DOF object motion), and every
    disagreement becomes a constraint violation a rigid weld must absorb as
    an impulse.
    """
    JL = jacobian(qL, "left")
    JR = jacobian(qR, "right")
    M = [JL[r] + [-v for v in JR[r]] for r in range(6)]
    return _matrix_rank(M)


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

    print()
    print("=== Bimanual core self-check (docs/bimanual/README.md section 5) ===")
    obj = BIMANUAL_OBJ_NOMINAL
    offsets = BIMANUAL_OFFSETS
    targets = grasp_targets(obj, offsets)
    print(f"  object pose {obj}, handle separation {BIMANUAL_HANDLE_SEP*100:.0f} cm")
    print(f"  left handle target  {tuple(round(v, 4) for v in targets['left'])}")
    print(f"  right handle target {tuple(round(v, 4) for v in targets['right'])}")

    try:
        qL, qR = pair_ik(obj, offsets, BIMANUAL_PSI, BIMANUAL_PSI)
        fracL, fracR = pair_reach(obj, offsets, BIMANUAL_PSI, BIMANUAL_PSI)
        feasible = pair_feasible(obj, offsets, BIMANUAL_PSI, BIMANUAL_PSI)
        gap = math.dist(fk(qL, "left"), fk(qR, "right"))
        print(f"  qL {tuple(round(v, 4) for v in qL)}  reach {fracL*100:.1f}%")
        print(f"  qR {tuple(round(v, 4) for v in qR)}  reach {fracR*100:.1f}%")
        print(f"  grasp-frame gap {gap*100:.2f} cm, pair_feasible={feasible}")
        print(f"  coupling_error at nominal (should be ~0): "
              f"{coupling_error(qL, qR, offsets)*1000:+.3f} mm")

        mob_post = pair_mobility(qL, qR)
        print(f"  pair_mobility (post/revolute handle grasp) rank={mob_post} "
              f"-> {8 - mob_post} spare joint DOF beyond the equal-twist constraint")
    except Unreachable as e:
        print(f"  PLAN INFEASIBLE at nominal pose: {e}")

    # Rigid-weld comparison: force both arms to the SAME orientation (psi
    # chosen so j1_L != j1_R, i.e. NOT the parallel-heading singularity) to
    # reproduce the section 2a "2 admissible motions" rigid-grasp finding.
    try:
        qL2, qR2 = pair_ik(obj, offsets, BIMANUAL_PSI, BIMANUAL_PSI, elbow="up")
        mob_rigid = pair_mobility(qL2, qR2)
        print(f"  pair_mobility (elbow-up both arms, same psi) rank={mob_rigid} "
              f"-> {8 - mob_rigid} spare DOF")
    except Unreachable as e:
        print(f"  rigid-weld comparison pose infeasible: {e}")
