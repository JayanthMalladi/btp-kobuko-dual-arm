# Bimanual Co-Manipulation — Design Notes

Working notes on making `dual_arm_kobuki` actually behave as a dual-arm *collaborator*: both grippers
on one object at the same time, arms mutually constrained.

**Status: nothing here is built.** This is analysis plus three unresolved design questions. No code,
world, or URDF changes have been made. See [`../SPEC.md`](../SPEC.md) for what *is* implemented (all of
it single-arm) and [`../RUNNING.md`](../RUNNING.md) for how to run it.

---

## 1. The question

> Once both jaws are on the object, both arms are constrained to each other's movements — right? What
> do we implement?

Right. Two arms holding one rigid object form a **closed kinematic chain**; the arms stop being
independent. The binding constraint is that the relative transform between the two grasp frames is
fixed by the object's geometry:

```
T_left⁻¹ · T_right = constant
```

The architectural consequence: you do **not** plan two arm trajectories and try to keep them agreeing.
You plan one trajectory for the **object's pose** and derive both arms' targets from it at every
timestep. The constraint is then satisfied *by construction* rather than enforced after the fact.

That much is standard. What is *not* standard is how badly this particular mechanism constrains what
that object trajectory is allowed to be.

---

## 2. The mechanism fights you (two findings that shape everything)

### 2a. A rigid dual grasp is a 2-DOF mechanism — it cannot carry

Each arm's end-effector orientation is always `R = Rz(j1) · Ry(psi)`, so `ŷ_EE = Rz(j1)ŷ` is **always
horizontal**. Differentiating, `ω = j̇1·ẑ + ψ̇·ŷ_EE`, and critically `j̇1` is *slaved* to translation:

```
j̇1 = v⊥ / ρ        ρ = hypot(x − s_x, y − s_y)     (v⊥ = horizontal velocity ⊥ to reach direction)
```

Roll about the horizontal reach direction is structurally impossible. For a rigid grasp the shared
angular velocity must lie in `span{ẑ, ŷ_L} ∩ span{ẑ, ŷ_R} = span{ẑ}` (pure yaw) whenever
`j1_L ≠ j1_R`. Imposing both arms' yaw-slaving constraints leaves exactly **two** admissible motions:

1. **Pure vertical lift** (both bearings unchanged, so `j̇1 = 0` on both arms).
2. **Yaw about one specific vertical axis `c`** — the intersection of the lines through each shoulder
   perpendicular to that arm's reach direction. An Ackermann-like instantaneous center.

**Therefore a rigidly-grasped object cannot be translated horizontally at all.** Pure horizontal
translation would require velocity radial from *both* shoulders simultaneously, impossible unless
`j1_L = j1_R`. (That parallel-heading case — handles at exactly `y = ±0.08875`, both arms pointing
`+x` — is a singularity that *gains* a DOF: lift + translate along `+x` + common pitch.)

**The fix is the grasp interface, not the arms.** Make each handle a **vertical cylindrical post** and
each grasp behaves as a *revolute joint about the post axis*. Constraints drop from 6 to 5 per arm, so
`8 + 6 − 10 = 4` object DOF, the free yaw at each jaw absorbs `j1` changes, and horizontal carry
becomes possible. This is the single most important conclusion here.

> This also reconciles the workspace numbers in §3: those were obtained by solving **position only**
> with free tool pitch, which implicitly assumes exactly this post/revolute interface. They are valid
> for a post grasp and **not** valid for a rigid weld.

Vertical posts have a second, independent virtue: the fingers close horizontally, so they are always
perpendicular to a vertical post axis *regardless of arm heading*. The grasp works at any `j1`.

### 2b. The 18 cm barrel cannot be squeeze-grasped — structural, not tuning

There is no wrist roll or yaw, so each palm's face normal **is** the arm's reach direction, and that
direction is forced by `j1 = atan2(contact_y − shoulder_y, contact_x − shoulder_x)`. Both palms
therefore always face forward, and a "squeeze" is mostly a forward shove:

| Barrel centre x | inward | forward | squeeze/forward |
|---|---|---|---|
| 0.44 m (its actual position) | 0.302 | 0.953 | 0.32 |
| 0.35 m | 0.440 | 0.898 | 0.49 |
| 0.2675 m (closest — hits deck edge) | 0.699 | 0.715 | 0.98 |

*Criterion: contact bearing chosen to maximise the inward component.* The exact ratios are
criterion-dependent — requiring push exactly along the inward normal gives 0.26 / 0.35 / 0.51; taking
the most-inward contact still inside the μ=1.6 friction cone gives 0.44 / 0.72 / 1.21. **The conclusion
is the same under every criterion:** at the barrel's real position the forward component dominates,
both arms' forward components *add*, and there is no backstop in the world to react against.

Compounding it: arm separation S = 17.75 cm is 2.5 mm **narrower** than the 18.00 cm object, so the
contact points sit outboard of the shoulders.

This is stronger than [`../../README.md`](../../README.md) §9, which frames the barrel as inefficient
(21–46% force fraction) rather than as impossible. That section should be corrected.

---

## 3. What *is* feasible (measured)

All figures from sweeps against `nodes/arm_kinematics.py`, for a **post/revolute** grasp interface.

- **Two-handle grasp is comfortable.** Sweeping x ∈ [0.16, 0.42], separation ∈ [0.12, 0.48],
  z ∈ [0.16, 0.36], psi ∈ {1.57 … 3.14}: **~690 feasible combinations**, best reach fraction 0.38 —
  lots of margin.
- **Generous coupled workspace.** From a nominal object centre (0.28, 0, 0.24) with 20 cm handle
  separation: lift 38 cm, reach 34.5 cm, lateral 40 cm, yaw ±90°.
- **One spare internal DOF per arm.** Tool pitch psi is free over 109°–178° at the nominal pose —
  usable for conditioning.
- **No arm-arm collisions** in that envelope: min link-to-link gap 9.3–17.7 cm against a 5.2 cm contact
  threshold, for separations up to 24 cm. At 28 cm separation usable yaw collapses to ±35°.

### The interior-hole problem (most important practical finding)

The coupled workspace is **non-convex with interior holes**. At the nominal pose, yaw +56° to +72° is
unreachable while +50° and +80° are both fine. Cause: j3 needs 2.3596 rad against its 2.3562 rad limit
— it misses by **0.19°**, arm folded nearly double, and *both* elbow branches are blocked.

Consequences:
- Feasibility must be checked at **every trajectory sample**, never just at waypoints. The existing
  waypoint-only check in `nodes/pick_and_place.py:build_plan()` would fly straight through these holes.
- Under a shared grasp, mid-motion IK failure is not a graceful abort — one arm stalls while the other
  keeps pulling, and the object gets wrenched.
- `j3_upper` is a `[PLACEHOLDER]` in `urdf/common.xacro`, so widening it (2.3562 → ~2.44 rad) closes
  this particular hole. That is a spec decision, not a code fix.

---

## 4. Open questions

Deliberately unresolved. Each lists plausible solutions, cost, what it unlocks, and what evidence would
settle it.

### Q-A — Mechanism: how to make barrel-style co-manipulation possible at all

| Option | Cost | Unlocks | Settled by |
|---|---|---|---|
| **(a) Handle posts on a shared object** | Modify `object_max` in the SDF; ~zero code | The ~690-combination feasible set, and (with *cylindrical* posts) the 4-DOF closed chain of §2a | Paired-IK sweep + a lift-and-carry sim |
| **(b) 5th DOF (wrist roll)** | Breaks the planar-3R decomposition — `ik()` must be rewritten, +2 controllers and bridge topics, invalidates SPEC §6 | True squeeze grasps *and* the missing roll DOF, raising the rigid closed chain from 2 to 4 DOF | Rank of `[J_L │ −J_R]` before/after |
| **(c) Deck as backstop** | Cheap | — | **Dead end**: deck edge is at r = 0.1775 and the barrel is 18 cm dia, so the object centre lands at x ≈ 0.27 — inside the J1 dead zone and colliding with both risers |
| **(d) Cradle / scoop from underneath** | `psi ≈ 0.5` gives upward palms; barrel rests on both forearms | A demo-able lift | Topple test: 25 cm tall × 0.5 kg on soft P=4..9 PIDs with no lateral capture. Viable as a demo, not as a grasp |
| **(e) Sequential single-arm regrasp** | No hardware change | Object repositioning (one arm stabilises, the other moves) | — but it is not co-manipulation, so it dodges the question |
| **(f) Object-side compliance** | Soft/deformable handles, or a keyed socket that self-aligns | Physically absorbs the over-constraint instead of solving it in software | Whether the 2-DOF mismatch shows up as measurable internal force |

(a) is the natural default; (b) is the highest-payoff and highest-cost option and is the only one that
makes the *barrel specifically* work.

### Q-B — Coupling: how tightly to enforce it during motion

| Option | Cost | Notes |
|---|---|---|
| **Feedforward only** — one object trajectory, one shared clock, one publish loop | Free | Arms cannot desynchronise, but nothing detects slip or lag. Only valid on the feasible manifold. Verify: FK-derived separation drift < 5 mm across the trajectory |
| **Feedforward + FK monitor/abort** | Small | Subscribe `/joint_states`, run `fk()` on both arms, compare `‖p_L − p_R‖` against nominal, abort past threshold. Cheap, diagnostic, requires no control change. The pragmatic middle |
| **Full internal-wrench compliance** | Large | `JointPositionController` cannot accept torque commands at all. Would need `gz-sim-joint-controller` in force mode (or ros2_control effort interfaces), joint-torque sensing, an arm Jacobian (absent from `arm_kinematics.py` today), and a grasp-matrix pseudoinverse to split internal from external wrench. `joint_effort = 5.0` N·m also saturates quickly |

### Q-C — Grasp physics in simulation

| Option | Notes |
|---|---|
| **Friction only** (as the existing single-arm demo) | Honest and consistent. 0.5 kg needs only ~1.5 N normal force, but soft arm PIDs plus a 25 cm lever arm make it marginal |
| **Weld both palms via `gz-sim-detachable-joint-system`** | **Actively harmful.** By §2a this welds a 2-DOF mechanism and then commands 8 independent position setpoints. DART resolves loops by constraint projection so it will not explode outright, but every inconsistency between the two arms' setpoints becomes a constraint violation the stiff weld absorbs as large impulses — jitter, drift, and genuine energy injection. This is the classic dual-arm "fighting" failure |
| **Friction with weld fallback** | Best pragmatic option, **provided at most one weld is rigid** — e.g. weld the left palm and leave the right friction-only. That keeps the loop open and lets the right arm ride |

---

## 5. The decision-independent core

These are identical no matter how Q-A/Q-B/Q-C resolve, so they can be built first without prejudging
anything. All extend `nodes/arm_kinematics.py`, reusing the existing `ik`, `fk`, `shoulder`,
`reach_fraction`, `Unreachable`, and `JOINT_LIMITS`:

```
grasp_targets(obj_pose, offsets)      -> {"left": (x,y,z), "right": (x,y,z)}
                                         obj_pose is (x, y, z, yaw) — yaw-only by design,
                                         since roll/pitch are unreachable (§2a)
pair_ik(obj_pose, offsets, psi_l, psi_r, elbow="auto") -> (qL, qR)
                                         raises Unreachable tagged with which arm failed
pair_reach(obj_pose, offsets, psi_l, psi_r)  -> (fracL, fracR)
pair_feasible(...)                    -> bool   limits + reach < 0.9 + inter-arm clearance
coupling_error(qL, qR, offsets)       -> float  fk-based measured-vs-nominal separation (for Q-B)
pair_mobility(qL, qR)                 -> int    numeric rank of [J_L | −J_R]
```

`pair_mobility` needs a small new `jacobian(q, arm)` helper — the only genuinely new math, and itself
decision-independent. It is also the test that settles Q-A(b) and that would have predicted the Q-C
welding problem directly.

Whatever is built, the trajectory checker must sample **every step**, not every waypoint (§3).

---

## 6. Not decided here

Mechanism (Q-A), coupling strictness (Q-B), and grasp physics (Q-C) are all open. Until Q-A is
answered, no world or URDF changes should be made — the shape of the shared object follows from that
choice. The barrel stays in the world for now as a documented negative result.
