# What You Need to Understand to Build a Pick-and-Place

**Who this is for:** you want to build a scripted pick-and-place — the thing in
`nodes/pick_and_place.py` — and you want to know *which ideas that actually
requires*, and understand each one properly rather than copying code.

This is a concept map, not a tutorial. Each concept gets: what it is, why it
exists, the maths where maths is the point, where it shows up in this repo, and
the mistake people make with it.

**The honest summary up front:** a scripted pick-and-place needs about
**nine ideas**, and the rest of this document is those nine unpacked into the
forty-odd sub-concepts they're made of.

1. Pose — how you say where something is
2. Frames and transforms — how you convert between "where" answers
3. Kinematic chains and DOF — what the arm can and cannot do
4. Forward kinematics — angles in, hand position out
5. Inverse kinematics — hand position in, angles out
6. Trajectory generation — how to get between two sets of angles smoothly
7. Grasping — why the object stays in the hand
8. Control — how a joint actually reaches the angle you asked for
9. Sequencing and verification — the order of operations, and how you know it worked

If you only have time for a subset, do 1–6 and 9. Grasping and control you can
initially fake; kinematics and sequencing you cannot.

---

# Contents

- **Layer 0** — [The shape of the problem](#layer-0)
- **Layer 1** — [Describing space](#layer-1) · pose, frames, transforms, rotations
- **Layer 2** — [Describing the mechanism](#layer-2) · links, joints, DOF, config space
- **Layer 3** — [Kinematics](#layer-3) · FK, IK, Jacobian, singularities, workspace
- **Layer 4** — [Motion](#layer-4) · paths vs trajectories, interpolation, smoothness
- **Layer 5** — [Grasping](#layer-5) · friction, closure, force budgets, compliance
- **Layer 6** — [Control](#layer-6) · PID, control modes, saturation
- **Layer 7** — [Physics and simulation](#layer-7) · inertia, timesteps, contact
- **Layer 8** — [Software plumbing](#layer-8) · descriptions, middleware, TF, launch
- **Layer 9** — [Task structure and verification](#layer-9)
- **Afterword** — [What this project left out, and what each omission is called](#afterword)

---

<a name="layer-0"></a>
# Layer 0 — The shape of the problem

Before any concept, get the decomposition right. "Pick and place" sounds like one
task. It is six questions, and each one is answered by a different body of
knowledge:

| Question | Concept that answers it | Layer |
|---|---|---|
| Where is the object? | pose, frames, transforms | 1 |
| Can the arm even get there? | DOF, workspace, reachability | 2–3 |
| What joint angles put the hand there? | inverse kinematics | 3 |
| How do I move between angle sets without flailing? | trajectory generation | 4 |
| Why does the object stay in the gripper? | grasp mechanics, friction | 5 |
| How does a joint actually reach its target? | control | 6 |

And two more that aren't about the robot at all but decide whether your project
works:

| Question | Concept | Layer |
|---|---|---|
| In what order do all these happen? | task sequencing | 9 |
| How do I know any of it is right? | verification | 9 |

**The most important structural insight:** these layers are a
*stack*, and each layer only talks to its neighbours. IK does not know about
friction. The trajectory generator does not know what a gripper is. The PID loop
does not know there is a task. If you find yourself writing a function that
reasons about two distant layers at once, you have probably mislaid a concept.

---

<a name="layer-1"></a>
# Layer 1 — Describing space

## C1. Rigid body

**What it is.** An object whose points never move relative to each other. A
coffee mug is a rigid body; a rope isn't.

**Why it matters.** If a body is rigid, then knowing the position of *one* point
on it plus its orientation tells you where *every* point on it is. That's an
enormous simplification, and it's the assumption the entire field rests on. Every
link in a robot arm is treated as rigid.

**The cost of the assumption.** Real links flex, real gears have backlash, real
belts stretch. Rigid-body models ignore all of it. For a slow 80-gram
pick-and-place this is fine. For a fast arm at full extension carrying its
payload limit, link flex is exactly what makes the hand miss.

## C2. Pose — position plus orientation

**What it is.** A **pose** is the complete answer to "where is this body?" It has
two halves:

- **Position** — 3 numbers (x, y, z). Where a reference point on the body is.
- **Orientation** — 3 more numbers. Which way the body is turned.

So a pose is **6 numbers**, and we say rigid-body pose has **6 degrees of
freedom**.

**Why the 6 matters.** This is the number every arm is measured against. An arm
that can achieve any pose in its workspace needs at least 6 joints. An arm with
fewer is **under-actuated**: it can't hit arbitrary poses, and you have to decide
which parts of the pose you're willing to give up.

**In this repo.** The arms have 4 joints. So 4 of the 6 pose numbers are
controllable and 2 are not. `ik()` in `nodes/arm_kinematics.py` spends them as
3 position + 1 orientation, and the docstring says so explicitly. The two you
give up are tool roll and tool yaw.

**The classic mistake.** Asking a 4-DOF arm for a pose and being surprised when
it can't. It isn't a bug or a tuning problem — you asked for 6 numbers from a
mechanism that has 4.

## C3. Frames — why "where" is always relative

**What it is.** A **frame** (or "coordinate frame") is an origin plus three
perpendicular axis directions, glued to a body. A position is only meaningful
*relative to a frame*.

"The object is at (0.31, 0.09, 0.225)" is meaningless on its own. Relative to
what? In this repo, relative to `base_footprint` — a frame on the floor directly
under the robot's centre.

**Why it matters.** A robot task involves many "where" answers held in different
frames at once:

- the object's pose, in the world frame
- the object's pose, as seen by the camera (in the camera's frame)
- the hand's pose, in the shoulder's frame (which is where IK wants it)
- each joint's angle, which is a frame relationship all by itself

Getting between these is most of the bookkeeping in a robot program. Attaching
frames to things — and naming them — is how you keep it straight.

**In this repo.** 21 frames, rooted at `base_footprint`. The important ones are
`left_grasp_frame` and `right_grasp_frame`, declared in `urdf/arm.xacro` as
massless fixed-joint children of the palm, offset by `grasp_offset` along the
palm's Z. They mark the point *between the finger pads* where an object is held.
That's a frame that corresponds to no physical part — it's in mid-air — which is
exactly the point: **frames are bookkeeping devices, not objects.**

**The classic mistake.** Mixing frames silently. Taking a number the camera
produced and feeding it to IK without converting. The units are right, the
magnitude looks plausible, and the arm reaches somewhere wrong.

## C4. Transforms — converting between frames

**What it is.** A **transform** is the operation that converts a position
expressed in frame A into the same position expressed in frame B. It's a rotation
plus a translation.

The notation worth learning is `T_B_A`: "the transform that takes A-frame
coordinates into B-frame coordinates." Two facts make transforms pleasant:

1. **They compose.** `T_C_A = T_C_B · T_B_A`. Chain them and you can hop across
   any number of frames.
2. **They invert.** `T_A_B = (T_B_A)⁻¹`. Every transform runs backwards.

**Why it matters.** Composition is what makes forward kinematics trivial
(C9): the arm is a chain of frames, each joint contributes one transform, and the
hand's pose is the product of them all. It's why robotics can describe an
arbitrarily long arm with one rule.

**The 4×4 homogeneous transform.** The standard trick for making transforms
compose as plain matrix multiplication: pack the 3×3 rotation `R` and the 3×1
translation `t` into one 4×4 matrix:

```
T = [ R  t ]      and positions become 4-vectors [x y z 1]ᵀ
    [ 0  1 ]
```

Now composing transforms is just multiplying matrices, and inverting one is a
known closed form (`R⁻¹ = Rᵀ` for rotations, so the inverse is
`[Rᵀ, −Rᵀt]`). Every robotics library does this internally.

**In this repo.** Deliberately absent, and it's instructive. Because the arm is
planar (C11), `fk()` in `arm_kinematics.py` doesn't build matrices at all — it
accumulates two scalars, `s` (radial distance) and `h` (height), and rotates once
at the end by J1. This is the payoff of a special-case mechanism: the general
tool isn't needed. **Learn the general 4×4 method anyway**, because the moment
your arm isn't planar, you need it.

## C5. Representing orientation — and why it's the messy part

Position is easy: three numbers, no ambiguity. Orientation is where the
subtleties live. Four representations, all common:

**Rotation matrix (3×3).** Nine numbers with six constraints (columns are unit
length and mutually perpendicular). Composes by matrix multiplication. No
ambiguity, no singularities. Verbose, and not human-readable.

**Euler angles / roll-pitch-yaw (3 numbers).** Three successive rotations about
named axes. Human-readable, which is why URDF uses them (`rpy="0 0 0"`). Two
problems: the order of the three rotations matters and there are a dozen
conventions, and they have **gimbal lock** — configurations where two of the three
angles do the same thing and one degree of freedom vanishes from the
representation.

**Quaternion (4 numbers).** A 4-vector with unit length. No gimbal lock,
composes cheaply, interpolates beautifully (`slerp`). Not human-readable at all.
This is what ROS messages use internally (`geometry_msgs/Quaternion`).

**Axis-angle (3 or 4 numbers).** A unit axis plus an angle about it. Very
intuitive, and the natural form for angular velocity.

**Which to use.** Store and transmit quaternions. Author URDF in RPY because
humans write it. Compute with matrices when you're composing. Think in
axis-angle.

**In this repo.** The URDF uses RPY, as URDF does. But the kinematics never
represents orientation as a 3-vector at all: it uses the single scalar
**ψ = J2 + J3 + J4**, the tool pitch. That's legitimate precisely *because* the
mechanism has only one orientation freedom — a full representation would be
three numbers, two of which are not yours to set. **Match your representation to
your mechanism's actual freedoms**, and a lot of code disappears.

**The classic mistake.** Interpolating Euler angles. Averaging two RPY triples
does not give you the rotation halfway between them, and near gimbal lock it
gives nonsense. Interpolate quaternions.

---

<a name="layer-2"></a>
# Layer 2 — Describing the mechanism

## C6. Links and joints

**What it is.** A robot is modelled as **links** (rigid bodies, C1) connected by
**joints** (constraints that permit specific relative motion). That's it. Every
robot description format encodes exactly this.

Joint types you'll meet:

| Type | Motion | DOF | Here |
|---|---|---|---|
| revolute | rotates about an axis, limited | 1 | all four arm joints |
| continuous | rotates about an axis, unlimited | 1 | wheels (none here) |
| prismatic | slides along an axis, limited | 1 | the gripper fingers |
| fixed | none | 0 | sensors, grasp frames, mast |
| floating / planar | 6 / 3 | 6 / 3 | rare, avoid |

**In this repo.** `urdf/arm.xacro` declares four revolute joints and two
prismatic finger joints per arm. The finger joints are worth reading closely:
both read 0 when closed, both take the *same* commanded value, and their axes are
mirrored (`axis xyz="0 ±1 0"`) so one value opens them symmetrically. That's a
small design decision that makes the control code trivial — one number, not two.

**The classic mistake.** Modelling something as `fixed` that actually moves, or
forgetting that a `fixed` joint still needs a sensible origin. A camera at the
wrong offset produces perception errors that look like calibration problems
forever.

## C7. Kinematic chain, tree, and loop

**What it is.** How the links are wired together.

- **Chain** — links in a line, each with one parent. An arm.
- **Tree** — branches, each link still has exactly one parent. This robot: the
  base branches into two arms and a mast.
- **Closed loop** — a link with two parents. A four-bar linkage, or *two arms
  both holding the same object.*

**Why it matters.** Trees are easy: FK is one pass from root to leaf, and URDF
can only express trees. Loops are hard: they impose constraints that must be
solved simultaneously, and URDF cannot represent them at all.

**In this repo.** This is the deep reason the bimanual case in
`docs/bimanual/` is difficult. The instant both grippers hold one object, the
kinematic tree becomes a **closed loop**, and the two arms can now *disagree*
about where the object should be. The comment on `pair_mobility()` in
`arm_kinematics.py` is precisely about this: the loop leaves 2 spare joint DOF
for the arms to fight over, and a rigid weld turns every disagreement into an
impulse. **A single-arm pick-and-place is a tree and stays easy. The moment you
add the second arm you have changed problem class.**

## C8. Degrees of freedom, configuration space, and task space

**Configuration space (C-space, joint space).** The set of all joint-angle
combinations. For this arm it's 4-dimensional: a point in it is
`q = (j1, j2, j3, j4)`. Every reachable pose of the arm is *some* point in
C-space, and moving the arm means tracing a path through it.

**Task space (operational space, Cartesian space).** The space the task is
described in — here, the 3D position of the grasp frame (plus ψ). A point in it
is where you want the hand.

**The whole of kinematics is the mapping between these two spaces.**

- Joint space → task space is **forward kinematics**. Always one answer.
- Task space → joint space is **inverse kinematics**. Zero, one, several, or
  infinitely many answers.

Compare the dimensions and you get the three regimes:

| dim(C-space) vs dim(task) | Name | Consequence |
|---|---|---|
| equal | exactly actuated | isolated IK solutions |
| greater | **redundant** | infinitely many solutions; you can optimise among them |
| less | **under-actuated** | most task-space poses are unreachable |

**In this repo.** 4 joints against a 6-DOF pose task: under-actuated. The fix is
not to complain but to *shrink the task description* to 4 numbers —
(x, y, z, ψ) — so the mapping becomes square and solvable. **That reframing is
the single most useful move in the whole codebase**, and it's what makes the IK a
closed-form solve instead of an optimisation.

**The classic mistake.** Believing "more DOF is better." Redundancy is genuinely
useful (you can avoid obstacles and joint limits with the spare freedom) but it
means IK no longer has isolated answers, so you need a resolution strategy —
pseudo-inverse, null-space projection, an optimisation objective. Under-actuation
is simpler to reason about; it's just more restrictive.

---

<a name="layer-3"></a>
# Layer 3 — Kinematics

## C9. Forward kinematics

**What it is.** Given joint angles, where is the hand? Compose one transform per
joint from base to tip:

```
T_base_hand(q) = T_0_1(q1) · T_1_2(q2) · … · T_n-1_n(qn)
```

**Why it's easy.** It's a product of known matrices. There is exactly one answer,
it always exists, and it's cheap. FK is the bedrock: you will use it to verify
IK (C36), to compute Jacobians numerically (C12), and to sanity-check
everything.

**Denavit–Hartenberg parameters.** The classical convention for writing down a
chain as 4 numbers per joint (`a, α, d, θ`) so FK becomes a fixed formula. Worth
knowing because half the literature uses it. Worth skipping in your own code —
most modern stacks just compose explicit transforms, which is far easier to debug.

**In this repo.** `fk()` in `arm_kinematics.py`, and it's a good example of
exploiting structure instead of reaching for the general method:

```python
cum = 0.0
for angle, length in ((j2, L1), (j3, L2), (j4, L3)):
    cum += angle
    s += length * math.sin(cum)
    h += length * math.cos(cum)
return (sx + s * math.cos(j1), sy + s * math.sin(j1), sz + h)
```

Two things to extract from those five lines:

- **`cum` accumulates.** Each joint angle is relative to the link before it, so
  a link's absolute tilt is the *sum* of every joint angle up to it. This is
  matrix composition, hand-rolled for the planar case.
- **Angles are measured from vertical**, hence `sin` for radial and `cos` for
  height. That convention means all-zeros = arms straight up, which has a
  visible consequence in the launch sequence (C34).

## C10. Inverse kinematics

**What it is.** Given a desired hand pose, what joint angles achieve it? This is
the one you actually need for pick-and-place, because tasks are described in task
space and robots are commanded in joint space.

**Why it's hard.** Three separate difficulties:

1. **It may have no solution.** The target may be outside the workspace, or
   inside it but unreachable at the requested orientation.
2. **It may have many.** Typically a discrete set (elbow up / elbow down,
   shoulder front / back, wrist flip), or a continuum if the arm is redundant.
3. **It's nonlinear.** The equations are trigonometric; there's no general
   closed form.

**Two families of method:**

**Analytic (closed-form).** Solve the trigonometry by hand for your specific
mechanism. Exact, instant, enumerates every solution, and tells you precisely
*why* a target fails. Only possible for mechanisms with exploitable structure —
which, happily, includes most real arms (a spherical wrist, or a planar chain).

**Numerical (iterative).** Start from a guess, compute the error, use the
Jacobian to step downhill, repeat. Works for any mechanism. But: needs a seed,
may converge to a solution you didn't want, may not converge at all, gets slow
and unstable near singularities, and gives you one answer rather than all of them.

**In this repo — the analytic solve, and why it works.** `ik()` is worth
reading as the canonical example of exploiting structure. The mechanism is J1
(yaw) then J2/J3/J4 (all pitching about the *same* axis), so the arm is a
**planar 3R chain in a vertical plane whose heading J1 selects.** Four steps:

**Step 1 — decouple J1.** Look at the target from above; its bearing off the
shoulder *is* J1. One `atan2` call, and the problem drops from 3D to 2D.

```python
j1  = math.atan2(dy, dx)
s_t = math.hypot(dx, dy)      # radial distance, in the plane
h_t = z - sz                  # height above the shoulder
```

Use `atan2(y, x)`, never `atan(y/x)`: it resolves all four quadrants and doesn't
blow up when `x` is zero.

**Step 2 — choose ψ.** Three joints remain and the 2D position consumes two, so
exactly one freedom is left over and must be spent on orientation. That's ψ, the
cumulative tool pitch from vertical. **This is the under-actuation from C8 made
concrete:** you don't get to not choose, and you don't get to choose more.

**Step 3 — reduce to a 2R problem.** The gripper is a rigid stick of length L3
pointing along ψ, so walk backwards along it from the target to find where the
wrist must be:

```python
s_w = s_t - L3 * math.sin(psi)
h_w = h_t - L3 * math.cos(psi)
```

Now you need J2 and J3 to place the wrist at a known point: two links of known
length, tip on a known spot. A triangle. **Reachability is a triangle-inequality
check**, which is why the errors are geometric rather than numerical:

```python
if r > L1 + L2:      raise Unreachable   # too far to stretch
if r < abs(L1 - L2): raise Unreachable   # inside the folded dead zone
```

Then the law of cosines gives the elbow, and one `atan2` pair gives the shoulder:

```python
c3 = (r2 - L1*L1 - L2*L2) / (2.0 * L1 * L2)
c3 = max(-1.0, min(1.0, c3))                  # clamp: see below
j3 = ±math.acos(c3)
j2 = math.atan2(s_w, h_w) - math.atan2(L2*math.sin(j3), L1 + L2*math.cos(j3))
```

That clamp is not decoration. Floating-point rounding will eventually hand
`acos` a value like `1.0000000001`, which is outside its domain and raises. Clamp
every `acos`/`asin` argument you didn't personally construct.

**Step 4 — J4 is whatever's left.** `j4 = psi - j2 - j3`. Free, by construction.

**Multiple solutions, handled explicitly.** The `±` on J3 is elbow-up versus
elbow-down: same hand position, mirrored arm shape. `ik(..., elbow="auto")` tries
both and returns the first that satisfies every joint limit, and when neither
does it raises with the offending joint names —
`elbow up: j3 out of range; elbow down: j2,j4 out of range`. **Error messages
that name the constraint are worth writing.**

**The classic mistake.** Reaching for a numerical solver before checking whether
your mechanism has structure. A planar chain, or any arm with a spherical wrist,
has a closed form — and closed form gives you every solution plus a real
diagnosis of failure, which no iterative solver will.

## C11. Decoupling — the idea behind all analytic IK

Worth naming on its own, because it's the transferable skill.

**Decoupling** means finding a joint whose effect you can determine independently
of the others, solving it, and reducing the remaining problem. Every analytic IK
is a chain of decouplings:

- Here: J1 is determined by bearing alone → 3D becomes 2D.
- Here: ψ is chosen, not solved → 3 unknowns become a 2R problem.
- On a classic 6-DOF arm: the spherical wrist's three axes intersect at a point,
  so position depends only on the first three joints → position and orientation
  decouple entirely.

**The habit:** before solving anything, ask which joints are separable. Mechanism
designers *deliberately* build in decoupling (that's why spherical wrists exist)
precisely so the IK is tractable.

## C12. The Jacobian

**What it is.** The matrix of partial derivatives mapping **joint velocities** to
**end-effector velocity**:

```
ẋ = J(q) · q̇
```

where `ẋ` is the 6-vector twist (3 linear + 3 angular) and `q̇` is the joint
velocity vector. `J` is 6×n and depends on the current configuration `q`.

**Why it matters.** Five distinct uses, all important:

1. **Velocity control.** Command a hand velocity, get joint velocities — by
   inverting or pseudo-inverting `J`.
2. **Numerical IK.** The downhill direction in iterative IK *is* the Jacobian.
3. **Singularity detection.** Where `J` loses rank, the arm has lost a direction
   of motion (C13).
4. **Force mapping.** `τ = Jᵀ · F` — the transpose maps end-effector forces to
   joint torques. This is how you check whether your motors can produce a
   required grip or resist a payload.
5. **Multi-arm constraint analysis.** Stack two arms' Jacobians and the rank
   tells you how the closed loop behaves.

**In this repo.** `jacobian()` in `arm_kinematics.py`, 6×4, and the way it's
built is a nice practical lesson:

- **Linear rows: central differences on `fk()`.** `(fk(q+ε) − fk(q−ε)) / 2ε`.
  Central differences (not forward) because the error is O(ε²) rather than O(ε).
  With 4 joints this costs 8 FK calls — trivially cheap, so not worth
  differentiating by hand.
- **Angular rows: analytic**, from a mechanism identity — since J2/J3/J4 share
  one axis, `ω = j̇1·ẑ + ψ̇·ŷ_EE`, so the angular columns are just `ẑ` and three
  copies of `ŷ_EE`. The docstring notes this avoids the ε-cancellation error a
  numerically-differentiated rotation would suffer.

**The lesson:** finite-difference the parts that are awkward, derive the parts
where differencing is ill-conditioned. Mixing the two is normal and fine.

**Where it's used:** `pair_mobility()` stacks `[J_L | −J_R]` into a 6×8 matrix
and takes its rank — a direct application of use 5 above, asking how much freedom
two arms holding one object actually have. The answer (rank 6, leaving 2 spare
joint DOF) is what predicted the welding failure documented in
`docs/bimanual/`.

## C13. Singularities and conditioning

**What it is.** A **singularity** is a configuration where the Jacobian loses
rank — the arm instantaneously cannot move the hand in some direction, no matter
what the joints do.

The canonical example is full extension. A straight arm can't move its hand
further out: radial velocity is unavailable at that instant. Others: a wrist with
two axes aligned, or a shoulder directly over its own axis.

**Why it matters.** Near (not even at) a singularity:

- Small hand motions demand enormous joint velocities. Inverting `J` blows up.
- Numerical IK becomes ill-conditioned and may fail to converge.
- The controller produces visible jerk.
- Solution branches merge — elbow-up and elbow-down become the same
  configuration, so the arm can flip unpredictably between them.

**The practical takeaway: don't get close.** Not "handle singularities robustly"
— *plan targets that stay away from them*. That's a workspace design decision,
made before any code runs.

**In this repo.** `reach_fraction()` is exactly a cheap, interpretable proxy for
distance-from-singularity:

```python
def reach_fraction(target, arm="left", psi=2.35):
    ...
    return math.hypot(s_w, h_w) / (L1 + L2)
```

1.0 means fully extended, i.e. singular. The docstring flags 0.9 as the point
where the solve becomes poorly conditioned, and `report()` in
`pick_and_place.py` prints `<-- extended` on any waypoint over 0.90. The
comments in `worlds/manipulation_task.sdf` say the furniture positions were
*chosen* so every waypoint stays under 75%, and the dry-run confirms the plan
spans 54.7%–73.1%.

**That's the pattern worth copying:** define a scalar conditioning metric, print
it for every waypoint, and put a threshold on it. You get singularity avoidance
for about ten lines of code, and you can see it in a table.

## C14. Workspace and reachability

**What it is.** The set of poses the hand can attain.

- **Reachable workspace** — positions attainable at *some* orientation.
- **Dexterous workspace** — positions attainable at *any* orientation. Much
  smaller, and empty for an under-actuated arm.

**Why it matters.** Whether your task is possible at all. This is a question to
answer *before* building anything: place the object, place the destination, check
reach. Discovering a target is unreachable after mounting hardware is expensive;
discovering it in a dry run is free.

**In this repo.** The 2R envelope is `L1 + L2` = 0.32 m, and the inner dead zone
is `|L1 − L2|` = 0 (equal links). Note that the envelope is computed at the
**wrist**, not the hand — the reachable set for the *grasp point* depends on ψ,
which is why the same target consumes 78.2% of the envelope top-down but only
56.1% diagonally. **Reachability is a function of the approach you choose**, not
a fixed property of the target.

**The classic mistake.** Checking that the target is within `L1+L2` of the
shoulder and calling it reachable. You also need: the *wrist* point (after
backing off L3 along ψ) inside the envelope, every joint inside its limits, and
enough margin that approach and retreat waypoints are reachable too.

---

<a name="layer-4"></a>
# Layer 4 — Motion

Solving IK gives you a *pose*. A task needs *motion*. This layer is the gap, and
it's the one beginners most often skip — with the result that the arm snaps
violently between poses.

## C15. Path vs trajectory — learn this distinction

Genuinely two different objects, and the words are not interchangeable:

- A **path** is a geometric sequence of configurations. No time. "Go here, then
  here, then here."
- A **trajectory** is a path **plus a time parameterisation**. "Be here at
  t = 0.4 s, here at t = 0.9 s."

**Why it matters.** Path planning is about *where* (obstacles, reachability).
Trajectory generation is about *when* (velocity, acceleration, smoothness, motor
limits). They're separate problems with separate literatures and separate failure
modes, and conflating them is why "the arm gets there but violently" is such a
common bug.

**In this repo.** Cleanly separated, which is worth noticing.
`build_plan()` produces the path: a list of `(label, q, grip, duration, reach)`
with no notion of instants. `run()` turns that into a trajectory by expanding
each waypoint's duration into `int(dur * RATE)` steps and interpolating. Same
waypoints, and you could re-time the whole motion by touching one number.

## C16. Interpolation space — joint vs Cartesian

You have a start configuration and an end configuration. What happens *between*?
Two choices, and they produce genuinely different motions:

**Joint-space interpolation.** Move each joint linearly (or smoothly) from its
start angle to its end angle.

- ✅ Trivial to compute; no IK during motion.
- ✅ Cannot fail mid-motion — every intermediate point is a valid configuration.
- ✅ No singularity trouble; you never invert the Jacobian.
- ❌ **The hand path is a curve you don't control.** Each joint moves linearly,
  but FK is nonlinear, so the composition isn't a straight line.

**Cartesian-space interpolation.** Interpolate the hand *pose* along a straight
line (or arc) and run IK at every step.

- ✅ Predictable hand path — essential for welding, painting, inserting a peg.
- ❌ IK at every timestep, so it can *fail mid-motion* if the straight line exits
  the workspace or crosses a singularity — with the arm already moving and
  holding something.
- ❌ Joint velocities can spike near singularities.

**In this repo.** Joint-space, with the trade-off documented in the module
docstring:

> Joint-space interpolation means the path between waypoints is not a straight
> line in Cartesian space — fine here because the waypoints are close together
> and the space is empty, but it is the reason for the explicit lift and transit
> waypoints rather than a direct pick-to-place move.

**That's the key consequence and it's the reason the waypoint list has nine
entries instead of two.** Since you don't control the shape of each leg, you keep
each leg *short* and *fenced away from obstacles*: come straight down onto the
object, lift straight up off it, and only travel sideways at altitude. The
`APPROACH = 0.08` standoff is that fence.

**The classic mistake.** Using Cartesian interpolation because it sounds better,
then having IK fail halfway through a move. Joint-space with well-chosen
waypoints is the right default; go Cartesian only when the hand's *path shape*
is part of the requirement.

## C17. Smoothness — position, velocity, acceleration, jerk

The derivatives of position with respect to time, and why each one matters:

| Order | Name | Physical meaning | What a discontinuity does |
|---|---|---|---|
| 0 | position | where | teleportation — physically impossible |
| 1 | velocity | how fast | requires infinite acceleration → infinite force |
| 2 | acceleration | force / torque | instantaneous force step — the arm "hammers" |
| 3 | **jerk** | rate of force change | excites structural vibration, wears gearing |

**Continuity classes.** A trajectory is C⁰ if position is continuous, C¹ if
velocity is too, C² if acceleration is too. **You want at least C², and C²
requires acceleration to be zero at the endpoints** — otherwise the very first
instant is a force step.

**Why linear interpolation is not good enough.** Interpolate linearly in time and
velocity is constant throughout — which means it jumps from 0 to full at t=0 and
back to 0 at the end. Two instants of infinite acceleration. In simulation the
arm visibly snaps; on hardware you hear it.

## C18. Time-scaling functions and the minimum-jerk profile

**What it is.** A scalar function `s(t)` that maps normalised time
`t ∈ [0,1]` to normalised progress `s ∈ [0,1]`. You then interpolate anything
with it:

```python
q = q_start + (q_end - q_start) * s(t)
```

The trajectory's smoothness is entirely a property of `s`. Common choices:

| Profile | Continuity | Notes |
|---|---|---|
| linear | C⁰ | infinite accel at both ends. Don't. |
| trapezoidal velocity | C¹ | industry standard; time-optimal under accel limits; accel steps |
| S-curve / cubic | C¹–C² | ramps the accel; most commercial controllers |
| **quintic (minimum jerk)** | **C²** | zero velocity *and* acceleration at both ends |

**The minimum-jerk polynomial.** The quintic that starts and ends at rest with
zero acceleration:

```python
def minimum_jerk(t):
    return 10 * t**3 - 15 * t**4 + 6 * t**5
```

Check it yourself — this is worth doing once by hand:

```
s(t)   = 10t³ − 15t⁴ + 6t⁵        s(0) = 0        s(1) = 10 − 15 + 6 = 1   ✓
s'(t)  = 30t² − 60t³ + 30t⁴       s'(0) = 0       s'(1) = 30 − 60 + 30 = 0  ✓
s''(t) = 60t − 180t² + 120t³      s''(0) = 0      s''(1) = 60 − 180 + 120 = 0 ✓
```

Six boundary conditions (position, velocity, acceleration at each end) determine
six coefficients — which is exactly why the polynomial is quintic. **That's the
general recipe: count your boundary conditions, and that's your polynomial
order.**

**The price of smoothness.** Smooth isn't free. Peak velocity is
`s'(0.5) = 15/8 = 1.875×` the average, and peak acceleration is
`10/√3 ≈ 5.77` (normalised). So a minimum-jerk move needs **87.5% more peak
velocity headroom** than a constant-velocity move covering the same distance in
the same time. If your joint velocity limit is what binds, that matters — it's
the reason industry often prefers trapezoidal profiles, which are time-optimal
under acceleration limits.

**In this repo.** `minimum_jerk()` in `pick_and_place.py`, applied identically to
all four joint angles and to the gripper position, inside `run()`:

```python
for k in range(1, steps + 1):
    s = minimum_jerk(k / steps)
    q = [a + (b - a) * s for a, b in zip(q_start, q_target)]
    g = grip_start + (grip_target - grip_start) * s
    send(q, g)
```

Note that every joint uses the *same* `s`. That's **synchronised** interpolation:
all joints start together, finish together, and pass through their midpoints
together. The alternative — each joint running at its own pace — makes the hand
path even less predictable.

## C19. Discretisation and control rate

**What it is.** You can't send a continuous function to a robot. You sample it at
some rate and send a sequence of setpoints.

**Choosing the rate.** Fast enough that the mechanism can't tell it's discrete —
the samples should be dense compared to the arm's own response time. Too slow and
you get visible stepping and a controller chasing a staircase; too fast and
you're flooding the middleware for nothing.

50–100 Hz is typical for position setpoint streaming; inner torque loops on real
hardware run at 1 kHz or more.

**In this repo.** `RATE = 50.0` Hz, so `dt = 20 ms`, and a 1.5-second move
becomes 75 setpoints. The loop uses `time.sleep(dt)`, which is honest but naive —
it accumulates drift because it doesn't account for the time the loop body itself
took. A production loop measures elapsed time and sleeps the remainder, or uses a
proper ROS timer. **For a scripted demo the drift is irrelevant; know that it's
there.**

---

<a name="layer-5"></a>
# Layer 5 — Grasping

## C20. Grasp geometry — aperture, approach, and the grasp frame

Three questions before any physics:

**Does it fit?** The jaw's maximum opening must exceed the object's width at the
grasp location, with margin. Here: 7.0 cm opening, 5 cm object — 2 cm of margin.
The 18 cm object simply doesn't fit, and no amount of control fixes that.

**From which direction?** The **approach axis** is the direction the gripper
travels along as it closes on the object. It must be clear of obstacles, and it's
constrained by your DOF. On this arm the approach axis is locked inside the J1
plane (no roll, no yaw), so "reach around the far side" is not available.

**Where exactly is the hold point?** That's the **grasp frame** — and it should
be the thing your IK targets, not the wrist or the palm. Here it's declared in
`arm.xacro` as a massless link between the pads, at
`palm_len + 0.6 × finger_len` along the palm axis. The `0.6` is a judgement: not
at the fingertips (unstable), not at the knuckles (object hits the palm).

## C21. Friction — the Coulomb model and the friction cone

**The model.** Dry friction, to first order:

```
F_friction ≤ μ · N
```

`N` is the normal force pressing the surfaces together, `μ` the coefficient. Note
it's an **inequality**: friction provides *up to* `μN` of resistance, no more. It
doesn't push; it only opposes.

**The friction cone.** The geometric picture, and worth internalising. At a
contact point, the total force the surface can exert lies inside a cone about the
surface normal, with half-angle `arctan(μ)`. A force inside the cone is resisted;
a force outside it means sliding.

`μ = 1.6` (this repo's pads) gives a half-angle of `arctan(1.6) ≈ 58°` — a very
wide cone, which is what soft grippy rubber buys you.

**Why this is the central concept in grasping.** A friction grasp holds *only*
because the contact forces stay inside their cones. Everything else — how hard to
squeeze, whether the object slips, whether you can accelerate while carrying —
reduces to that.

**The caveat.** Coulomb friction is a rough model. Real friction depends on
speed, contact area, surface history, and static exceeds dynamic. Simulators
implement approximations of an approximation. **Treat computed friction margins
as order-of-magnitude, and design in 5–10× slack** — which is exactly what this
repo does.

## C22. Force closure vs form closure

Two ways to hold something, and knowing which you have tells you what will make
you drop it.

**Form closure** — geometry alone prevents motion. A peg in a matching hole; a
ball in a socket. No friction needed, doesn't care how hard you squeeze.
Extremely robust.

**Force closure** — friction resists motion. A book between two palms. Needs
continuous squeeze; fails if you press too lightly, if the object is heavier than
you assumed, or if you accelerate hard enough to exceed the cone.

**In this repo.** Pure force closure, stated plainly in the module docstring:
*"The grasp is friction-only: stiff finger PIDs squeezing against mu = 1.6 pads.
No attach plugin."*

**Why this is the honest choice.** Many simulation tutorials cheat by welding
object to gripper with a `DetachableJoint` plugin. That converts the problem to
form closure and hides the whole of grasp mechanics. This project does the real
thing, which means it can genuinely drop the object — and so the force budget
below is a real calculation, not a formality. (The README documents the
`DetachableJoint` fallback, explicitly marked untested, in case contact solving
proves unreliable.)

## C23. The force budget — do this arithmetic before you debug

**What it is.** Comparing the grip force you *need* against the grip force you
*produce*. Three lines, and it will save you hours.

**Needed.** To hold a mass `m` against gravity with `k` friction contacts:

```
k · μ · N  ≥  m · g        →        N ≥ m·g / (k·μ)
```

With m = 0.08 kg, g = 9.8, k = 2 pads, μ = 1.6:

```
N ≥ 0.784 / 3.2 = 0.245 N
```

**Produced.** This repo produces grip force through **deliberate interference**
(C24): the finger is commanded 4 mm past the object's surface, so the position
controller sits at permanent error, pushing. For a proportional controller,
`F = Kp · error`:

```
F = 600 × 0.004 = 2.4 N
```

**Compare.**

```
needed     0.245 N
produced   2.4   N      ≈ 10× margin
ceiling    25    N      → operating at 10% of the actuator limit
```

Healthy on both sides: an order of magnitude more grip than required, and nowhere
near saturation or crushing.

**Do this before you touch a gain.** If the object slips and the budget says
10× margin, your problem is contact modelling, not grip force — and you'd have
wasted the afternoon turning up `Kp`.

**What the simple budget ignores** (fine here, not always): inertial load when
accelerating (add `m·a`), torque about the grasp point if the object's centre of
mass is off-axis, contact patch geometry, and the fact that static and dynamic μ
differ.

## C24. Compliance, interference, and why you command an impossible target

**What it is.** Commanding the fingers to a position *inside* the object, which
they can't reach, so the controller pushes steadily against the obstruction.

```python
GRIP_CLOSED = OBJECT_RADIUS - 0.004     # 4 mm of interference -> real squeeze
```

**Why do it this way.** A pure position controller commanded to exactly the
object's surface would settle at near-zero error and therefore near-zero force —
and drop the object. Interference converts a position command into a *force*
command, using the controller's own stiffness as the conversion factor. It's the
standard trick for getting force out of a position-controlled gripper.

**What it depends on:** knowing the object's size. 4 mm of interference on a
2.5 cm radius is tuned to *this object*. A larger object means more interference
and more force — possibly crushing; a smaller one means none at all and a drop.
**A real gripper uses force sensing or a genuinely compliant mechanism** (series
elastic, spring-loaded jaws) so the grip force doesn't depend on knowing the
geometry in advance.

**The concept to take away: stiffness sets the position–force exchange rate.**
Stiff joints track position well and resist disturbance; compliant joints are
gentle and safe on contact. Fingers want stiff (they must hold); arm joints want
softer (they must not fight the world). That is precisely why the gains in this
repo differ by 100× between arm and fingers (C26).

---

<a name="layer-6"></a>
# Layer 6 — Control

## C25. Control modes — what you're allowed to command

A joint actuator generally accepts one of three kinds of command, and which one
you get shapes everything above it:

| Mode | You command | Good for | Weak at |
|---|---|---|---|
| **position** | an angle | point-to-point, holding | contact, force control |
| **velocity** | a rate | smooth tracking, mobile bases | position drifts over time |
| **effort** (torque) | a torque | contact, compliance, force control | needs good dynamics model |

**In this repo.** Position control, throughout. Twelve
`gz-sim-joint-position-controller-system` plugin instances declared in
`urdf/robot.urdf.xacro`, each listening on its own
`/arm/<joint>/cmd_pos` topic carrying a single `std_msgs/Float64`.

**What position control implies.** You can't directly command a grip *force* —
hence the interference trick (C24). You also can't make the arm compliant on
contact; it will push until its effort limit. For a pick-and-place in free space
that's fine, and it's much easier than torque control, which needs an accurate
dynamics model.

## C26. PID

**What it is.** The workhorse feedback law. Given error `e = target − actual`:

```
output = Kp·e  +  Ki·∫e dt  +  Kd·(de/dt)
```

- **P** — proportional to current error. The main restoring push. Alone, it
  leaves steady-state error under constant load (gravity), because zero error
  would mean zero force.
- **I** — accumulated past error. Kills that steady-state droop. Too much and it
  "winds up", overshooting after a saturation or a long error period — hence
  `i_max` / `i_min` clamps in the plugin config.
- **D** — proportional to the *rate* of error change. Damping; reduces
  overshoot and oscillation. Amplifies sensor noise.

**Tuning intuition.** Raise P until it responds briskly and just begins to
oscillate; add D to damp the oscillation; add just enough I to remove residual
droop. Gains scale with the *load* a joint carries.

**In this repo.**

| Joint | P | I | D | effort limit |
|---|---|---|---|---|
| J1 | 6.0 | 0.05 | 0.6 | 5 N·m |
| J2 | 9.0 | 0.10 | 0.9 | 5 N·m |
| J3 | 7.0 | 0.08 | 0.7 | 5 N·m |
| J4 | 4.0 | 0.05 | 0.4 | 5 N·m |
| fingers | **600.0** | **8.0** | **20.0** | **25 N** |

Two readable patterns:

1. **Arm gains descend outward** (J2 = 9 → J4 = 4). J2 carries the whole arm's
   weight plus payload; J4 carries only the gripper. **Gains track load.**
2. **Fingers are 100× stiffer.** That's C23/C24 showing up in a config file:
   `Kp = 600` is precisely what turns 4 mm of interference into 2.4 N of grip. A
   soft finger controller would let go.

**The diagnostic worth knowing.** If arms *droop* instead of holding their pose,
the position controllers didn't load at all — check the Gazebo console for
`JointPositionController` errors. Drooping is not a tuning symptom; it's an
absence-of-controller symptom.

## C27. Saturation, limits, and feedforward

**Saturation.** Every actuator has a ceiling (`cmd_max` here; 5 N·m for arm
joints, 25 N for fingers). Past it, the controller's output is clipped, and while
clipped the loop is effectively open — feedback has stopped working. The I term
keeps integrating, which is windup. **Always know your saturation margin**; this
repo's grasp uses 10% of the finger ceiling.

**Joint limits.** Mechanical range. Enforced in the URDF `<limit>` tags, and
independently checked in `ik()` — which is the right place, because a limit
violation should be caught *before* motion, as a planning failure, not
discovered by the hardware.

**Feedforward and gravity compensation.** PID is purely reactive: it needs error
before it acts. If you can *predict* the required effort — gravity on a known
payload, say — you can add it directly and let PID correct only the residual.
This is how good arms hold position without large I terms. Not present in this
repo (position control with adequate gains is enough at these masses) but it's
the standard next step.

---

<a name="layer-7"></a>
# Layer 7 — Physics and simulation

Only needed because this project is simulated — but a badly-specified simulation
produces bugs that look exactly like control bugs, so it's worth knowing which
is which.

## C28. Mass, centre of mass, and the inertia tensor

Three quantities per link, and all three must be specified:

**Mass** — resistance to linear acceleration.

**Centre of mass** — the point where mass effectively acts. Offset from the
joint, it creates a gravity torque, which is what your controller fights.

**Inertia tensor** — resistance to *angular* acceleration. A 3×3 symmetric
matrix (6 independent numbers); the diagonal `ixx, iyy, izz` are the moments
about each axis. It's "rotational mass", and it depends on how mass is
*distributed*, not just how much there is.

**Why you must get it roughly right.** Bad inertia doesn't produce a clean error
— it produces a simulation that *runs* but behaves wrongly: links that spin
absurdly easily, or refuse to move, or make the solver unstable. It is a
miserable bug to chase because it looks like a control problem.

**Don't guess — use the formula for the shape.** For a solid cylinder of mass m,
radius r, length L:

```
about its own axis:               izz = m·r²/2
about a perpendicular axis:  ixx = iyy = m·(3r² + L²)/12
```

**In this repo.** `common.xacro` defines `cylinder_inertial` and `box_inertial`
macros so no link's inertia is ever typed by hand — it's computed from the mass
and dimensions already declared. That's the same "push the arithmetic into the
description" habit as the sensor heights. The world objects are done by hand and
they check out:

```
object_min (m = 0.08, r = 0.025, L = 0.05):
  izz = 0.08 × 0.025²/2              = 0.0000250   ✓ matches the SDF
  ixx = 0.08 × (3×0.025² + 0.05²)/12 = 0.0000292   ✓ matches the SDF
```

## C29. The parallel-axis theorem

**What it is.** How to get the inertia of a compound body: compute each piece's
inertia about *its own* centre, then shift it to the shared reference point:

```
I_about_new_axis = I_about_own_com + m·d²
```

where `d` is the distance between the axes. Sum the shifted inertias.

**Why you need it.** Any link made of more than one primitive. Here:
`object_bimanual` is a tray box plus two handle posts, and its inertia was
computed by summing three shifted inertias — the derivation is in
`docs/bimanual/IMPLEMENTATION.md`.

## C30. Numerical integration and the timestep

**What it is.** A physics engine advances the world in discrete steps: compute
forces, integrate to get velocities and positions, repeat.

**Why the step size matters.** Too large and the integration is inaccurate and
can go unstable — objects jitter, sink through floors, or gain energy. Too small
and the simulation is slow. Stiff systems (high contact stiffness, high gains)
need smaller steps, because a stiff spring's forces change fast.

**In this repo.** `max_step_size = 0.001` — 1 ms, i.e. 1000 Hz. That's
fine-grained, and it's a deliberate trade: this project's entire grasp depends on
contact (C31), contact is the least stable part of any physics engine, and small
steps are what buy stability.

**Real-time factor.** `real_time_factor = 1.0` asks simulated time to track
wall-clock time. If your machine can't keep up, simulation runs slower than
real time — which matters here, because `run()` paces itself with
`time.sleep()` on the *wall clock*. If the sim lags, your setpoints arrive faster
in sim-time than intended. (Bridging `/clock` is what lets nodes use simulated
time instead; this repo bridges it.)

## C31. Contact modelling

**What it is.** How the engine resolves two bodies touching. Most engines let
bodies interpenetrate slightly and generate a restoring force from the
penetration depth — a stiff spring-damper:

```
F = kp · penetration  +  kd · penetration_rate
```

**The parameters.**

- `kp` — contact stiffness. High = rigid, realistic, but demands small timesteps.
  Low = squishy, objects visibly sink in.
- `kd` — contact damping. Absorbs impact energy and stops bouncing.
- `mu`, `mu2` — friction coefficients along the two tangent directions (C21).

**In this repo.** The finger pads (`arm.xacro`) and `object_min`
(`manipulation_task.sdf`) both set `mu = 1.6, kp = 1e6, kd = 100`. The comment on
the pad block is blunt and correct: *"Grippy pads. Without this the object
squirts out under contact."*

**Why contact is the hard part.** Everything else in a physics simulation is
smooth differential equations. Contact is discontinuous — a force that doesn't
exist one instant and is large the next — and friction adds a non-smooth
inequality on top. Nearly all "why does my simulated grasp fail" questions are
contact questions. This is the layer where sim and reality diverge most.

---

<a name="layer-8"></a>
# Layer 8 — Software plumbing

## C32. The robot description

**What it is.** A machine-readable statement of the links, joints, geometry,
inertias, and sensors. Everything else — the visualiser, the physics engine, the
TF tree, the collision checker — reads it. **One description, many consumers**,
which is why the format matters.

- **URDF** — the ROS standard. Links and joints. No variables, no loops, so
  hand-writing it is painfully repetitive. Trees only.
- **xacro** — a macro preprocessor that adds variables, arithmetic, includes and
  macros, then expands to plain URDF. Every `.xacro` here is URDF with the
  repetition removed.
- **SDF** — Gazebo's native format. Richer (worlds, lights, physics, plugins).
  Used here for the world.

**In this repo — the pattern worth stealing.** All dimensions live in
`urdf/common.xacro` and everything else derives from them. The arm is *one*
macro in `arm.xacro`, instantiated twice with a mirror flag. Sensor mount offsets
are *computed* from specified above-ground heights rather than typed. Inertias
are *computed* by macro from mass and dimensions.

**Single source of truth, and where it breaks.** The discipline has one
documented hole, and it's the biggest trap in the codebase:
`nodes/arm_kinematics.py` re-declares `MOUNT_X`, `MOUNT_Y`, `J2_Z`, `L1`, `L2`,
`L3` as **hand-copied constants**, because Python can't read xacro. Change the
geometry in one and not the other and **IK silently solves for a robot that
doesn't exist** — no error, just an arm that misses. Same for
`BIMANUAL_HANDLE_SEP` versus the world SDF. The general lesson: when a value must
live in two languages, make the duplication loud, and say so in a comment.

## C33. Middleware — nodes, topics, messages

**What it is.** Robot software is many processes that must exchange data.
ROS 2 provides the plumbing:

- **Node** — a process (or component) that does one job.
- **Topic** — a named channel with a fixed message type.
- **Publisher / Subscriber** — write to and read from topics, anonymously and
  many-to-many.
- **Message** — a typed struct.

**Why anonymous pub/sub.** Producers don't know who consumes. You can add a
logger, a plotter, a second controller, without touching the producer. It's what
makes robot stacks composable.

**Other patterns you'll meet:** *services* (request/response, for one-shot
queries), *actions* (long-running goals with feedback and cancellation — the
right pattern for "execute this trajectory"), and *parameters* (per-node config).

**In this repo.** Minimal and explicit: 12 publishers of
`std_msgs/Float64`, one per joint, created in `run()`. Each message is literally
one number.

**Two practical details in that function worth copying.**

```python
# let the bridge match subscriptions before the first setpoint
end = time.time() + 2.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
```

Pub/sub needs time to *discover* peers. Publish immediately after creating a
publisher and the message goes nowhere — nobody is connected yet. This is the
single most common "my first ROS node doesn't work" bug.

```python
# park the idle arm out of the way and hold it there
for n, v in zip([f"{other}_j{i}" for i in (1, 2, 3, 4)], HOME):
    pubs[n].publish(Float64(data=float(v)))
```

And: **there is no "arm" abstraction here.** Twelve independent joints, twelve
independent topics, nothing synchronising them. Any coordination lives entirely
in the publishing node. That's the minimum viable control layer — and knowing
that's what you have prevents a lot of confusion about why the two arms don't
move together.

## C34. TF and `robot_state_publisher`

**What it is.** `tf2` is the subsystem that maintains the tree of frames (C3)
over time and answers "where is frame A relative to frame B, at time t?"

**How it gets populated.** `robot_state_publisher` reads the robot description,
subscribes to `/joint_states` (the measured joint angles), computes FK for the
whole tree, and publishes every frame relationship to `/tf`. **This is forward
kinematics as a service** — and it means everything downstream (RViz, perception,
any node that needs a transform) gets frames for free.

**In this repo.** Launched in `spawn.launch.py`. `/joint_states` comes out of
Gazebo through the bridge. Result: 21 frames, verifiable with
`ros2 run tf2_tools view_frames`.

**One launch-file landmine, documented because it costs people an afternoon:**
the description must be passed as

```python
ParameterValue(Command(["xacro ", ...]), value_type=str)
```

Omit `value_type=str` and parameter type inference **fails silently** — no error,
nothing spawns.

## C35. Launch orchestration and startup ordering

**What it is.** Bringing up several processes in a valid order. Robot systems have
real startup dependencies: the simulator must be running before you can spawn
into it; the description must be published before `robot_state_publisher` can use
it; controllers must be loaded before setpoints mean anything.

**Two approaches.** *Readiness checks* — wait for the actual condition (service
available, topic present). Robust, more code. *Timers* — wait a fixed duration
and hope. Simple, and it can lose the race on a slow machine.

**In this repo.** Timers, with the trade-off acknowledged in the code:

```
t = 0 s   gz-sim starts, loads the world
t = 0 s   robot_state_publisher starts (runs xacro)
t = 4 s   ros_gz_sim create  — spawn the robot
t = 8 s   pose_cmd.py home   — drive the arms off their zero pose
```

The 4 s wait is for Gazebo to advertise its spawn service. The 8 s one is a
consequence of C9's angle convention: all-joints-zero means arms straight up,
where the three links stack into what looks like a single fused rod, so the
launch drives them to a folded `HOME` to make the articulation visible.

## C36. Bridging two middlewares

**What it is.** Gazebo has its own transport, separate from ROS 2. A bridge
process translates messages between them.

**In this repo.** `ros_gz_bridge`, configured by `config/bridge.yaml`, 17 topics:

| Direction | Topics |
|---|---|
| Gazebo → ROS | `/clock`, `/joint_states`, `/scan`, `/camera/image_raw`, `/camera/camera_info` |
| ROS → Gazebo | the 12 `cmd_pos` command topics |

**Worth knowing:** the bridge is a real process with real latency and a real
chance of being misconfigured. A topic missing from `bridge.yaml` simply doesn't
exist on the other side — the publisher succeeds, and nothing happens. When a
command has no effect, check the bridge before you check your logic.

---

<a name="layer-9"></a>
# Layer 9 — Task structure and verification

## C37. Task decomposition — the canonical pick-and-place phases

**What it is.** The standard phase structure. Nearly every pick-and-place, at
every scale, is this sequence:

```
  approach → descend → close → lift → transit → pre-place → descend → open → retreat
```

**Why each phase exists** — these are not padding:

| Phase | Why it's separate |
|---|---|
| **approach / pre-grasp** | arrive at a standoff above the object, so the final motion is a clean straight descent along the approach axis |
| **descend** | move onto the object along that axis only |
| **close** | grip with the arm *stationary* — never grab while moving |
| **lift** | separate from the support surface before any lateral motion |
| **transit** | travel at altitude, clear of everything |
| **pre-place** | standoff above the destination |
| **descend / open** | set down, then release stationary |
| **retreat** | withdraw along the approach axis before doing anything else |

**Approach and retreat standoffs are the important part.** They exist because of
C16: with joint-space interpolation you don't control the shape of a leg, so you
keep legs short and vertical near obstacles, and do the long travel in open
space. `APPROACH = 0.08` is that 8 cm margin.

**In this repo.** `build_plan()` is exactly the canonical nine, and the dry-run
table shows the structure clearly: `grasp` and `close` are identical
configurations differing only in grip, as are `place` and `open` — which is the
"grip while stationary" rule, visible as data.

## C38. Plan-then-execute, and failing before you move

**What it is.** Solve the entire task *before* commanding any motion.

**Why it matters enormously.** The alternative — solve each waypoint as you reach
it — means an infeasible waypoint is discovered **mid-motion, with the arm
extended and the object in the gripper.** There's no good recovery from that.
Plan-first turns a runtime failure into a validation failure, which is the
cheapest possible place to find it.

**In this repo.** The docstring on `build_plan()` states the contract —
*"Solve every waypoint up front. Raises before anything moves if the task is not
kinematically feasible."* — and `main()` honours it:

```python
try:
    plan = build_plan()
except K.Unreachable as e:
    print(f"PLAN INFEASIBLE: {e}", file=sys.stderr)
    return 1
```

Nothing has been published at that point. Nothing can be left in a bad state.

**What you'd grow into.** A fully pre-computed plan can't react. Real systems add
**state machines** or **behaviour trees** — explicit states with transitions on
sensed events (contact detected, grasp confirmed, object missing), so the task can
retry, skip, or abort. That's the natural next step once perception is in the
loop; a scripted demo doesn't need it, and adding it before you need it just
obscures the sequence.

## C39. Verification — round-tripping, dry runs, observability

The concept people skip, and then pay for.

**Round-trip testing.** The strongest available test for kinematics. IK is hard
to verify directly — you'd need an independent source of truth. But **FK is easy
and exact**, so you can generate that truth yourself:

```
random joint angles q  →  FK  →  a position p  →  IK  →  q'  →  FK  →  p'
```

Then assert `p ≈ p'`. If IK is wrong, the loop doesn't close. No reference
implementation needed, and it covers thousands of configurations including ones
you'd never think to test.

**In this repo.** `python3 nodes/arm_kinematics.py` runs exactly this over 4000
random poses per arm. Current output:

```
=== FK/IK round-trip, 4000 random reachable poses ===
  solved 3948, unreachable 4052
  worst position error: 0.000 micrometres
```

Two things to read there. The worst error is zero to float precision — expected,
since both directions are closed-form, and any nonzero value would mean a real
bug. And **more samples were rejected than solved** (4052 vs 3948), because
random joint angles frequently produce poses whose mirrored IK solution violates
a limit. That's not a failure; it's the joint limits doing their job, and the
test correctly counts rather than ignores them.

Note also that the test compares *positions*, not joint angles — because with two
valid elbow solutions, `q'` legitimately need not equal `q`. **Compare the thing
you actually care about**, not an intermediate representation.

**Dry runs.** Execute all the computation and none of the actuation.
`pick_and_place.py --dry-run` solves and prints the full plan without importing
`rclpy` at all — so it runs with no simulator, no ROS, in a fraction of a second.
That makes the expensive part of the task (is this feasible, is it
well-conditioned) testable in a loop fast enough to iterate in.

**Observability.** Know what to look at, and what the healthy value is:

```bash
ros2 run tf2_tools view_frames            # 21 frames rooted at base_footprint
ros2 topic hz /scan                       # ~10 Hz
ros2 topic hz /camera/image_raw           # ~15 Hz
ros2 topic echo /joint_states --once      # 8 names: left_j1..j4, right_j1..j4
```

Plus one visual check no command gives you: **do the arms hold their pose, or
droop?** Drooping means the position controllers never loaded.

**The general principle: "it ran without crashing" is not verification.** Define,
in advance, the observable that tells you it worked — a number, a frame count, a
rate, a round-trip error — and check that.

---

<a name="afterword"></a>
# Afterword — what this project left out, and what each omission is called

Everything above is what a *scripted* pick-and-place needs. Here's what a
*robust* one adds, named so you can go read about each:

| Omitted | Concept | What it buys |
|---|---|---|
| `ros2_control` | hardware abstraction, controller manager | the same code drives sim and real hardware; real trajectory controllers with proper action interfaces |
| **MoveIt** | motion planning | collision-aware paths (this project relies on an empty workspace and hand-chosen waypoints) |
| collision checking | self- and environment-collision geometry | knowing a path is safe rather than assuming it |
| **perception** | pose estimation from sensors | the camera and LiDAR are *published but unused* — object positions are hard-coded constants. Closing that loop is the single biggest step toward a real system |
| force/tactile sensing | closed-loop grasping | grip force from measurement instead of assumed geometry (C24) |
| torque control | inverse dynamics, gravity compensation | compliance, safe contact, faster motion |
| navigation | SLAM, path planning, wheel control | the base is a rigid body with no wheels; approach is not modelled |
| behaviour trees / state machines | reactive task execution | retry, recover, abort (C38) |
| bimanual coordination | closed-loop kinematics, mobility analysis | see `docs/bimanual/` — and note it's a **different problem class** (C7), not an increment |

**The order I'd add them in:** perception first (it's the one that turns a
demo into a system), then `ros2_control` (so the code survives contact with
hardware), then collision-aware planning. Force sensing when the grasp starts
being the thing that fails.

---

# The dependency graph — a suggested learning order

Concepts that must come before others:

```
  pose (C2) ──► frames (C3) ──► transforms (C4) ──┐
                                                  ├──► forward kinematics (C9)
  links & joints (C6) ──► DOF / C-space (C8) ─────┘         │
                                                            ▼
                                          Jacobian (C12) ◄── inverse kinematics (C10)
                                                │                     │
                                                ▼                     ▼
                                     singularities (C13) ──► workspace (C14)
                                                                      │
                                                                      ▼
                            path vs trajectory (C15) ──► interpolation space (C16)
                                                                      │
                                                                      ▼
                                         smoothness (C17) ──► time scaling (C18)
                                                                      │
   friction (C21) ──► closure (C22) ──► force budget (C23)            │
                                             │                        │
   PID (C26) ──► stiffness / compliance (C24)┘                        │
                                             │                        │
                                             └────────┬───────────────┘
                                                      ▼
                                       task decomposition (C37)
                                                      │
                                                      ▼
                                      plan-then-execute (C38) ──► verification (C39)
```

**A concrete study path, in this repo:**

1. Read `urdf/common.xacro` end to end. It's the whole mechanism in one screen.
2. Read `nodes/arm_kinematics.py`. `fk()` then `ik()`, with C9–C11 beside you.
3. Run `python3 nodes/arm_kinematics.py` — see the round-trip close (C39).
4. Run `pick_and_place.py --dry-run` — read the reach column against C13.
5. Read `build_plan()` against C37, then `run()` against C15–C19.
6. Only then launch the simulator. By that point you'll know what you're looking
   at, and what "working" is supposed to look like.

**And the meta-lesson from this codebase**, which generalises past robotics: the
project's best decisions were all *reframings*, not implementations — shrinking a
6-DOF task to 4 numbers so IK closes; choosing ψ = 135° to buy back a quarter of
the reach envelope; putting handles on the object instead of adding a joint to
the arm; and writing down what's impossible, with the number that proves it.
