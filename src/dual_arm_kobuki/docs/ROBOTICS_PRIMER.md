# A Robotics Primer: The Concepts Behind Any Manipulator Project

**Who this is for:** you're new to robotics and looking at a project like this one
— a simulated robot with arms, sensors, and a scripted task — and the
vocabulary is a wall. This document is not about this project. It's the
general-purpose robotics knowledge that makes *any* project like it readable:
a mobile base, one or more arms, a gripper, some sensors, a physics simulator,
and a program that moves things around.

Every concept below is explained with generic, made-up examples — a
hypothetical 2-link arm, a hypothetical sensor — not this specific robot's
numbers. Once you understand the concept here, go back to whatever project
you're reading and you'll recognize it immediately, whatever the actual
numbers turn out to be.

**How to use this document:** it's organized as ten layers, each building on
the one before. Read it in order the first time. After that, it's a
reference — jump to whichever concept you've hit.

---

# Contents

1. [The vocabulary of "a robot"](#1)
2. [Describing where things are](#2) — pose, coordinate frames, transforms
3. [Describing how a robot is built](#3) — links, joints, degrees of freedom
4. [Kinematics](#4) — forward and inverse, the Jacobian, singularities
5. [Motion](#5) — paths, trajectories, smoothness
6. [Grasping](#6) — friction, closure, force
7. [Control](#7) — feedback loops, PID, control modes
8. [Sensing](#8) — cameras, range sensors, proprioception
9. [Simulation](#9) — why simulators exist and where they lie to you
10. [Software architecture](#10) — how the pieces talk to each other
- [Putting it together](#together) — reading a typical project end to end
- [Glossary](#glossary)

---

<a name="1"></a>
## 1. The vocabulary of "a robot"

Strip away the specifics and almost every robot is the same four things:

```
   SENSORS ──► [ SOFTWARE / "BRAIN" ] ──► ACTUATORS
                       │
                   STRUCTURE
              (the rigid body it's all bolted to)
```

- **Structure** — the physical body: links, a chassis, a gripper. It doesn't
  do anything by itself; it's what everything else acts on and through.
- **Actuators** — things that make parts move: motors at joints, wheels,
  pneumatic cylinders. An actuator turns a command ("go to this angle") into
  physical motion.
- **Sensors** — things that produce information about the world or the robot
  itself: cameras, range finders, encoders that report a joint's current
  angle.
- **Software** — the part that decides what to command, using what the
  sensors report. This ranges from "a fixed script" (do these five things in
  order, ignore the sensors) to "a fully autonomous system" (build a plan
  from what the sensors currently show).

**The first useful distinction: open-loop vs. closed-loop.**

- **Open-loop**: the software issues commands based on a plan, without
  checking whether they actually happened. "Move joint 1 to 30°, then joint 2
  to 45°" — sent once, trusted to work.
- **Closed-loop**: the software checks sensor feedback and corrects. "Move
  joint 1 toward 30°, keep checking the encoder, keep adjusting until it's
  actually at 30°."

Nearly all real motion control is closed-loop at some level (see §7), but
whole *tasks* are often scripted open-loop on top of that — a fixed sequence
of moves, each one individually closed-loop, but with no sensing that changes
the sequence itself. That's the simplest kind of robot program, and a
reasonable one to build first.

**The second useful distinction: simulation vs. reality.**

A huge amount of robotics work happens in **simulation**: software that
models physics (gravity, contact, friction) and pretends to be the sensors
and actuators, so you can write and test robot code without any hardware.
Simulation is cheap, safe, and repeatable, but it is always an approximation
— see §9 for where it typically diverges from reality.

---

<a name="2"></a>
## 2. Describing where things are

### 2.1 Rigid bodies

A **rigid body** is an object whose points never move relative to each other
— it can move through space, but it doesn't bend, stretch, or deform. A brick
is a rigid body; a rope is not.

**Why this assumption matters so much:** if a body is rigid, knowing the
position of *one* reference point on it, plus which way it's turned, tells
you where *every other point on it* is. That one fact is why robotics
problems are tractable at all — every rigid body reduces to a small, fixed
number of quantities instead of an infinite cloud of points. Virtually every
part of a robot's structure — each link, the chassis, a rigid tool — is
modeled as a rigid body, even though in reality every material flexes a
little.

### 2.2 Pose: position + orientation

The **pose** of a rigid body is the complete answer to "where is it, and
which way is it facing?" It splits into two halves:

- **Position** — where a chosen reference point on the body is, given as
  three numbers (x, y, z) relative to some origin.
- **Orientation** — which way the body is rotated, also expressible in three
  independent numbers (though not always stored that way — see 2.5).

So a full pose is **6 numbers**, and a rigid body in free space is said to
have **6 degrees of freedom (DOF)**: 3 translational + 3 rotational.

**Why the number 6 keeps showing up:** to place a hand at any position, in
any orientation, you generally need a mechanism with 6 independently
controllable motions. A mechanism with fewer can't achieve arbitrary poses —
it's called **under-actuated**, and you have to decide in advance which parts
of the 6 you're willing to give up. This single number is the most important
constant in manipulator design, and it comes up again in §3 and §4.

### 2.3 Coordinate frames

A **coordinate frame** ("frame" for short) is an origin plus a set of three
mutually perpendicular axis directions, attached to some point of reference.
A position is *only* meaningful relative to a frame — "the object is at
(0.3, 0.1, 0.2)" means nothing until you say relative to what.

Robots need many frames simultaneously, because different parts of the
system naturally describe things relative to different points:

- A camera reports what it sees relative to *itself*.
- A gripper's target is most naturally described relative to *the arm's
  base*.
- The whole robot's position is most naturally described relative to *the
  room, or the world*.

```
        world frame
            │
        (robot's position in the room)
            │
        robot base frame
            │
        (arm mounted somewhere on the robot)
            │
        arm base frame
            │
        (chain of joints — see §3)
            │
        end-effector frame  ◄── "where the hand actually is"
```

Every rigid part of a robot conceptually has its own frame attached to it.
Keeping track of "which frame is this number expressed in" is one of the
most common sources of confusing bugs in robotics — a number with the right
units and a plausible magnitude, computed in the wrong frame, produces
motion that goes confidently in the wrong direction.

### 2.4 Transforms

A **transform** converts a position (or a full pose) expressed in one frame
into the equivalent expressed in another frame. It bundles a rotation and a
translation together.

Two properties make transforms practical:

- **They compose.** If you know the transform from frame A to frame B, and
  from B to C, you can chain them to get directly from A to C. This is *why*
  a long kinematic chain (§3) can be reasoned about one joint at a time: each
  joint contributes one local transform, and the overall transform from base
  to hand is just the product of all of them.
- **They invert.** If you know the transform from A to B, you automatically
  know the transform from B to A — it's the mathematical inverse.

The standard computational trick is the **4×4 homogeneous transformation
matrix**, which packs a 3×3 rotation and a 3×1 translation into one matrix so
that composing transforms becomes ordinary matrix multiplication. You don't
need to hand-derive this to use robotics software — libraries do it for you
— but recognizing the pattern (a chain of frames, each one a transform away
from the last) is the mental model that makes forward kinematics (§4.1)
click.

### 2.5 Representing orientation

Position is simple: three numbers, unambiguous. Orientation is where
robotics math gets genuinely fussy, because there are several ways to write
down "which way is it turned," each with a real trade-off.

| Representation | How many numbers | Strength | Weakness |
|---|---|---|---|
| **Rotation matrix** | 9 (with 6 constraints) | Unambiguous; composes by matrix multiply | Verbose, not human-readable |
| **Euler angles** (roll / pitch / yaw) | 3 | Intuitive, human-friendly | Order-dependent; suffers **gimbal lock** |
| **Quaternion** | 4 (unit length) | No gimbal lock, compact, interpolates smoothly | Not human-readable at all |
| **Axis-angle** | 3–4 | Very intuitive, natural for rotational velocity | Awkward to combine multiple rotations |

**Gimbal lock**, briefly: Euler angles apply three rotations in sequence
about named axes. In certain orientations, two of those three rotation axes
become parallel, and the system permanently loses the ability to represent
rotation around one direction — a full degree of freedom vanishes from the
*representation*, even though the physical body can still rotate freely.
It's a famous trap and the main reason quaternions exist.

**The practical rule of thumb:** humans write and read Euler angles (that's
why robot description files usually use them for readability), software
*stores and computes with* quaternions or matrices internally, and nobody
should ever try to average or blend two sets of Euler angles directly —
interpolate quaternions instead.

**A shortcut worth knowing:** if a mechanism genuinely can't achieve every
orientation (because it's under-actuated, §2.2), its orientation is often
best described with fewer numbers than the general case — sometimes just a
single angle. Using a representation that matches what the mechanism can
*actually* do, rather than always reaching for the full 3-number general
case, is one of the more elegant simplifications in kinematics.

---

<a name="3"></a>
## 3. Describing how a robot is built

### 3.1 Links and joints

A robot's moving structure is built from two ingredients, repeated:

- **Links** — rigid bodies (§2.1): a rod, a plate, a housing.
- **Joints** — connections between two links that constrain how they can
  move relative to each other.

| Joint type | Motion allowed | Typical use |
|---|---|---|
| **Revolute** | Rotates about one axis, usually within limits | Elbows, shoulders, wrists |
| **Prismatic** | Slides along one axis, within limits | Linear rails, gripper fingers |
| **Continuous** | Rotates about one axis, no limits | Wheels |
| **Fixed** | No motion at all | Bolting a sensor or tool rigidly to a link |

A **fixed** joint might look pointless — nothing moves — but it's how you
say "this sensor sits at exactly this offset from that link," which is
essential bookkeeping even though it contributes no motion.

### 3.2 Kinematic chains, trees, and loops

How links and joints connect to each other has its own vocabulary, and it
matters for how hard the resulting math is:

- **Chain** — a straight-line sequence of links: link–joint–link–joint...
  A single arm is a chain.
- **Tree** — chains that branch, but every link still has exactly one
  parent. A robot with a body that branches into two arms and a sensor mast
  is a tree.
- **Closed loop** — a link that has *two* parents, because two separate
  chains both connect back to it. A four-bar linkage is a closed loop. So is
  the moment two independent arms both grip the *same* rigid object — the
  object becomes a link with two parents, one through each arm.

**Why this distinction matters:** trees are comparatively easy. You can
compute where anything is with one pass from the root out to the tips (see
§4.1), and most description formats can only represent trees at all. Closed
loops are qualitatively harder: the two paths into the shared link must agree
with each other, which becomes an extra set of constraints that has to be
solved simultaneously rather than computed by simple traversal.

**The generalizable lesson:** the instant a task requires two independently
actuated chains to jointly hold one rigid thing, you have changed problem
class, not just added complexity. It is worth recognizing that transition
explicitly rather than discovering it as a mysterious wave of new bugs.

### 3.3 Degrees of freedom, and the two spaces of a mechanism

A joint that allows one independent motion (an angle, or a sliding distance)
contributes **one degree of freedom**. A mechanism's total DOF is the sum
across all its joints, and it defines two related but distinct spaces:

- **Configuration space** (also "joint space" or "C-space") — every possible
  combination of joint values. For a mechanism with *n* joints, a point in
  this space is one specific set of *n* numbers: "joint 1 at this angle,
  joint 2 at this angle, ..." This is the space a mechanism is *actually
  commanded in*.
- **Task space** (also "operational space" or "Cartesian space") — the space
  the *task* is naturally described in: usually the pose of some tool or
  hand. This is the space a task is *naturally described in*.

Comparing the dimensions of these two spaces tells you what kind of problem
you have:

| DOF vs. task dimension | Name | What it means |
|---|---|---|
| Equal | **Exactly actuated** | A target pose typically has a small, fixed number of joint solutions |
| DOF > task dimension | **Redundant** | Infinitely many joint solutions reach the same pose — extra freedom to optimize with (avoid obstacles, stay away from limits) |
| DOF < task dimension | **Under-actuated** | Most task-space poses are simply unreachable; you must give something up |

Under-actuated mechanisms are common and completely normal — a huge amount
of practical mechanism design deliberately trades DOF for simplicity, cost,
or reliability. The engineering skill is knowing exactly *which* freedoms
you're giving up and designing the task around that, rather than treating it
as a limitation to fight.

### 3.4 Workspace and reachability

The **workspace** of a mechanism is the set of poses its end-effector (hand,
tool tip) can actually reach.

- The **reachable workspace** is every *position* attainable at *some*
  orientation.
- The **dexterous workspace** is every position attainable at *any*
  orientation — always smaller, often much smaller, and can be empty for an
  under-actuated mechanism.

**A subtlety worth internalizing:** for an under-actuated mechanism, how much
of the workspace envelope a given target consumes depends not just on where
the target is, but on *which orientation you approach it with* — because
achieving different orientations consumes different amounts of the
mechanism's limited reach. Choosing an efficient approach orientation can
make an otherwise marginal target comfortable, and choosing a poor one can
make an easily-reachable target awkward or impossible. This is a genuinely
useful design lever, not a fixed property of the geometry alone.

---

<a name="4"></a>
## 4. Kinematics

Kinematics is the study of motion and geometry *without* reference to the
forces that cause it — purely "if the joints are at these values, where is
everything," and its inverse. It's the mathematical core of any manipulator
project.

### 4.1 Forward kinematics (FK)

**The question:** given every joint's current value, where is the
end-effector?

**Why it's the easy direction:** you walk the chain from the base outward,
composing one transform (§2.4) per joint, and multiply them together. There
is exactly one answer, it always exists, and it's cheap to compute. Forward
kinematics is the bedrock operation everything else is checked against.

A tiny illustrative example: imagine a 2-link arm in a single vertical
plane, with link lengths `L1` and `L2`, and two joint angles `θ1` (measured
from the base) and `θ2` (measured relative to the first link, so it "carries
along" with it). The tip's position works out to:

```
x = L1·cos(θ1) + L2·cos(θ1 + θ2)
y = L1·sin(θ1) + L2·sin(θ1 + θ2)
```

Notice the **cumulative angle** `(θ1 + θ2)` in the second term — because
`θ2` is measured relative to the first link, the second link's true
orientation in the world is the *sum* of both joint angles. This "angles
accumulate down the chain" idea generalizes directly: with more joints, each
link's absolute orientation is the running sum of every joint angle between
it and the base. It's the same mathematical idea as composing transforms
(§2.4), just written out arithmetically instead of as matrices.

### 4.2 Inverse kinematics (IK)

**The question:** given a desired end-effector pose, what joint values
achieve it? This is the direction you actually need for most tasks, because
tasks are naturally specified in task space ("put the hand *here*") while
the robot is commanded in joint space.

**Why it's hard**, in three distinct ways:

1. **It might have no solution.** The target could be outside the
   workspace, or inside it but unreachable at the requested orientation.
2. **It might have many solutions.** A common case is a discrete handful —
   "elbow up" vs. "elbow down," "shoulder forward" vs. "shoulder back" — but
   a redundant mechanism (§3.3) can have an entire continuous family of
   solutions.
3. **It's nonlinear.** The equations involve trigonometric functions of the
   unknowns, so there's no simple algebraic rearrangement in general.

**Two families of solution method:**

- **Analytic (closed-form)** — solve the trigonometry by hand for a specific
  mechanism's geometry. When a mechanism has exploitable structure (see
  below), this gives an exact answer instantly, enumerates *every* solution,
  and can tell you precisely *why* a particular target fails. Only possible
  when the mechanism's geometry cooperates.
- **Numerical (iterative)** — start from a guess, measure the error, take a
  small step that reduces it (using the Jacobian, §4.3), and repeat until
  converged. Works for essentially any mechanism, including very complex
  ones, but needs a starting guess, may converge to a solution you didn't
  want, may fail to converge at all, and gets numerically unstable near
  singularities (§4.4).

**The key transferable skill: decoupling.** Almost every analytic IK
solution works by finding one joint (or a small group) whose effect can be
determined *independently* of the rest, solving that piece first, and using
it to reduce the remaining problem. A classic example on 6-DOF industrial
arms: if the last three joints' rotation axes all intersect at one point (a
"spherical wrist"), then the *position* of that point depends only on the
first three joints — position and orientation decouple entirely, and each
half becomes a much simpler sub-problem. Mechanism designers often build
this kind of structure in *on purpose*, precisely so the inverse kinematics
stays tractable. Before reaching for a numerical solver, it's always worth
asking: does this mechanism have exploitable structure?

**Multiple solutions are a feature, not a bug, to detect explicitly.** A
sound IK implementation for a mechanism with several solution branches
should enumerate the candidates, check each against the mechanism's actual
joint limits, and pick (or return) a valid one — with a specific, useful
error when *none* is valid, ideally naming which constraint was violated.
"No solution" and "a solution exists but violates joint limits" are
different failure modes worth distinguishing.

### 4.3 The Jacobian

**What it is:** the matrix of partial derivatives relating joint
*velocities* to end-effector *velocity*:

```
ẋ = J(q) · q̇
```

where `q̇` is the vector of joint velocities, `ẋ` is the end-effector's
velocity (typically 3 linear + 3 angular components — a "twist"), and `J`
is the Jacobian matrix, which itself depends on the current joint
configuration `q`.

**Why it matters, in five distinct ways:**

1. **Velocity control** — command a desired hand velocity, and invert (or
   pseudo-invert) the Jacobian to find the joint velocities that produce it.
2. **Numerical IK** — the Jacobian gives the local "downhill" direction that
   an iterative solver steps along.
3. **Singularity detection** — where the Jacobian loses rank (§4.4), the
   mechanism has momentarily lost the ability to move in some direction.
4. **Force mapping** — the *transpose* of the Jacobian maps forces at the
   end-effector to torques required at each joint: `τ = Jᵀ·F`. This is how
   you check whether available actuator torque is enough to produce a
   required grip force or resist an external load.
5. **Multi-mechanism analysis** — when two chains jointly constrain one
   object (the closed-loop case, §3.2), stacking their Jacobians and
   examining the combined matrix's rank tells you how the two chains'
   freedoms interact — including whether they have room to disagree with
   each other, which matters enormously for anything like a two-armed carry.

A Jacobian can be derived analytically when the mechanism's FK is written
out symbolically, or approximated numerically by nudging each joint slightly
and measuring how much the end-effector moves (a **finite-difference**
approximation) — a fine and common approach when the mechanism has few
joints, since the cost is only a couple of extra FK evaluations per joint.

### 4.4 Singularities and conditioning

A **singularity** is a configuration where the Jacobian loses rank — the
mechanism has instantaneously lost the ability to move its end-effector in
at least one direction, no matter how the joints move. The textbook example
is full extension: a fully straightened arm cannot, in that instant, move
its hand any further away from the base — that direction of motion has
vanished.

**Why singularities matter in practice**, even when you never sit exactly
on one:

- **Near** a singularity, small desired motions at the hand require
  disproportionately large joint velocities — any calculation that inverts
  the Jacobian becomes numerically unstable.
- Iterative IK solvers slow down or fail to converge.
- Motion controllers produce visibly jerky output.
- Distinct solution branches (like elbow-up and elbow-down) merge into one,
  so a small perturbation can cause the mechanism to unpredictably flip
  between them.

**The practical takeaway is almost always "avoid," not "handle
gracefully."** The standard engineering response is to define a simple,
cheap scalar metric — some measure of how close the current configuration is
to full extension or another known singular pose — and keep planned targets
comfortably below a threshold on that metric. This turns an intimidating
piece of theory into a single number you can compute per target and print in
a table, which is a far more tractable engineering habit than trying to
"robustly handle" the singularity in real time.

### 4.5 Workspace design as a first-class activity

Because of §4.4 (avoid, don't handle) and §3.4 (approach orientation affects
reach), a recurring and very general pattern in manipulator projects is:
**choose the geometry of the task itself** — where fixtures sit, how far
apart two work surfaces are, which orientation the tool approaches with — so
that every point the robot needs to reach falls comfortably inside a
well-conditioned part of the workspace, with margin. This is done *before*
writing any motion code, using nothing but the forward-kinematics-derived
reach metric above. It's cheap, and it prevents an entire category of
failures that would otherwise only surface once real motion is attempted.

---

<a name="5"></a>
## 5. Motion

Solving inverse kinematics gives you a single target *pose*. A task requires
*motion* between poses — and this is the layer where a surprising amount of
"my simulated or real robot moves violently" bugs originate, because it's
easy to skip past it.

### 5.1 Path vs. trajectory — a distinction worth holding onto precisely

These two words are often used loosely, but they refer to genuinely
different things:

- A **path** is a purely geometric sequence of configurations, with no
  notion of time: "go through this point, then this one, then this one."
- A **trajectory** is a path *plus* a time parameterization: "be at this
  point at this instant, and at that point at that later instant."

Path planning is about *where* — reachability, obstacles, geometric
feasibility. Trajectory generation is about *when* — velocity, acceleration,
smoothness, actuator limits. They are genuinely separate problems, and
keeping them conceptually and often literally separate in code (compute the
path first, worry about timing second) is a clean and common architecture.

### 5.2 Interpolation space: joint space vs. Cartesian space

Once you have a start configuration and an end configuration, *how* do you
move between them? Two fundamentally different choices exist, and they
produce genuinely different physical motion:

**Joint-space interpolation** — move each joint smoothly and independently
from its start value to its end value.

- ✓ Cheap: no inverse kinematics needed during the motion itself.
- ✓ Cannot fail partway through — every intermediate configuration is, by
  construction, a valid set of joint values.
- ✓ No singularity concerns during the move, since you never invert a
  Jacobian.
- ✗ **The resulting path of the end-effector through space is a curve you
  don't directly control.** Each joint moves in a straight line through its
  own value, but forward kinematics is nonlinear, so the *combination* is
  generally not a straight line in task space.

**Cartesian-space interpolation** — interpolate the end-effector's *pose*
directly (e.g., along a straight line), solving inverse kinematics fresh at
every intermediate point.

- ✓ Predictable, controllable end-effector path — essential when the shape
  of that path matters (drawing a line, inserting a part along an axis).
- ✗ Requires solving IK repeatedly during motion, which can fail *mid-move*
  if the straight line happens to cross outside the workspace or through a
  singularity — a much worse time to discover infeasibility than before the
  motion started.
- ✗ Joint velocities can spike unpredictably near singularities.

**The generalizable design consequence:** if you choose joint-space
interpolation (often the simpler and more robust default), you no longer
control the exact shape of the path between two waypoints. The standard
mitigation is to insert *more* waypoints — an approach/retreat point held
directly above a work surface, say — so that each individual leg of motion
is short and happens in open space, and the parts of the motion near
obstacles are simple straight vertical moves rather than long curved
unknowns.

### 5.3 Smoothness: position, velocity, acceleration, jerk

Every additional time-derivative of position tells you something physically
important, and a discontinuity in one has a specific consequence:

| Derivative order | Name | Physical meaning | Consequence of a jump |
|---|---|---|---|
| 0th | Position | Where something is | A jump is teleportation — impossible |
| 1st | Velocity | How fast it's moving | A jump requires infinite acceleration → infinite force |
| 2nd | Acceleration | Proportional to force/torque | A jump is an instantaneous force step — visible as a jerk or "hammer" |
| 3rd | **Jerk** | Rate of change of force | A jump excites vibration and wears mechanical components over time |

**Continuity classes**, a standard shorthand: a trajectory is `C⁰` if
position is continuous, `C¹` if velocity is also continuous, `C²` if
acceleration is also continuous. **You generally want at least `C²`**, and
achieving it requires the trajectory's acceleration to be *zero* at both
endpoints — otherwise the very first instant of motion is already a step in
force.

Plain linear interpolation over time has *constant* velocity throughout,
which means velocity jumps instantly from zero to its full value at the
start and back to zero at the end — two instants of theoretically infinite
acceleration. In a simulator this shows up as a visible snap; on physical
hardware it's audible and, over time, damaging.

### 5.4 Time-scaling functions and smooth profiles

A clean way to generate a smooth trajectory: define a single scalar function
`s(t)` that ramps from 0 to 1 as normalized time `t` goes from 0 to 1, with
whatever smoothness properties you need, and then use it to interpolate
*anything* — every joint's angle, a gripper's opening, all synchronized to
the same `s`:

```
value(t) = start + (end - start) · s(t)
```

Common choices for `s(t)`, in increasing order of smoothness:

| Profile | Continuity | Note |
|---|---|---|
| Linear | `C⁰` | Infinite acceleration at both ends — generally avoid |
| Trapezoidal velocity | `C¹` | Ramp up, cruise, ramp down; the traditional industrial default; still has acceleration steps |
| S-curve | `C¹`–`C²` | Smooths the acceleration ramps themselves too |
| **Quintic ("minimum-jerk")** | `C²` | Zero velocity *and* zero acceleration at both endpoints |

A minimum-jerk profile is a fifth-degree polynomial in normalized time,
`s(t) = 10t³ − 15t⁴ + 6t⁵`, chosen because a 5th-order polynomial has exactly
six free coefficients, and there are exactly six boundary conditions to
satisfy: position, velocity, and acceleration, each specified at both the
start and the end. **This "count your boundary conditions, that's your
polynomial order" logic generalizes** — it's the same reasoning behind any
polynomial trajectory profile, however many derivatives you want to
constrain.

```
        1.0 ┤                              ╭────
            │                          ╭────╯
  progress  │                     ╭────╯
    s(t)    │                ╭────╯
            │            ╭───╯
            │        ╭───╯
        0.0 ┼────────╯
            0                                  1
                    normalized time  t

  slope (velocity) starts at 0, peaks in the
  middle, and returns to 0 — no snap at either end
```

**Smoothness has a cost, and it's worth knowing the number:** a minimum-jerk
profile's *peak* velocity, for a move covering a given distance in a given
time, is about **1.875× the average velocity** the same move would need
under constant speed. So a smoother motion needs meaningfully more velocity
*headroom* to complete in the same time — which is exactly why some
industrial systems prefer trapezoidal profiles (they're time-optimal under a
hard acceleration limit) over minimum-jerk ones, trading a small amount of
smoothness for a smaller peak-velocity requirement.

### 5.5 Sampling and control rate

A continuous trajectory function eventually has to become a *sequence* of
discrete commands, sent at some fixed rate. Choosing that rate is a genuine
trade-off: too slow, and the motion visibly "steps" and the low-level
controller (§7) is left chasing a staircase instead of a smooth curve; too
fast, and you're spending computation and communication bandwidth for no
perceptible benefit, since the mechanism physically can't respond that
quickly anyway. Setpoint streaming for position control is commonly in the
tens-to-low-hundreds of Hz; tight inner control loops (particularly torque
control) often run at 1 kHz or faster.

---

<a name="6"></a>
## 6. Grasping

### 6.1 Grasp geometry: three questions before any physics

- **Does it fit?** The gripper's maximum opening has to exceed the object's
  width at the intended grasp point, with margin for controller imprecision
  and manufacturing tolerance.
- **From which direction?** The **approach axis** is the direction the
  gripper travels as it closes in on the object. It has to be clear of
  obstacles, and it's constrained by the mechanism's actual orientation
  freedoms (§2.2, §3.3) — an under-actuated arm may simply be unable to
  approach from certain directions at all.
- **Where exactly, on the object, is the hold point?** This matters enough
  that it's usually given its own frame (§2.3) rather than being computed
  ad-hoc every time — a fixed point conceptually located between the
  gripper's contact surfaces, which is what inverse kinematics actually
  targets.

### 6.2 Friction: the Coulomb model and the friction cone

The simplest useful model of dry friction:

```
F_friction ≤ μ · N
```

`N` is the normal force pressing two surfaces together, and `μ`
("mu") is the coefficient of friction between them. This is an
**inequality**, not an equation — friction resists motion *up to* that
maximum; it never actively pushes on its own.

The geometric picture that follows from this is the **friction cone**: at
any point of contact, the full range of forces the surface can transmit
without slipping forms a cone around the surface's normal direction, with a
half-angle of `arctan(μ)`. A contact force pointing inside that cone is held
in place; a force pointing outside it causes sliding. Higher friction
coefficients mean wider cones and more forgiving grasps.

```
                    surface normal
                          │
                    ╲     │     ╱
                     ╲    │    ╱      ← the friction cone:
                      ╲   │   ╱          any contact force
                       ╲  │  ╱           inside here is held
                     ────●────
                      the contact
                         point
```

**Why this is the central idea in grasping:** whether an object stays in a
gripper reduces, fundamentally, to whether the contact forces at every
touching surface stay inside their respective friction cones under all the
loads the object experiences — its own weight, any acceleration, any
external disturbance.

**A necessary caveat:** Coulomb friction is a simplified model. Real
friction depends on speed, surface area, surface wear, and history, and the
coefficient that applies just before sliding starts (static friction) is
generally higher than the one that applies once sliding is underway (kinetic
friction). Any calculation based on this model should be treated as an
order-of-magnitude estimate, not an exact prediction — which is exactly why
sound grasp designs build in generous safety margin (often 5–10×) rather
than targeting the calculated minimum.

### 6.3 Force closure vs. form closure

Two fundamentally different ways an object can be held in place:

- **Form closure** — the geometry itself prevents motion, independent of
  friction: a peg fitting a matching hole, a ball seated in a socket. No
  ongoing force is required to maintain it; it doesn't matter how hard
  anything is squeezing.
- **Force closure** — friction (or another applied force) is actively
  resisting motion: a book held between two flat palms. This requires
  continuous, sufficient force; it fails if the squeeze is too light, if the
  object turns out heavier than assumed, or if an acceleration pushes the
  required force outside the friction cone.

Knowing which kind of grasp you have tells you exactly what will make you
drop the object — a form-closure grasp is essentially immune to squeeze
force as long as the geometry stays engaged, while a force-closure grasp is
only as good as its ongoing force budget (§6.4).

### 6.4 The grasp force budget

A concrete habit worth adopting whenever grasping is friction-based: before
debugging *anything* about a slipping grasp, do the arithmetic that compares
force *needed* against force *available*. For an object of mass `m` held by
`k` frictional contact points, each with coefficient `μ`, the minimum total
normal force needed to resist gravity is:

```
k · μ · N  ≥  m · g          (g = gravitational acceleration)
```

Solve for `N`, and compare that "needed" number against what the gripper's
actuator is actually capable of producing, given its own physical limits.
If the ratio between available and needed force is comfortably large (a
healthy margin, given the caveats in §6.2), and an object is still slipping
in practice, the likely culprit is a *modeling* problem — contact surfaces
not actually touching as intended, an incorrect friction coefficient, or a
control problem in how force is actually being commanded (§6.5) — not
insufficient squeeze force. Doing this arithmetic first, before touching any
tuning parameter, routinely saves hours of misdirected debugging.

### 6.5 Compliance and commanding force through position control

A subtlety that surprises many newcomers: many simple grippers are
**position-controlled** (§7.3) — you command an angle or a distance, not a
force directly. So how do you make a position-controlled gripper squeeze
with a specific force?

The standard trick is **deliberate interference**: command the gripper's
fingers to a position slightly *inside* where the object's actual surface
is — a position the fingers physically cannot reach, because the object is
in the way. The position controller then sits at a small, permanent
position error, continuously pushing against the obstruction. For a simple
proportional controller (§7.2), the resulting force is approximately:

```
F ≈ Kp × (commanded position − actual achievable position)
```

This converts a position command into a *force* command, using the
controller's own stiffness as the conversion factor between the two. It's a
clever, widely-used technique — and it comes with a real limitation worth
knowing: because it depends on knowing the object's geometry in advance, a
larger-than-expected object produces more interference and more force
(potentially damaging), while a smaller one produces none at all (and drops
the object). A gripper with genuine force sensing, or true mechanical
compliance (a spring-loaded mechanism), avoids this dependency, at the cost
of additional hardware.

---

<a name="7"></a>
## 7. Control

### 7.1 Control modes: what you're actually allowed to command

An actuator generally accepts commands in one of three modes, and this
choice shapes everything built on top of it:

| Mode | What you send it | Strong at | Weak at |
|---|---|---|---|
| **Position** | A target angle or distance | Point-to-point motion, holding a pose | Anything involving contact or force |
| **Velocity** | A target speed | Smooth continuous tracking, driving wheels | Position drifts over time if not corrected |
| **Effort / torque** | A target torque or force | Contact, compliance, direct force control | Needs an accurate model of the mechanism's own dynamics to use well |

Position control is by far the simplest to reason about and is extremely
common for point-to-point tasks, but it fundamentally cannot express "push
with this much force" directly — which is exactly why techniques like
deliberate interference (§6.5) exist as a workaround.

### 7.2 PID: the standard feedback loop

**PID** (Proportional-Integral-Derivative) is the default feedback control
law used across almost all of engineering, not just robotics. Given an
error `e = target − actual`, the controller's output is:

```
output = Kp·e  +  Ki·∫e dt  +  Kd·(de/dt)
```

- **P (proportional)** — pushes in proportion to the *current* error. Alone,
  it typically leaves a small steady-state error under any constant load
  (like gravity), because zero error would mean zero corrective force —
  which isn't enough to hold something up against a constant pull.
- **I (integral)** — accumulates *past* error over time, which eliminates
  that steady-state droop. Too much integral gain causes "windup" — the
  accumulated term overshoots and takes a while to unwind, especially after
  the actuator has been saturated (§7.4) for a while.
- **D (derivative)** — reacts to the *rate of change* of error, which damps
  oscillation and overshoot. It amplifies whatever noise is present in the
  sensor signal, so it's used carefully.

**Practical tuning intuition:** raise P until the response is brisk and just
starts to oscillate; add D to damp that oscillation back out; add just
enough I to eliminate any remaining steady-state error, being mindful of
windup.

A joint carrying more load (weight, or a payload) generally needs higher
gains than one carrying less — gains should roughly track the mechanical
load a given joint or actuator has to support. And an actuator that must
*hold firmly* (like a gripper maintaining a grasp, §6.5) typically uses much
higher stiffness (much higher `Kp`) than one that mostly needs to move
smoothly through free space without fighting anything — a soft, low-gain
joint is gentler and safer around unexpected contact, while a stiff,
high-gain one tracks its target precisely and resists disturbance.

### 7.3 A quick diagnostic worth remembering

If a joint (or an entire limb) simply **sags** under gravity instead of
holding its commanded position, that's usually not a tuning problem at all —
it's a sign that no position controller is actually running on that joint
(the software layer that's supposed to be closing the feedback loop isn't
active, or never started). Sagging under load and "the gains are too low"
look superficially similar but usually have completely different root
causes, and it's worth checking for the simpler one (a missing or crashed
controller) before spending time re-tuning.

### 7.4 Saturation, limits, and feedforward

**Saturation** — every real actuator has a maximum output (a torque or force
ceiling). Once a controller's computed output exceeds that ceiling, the
actual output is clipped, and while clipped, the feedback loop is
effectively broken — the actuator can't produce any more corrective effect
no matter how large the computed error grows. An integral term that keeps
accumulating error during this period causes windup (§7.2); many practical
controllers explicitly clamp the integral term's contribution to guard
against this.

**Joint and mechanism limits** — physical range-of-motion boundaries. These
are best checked as early as possible in a pipeline — ideally during
planning (§4.2), *before* any motion begins — rather than discovered only
when hardware physically hits a limit stop.

**Feedforward** — a pure feedback controller (like PID above) only reacts
*after* an error already exists; it fundamentally needs some error before it
can act. If part of the required effort can be *predicted* in advance — most
commonly, the torque needed just to hold a known payload against gravity —
adding that predicted amount directly, and letting the feedback loop correct
only the smaller *remaining* error, produces much steadier, tighter control
with less reliance on aggressive gains. This is the standard next step
beyond plain PID for anything that has to hold a known, predictable load.

---

<a name="8"></a>
## 8. Sensing

### 8.1 Two broad sensor categories

- **Proprioceptive** sensors measure the robot's *own* internal state —
  joint encoders (current angle), IMUs (orientation and acceleration of the
  body itself), current sensors on motors.
- **Exteroceptive** sensors measure the *external* world — cameras, range
  finders, force/torque sensors on a tool.

A basic scripted task can often get by on proprioception alone (open-loop
with respect to the world, closed-loop only at the joint level). Anything
that needs to react to *where objects actually are* — rather than assuming
fixed, pre-known positions — needs exteroceptive sensing, and that's
generally the single biggest step from "a scripted demo" to "a system that
actually perceives and adapts."

### 8.2 Cameras

A camera reports a 2D grid of color or intensity values. Key parameters to
understand for any camera in a robotics context:

- **Resolution** — the width × height of that grid, in pixels.
- **Field of view (FOV)** — the angular extent of the world visible in the
  frame, usually given in degrees horizontally and vertically.
- **Frame rate** — how many images per second it produces, in Hz.

```
                    camera
                       ▲
                      ╱ ╲
                     ╱   ╲   ← field of view: the angular
                    ╱     ╲     wedge of the world that
                   ╱───────╲    appears in every frame
              (nothing outside this wedge is visible)
```

A single ordinary camera gives you a 2D projection — it tells you *what*
is in a given direction, but not directly *how far away* it is; recovering
distance from a single camera image generally requires additional
processing (stereo pairs, known object sizes, or other cues).

### 8.3 Range sensors (LiDAR and similar)

A range sensor measures *distance* directly, typically by timing a
reflected signal (commonly a laser, hence "LiDAR" — light detection and
ranging). A **2D LiDAR** sweeps a single flat plane and reports distance as
a function of angle within that plane — it produces a "slice" through the
world at one fixed height, not a full 3D picture. Key parameters:

- **Angular range and resolution** — how much of a full circle it covers,
  and how finely it samples angle (e.g., one reading per degree).
- **Range** — the minimum and maximum distance it can reliably measure; most
  range sensors have a "dead zone" too close to the sensor where they can't
  measure at all.
- **Update rate** — how many full sweeps per second.

2D LiDAR is a very common, comparatively cheap way to give a mobile robot
awareness of nearby obstacles in a horizontal plane, without needing to
interpret camera imagery at all.

### 8.4 Publish, but not necessarily use

A common and worth-noting pattern in robotics projects, especially early or
simplified ones: sensors are physically or virtually present and actively
producing data, but the software driving the robot's actual behavior doesn't
read that data at all — object positions or navigation decisions are
hard-coded or otherwise assumed rather than perceived. This isn't
necessarily a flaw; it's often a deliberate, reasonable simplification for
an early milestone. But it's worth explicitly recognizing when a project is
in this state, because "the sensors are there" and "the sensors are actually
used" are very different claims, and closing that gap — genuinely perceiving
and reacting to the world rather than assuming it — is usually the single
highest-value next step toward a more capable system.

---

<a name="9"></a>
## 9. Simulation

### 9.1 Why simulate at all

Physics simulators model gravity, rigid-body dynamics, and contact well
enough to let you write and test robot control software without any
physical hardware. This is cheap, perfectly repeatable (the same script
produces the same outcome every time, modulo any randomness you introduce
deliberately), and safe — a bug can't damage anything. It is, correspondingly,
always an *approximation* of reality, and knowing where that approximation
tends to break down is a core simulation skill in itself.

### 9.2 Mass, center of mass, and the inertia tensor

Every rigid body in a simulation needs three physical properties specified:

- **Mass** — resistance to *linear* acceleration (how hard it is to speed
  up or slow down in a straight line).
- **Center of mass** — the effective point where all the mass "acts." If
  it's offset from a joint's axis, gravity creates a torque about that
  joint that a controller has to actively counteract.
- **Inertia tensor** — resistance to *angular* acceleration (how hard it is
  to start or stop something spinning). It's a 3×3 matrix that depends on
  how the mass is *distributed* through the body's shape, not merely on
  the total amount of mass.

**Why this matters more than it might seem:** incorrect inertia values
don't usually produce an obvious error message — they produce a simulation
that runs without complaint but behaves subtly wrong: parts that spin
absurdly easily, refuse to rotate at all, or destabilize the whole physics
solver. This looks exactly like a control-tuning problem and can be a
genuinely frustrating bug to track down. The reliable fix is to compute
these values from the standard formula for the body's actual shape (a
cylinder, box, sphere — each has a known closed-form formula) rather than
guessing plausible-looking numbers.

For compound shapes (a body made of multiple simple pieces), the
**parallel-axis theorem** lets you combine each piece's own inertia,
computed about its own center, into a combined inertia about one shared
reference point — sum each piece's own inertia plus a correction term based
on how far that piece's center is from the shared point.

### 9.3 Discrete time steps

A simulator doesn't solve physics continuously — it advances the world in
small discrete steps: compute the forces acting on everything, integrate
those forces forward by one small time increment to get new velocities and
positions, and repeat. The size of that time step is a genuine engineering
trade-off:

- **Too large**, and the simulation becomes inaccurate and can go
  numerically unstable — objects visibly jitter, sink through surfaces
  they should rest on, or spontaneously gain energy they shouldn't have.
- **Too small**, and the simulation consumes more computation for the same
  amount of simulated time — it runs slower relative to real time.

**Stiffer systems need smaller steps.** Anything involving high-stiffness
contact (a hard collision) or high-gain control (very stiff feedback,
§7.2) changes forces very rapidly, and rapidly-changing forces need finer
time resolution to integrate accurately. This is exactly why a project
whose core mechanism is friction-based grasping (§6) — which depends
entirely on contact being modeled well — often deliberately runs a finer
time step than a project without any contact-critical behavior, even
though that costs more computation.

### 9.4 Contact modeling

Simulating two solid objects touching is one of the hardest and least
"physically clean" parts of any simulator, because real contact is
genuinely discontinuous: a force that simply doesn't exist the instant
before touching, and can be very large the instant after.

Most practical simulators handle this by allowing a small, unphysical
amount of interpenetration between touching bodies, and generating a
restoring force from that penetration depth — essentially treating contact
as an extremely stiff spring-damper:

```
F_contact = kp · (penetration depth)  +  kd · (rate of penetration change)
```

- **`kp`** (contact stiffness) — higher values push interpenetrating bodies
  apart more forcefully and behave more like a truly rigid, realistic
  contact, but demand a smaller time step (§9.3) to remain numerically
  stable.
- **`kd`** (contact damping) — absorbs impact energy and prevents unwanted
  bouncing.
- Separately, friction coefficients (§6.2) determine how much *tangential*
  (sideways) force the contact can resist before slipping.

**Why this is often the hardest layer to get right:** every other part of a
rigid-body simulation is a smooth differential equation with well-understood
numerical methods. Contact is discontinuous, and friction adds a further
non-smooth inequality constraint on top of that. In practice, the large
majority of "why does my simulated grasp or contact behave strangely"
questions trace back to contact parameters, not to anything in the control
or planning layers above it — and this layer is also where simulated
behavior most often diverges noticeably from real-world behavior.

### 9.5 Real-time factor

Most simulators can report (and sometimes target) a **real-time factor**:
the ratio of simulated time elapsed to actual wall-clock time elapsed. A
real-time factor of 1.0 means the simulation is keeping pace with a real
clock; less than 1.0 means it's running slower than real time (common on
underpowered hardware or with very fine time steps). This matters
practically whenever external software paces itself using the real,
wall-clock time rather than the simulator's own internal clock — if the
simulation is running behind, that software's commands can arrive "too
fast" relative to what the simulated robot has actually had time to do.

---

<a name="10"></a>
## 10. Software architecture

### 10.1 Describing a robot as data, not code

Rather than hand-writing code that constructs every link and joint, robot
projects typically describe the whole structure in a **declarative
description file**: a data format listing every link's shape and mass,
every joint's type, axis, and limits, and every sensor's mounting position
— read by many different pieces of software (a visualizer, a physics
engine, motion-planning code, kinematics code) rather than reimplemented in
each one.

Because hand-authoring such a file for anything non-trivial is extremely
repetitive (a robot with two arms needs the same arm structure described
twice, for instance), a common pattern is a **macro/templating layer** on
top of the base format — parameterized building blocks that expand out
into the full, repetitive description automatically. This lets one set of
numbers (link lengths, mount offsets) drive an entire, correctly-repeated
structure, and lets you change the robot's dimensions in one place rather
than several.

**A structural trap worth knowing in advance:** any numeric value that has
to be duplicated by hand into a *second* file or language — because, say,
a lightweight scripting or math module can't directly read the primary
declarative description format — creates an easy-to-miss failure mode:
change the value in one place, forget the other, and the two no longer
agree. Nothing necessarily raises an error; code that depends on the
stale copy simply computes against the wrong geometry from then on,
silently. The general mitigation is either to eliminate the duplication
entirely (generate the second copy automatically from the first) or, when
that's genuinely impractical, to make the duplication loud and obvious —
comments in both locations explicitly cross-referencing each other, so a
future edit in one place is more likely to trigger an edit in the other.

### 10.2 Middleware: nodes, topics, and messages

Robot software is rarely one monolithic program — it's typically several
independent processes ("**nodes**") that need to exchange data: a
perception process, a planning process, a low-level control process. A
**middleware** layer provides the plumbing for this, most commonly through
**publish/subscribe** messaging:

- A **topic** is a named channel carrying a specific, fixed type of message.
- **Publishers** write data to a topic; **subscribers** read from it.
- Critically, publishers and subscribers don't know about each other
  directly — many-to-many, and anonymous.

**Why this anonymity is valuable:** you can add a new logger, a debugging
visualizer, or an entirely new consumer of some data stream without
touching the code that originally produced it. It's what makes a robot's
software genuinely composable rather than one large, tightly-coupled
program.

Beyond simple pub/sub, two other common middleware patterns are worth
knowing: **services** (a direct request/response call — "give me this one
piece of information right now") for one-shot queries, and **actions**
(a long-running goal with ongoing feedback and the ability to cancel) —
the natural fit for something like "execute this entire trajectory and
tell me your progress."

**A practical gotcha that trips up nearly everyone starting out:**
publish/subscribe connections take a small but nonzero amount of time to
be *discovered* after a program starts up. Publishing a message
immediately after creating a publisher, before any subscriber has had time
to discover and connect to it, means that first message simply goes
nowhere — silently. The common fix is a short deliberate pause (or an
explicit handshake) after setting up communication, before sending
anything that actually matters.

### 10.3 Frame trees over time

Building on §2.3 (coordinate frames): a running robot's frame relationships
change continuously as joints move, so there's typically a dedicated
subsystem whose entire job is maintaining the full tree of frame
relationships *over time* and answering, on demand, "where is frame A
relative to frame B, right now (or at some specific past time)?" Any
downstream software — a visualizer, a perception pipeline, motion planning
— can then ask for a transform between any two frames without needing to
know or recompute the mechanism's own kinematics itself; that computation
happens once, centrally, and everyone else just consumes the result. This
is effectively forward kinematics (§4.1) running continuously as a shared
service, rather than being recomputed independently and possibly
inconsistently by every piece of software that happens to need it.

### 10.4 Startup ordering

Bringing up a real or simulated robot system involves genuine sequencing
dependencies: a physics or hardware layer generally needs to be running
before anything can be spawned or connected into it; a robot's description
needs to be available before other software can use it; low-level
controllers need to be loaded and active before sending them any
setpoint is meaningful. Two broad strategies exist for enforcing this
ordering:

- **Readiness checks** — explicitly wait for the real, observable condition
  to become true (a required service becomes available, a particular topic
  starts publishing) before proceeding to the next step. More robust,
  generally more code to write.
- **Fixed delays** — simply wait a hardcoded amount of time and assume
  that's long enough. Much simpler to write, but can fail unpredictably on
  a slower machine or under unusually heavy system load, where the
  hardcoded delay turns out not to have been long enough after all.

Fixed delays are a completely reasonable, pragmatic choice for a
straightforward setup, as long as the trade-off being made is understood
and, ideally, noted for whoever reads the code later.

### 10.5 Plan-then-execute, and failing early

A widely applicable and valuable pattern: fully compute an entire task
plan — resolve *every* step's feasibility, run all the necessary
kinematics — **before** issuing a single real command that causes physical
motion.

**Why this matters so much:** the alternative — solving each step only when
the robot actually reaches it — means that if some later step in the
sequence turns out to be infeasible, you discover that fact only *midway
through execution*, quite possibly with an arm already extended and
something already held in the gripper. There is rarely a good, safe
recovery from that state. Planning everything up front converts what would
otherwise be a messy *runtime* failure into a clean *validation* failure,
caught and reported before anything physical has happened — which is
close to the cheapest possible place in the whole process to discover a
problem.

The natural next step beyond a single fully-precomputed plan, once a
system needs to genuinely react to sensed events (an object turns out not
to be where it was assumed to be, a grasp doesn't succeed, contact is
detected earlier or later than expected) is a **state machine** or
**behavior tree**: an explicit set of named states or behaviors with
defined transitions triggered by sensed conditions, so the software can
retry a step, skip it, or abort the whole task cleanly, rather than blindly
continuing to execute a plan built on assumptions that have since turned
out to be wrong.

### 10.6 Verification: what actually counts as "it worked"

**The single most important habit in this entire list:** define, in
advance, a specific, checkable thing that tells you a piece of code or a
running system is actually correct — and then genuinely check that thing.
"It ran without crashing" or "the simulator window opened and something
moved" is not verification; it's merely an absence of the most obvious
possible failure.

Some broadly reusable techniques:

- **Round-trip testing** for any pair of inverse operations (like FK and
  IK): apply the forward operation to a large number of random valid
  inputs, feed each result into the inverse operation, apply the forward
  operation again, and check that you land back where you started. This is
  a powerful test because it needs no independently-obtained "correct"
  reference answer — only the property that a correct forward operation and
  a correct inverse operation should exactly undo each other.
- **Dry runs** — running all of a system's expensive *computation* (motion
  planning, feasibility checking) while explicitly skipping the actual
  *actuation* step. This lets you validate the hardest, most failure-prone
  logic in a fast, cheap, repeatable way, without needing a simulator or
  hardware running at all.
- **Defined observables** — for a live running system, know in advance
  exactly which specific numbers, rates, or counts indicate healthy
  operation (an expected sensor update rate, an expected count of
  coordinate frames, an expected list of reported joint names) — and
  actually check those numbers, rather than relying on "it looks like it's
  probably working."

---

<a name="together"></a>
## Putting it together: reading a typical project end to end

With the ten layers above, here's roughly how they combine in a typical
manipulator project, and a sensible order to approach an unfamiliar one in:

1. **Find the description.** Locate whatever file declares the robot's
   links, joints, and dimensions (§10.1). This tells you the mechanism's
   actual structure before you read a line of behavior code — how many
   arms, how many joints each, roughly what size.
2. **Find the kinematics.** Locate the forward and inverse kinematics
   (§4.1, §4.2). This tells you the mechanism's fundamental capabilities
   and limitations — its DOF, what it gives up if under-actuated, its
   reach.
3. **Find the self-tests, and run them** (§10.6). A round-trip FK/IK test,
   in particular, is usually the fastest way to gain confidence that the
   kinematics is actually correct, independent of ever touching a
   simulator.
4. **Find the task plan.** Locate wherever a sequence of waypoints or
   actions is assembled (§5.1, §10.5) and, ideally, run it in a
   compute-only "dry run" mode if one exists, before ever launching a full
   simulation.
5. **Find the motion execution.** Locate where the plan is actually turned
   into a stream of commands over time (§5.2–§5.5) — this tells you what
   kind of interpolation and smoothing is used, and therefore what the
   *shape* of the actual motion will look like.
6. **Find the grasp mechanism**, if there is one (§6). Is it force-based or
   form-based? What's the actual force budget?
7. **Find the control layer** (§7). What mode is each actuator commanded
   in, and what does that imply about what the system can and can't do
   directly?
8. **Only then, launch the full simulation or hardware** (§9, §10.4). By
   this point you already know what the mechanism is capable of, what the
   plan is supposed to do, and what "working correctly" should actually
   look like — which turns watching it run from a guessing game into an
   actual verification step.

**A last, general observation, worth carrying into any new project:** the
best decisions in a well-built manipulator project are very often
*reframings* of a problem, not additional lines of implementation — noticing
that a mechanism is under-actuated and deliberately choosing what to give up
(§2.2, §3.3), picking an approach orientation that costs less of the
workspace envelope (§3.4), redesigning the object being manipulated rather
than the manipulator when a task turns out to be geometrically infeasible
(§3.2, §6.1), or simply writing down precisely *why* something is impossible,
with the specific number or constraint that proves it, rather than leaving
it as a vague unexplained limitation. Recognizing that kind of move when you
see it in someone else's project — and reaching for it yourself — is most of
what separates experienced robotics engineering from simply writing code
that happens to work once.

---

<a name="glossary"></a>
## Glossary

| Term | Meaning |
|---|---|
| **Actuator** | A device that produces physical motion from a command (motor, cylinder) |
| **C-space** | Configuration space — the space of all joint value combinations |
| **Closed loop** | Control that uses sensor feedback to correct itself |
| **DOF** | Degree of freedom — one independent way something can move |
| **Encoder** | A sensor that reports a joint's current position or velocity |
| **End-effector** | The functional tip of a manipulator — a hand, gripper, or tool |
| **FK** | Forward kinematics — joint values → end-effector pose |
| **Force closure** | A grasp held by friction / applied force, not geometry alone |
| **Form closure** | A grasp held by geometry alone, independent of friction |
| **Frame** | A coordinate origin + axes attached to some reference point |
| **Friction cone** | The set of contact forces a frictional surface can resist without slipping |
| **IK** | Inverse kinematics — desired pose → joint values |
| **Jacobian** | The matrix relating joint velocities to end-effector velocity |
| **Jerk** | The rate of change of acceleration; discontinuities cause vibration |
| **Joint** | A constrained connection between two links (revolute, prismatic, fixed, ...) |
| **Kinematic chain** | A sequence of links connected by joints |
| **Kinematics** | The study of motion/geometry, independent of the forces causing it |
| **Link** | A rigid body forming part of a robot's structure |
| **Middleware** | Software plumbing that lets independent processes exchange data |
| **Open loop** | Control that doesn't check whether a command actually took effect |
| **Pose** | Position + orientation of a rigid body (6 numbers) |
| **PID** | Proportional-Integral-Derivative — the standard feedback control law |
| **Proprioception** | Sensing of a robot's own internal state (vs. the external world) |
| **Quaternion** | A 4-number representation of orientation, avoiding gimbal lock |
| **Redundant (mechanism)** | Has more DOF than the task strictly requires |
| **Rigid body** | An object whose points never move relative to each other |
| **Singularity** | A configuration where the mechanism loses a direction of motion |
| **Task space** | The space a task is naturally described in (usually a pose) |
| **Topic** | A named publish/subscribe communication channel |
| **Trajectory** | A path plus a time parameterization |
| **Transform** | An operation converting a pose from one coordinate frame to another |
| **Under-actuated** | Has fewer DOF than the task ideally requires |
| **Workspace** | The set of poses a mechanism's end-effector can actually reach |
