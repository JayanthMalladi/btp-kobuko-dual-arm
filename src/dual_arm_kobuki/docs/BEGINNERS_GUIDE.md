# A Beginner's Guide to This Robot

**Who this is for:** you have never built a robot, never opened a URDF file, and
you would like to understand what this simulation actually *is* — not just how
to run it. No robotics background assumed. Every number in the robot gets
explained, and where a number was calculated, the calculation is shown.

Read this top to bottom. Each section builds on the one before it.

---

## Part 0 — What we actually built, in one paragraph

Imagine a robot vacuum cleaner: a squat cylinder about 35 cm across. Now bolt a
flat deck on top of it, and stand **two robot arms** on that deck, side by side,
like a person's shoulders. Each arm has four powered joints and a two-finger
gripper (a claw). Behind the arms, stand a little goalpost-shaped mast carrying
a **camera** and a spinning **laser rangefinder**. Put this robot in a simulated
room with two small tables and three objects on them. That's it. That's the
whole thing.

Everything below is the detail of *how big each piece is, and why*.

> **Important:** this is a **simulation only**. Nothing here was machined or
> wired. "The robot" means a description file that a physics simulator
> (Gazebo) reads and then pretends to be. That distinction matters later,
> because some of the numbers are real design decisions and others are
> plausible guesses — and the files label which is which.

---

## Part 1 — The five words you need

Robotics has a small core vocabulary. Learn these five and 90% of the files
become readable.

### Link
A **link** is a rigid piece. A bone. It does not bend. The robot's body is a
link. The upper arm is a link. Each finger is a link. A link has a shape (for
drawing and for collisions) and a mass.

### Joint
A **joint** connects two links and says how they may move relative to each
other. Three kinds appear here:

| Joint type | What it does | Example here |
|---|---|---|
| `revolute` | rotates about an axis, with limits | every arm joint |
| `prismatic` | slides along an axis, with limits | the gripper fingers |
| `fixed` | doesn't move at all — it's glue | camera bolted to the mast |

A `fixed` joint sounds pointless, but it is how you say "the camera is
*exactly here* relative to the body." That's most of what a robot description
is: a tree of pieces glued into known positions.

### Degrees of freedom (DOF)
**DOF = how many independently moving joints there are.** Each arm here has
4 revolute joints, so it's a "**4-DOF arm**". The gripper fingers are counted
separately, the way a commercial robot is advertised as "6-DOF *plus* gripper".
So this robot is: 4 DOF + gripper, twice over.

Why do people care so much about this number? Because **to put a hand anywhere
you like, in any orientation you like, you need 6 DOF** — three numbers for
position (x, y, z) and three for orientation (roll, pitch, yaw). With only 4,
you must give up two of those six. Which two you give up is the single most
important design fact about this robot, and Part 6 is about exactly that.

### Frame
A **frame** is a little set of x/y/z arrows attached to a link, so you can say
"5 cm in front of *this thing*". Every link has one. The whole robot is a tree
of frames, and the software (`tf2`) constantly computes how to convert a
position in one frame into a position in another.

This robot has **21 frames**, rooted at a frame on the floor called
`base_footprint`, and the two most useful ones are at the very tips:
`left_grasp_frame` and `right_grasp_frame` — the exact points between the
finger pads where an object gets held. When we say "move the arm to
(0.31, 0.09, 0.225)", we mean *put the grasp frame there*.

### URDF / xacro / SDF
Three file formats, all XML, all describing the same kind of thing:

- **URDF** — "Unified Robot Description Format". The standard way to describe a
  robot as links and joints. It has no variables and no loops, so writing one
  by hand is miserably repetitive.
- **xacro** — "XML macros". A preprocessor that adds variables and reusable
  macros to URDF, then spits out plain URDF. **All the `.xacro` files here are
  just URDF with the repetition removed.** This is why one arm macro can be
  called twice (once mirrored) instead of copy-pasting an arm.
- **SDF** — Gazebo's own format. Used here for the **world** (the room, the
  tables, the objects, gravity, the lighting) rather than the robot.

So: the **robot** is xacro → URDF, and the **world** is SDF.

---

## Part 2 — The base: where the numbers come from

Open `urdf/common.xacro`. Every dimension in the entire robot lives in that one
file, and every other file reads its values from there. That's deliberate: if
you want the arms to be longer, you change **one number in one file**.

```
base_radius      0.1775 m      [POSTER]
base_height      0.09 m        [PLACEHOLDER]
ground_clearance 0.015 m       [PLACEHOLDER]
base_mass        2.5 kg        [PLACEHOLDER]
deck_top_z       0.14 m        [POSTER]
```

### Read the tags — they are the most honest thing in the repo

Every value carries a tag, and understanding them will save you from a classic
beginner mistake:

- **`[POSTER]`** — a **locked design decision**. It came from the project spec
  and is not yours to fiddle with. 17.75 cm base radius, 14 cm deck height,
  16 cm arm links — these are the spec.
- **`[PLACEHOLDER]`** — **a plausible-looking guess**. Nobody decided this. The
  base mass of 2.5 kg is "a real Kobuki is about 2.35 kg dry, so 2.5 is
  sensible." The joint limits are "±90° feels about right."

The mistake to avoid: treating a `[PLACEHOLDER]` as if it were measured truth.
If your simulation does something strange and the cause traces back to a
placeholder, the answer may simply be that the guess was bad.

### What these numbers mean physically

`base_radius` = 0.1775 m means **a cylinder 35.5 cm across** — about the
footprint of a large dinner plate. `deck_top_z` = 0.14 m means the flat surface
the arms stand on is **14 cm above the floor**. Not 14 cm above the base's
bottom — above the *floor*. Heights in this project are measured from the
ground, which turns out to matter a lot in Part 8.

---

## Part 3 — Where the arms mount: your first real calculation

The two shoulders sit forward and to the sides on the deck:

```
arm_offset_fwd  0.095 m     (9.5 cm forward of centre)
arm_offset_lat  0.08875 m   (8.875 cm to each side)
```

The spec, however, asked for something different. It asked for the shoulders to
be **13 cm from the robot's centre**, and **17.75 cm apart from each other**.
Those are the two constraints. The x/y numbers above are what satisfies them.
Here's how:

### Constraint 1: the two arms are 17.75 cm apart

If each shoulder is `y` to one side, the distance between them is `2y`. So:

```
2y = 0.1775  →  y = 0.08875 m
```

That's `arm_offset_lat`. One line of arithmetic.

### Constraint 2: each shoulder is 13 cm from centre — Pythagoras

Now, how far forward? Looking straight down at the deck, a shoulder sits at
(x, y) = (forward, sideways) from the centre. Its straight-line distance from
the centre is the hypotenuse of a right triangle with legs `x` and `y`:

```
        shoulder
           /|
          / |
   r =13 /  | y = 8.875 cm
        /   |
       /____|
      O   x = ?
```

So `r² = x² + y²`, and we want `r` = 0.13:

```
x = sqrt(r² - y²)
  = sqrt(0.13² - 0.08875²)
  = sqrt(0.0169 - 0.007877)
  = sqrt(0.009023)
  = 0.0950 m        ← that's arm_offset_fwd
```

**Check it forwards** (always check a derivation by running it backwards):

```
sqrt(0.095² + 0.08875²) = sqrt(0.009025 + 0.0078766) = sqrt(0.0169016) = 0.13001 m ✓
```

13.001 cm against a target of 13 cm. That 0.001 cm of rounding is the comment
in `common.xacro` that reads `derived r = 0.1300 m [POSTER: 13 cm]`.

**This is the pattern for the whole project:** the spec states a *meaningful*
quantity (how far from centre, how high off the ground), and the file stores the
*x/y/z offsets* that produce it. Wherever you see a number like `0.1212` with a
square root in the comment, that's what happened.

---

## Part 4 — One arm, joint by joint

The arm is defined once, in `urdf/arm.xacro`, as a macro — then called twice,
the second time mirrored. Walking from the shoulder outward:

| # | Joint | Type | Rotates about | Link that follows | Length |
|---|---|---|---|---|---|
| J1 | shoulder yaw | revolute | **vertical (Z)** | riser | h0 = 4 cm |
| J2 | shoulder pitch | revolute | horizontal | upper arm | L1 = 16 cm |
| J3 | elbow | revolute | horizontal | forearm | L2 = 16 cm |
| J4 | wrist pitch | revolute | horizontal | wrist + palm + fingers | L3 = 9.3 cm |
| — | finger A | prismatic | slides sideways | finger | 3.5 cm travel |
| — | finger B | prismatic | slides sideways | finger | 3.5 cm travel |

### The one structural fact that explains everything else

Look at the "rotates about" column. **J1 turns about the vertical axis. J2, J3
and J4 all turn about the *same* horizontal axis.**

Hold your right arm out. Now:

1. **J1** is you swivelling your whole arm left and right at the shoulder,
   like sweeping a searchlight. It chooses a **direction** — a vertical slice
   of the world, like a page standing on edge.
2. **J2, J3, J4** are then all confined to that one page. Shoulder up/down,
   elbow up/down, wrist up/down, all in the same flat plane.

So the arm is: *pick a vertical page with J1, then draw on that page with three
joints.* In robotics language, it's a **planar 3R chain in a vertical plane
whose heading J1 selects**. Everything good and everything bad about this robot
follows from that sentence.

**The good:** a 3-joints-in-a-plane problem has a *closed-form* solution — you
can solve it with school trigonometry, exactly, instantly (Part 6). Most real
arms need iterative numerical solvers.

**The bad:** the gripper's approach direction is trapped inside the page. There
is **no tool roll and no tool yaw**. You cannot turn a doorknob. You cannot
reach around the side of something. And — as Part 11 explains — you cannot
squeeze a big object between the two arms.

### Where the 9.3 cm for L3 came from

J1 through J3 have clean spec lengths (4, 16, 16 cm). L3 is different: it isn't
a link, it's the distance from the wrist joint out to **the point where an
object is actually held** — and that point is in mid-air between the fingers.
So it's a sum of three pieces:

```
L3 = wrist_len + palm_len + 0.6 × finger_len
   = 0.030    + 0.030    + 0.6 × 0.055
   = 0.030    + 0.030    + 0.033
   = 0.093 m
```

The curious term is `0.6 × finger_len`. The fingers are 5.5 cm long, and an
object doesn't get gripped at the fingertips (unstable) or right at the
knuckles (the object would collide with the palm). It gets gripped somewhere in
the middle — this design says **60% of the way down the fingers**. That's a
judgement call, and if you changed it to 0.5 the whole arm's reach maths would
shift by 5.5 mm.

### How high is the shoulder?

The inverse kinematics needs to know where the J2 axis is in space. Stack up the
heights from the floor:

```
deck top surface       0.140 m     (deck_top_z, from spec)
+ J1 mechanism          0.010 m     (sits in the deck thickness)
+ riser h0              0.040 m     (J1 → J2)
─────────────────────────────────
  J2 axis height        0.190 m     ← this is J2_Z in arm_kinematics.py
```

### The arm's reach

J2→J3 is 16 cm and J3→J4 is 16 cm, so the wrist can be at most
**16 + 16 = 32 cm** from the shoulder — arm dead straight. That 0.32 m number
is called the **2R envelope** ("2R" = two revolute joints), and it is the
yardstick everything gets measured against in Part 7.

There's also an inner limit. With two equal 16 cm links, folding the elbow
completely brings the wrist back to the shoulder, so the arm can technically
reach `|L1 − L2| = 0`. In the code you'll see this checked as a "dead zone" —
for equal-length links it's degenerate, but the check is there because it
matters the moment the two links differ.

---

## Part 5 — Forward kinematics: the easy direction

**Forward kinematics (FK)** answers: *given the four joint angles, where is the
hand?* This is the easy direction — there's exactly one answer, and you get it
by walking the chain.

Because of the planar structure, you can do it in 2D and then rotate. Track two
numbers as you walk outward:

- `s` — how far out horizontally, within the page
- `h` — how far up

Start both at zero (at the shoulder). Then for each of the three in-plane
joints, add its length, tilted by the **cumulative** angle so far:

```python
cum = 0.0
for angle, length in ((j2, L1), (j3, L2), (j4, L3)):
    cum += angle                    # angles accumulate down the chain
    s += length * sin(cum)
    h += length * cos(cum)
```

Why cumulative? Because **each joint's angle is measured relative to the link
before it**. If you rotate your shoulder 30°, your whole arm goes with it —
including the elbow. So the forearm's true tilt in the world is J2 + J3, and
the gripper's true tilt is J2 + J3 + J4. That running total is what `cum` is.

Why `sin` for outward and `cos` for upward? Because angles here are measured
**from straight up**. At angle 0, `sin(0) = 0` (nothing outward) and
`cos(0) = 1` (fully upward) — so all joints at zero means **the arms point
straight at the sky**. Which they do, and this has a visible consequence: at
zero, the three links stack into what looks like a single fused rod rather than
an articulated arm. (This is exactly why the launch file drives the arms to a
folded `HOME` pose a few seconds after spawning — Part 13.)

Finally, swing that 2D answer out into 3D along the direction J1 chose:

```python
x = shoulder_x + s * cos(j1)
y = shoulder_y + s * sin(j1)
z = shoulder_z + h
```

That's the entire FK. Eleven lines, no matrices.

---

## Part 6 — Inverse kinematics: the hard direction, made easy

**Inverse kinematics (IK)** is the reverse and the one that matters: *I want the
hand at this point — what should the four joint angles be?* This is genuinely
harder, because there may be no answer, or many answers.

For most robots you throw a numerical solver at it. Here we don't have to,
because of the planar structure. Four steps.

### Step 1 — J1 is just "which way do I point?"

Take the target, subtract the shoulder position, and look at it from above.
The compass bearing of the target is J1. Done.

```python
dx, dy = x - shoulder_x, y - shoulder_y
j1 = atan2(dy, dx)
```

`atan2(dy, dx)` is "the angle to the point (dx, dy)", and it's used instead of
`atan(dy/dx)` because it gets all four quadrants right and doesn't explode when
`dx` is zero. Reach for `atan2` by reflex.

With J1 fixed, the page is chosen, and the remaining problem is 2D:

```python
s_t = hypot(dx, dy)      # how far out, in the page
h_t = z - shoulder_z     # how far up, relative to the shoulder
```

### Step 2 — pick the tool pitch, ψ

Here is where the missing DOF shows up. We have **three** joints left
(J2, J3, J4) and the target position uses up **two** of them (`s_t` and `h_t`
— we're in 2D now). So exactly **one** freedom is left over, and we must spend
it on orientation.

That one number is called **ψ (psi)** — the **tool pitch**, the total angle of
the gripper measured from straight up. It's the `cum` from Part 5, summed all
the way:

```
ψ = J2 + J3 + J4
```

| ψ | radians | The gripper... |
|---|---|---|
| 0° | 0 | points straight up |
| 90° | 1.571 | points horizontally outward |
| **135°** | **2.35** | **points down at 45° — a diagonal approach** |
| 180° | 3.142 | points straight down (top-down grasp) |

**You choose ψ; you don't get to choose anything else about the orientation.**
No roll, no yaw. This is what "4 DOF = 3 position constraints + exactly 1
orientation constraint" means. The demo uses ψ = 2.35, and Part 7 is entirely
about why.

### Step 3 — back off to the wrist, then solve a triangle

Now a lovely trick. We know where we want the *grasp point*, and we know the
gripper is a rigid 9.3 cm stick pointing along ψ. So **walk backwards along
that stick** to find where the wrist joint (J4) has to be:

```python
s_w = s_t - L3 * sin(psi)
h_w = h_t - L3 * cos(psi)
```

And with that, the hard part is over — because now we need J2 and J3 to put the
wrist at a known point, and that's the classic **two-link (2R) problem**: two
sticks of known length, hinged, tip must land on a known spot. A triangle.

```
                 wrist (s_w, h_w)
                   o
                  /
        L2=16cm  /
                /  ) J3  ← the angle we want
       elbow   o
               \
        L1=16cm \
                 \
     shoulder     o————————  r = distance shoulder→wrist
```

The triangle has sides L1, L2, and `r` = the distance from shoulder to wrist.
All three sides known → the **law of cosines** gives the enclosed angle. That's
J3, the elbow:

```python
r2 = s_w**2 + h_w**2
c3 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
c3 = max(-1.0, min(1.0, c3))        # clamp — see below
j3 = ± acos(c3)
```

That clamp line is defensive and worth copying into your own code: floating-
point rounding can hand `acos` a value like 1.0000000001, which is outside its
domain and crashes. Clamping to [−1, 1] makes it impossible.

**Before** doing any of this, the code checks the triangle can exist at all:

```python
if r > L1 + L2:      raise Unreachable   # too far — arm can't stretch that far
if r < abs(L1 - L2): raise Unreachable   # too close — inside the folded dead zone
```

A triangle with one side longer than the other two combined doesn't exist. So
"unreachable" isn't a solver giving up — it's geometry saying no.

Then J2, the shoulder angle, is "point at the wrist, then correct for the fact
that the elbow bends away from that line":

```python
j2 = atan2(s_w, h_w) - atan2(L2*sin(j3), L1 + L2*cos(j3))
#    ^ direction of the wrist   ^ how far the bent elbow throws you off
```

### Step 4 — J4 falls out for free

We decided ψ = J2 + J3 + J4, and we now know J2 and J3. So:

```python
j4 = psi - j2 - j3
```

No work at all. The wrist is whatever's left over to make the total come out to
the ψ we asked for.

### Elbow up or elbow down?

Notice the `±` on J3. Both signs put the wrist in exactly the right place —
they're mirror images through the shoulder-wrist line. Same hand position, two
different arm shapes. Like touching your nose with your elbow raised or
lowered.

So IK has **two answers**, and the code tries both (`elbow="auto"`), checking
each against the joint limits and returning the first that fits. If neither
fits, it raises `Unreachable` with a message naming which joints were out of
range — a genuinely useful error, e.g. `elbow up: j3 out of range; elbow down:
j2,j4 out of range`.

The joint limits it checks against:

| Joint | Limits (rad) | In degrees |
|---|---|---|
| J1 | ±1.5708 | ±90° |
| J2 | ±1.5708 | ±90° |
| J3 | ±2.3562 | ±135° |
| J4 | ±2.0000 | ±114.6° |

All four are `[PLACEHOLDER]` — sensible-looking, not decided. Note that
1.5708 ≈ π/2 and 2.3562 ≈ 3π/4; these are round numbers in radians, which is
the tell-tale sign of a guess rather than a measurement.

---

## Part 7 — Why 135°? The most instructive number here

The demo grasps with ψ = 2.35 rad (135°) — a diagonal, knife-through-cake
approach — rather than the obvious straight-down 180°. This looks arbitrary. It
isn't, and it's the best worked example in the project.

### The measuring stick: reach fraction

`reach_fraction()` reports **how much of the arm's 32 cm envelope a target
consumes**. It's just the wrist distance divided by the maximum:

```python
reach_fraction = hypot(s_w, h_w) / (L1 + L2)
```

- **0.5** — comfortable, elbow nicely bent, plenty of margin
- **0.9+** — nearly straight out. Technically reachable, **practically bad**

Why is near-full-extension bad? Two reasons, both worth internalising:

1. **It's poorly conditioned.** Near full stretch, a tiny change in hand
   position demands a huge change in joint angle. The maths gets numerically
   twitchy and the controller gets jerky. (At *exactly* full extension the
   arm is singular — the two IK solutions collapse into one.)
2. **No room left.** At 95% there is 1.6 cm of reach left. Any error, any
   approach standoff, any nudge to the object's position, and the target goes
   from "reachable" to `Unreachable`.

### Now the comparison

Same target — the small object at (0.31, 0.09, 0.225) — same everything, only ψ
changed. These are measured, not estimated:

| Approach | ψ | Reach fraction | Verdict |
|---|---|---|---|
| Top-down | 180° | **78.2%** | works, but tight |
| **Diagonal** | **135°** | **56.1%** | comfortable ✓ |
| Horizontal | 90° | 39.7% | roomiest, but you'd hit the table |

**Tilting the wrist by 45° bought back 22% of the arm's entire reach envelope —
about 7 cm — for free.** No longer links, no stronger motors. Just a better
choice of the one orientation freedom we have.

### Where the 22% went

Step 3 was "back off L3 along ψ from the target." The *direction* you back off
in decides where the wrist ends up:

- **Top-down (ψ = 180°):** backing off means going **straight up**. The wrist
  lands 9.3 cm directly above the target — the same distance out horizontally,
  but much higher, so it's further from the shoulder in a diagonal direction
  the arm is bad at.
- **Diagonal (ψ = 135°):** backing off goes up *and back toward the robot*.
  The wrist lands 6.6 cm up and 6.6 cm nearer the shoulder. **Nearer is the
  whole point.**

Numerically, for that pick target (shoulder-relative `s_t` = 0.215, `h_t` = 0.035):

```
ψ=180°:  s_w = 0.215 - 0.093·sin(180°) = 0.215        (unchanged)
         h_w = 0.035 - 0.093·cos(180°) = 0.128        (pushed up 9.3 cm)
         r   = hypot(0.215, 0.128) = 0.250 m → 0.250/0.32 = 78.2%

ψ=135°:  s_w = 0.215 - 0.093·0.7086  = 0.149          (pulled 6.6 cm closer)
         h_w = 0.035 + 0.093·0.7056  = 0.101          (up only 6.6 cm)
         r   = hypot(0.149, 0.101) = 0.180 m → 0.180/0.32 = 56.1%
```

**The lesson:** on an under-actuated arm, the freedoms you *do* have are worth
spending deliberately. A beginner picks the intuitive top-down grasp and then
complains the arm is too short. The arm was never too short.

---

## Part 8 — The gripper, and a force budget you can check by hand

```
finger_stroke   0.035 m per finger   →  7.0 cm maximum opening
finger_len      0.055 m
pad friction    mu = 1.6  (both directions)
finger_effort   25 N maximum
```

Two fingers, each sliding 3.5 cm, so the jaw opens **7 cm** total. Remember
that number — it decides which objects in the world are pickable and which
aren't.

### There is no magic "attach" — the grasp is real friction

Many tutorials cheat: when the gripper is near an object, a plugin welds them
together. **This project does not do that.** The object is held by nothing but
friction between two rubber pads, computed by the physics engine, exactly as in
reality. Which means it can be *dropped* if the numbers are wrong. So let's
check the numbers.

### Step 1 — how hard must we squeeze?

The object is 0.08 kg. Its weight:

```
W = m·g = 0.08 × 9.8 = 0.784 N
```

Friction gives you at most `μ × N` of grip, where `N` is how hard you press.
Two pads press, so you get `2 × μ × N`. To not drop it:

```
2 × μ × N  ≥  W
2 × 1.6 × N ≥ 0.784
N ≥ 0.784 / 3.2
N ≥ 0.245 N
```

**We need to press with at least 0.245 N per pad.** (That's about the weight of
a AA battery — not much, because μ = 1.6 is very grippy rubber.)

### Step 2 — how hard do we actually squeeze?

Here's the trick the code uses. The object's radius is 2.5 cm, and the fingers
are commanded to **2.1 cm** — that is, 4 mm *inside* where the surface is:

```python
OBJECT_RADIUS = 0.025
GRIP_CLOSED = OBJECT_RADIUS - 0.004     # 4 mm of deliberate interference
```

The finger can't actually get there — the object is in the way. So the
controller sits there with a permanent 4 mm position error, pushing. The finger
controller is a P-controller with gain 600, and a P-controller's output is
`gain × error`:

```
F = p_gain × error = 600 × 0.004 = 2.4 N
```

### Step 3 — compare

```
Needed:    0.245 N
Delivered: 2.4 N        → about 10× margin
Ceiling:   25 N         → we're at 10% of the motor's limit
```

Comfortable on both sides: nearly 10× more grip than needed to hold on, and
nowhere near hard enough to crush anything or saturate the actuator. **That is
what a force budget looks like**, and it's three lines of arithmetic you should
do for any grasp before wondering why objects fall out.

(The open command is `GRIP_OPEN = 0.033` → 6.6 cm, deliberately just under the
7 cm hard stop, so the fingers don't slam into their end stops.)

---

## Part 9 — The sensors

A goalpost-shaped mast — two vertical posts and a crossbar — stands behind the
arms, carrying two sensors.

### The camera

| Property | Value | In plain English |
|---|---|---|
| Resolution | 640 × 480 | modest, like an old webcam |
| Format | R8G8B8 | 8 bits each of red/green/blue |
| Horizontal field of view | 1.204 rad = **69°** | roughly a phone's main camera |
| Update rate | 15 Hz | 15 pictures a second |
| Near / far clip | 0.05 m / 12.0 m | ignores closer than 5 cm, further than 12 m |
| Height | **0.46 m** above ground | pitched 23° downward |

It publishes to `/camera/image_raw`.

### The 2D LiDAR

LiDAR = a spinning laser rangefinder. "2D" means it sweeps a single flat
horizontal circle, so it sees a **slice** of the world, not a 3D volume.

| Property | Value | In plain English |
|---|---|---|
| Samples | 360 | one measurement per degree |
| Angular span | −π to +π | **full 360°** |
| Range | 0.12 – 8.0 m | blind closer than 12 cm |
| Range resolution | 0.01 m | distances to the nearest centimetre |
| Update rate | 10 Hz | ten full sweeps a second |
| Height | **0.32 m** above ground | |

It publishes to `/scan`, and (unlike the camera) is set to `visualize: true`, so
you can see its rays drawn in Gazebo — very handy for a sanity check.

### The height trick worth stealing

The spec says "camera 46 cm above the ground, LiDAR 32 cm above the ground."
But a URDF joint wants an **offset from its parent link**, not a height above
the floor. Rather than working out those offsets by hand and hard-coding them,
`common.xacro` writes down the goal and lets xacro do the subtraction:

```
mast_head_agl = deck_top_z + mast_height = 0.14 + 0.31 = 0.45 m
camera_z_off  = 0.46 - 0.45 = +0.01 m    (1 cm above the crossbar)
lidar_z_off   = 0.32 - 0.45 = -0.13 m    (13 cm below the crossbar)
```

Now if you raise the mast, **both sensors stay at their specified heights
automatically**, because the offsets are computed, not typed. Pushing arithmetic
into the description file instead of into your head is one of the most valuable
habits in this whole project.

### And the same Pythagoras as Part 3

The mast posts must sit 14 cm from the robot's centre. The design puts them
7 cm *behind* centre (`mast_offset_x = -0.07`), so:

```
mast_offset_y = sqrt(0.14² - 0.07²) = sqrt(0.0196 - 0.0049) = sqrt(0.0147) = 0.12124 m
```

That's the mysterious `0.1212` in the file. Identical move to the shoulders:
spec states the radius, file stores x and y.

---

## Part 10 — How the joints actually get moved

This part will surprise you if you've read any ROS 2 tutorials.

### What's missing: ros2_control

The normal way to drive a robot in ROS 2 is `ros2_control` — a framework with
hardware interfaces, controller managers, and trajectory controllers. **This
project deliberately doesn't use it**, and that's a scope decision for the
first milestone, not an oversight.

Nor is there **MoveIt** (motion planning with collision avoidance), or any
**navigation** stack. The base has no wheels at all — it's a rigid body that
sits where it's put.

### What's there instead: 12 independent position controllers

Each of the 12 moving joints gets its own little Gazebo plugin
(`JointPositionController`) that listens on its own topic and drives that one
joint toward whatever number it last received:

```
/arm/left_j1/cmd_pos        /arm/right_j1/cmd_pos
/arm/left_j2/cmd_pos        /arm/right_j2/cmd_pos
/arm/left_j3/cmd_pos        /arm/right_j3/cmd_pos
/arm/left_j4/cmd_pos        /arm/right_j4/cmd_pos
/arm/left_finger_a/cmd_pos  /arm/right_finger_a/cmd_pos
/arm/left_finger_b/cmd_pos  /arm/right_finger_b/cmd_pos
```

Each carries a `std_msgs/Float64` — literally one number, the desired angle in
radians (or metres, for fingers). Publish `1.2` to `/arm/left_j2/cmd_pos` and
that joint heads for 1.2 rad.

**The 12 joints are completely independent. Nothing synchronises them.** There
is no notion of "the left arm" in the control layer — only twelve unrelated
numbers. Any coordination (including both arms moving together) exists purely
in whichever Python node is doing the publishing.

### The PID gains, and why the fingers are so different

Each controller is a PID loop — it looks at the error between where the joint
is and where it was told to be, and produces effort. Three gains: **P**
(proportional to current error), **I** (accumulated past error, kills steady
droop), **D** (proportional to rate of change, damps oscillation).

| Joint | P | I | D | max effort |
|---|---|---|---|---|
| J1 | 6.0 | 0.05 | 0.6 | 5 N·m |
| J2 | 9.0 | 0.10 | 0.9 | 5 N·m |
| J3 | 7.0 | 0.08 | 0.7 | 5 N·m |
| J4 | 4.0 | 0.05 | 0.4 | 5 N·m |
| **fingers** | **600.0** | **8.0** | **20.0** | **25 N** |

Two patterns to read here:

1. **Arm gains descend outward** (J2 = 9 down to J4 = 4). J2 carries the entire
   arm's weight, so it needs the most authority. J4 carries only the gripper.
   Gains roughly track the load.
2. **The fingers are 100× stiffer**, and that's the Part 8 force budget showing
   up in the config file. P = 600 is exactly what converts 4 mm of squeeze into
   2.4 N of grip. A soft finger controller would simply let go.

### Bridging: two middlewares talking

Gazebo and ROS 2 use different messaging systems, so a translator process
(`ros_gz_bridge`) sits between them, configured by `config/bridge.yaml` with
**17 topics**:

| Direction | Topics |
|---|---|
| Gazebo → ROS | `/clock`, `/joint_states`, `/scan`, `/camera/image_raw`, `/camera/camera_info` |
| ROS → Gazebo | the 12 `cmd_pos` command topics |

---

## Part 11 — The world: the room and what's in it

`worlds/manipulation_task.sdf` describes everything that isn't the robot.

### Physics and scene

```
max_step_size     0.001 s      → physics recomputed every 1 ms (1000 Hz)
real_time_factor  1.0          → simulated time tracks wall-clock time
gravity           0 0 -9.8     → downward, Earth
ground_plane      50 × 50 m, friction mu = 1.0
lighting          one directional "sun" with shadows
```

A 1 ms step is quite fine-grained. Contact and friction are hard to simulate
stably, and since this project's entire grasp *depends* on contact (Part 8),
paying for small steps is the right trade.

Five Gazebo plugins are loaded. Four are routine (physics, user commands, scene
broadcasting, sensors). The fifth is **Contact** — and it's there specifically
because friction grasping needs contact reporting to work.

### Two tables

Both are `static: true`, meaning physics won't push them around — they're
scenery, immovable. Each is a pedestal with a flat plate on top, with the top
surface at **z = 0.20 m**:

| Model | Position (x, y) | Top plate | Pedestal |
|---|---|---|---|
| `starting_base` | (0.41, 0.02) | 0.30 × 0.28 × 0.02 m | 0.12 × 0.12 × 0.18 m |
| `destination_base` | (0.22, 0.30) | 0.24 × 0.24 × 0.02 m | 0.10 × 0.10 × 0.18 m |

Note where they are. The start table is straight ahead (x = 0.41, y ≈ 0); the
destination is closer in but off to the left (x = 0.22, y = 0.30). So the demo
task is a **pick, swing left, and place** — it exercises J1, not just the
in-plane joints. And the comment in the file says the positions were chosen so
every waypoint stays under 75% reach fraction. Part 7's yardstick decided where
the furniture went.

### Three objects, each making a point

| Model | Shape | Mass | Position | Why it exists |
|---|---|---|---|---|
| `object_min` | cylinder ⌀5 cm × 5 cm | 0.08 kg | (0.31, 0.09, 0.225) | smallest case — **the demo object** |
| `object_max` | cylinder ⌀18 cm × 25 cm | 0.50 kg | (0.44, 0.02, 0.325) | largest case — **deliberately impossible** |
| `object_bimanual` | tray + 2 handle posts | 0.50 kg | (0.28, 0, 0.24) | the two-arm case, done right |

All three use μ = 1.6 (grippy). `object_min` also gets contact tuning
`kp = 1e6, kd = 100` — a stiff, slightly damped contact, so the gripper doesn't
sink into it visually.

**Why z = 0.225 for the small object?** Table top is at 0.20, the object is
5 cm tall so its centre is 2.5 cm up: `0.20 + 0.025 = 0.225`. SDF poses are the
*centre* of a shape, not its bottom — a very common first bug.

### Where the inertia numbers came from

Each object has an `<inertia>` block full of numbers like `0.0000292`. Inertia
is "rotational mass" — how much it resists being spun. You don't guess these;
you use the standard formula for the shape. For a solid cylinder of mass m,
radius r, length L:

```
about its own axis (izz):        m·r²/2
about either perpendicular axis: m·(3r² + L²)/12
```

For `object_min` (m = 0.08, r = 0.025, L = 0.05):

```
izz = 0.08 × 0.025² / 2                    = 0.0000250  ✓ matches the file
ixx = 0.08 × (3×0.025² + 0.05²) / 12       = 0.0000292  ✓ matches the file
```

For `object_max` (m = 0.5, r = 0.09, L = 0.25):

```
izz = 0.5 × 0.09² / 2                      = 0.002025   ✓
ixx = 0.5 × (3×0.09² + 0.25²) / 12         = 0.003617   ✓
```

The tray is harder — a box plus two posts — so its inertia was computed by
summing the three pieces using the **parallel-axis theorem** (which is how you
combine inertias about a shared point when the pieces aren't centred there).
The derivation is written up in `docs/bimanual/IMPLEMENTATION.md`.

**The habit:** look up the formula for your shape, plug in your numbers, and put
a comment saying you did. Nonsense inertias produce objects that spin like
they're haunted, and it's a miserable bug to chase.

---

## Part 12 — The big object cannot be picked, and that's the interesting part

`object_max` is 18 cm across. The jaw opens 7 cm. It does not fit. Obviously —
so why keep it?

Because it's a **documented negative result**, and the reasoning is the best
illustration of the Part 4 structural limit.

The obvious idea: if one gripper can't span it, squeeze it between **both**
arms, like carrying a box between your palms. That fails, and not for a fixable
reason.

To squeeze an object sitting in front of the robot, both hands must push
**inward, sideways** — left hand pushes right, right hand pushes left. Now
recall Part 4: **J2, J3 and J4 all pitch about the same horizontal axis.** They
can only move the hand up/down and in/out *within the page J1 chose*. None of
them can push sideways.

The only joint that can produce any sideways push is J1, by yawing the arm
inward. But J1 swings the hand along an *arc*, so only a fraction of its force
points where you need it. Working through the geometry (in
`docs/bimanual/README.md` §2b) the best available is a **~46% angled friction
pinch** — less than half the needed force, with the rest wasted pushing the
object forward.

**So it's structural, not a tuning problem.** No gain, no stronger motor, no
cleverer controller fixes it. You'd need a fifth DOF — a tool roll — to
reorient the palms. The file says as much, in a comment telling you to read the
analysis before trying.

**This is worth more than a working demo.** "We tried and it was fiddly" is
noise. "It is geometrically impossible with this joint layout, here's the
46% number, here's what DOF you'd have to add" is engineering.

### The fix: change the object, not the arm

If the arms can't squeeze, give them something they *can* grip.
`object_bimanual` is a tray with **two vertical handle posts**, 3 cm in
diameter, 20 cm apart:

| Property | Value | Why |
|---|---|---|
| Handle diameter | 3 cm | well inside the 7 cm jaw — a normal grasp |
| Handle separation | 0.20 m | one arm each, both comfortably in reach |
| Tray body | 22 × 8 × 3 cm | connects the handles |
| Total mass | 0.50 kg | matches the payload ceiling |

The insight: each gripper closes **around a post**, not onto the object's own
surface. A jaw wrapped around a cylinder can **spin freely about that
cylinder's axis** — so the post acts as an extra rotational joint, granted by
the *object* rather than the robot. That recovers the freedom the 4-DOF arm
lacks, and makes a two-arm carry kinematically possible.

**The lesson:** when a manipulator can't do a task, redesigning the *thing being
manipulated* is a legitimate engineering answer. Real factories do this
constantly — that's what fixtures, jigs and handles are.

Two honest caveats, both stated in the file: `object_max` and `object_bimanual`
have no table under them, so they free-fall on startup — they're geometry for
reach analysis, not staged for a demo. And the handle-post numbers are
duplicated in `arm_kinematics.py` as `BIMANUAL_OBJ_NOMINAL` and
`BIMANUAL_HANDLE_SEP`, so **the two files must be changed together**.

---

## Part 13 — Putting it in motion

### Starting up: an ordering problem

`launch/spawn.launch.py` starts five things, and **the order matters**. It's
enforced with plain timers rather than readiness checks:

```
t = 0 s   start Gazebo, load the world
t = 0 s   start robot_state_publisher (runs xacro, publishes /robot_description)
t = 4 s   spawn the robot into the world
t = 8 s   drive both arms to the HOME pose
```

Why the waits? The spawn request needs Gazebo to have finished advertising its
spawn service — ask too early and nothing appears. And the HOME pose is sent
last because, per Part 5, **all-joints-zero means arms straight up**, which
looks like a single fused rod. `HOME` = (0.0, −0.55, 1.35, 0.50) folds them
back and up so all four joints are visibly distinct.

Timers are a blunt instrument — a slow machine can still lose the race — but
they're honest about it, and there's a note in the code saying so.

> One very ROS-specific landmine, documented because it cost someone an
> afternoon: the robot description must be passed as
> `ParameterValue(Command(["xacro ", ...]), value_type=str)`. Omit the
> `value_type=str` and type inference **fails silently** — no error, nothing
> spawns.

### The demo: `nodes/pick_and_place.py`

Only the **left** arm moves. The right is parked at HOME and held there.

The key design choice is **solve everything before moving anything**.
`build_plan()` runs IK on every waypoint up front, so an impossible move is
caught while the robot is still standing still — never halfway through, holding
an object. You can see the whole plan without even starting Gazebo:

```bash
ros2 run dual_arm_kobuki pick_and_place.py --dry-run
```

```
step         reach   j1      j2      j3      j4      grip
--------------------------------------------------------------
pre-grasp    73.1%  +0.006  -0.061  +1.503  +0.909    6.6cm
grasp        56.1%  +0.006  +0.002  +1.951  +0.397    6.6cm
close        56.1%  +0.006  +0.002  +1.951  +0.397    4.2cm
lift         73.1%  +0.006  -0.061  +1.503  +0.909    4.2cm
transit      72.0%  +0.505  -0.207  +1.533  +1.024    4.2cm
pre-place    72.0%  +1.046  -0.096  +1.534  +0.911    4.2cm
place        54.7%  +1.046  -0.033  +1.985  +0.398    4.2cm
open         54.7%  +1.046  -0.033  +1.985  +0.398    6.6cm
retreat      72.0%  +1.046  -0.096  +1.534  +0.911    6.6cm
--------------------------------------------------------------
total motion time: 14.5 s
```

Read that table with Part 7 in hand and it tells you a lot:

- **Every reach fraction is between 54% and 74%** — comfortably under the 90%
  danger zone. That's the workspace layout being validated.
- **J1 goes 0.006 → 1.046 rad** (0° → 60°). That's the swing from the start
  table to the destination, and the only joint doing it.
- **Grip alternates 6.6 cm → 4.2 cm → 6.6 cm** — open, squeeze (with the 4 mm
  interference), release.
- **`grasp` and `close` are identical except for the grip**, as are `place` and
  `open`. Closing the fingers is its own step with the arm held still; you don't
  grab while moving.

### Why so many waypoints?

Because of one subtle thing: **motion is interpolated in joint space, not
Cartesian space**. Between two waypoints, each joint slides smoothly from its
start angle to its end angle (at 50 Hz, on a minimum-jerk profile — a smooth
S-curve that starts and ends at zero velocity, so no jerky snaps).

The consequence: **the hand does not travel in a straight line.** Each joint
moves linearly, but the resulting hand path is a curve, and you don't fully
control its shape.

Which is exactly why the waypoints exist. If you asked for one direct move from
`grasp` to `place`, the curved path could easily drag the object through the
table edge. So explicit `lift`, `transit` and `pre-place` waypoints fence the
path in:

```
     lift ────── transit ────── pre-place
      │                              │      ← 8 cm standoff above everything
      │                              │
   grasp                          place
  [table 1]                     [table 2]
```

The 8 cm vertical standoff (`APPROACH = 0.08`) is that safety margin: come
straight down onto the object, lift straight up off it, and only move sideways
at altitude.

---

## Part 14 — Every number in one place

**Base**
| | |
|---|---|
| Radius | 0.1775 m (35.5 cm across) `[POSTER]` |
| Height / clearance | 0.09 m / 0.015 m `[PLACEHOLDER]` |
| Mass | 2.5 kg `[PLACEHOLDER]` |
| Deck top (arm mount height) | 0.14 m `[POSTER]` |
| Wheels | none — rigid body |

**Arm** (×2, mirrored)
| | |
|---|---|
| Shoulder position | (±0.095, ±0.08875) → r = 13 cm, span 17.75 cm `[POSTER]` |
| J2 axis height | 0.190 m (= 0.14 + 0.01 + 0.04) |
| h0 / L1 / L2 | 0.04 / 0.16 / 0.16 m `[POSTER]` |
| L3 (J4 → grasp point) | 0.093 m (computed) |
| Max reach (2R envelope) | 0.32 m |
| DOF | 4 revolute + 2 prismatic fingers |
| J1 axis | vertical (yaw) |
| J2, J3, J4 axes | all the **same** horizontal axis (pitch) |
| Limits | J1, J2 ±90°; J3 ±135°; J4 ±114.6° `[PLACEHOLDER]` |
| Effort / velocity | 5 N·m / 2 rad/s |
| HOME pose | (0.0, −0.55, 1.35, 0.50) rad |

**Gripper**
| | |
|---|---|
| Max opening | 7.0 cm (2 × 3.5 cm stroke) |
| Finger | 55 × 12 × 28 mm |
| Pad friction | μ = 1.6 |
| Max force | 25 N |
| Grasp squeeze | 4 mm interference → 2.4 N (need 0.245 N) |

**Sensors**
| | |
|---|---|
| Camera | 640×480, 69° HFOV, 15 Hz, 0.05–12 m, **0.46 m AGL**, 23° down |
| LiDAR | 360 samples over 360°, 10 Hz, 0.12–8 m, **0.32 m AGL** |
| Mast posts | r = 14 cm at (−0.07, ±0.1212), 0.31 m tall |

**World**
| | |
|---|---|
| Physics | 1 ms step, RTF 1.0, gravity −9.8 |
| Ground | 50 × 50 m, μ = 1.0 |
| Tables | tops at z = 0.20 m, both static |
| Objects | 5 cm/0.08 kg, 18 cm/0.5 kg, tray/0.5 kg — all μ = 1.6 |

**Software**
| | |
|---|---|
| ROS 2 / Gazebo | Jazzy / Harmonic (gz-sim 8) |
| Control | 12 independent `JointPositionController` plugins |
| Bridge topics | 17 |
| TF frames | 21, rooted at `base_footprint` |
| Not used | ros2_control, MoveIt, navigation |

---

## Part 15 — Things that will trip you up

A short list of traps, most of them documented in the files themselves.

**1. `arm_kinematics.py` duplicates `common.xacro` by hand.**
The constants `MOUNT_X`, `MOUNT_Y`, `J2_Z`, `L1`, `L2`, `L3` at the top of the
Python file are **hand-copied** from the xacro, not computed from it. Change the
arm geometry in one and forget the other, and **IK will silently solve for the
wrong robot** — no error, just an arm that misses. Same trap for
`BIMANUAL_HANDLE_SEP` vs the world SDF. Always change both.

**2. `[PLACEHOLDER]` means nobody decided.**
Don't build an analysis on top of a guess without noticing it's a guess.

**3. "The window opened" is not a passing test.**
Check in a second sourced terminal:

```bash
ros2 run tf2_tools view_frames               # expect 21 frames
ros2 topic hz /scan                          # expect ~10 Hz
ros2 topic hz /camera/image_raw              # expect ~15 Hz
ros2 topic echo /joint_states --once         # expect 8 names, left_j1..j4 / right_j1..j4
```

And look at the arms: if they **droop** instead of holding their pose, the
position controllers didn't load. Check the Gazebo console for
`JointPositionController` errors. Silent sensor topics usually mean a GPU/driver
problem with the rendering plugin — `gui:=false` (headless rendering) is the
known workaround.

**4. Nothing coordinates the two arms.**
Twelve independent joints, twelve independent topics. Any synchronisation is
yours to write.

**5. There's no test suite.**
`colcon build` mostly just validates the package metadata (there's no compiled
code). The real correctness check is the kinematics self-test, which round-trips
FK→IK over ~4000 random poses per arm and reports the worst error:

```bash
python3 src/dual_arm_kobuki/nodes/arm_kinematics.py
```

This is a lovely pattern to copy: IK is hard to verify directly, but FK is easy
and exact. So generate random joint angles, run FK to get a position, run IK on
that position, and check you get the angles back. **If IK is wrong, the round
trip won't close.**

**6. `--symlink-install` matters.**
Build with it and edits to `.xacro` / `.sdf` files take effect on the next
launch with no rebuild. Without it you'll rebuild constantly and wonder why
your changes aren't showing up.

**7. Arms face +X.**
If the robot appears to be reaching out of its own back, flip `arm_offset_fwd`
and `mast_offset_x` in `common.xacro`.

---

## Part 16 — What to read next, in order

1. **`urdf/common.xacro`** — every number, in one screen. Start here.
2. **`nodes/arm_kinematics.py`** — FK and IK, ~150 lines of pure maths with no
   ROS dependency. Run it directly to see the self-test.
3. **`nodes/pick_and_place.py --dry-run`** — the whole demo plan, no simulator
   needed.
4. **`urdf/arm.xacro`** — how one arm macro becomes two mirrored arms.
5. **`worlds/manipulation_task.sdf`** — the room and its objects.
6. **`README.md` §7–8** — the terse version of Parts 6 and 8 above.
7. **`docs/bimanual/README.md` §2b** — the full impossibility argument from
   Part 12. Read it before attempting any two-arm grasp.

### The four ideas worth taking with you

- **One source of truth for dimensions.** Every number lives in
  `common.xacro`; everything else derives from it. Where that discipline breaks
  (trap 1 above), the file says so loudly.
- **Spend your freedoms deliberately.** The 135° approach buying back 22% of
  the reach envelope is the whole lesson of under-actuated design.
- **Budget your forces before debugging them.** Three lines of arithmetic told
  us the grasp would hold, with 10× margin, before a single simulation ran.
- **Write down the things that don't work, and why.** "Impossible, here's the
  46% number, here's the DOF you'd need" is more useful than a demo that
  happens to work.
