# dual_arm_kobuki — Implementation Spec

Detailed record of what is built, how it works, the math behind it, and what is still open.
For "how do I run this," see [`RUNNING.md`](RUNNING.md). For a file-by-file map, see the
top-level [`README.md`](../README.md).

Status: **Milestone 1** — the robot and world spawn correctly, every joint is drivable, and a
scripted single-arm pick-and-place demo runs end to end. Nothing perceives or plans on its own;
there is no navigation, no `ros2_control`, no MoveIt.

Stack: ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8), connected via `ros_gz`.

---

## 1. What's implemented

| Area | Implemented | Not implemented |
|---|---|---|
| Base | Rigid cylinder body, correct radius/height/mass | No wheels, no drive, no odometry |
| Arms | 2x 4-DOF revolute chain + 2-finger prismatic gripper, fully articulated, driven by Gazebo position controllers | No `ros2_control`, no torque/effort control, no compliance |
| Kinematics | Closed-form analytic FK/IK, both arms, self-tested | No collision-aware IK, no null-space/redundancy handling (none needed — arm is fully constrained) |
| Trajectory | Joint-space minimum-jerk interpolation at 50 Hz | No Cartesian-space interpolation, no online replanning, no obstacle avoidance |
| Grasping | Friction-only pinch grasp of the small object, closed-loop plan-then-execute demo | No attach/detach plugin (documented but untested), no force/tactile feedback, no bimanual grasp of the large object |
| Sensing | RGB camera + 2D LiDAR modelled and bridged to ROS topics | Nothing consumes the sensor data — no perception, no object detection, no localization |
| World | One task world with two tables and two task objects (size envelope extremes) | No additional scenes/clutter, no randomization |
| Bring-up | Single launch file sequences world, description, spawn, bridge, home pose | No readiness handshaking — sequencing is done with fixed timer delays (see §5) |

---

## 2. Package layout and responsibilities

```
urdf/common.xacro        Single source of truth for every dimension, mass, joint limit, HOME pose.
urdf/base.xacro          Kobuki body (cylinder primitive) + top deck.
urdf/arm.xacro           4-DOF arm + wrist + 2-finger gripper, as a reflectable macro (x2).
urdf/sensors.xacro       Mast, RGB camera, 2D LiDAR links + Gazebo sensor definitions.
urdf/robot.urdf.xacro    Assembles the above; declares all Gazebo system plugins.
worlds/manipulation_task.sdf   Ground plane, starting/destination tables, small/large objects.
launch/spawn.launch.py   Orchestrates gz-sim, robot_state_publisher, spawn, bridge, home pose, RViz.
config/bridge.yaml       ros_gz_bridge topic map (17 topics).
nodes/arm_kinematics.py  Closed-form FK/IK. Pure Python/math, no ROS dependency, self-testing.
nodes/pick_and_place.py  Demo sequencer: builds a waypoint plan, executes it as ROS commands.
nodes/pose_cmd.py        Publishes one named pose/grip to both arms, then exits.
```

Every dimension in `common.xacro` is tagged:
- **`[POSTER]`** — a locked value taken directly from the design spec (arm mount radius, arm
  separation, mounting height, link lengths, mast offset, camera/LiDAR heights).
- **`[PLACEHOLDER]`** — a plausible but not-yet-decided value where the spec left the choice open
  (link masses, joint limits/effort/velocity, gripper jaw geometry, motor selection).

`nodes/arm_kinematics.py` is **not** generated from `common.xacro` — its `MOUNT_X`, `MOUNT_Y`,
`J2_Z`, `L1`, `L2`, `L3` constants are hand-mirrored copies, checked by comment against the xacro.
**If arm geometry changes in `common.xacro`, `arm_kinematics.py` must be updated by hand or the IK
will silently target the wrong physical geometry.** This is the single most important coupling to
remember when editing either file.

---

## 3. Robot model

### 3.1 Base

`base_footprint` (ground contact frame) → fixed joint → `base_link` (body) → fixed joint →
`top_deck`. Base is a cylinder primitive (radius 17.75 cm, height 9 cm, mass 2.5 kg — Kobuki is
~2.35 kg dry, so this is close but not exact) rather than a mesh; see §7 for why.

The top deck sits with its **top surface** exactly at `deck_top_z = 0.14 m` above ground — every
arm and mast mount is referenced off that surface, not off `base_link`, so raising/lowering
`deck_top_z` moves the whole superstructure without touching mount math.

### 3.2 Arm kinematic chain

Each arm is a macro (`four_dof_arm`, parameterized by `prefix` = `left`/`right` and `reflect` =
±1 for the lateral mirror) instantiated twice off the deck:

```
top_deck
  -> [fixed]   mount pad, offset (arm_offset_fwd, reflect*arm_offset_lat) from deck centre
  -> [J1 revolute, axis Z]   riser, length h0 = 4 cm            (yaw)
  -> [J2 revolute, axis Y]   upper_arm, length L1 = 16 cm       (shoulder pitch)
  -> [J3 revolute, axis Y]   forearm, length L2 = 16 cm         (elbow pitch)
  -> [J4 revolute, axis Y]   wrist, length 3 cm                 (wrist pitch)
  -> [fixed]   palm, length 3 cm
  -> [prismatic x2] finger_a (+Y), finger_b (-Y), stroke 3.5 cm each
  -> [fixed]   grasp_frame, offset palm_len + 0.6*finger_len down the palm Z axis
```

J2, J3, J4 all rotate about parallel Y axes, so **the arm is a planar 3R chain living in one
vertical plane**, and J1 alone selects which vertical plane (its heading about Z). This is the
mechanical fact the entire IK derivation in §4 rests on — it is not a solver choice, it is how the
joints are wired.

Every revolute joint carries its own visible housing cylinder oriented along its rotation axis
purely so articulation reads clearly in the GUI (otherwise 4 co-planar pitch joints at zero
deflection look like one fused rod — this bit the home-pose choice too, see §5).

**Verified geometry** (design sheet vs. built model, from the package README):

| Quantity | Spec | Model |
|---|---|---|
| Arm mount radius r = √(fwd² + lat²) | 13 cm | 13.001 cm |
| Arm separation S (between left/right mounts) | 17.75 cm | 17.750 cm |
| Arm mounting height (deck top) | 14 cm | 14.000 cm |
| Geometric reach, J2 to J4 (L1 + L2) | 32 cm | 32.0 cm |
| Reach to grasp point (L1 + L2 + L3) | not specified | 41.3 cm |
| Riser h0 | 4 cm | 4.0 cm |

### 3.3 Gripper

Two prismatic fingers per arm, both reading `0` fully closed, driven symmetrically (same commanded
position on both) for a max opening of `2 x finger_stroke = 7.0 cm`. Each finger has a raised
contact pad (`pad_green` material) set with high friction (`mu1 = mu2 = 1.6`) and stiff contact
parameters (`kp = 1e6`, `kd = 100`) — this is what makes the friction-only grasp in §6 viable at
all; without the pad-level friction override the default Gazebo contact would let the object slip.

### 3.4 Sensor mast

Two posts at radius 14 cm from deck centre, joined by a crossbar (`mast_head`). Camera and LiDAR
mount off the crossbar, and their heights are specified **above ground level (AGL)**, not above the
deck — `common.xacro` derives the actual mount offset from the AGL target so that changing
`camera_height_agl` or `lidar_height_agl` is the only edit needed to retarget a sensor height:

```
mast_head_agl = deck_top_z + mast_height
camera_z_off  = camera_height_agl - mast_head_agl
lidar_z_off   = lidar_height_agl  - mast_head_agl
```

- RGB camera: 46 cm AGL, pitched 0.40 rad (~23°) down, 1.204 rad (~69°) horizontal FOV, 640x480,
  15 Hz, published on `camera/image_raw` + `camera/camera_info`.
- 2D LiDAR: 32 cm AGL, level, 360 samples over a full 2π sweep, 0.12–8.0 m range, 10 Hz, published
  on `scan`.

Both use Gazebo Harmonic's native `<sensor>` blocks under a `gz-sim-sensors-system` plugin
(`ogre2` render engine) declared in the world file — not `gazebo_ros` sensor plugins.

---

## 4. Control architecture

**There is no `ros2_control` in this project.** Instead, `robot.urdf.xacro` instantiates one
`gz-sim-joint-position-controller-system` plugin **per joint** (12 total: 4 arm joints x 2 arms +
2 fingers x 2 arms), each listening on its own topic:

```
/arm/<side>_<joint>/cmd_pos    std_msgs/Float64   (e.g. /arm/left_j2/cmd_pos)
```

PID gains differ by joint role:

| Joint class | P | I | D | i_max | cmd_max |
|---|---|---|---|---|---|
| j1 (yaw) | 6.0 | 0.05 | 0.6 | 1.0 | 5.0 N·m |
| j2 (shoulder) | 9.0 | 0.10 | 0.9 | 1.0 | 5.0 N·m |
| j3 (elbow) | 7.0 | 0.08 | 0.7 | 1.0 | 5.0 N·m |
| j4 (wrist) | 4.0 | 0.05 | 0.4 | 1.0 | 5.0 N·m |
| fingers (prismatic) | 600.0 | 8.0 | 20.0 | 12.0 | 25.0 N |

Arm joints are tuned soft-ish (they only have to hold pose against gravity/link weight); fingers
are tuned very stiff because they must sustain a squeeze against contact reaction force without
backing off (see §6's force budget).

`ros_gz_bridge` (`config/bridge.yaml`) mirrors 17 topics between ROS 2 and Gazebo's transport
layer: `/clock`, `/joint_states`, `/scan`, `/camera/image_raw`, `/camera/camera_info` flow
Gazebo→ROS; all 12 `cmd_pos` topics flow ROS→Gazebo.

**The two arms and all 12 joints are fully independent.** There is no synchronization,
no coupling, no shared controller state — any node (or `ros2 topic pub`) can move exactly one
joint without affecting anything else. This was a deliberate design point (see README §5) so that
partial/manual control is trivially possible during development.

Joint states (all 8 revolute + 4 prismatic positions) are published by a
`gz-sim-joint-state-publisher-system` plugin onto `joint_states`, then bridged to ROS.

---

## 5. Bring-up sequencing (`spawn.launch.py`)

Bring-up has a real ordering dependency and is currently enforced with **fixed timer delays**,
not readiness checks:

1. `SetEnvironmentVariable(GZ_SIM_RESOURCE_PATH, ...)` so Gazebo can resolve the world file and any
   meshes shipped in package share.
2. Include `ros_gz_sim`'s `gz_sim.launch.py` with a computed `gz_args` string (world file path +
   `-r` run-immediately + `-v 3` info logging, plus `-s --headless-rendering` appended when
   `gui:=false`).
3. Start `robot_state_publisher` immediately, fed `robot_description` from
   `Command(["xacro ", xacro_file])` wrapped in `ParameterValue(..., value_type=str)`. **This
   explicit `value_type=str` is required** — without it, `launch` tries to infer the parameter
   type from the expanded URDF string, fails, and `robot_state_publisher` never starts. The
   failure is silent: `/robot_description` is simply never published, the spawn step has nothing
   to spawn from, and the world still loads normally, making the fault easy to miss.
4. **4 s later** (`TimerAction`), call `ros_gz_sim create` to spawn the robot from
   `/robot_description` at the origin. The delay exists because gz-sim needs time to finish
   loading the world and advertise its spawn service before `create` can call it; there is no
   poll/retry, just a flat wait.
5. Start `ros_gz_bridge` (`parameter_bridge`) immediately, config-driven.
6. **8 s later**, if `home_pose:=true` (default), run `pose_cmd.py home` once to drive both arms
   off their zero pose. This matters because the zero pose is arms-straight-up with all 4 pitch
   joints coplanar and undeflected — visually it reads as a single fused rod rather than an
   articulated arm, so without this step the demo looks broken even when it isn't.
7. Optionally start `rviz2` if `rviz:=true`.

Launch args: `gui` (default `true`), `rviz` (default `false`), `world` (default
`manipulation_task`), `use_sim_time` (default `true`), `home_pose` (default `true`).

---

## 6. The kinematics

### 6.1 Why closed-form works

J2, J3, J4 share a rotation axis (Y, after J1's yaw), so once J1 fixes a heading, the remaining
3 joints move the end effector only within that one vertical plane — a classic planar 3R chain.
4 DOF gives exactly 4 constraints: 3 for target position (x, y, z) and **one** for orientation.
That one orientation DOF is the tool pitch `psi` — the cumulative angle of the gripper from
vertical (`psi = j2 + j3 + j4`). **There is no free choice of tool roll or yaw** — the gripper's
approach axis is locked inside whatever plane J1 selects. This is a property of the mechanism, not
a limitation of the solver, and it is the fact behind the large-object grasp problem in §8.

### 6.2 Forward kinematics

For arm side `s ∈ {left, right}` with shoulder position (from `common.xacro`):

```
shoulder(s) = (MOUNT_X, sign(s) * MOUNT_Y, J2_Z)
  MOUNT_X = 0.095 m       (arm_offset_fwd)
  MOUNT_Y = 0.08875 m     (arm_offset_lat)
  J2_Z    = 0.190 m       (deck_top_z + riser_h0 + J1 offset)
  sign(left) = +1, sign(right) = -1
```

Walking the planar chain, accumulate the pitch angle and integrate radial (`s`) / vertical (`h`)
offsets from the shoulder:

```
cum = 0; s = 0; h = 0
for (angle, length) in [(j2, L1), (j3, L2), (j4, L3)]:
    cum += angle
    s += length * sin(cum)
    h += length * cos(cum)

grasp_xyz = ( shoulder.x + s*cos(j1),
              shoulder.y + s*sin(j1),
              shoulder.z + h )
```

where `L1 = L2 = 0.160 m` and `L3 = 0.093 m` (wrist + palm + 0.6 x finger length — the grasp point
sits between the finger pads, 60% down their length, matching the `grasp_offset` xacro property).

### 6.3 Inverse kinematics

Given a target `(x, y, z)` and a chosen tool pitch `psi`:

1. **J1** — bearing of the target off the shoulder in the XY plane:
   `j1 = atan2(y - shoulder.y, x - shoulder.x)`
2. **Back off to the wrist point** — the grasp frame is `L3` beyond the wrist along direction
   `psi`; subtract that offset in the (radial, vertical) plane:
   ```
   s_t = hypot(x - shoulder.x, y - shoulder.y);  h_t = z - shoulder.z
   s_w = s_t - L3*sin(psi);  h_w = h_t - L3*cos(psi)
   ```
3. **Textbook 2R solve for J2/J3** on the wrist point `(s_w, h_w)`:
   ```
   r² = s_w² + h_w²
   c3 = (r² - L1² - L2²) / (2*L1*L2)              # clamped to [-1, 1]
   j3 = ±acos(c3)                                  # elbow up (+) / down (-)
   j2 = atan2(s_w, h_w) - atan2(L2*sin(j3), L1 + L2*cos(j3))
   ```
4. **J4** closes the orientation constraint: `j4 = psi - j2 - j3`.

Reachability is checked against the 2R annulus (`|L1-L2| <= r <= L1+L2`) before solving, raising
`Unreachable` with the actual vs. max distance if violated. With `elbow="auto"` (the default used
by the demo), both elbow signs are tried and the **first solution that satisfies every joint
limit** is returned; if neither does, `Unreachable` is raised listing which joints were out of
range for each elbow choice.

`reach_fraction(target)` reports `r / (L1 + L2)` — the fraction of the 2R envelope consumed. Above
roughly 0.9 the Jacobian is poorly conditioned (near full extension); `pick_and_place.py` flags any
waypoint above 90% in its printed report.

**Self-test** (`python3 nodes/arm_kinematics.py`, no ROS needed): 4000 random joint configurations
per arm are run through FK, then the resulting Cartesian point is fed back through IK with the
matching `psi`, and the round-trip position error is measured. Result: worst-case error under one
micrometre across ~3900 solved poses (some fraction of the 8000 random configs land outside the
reachable annulus and are skipped, not counted as failures).

### 6.4 Trajectory generation

`pick_and_place.py` solves **every** Cartesian waypoint of the task up front via `arm_kinematics.ik`
before commanding any motion — an infeasible plan fails immediately (`build_plan()` raises before
`run()` is ever called), which is what `--dry-run` exercises.

Between consecutive joint-space waypoints, each joint (and the gripper) is interpolated with a
quintic minimum-jerk profile — zero velocity and zero acceleration at both endpoints:

```
s(t) = 10t³ - 15t⁴ + 6t⁵,   t ∈ [0, 1]
q(t) = q_start + (q_target - q_start) * s(t)
```

evaluated at 50 Hz for the waypoint's allotted duration. This is **joint-space**, not Cartesian,
interpolation — the path between two waypoints is not a straight line in task space. That's fine
because waypoints are close together and the workspace is empty of obstacles between them, but it
is precisely why the plan has explicit `lift`/`transit`/`pre-place` waypoints instead of one direct
pick-to-place move: a naive 2-waypoint plan would cut a joint-space-interpolated (and therefore
curved, unpredictable) path straight through the table edge.

Peak joint velocity across the full demo sequence is ~0.64 rad/s against a 2.0 rad/s limit — large
headroom to speed the demo up if needed.

---

## 7. The pick-and-place task

Defined entirely in `nodes/pick_and_place.py`, left arm only:

```
PICK    = (0.31, 0.09, 0.225)   object_min's centre, on the starting base
PLACE   = (0.20, 0.27, 0.225)   target point, on the destination base
TRANSIT = (0.26, 0.18, 0.32)    waypoint between the two tables, above both
PSI     = 2.35 rad (135°)       diagonal approach (see below)
APPROACH = 0.08 m               vertical standoff for pre-grasp / pre-place
```

Nine-waypoint plan: `pre-grasp -> grasp -> close -> lift -> transit -> pre-place -> place -> open
-> retreat`, each with its own duration (1.0–2.5 s), followed by a 2.5 s return to `HOME`. Total
motion time is reported by `build_plan()`/`report()`; end to end is roughly 17 s including the
return home.

**Why `psi = 2.35 rad` and not straight down (`psi = π`):** a top-down grasp puts the wrist 9.3 cm
(`L3`) directly above the object, which pushes every pick waypoint past ~95% of the reach envelope
— poorly conditioned and close to a singularity. The 135° diagonal approach keeps every waypoint
under 75% of the envelope while still being a stable, repeatable pinch angle.

The idle right arm is explicitly parked at `HOME` and held there for the whole sequence — it is
not simply "left alone," since leaving it uncommanded would leave it wherever it happened to be
after the launch-time home move, which is fine in practice but is deliberately made explicit for
determinism.

### Grasp force budget

The grasp is **friction-only** — there is no attach/weld plugin. Fingers close 4 mm past the
object's surface (`GRIP_CLOSED = OBJECT_RADIUS - 0.004`), and the stiff finger PID (P=600, see
§4) sustains that interference against the object's elastic pushback.

```
object mass............ 0.08 kg
weight to hold......... 0.08 * 9.81 = 0.245 N   (two pads must jointly resist this via friction)
contact friction coeff.. mu = 1.6 (both finger pad and object surface)
interference............ 4 mm -> roughly 2.4 N of squeeze force at the tuned finger stiffness
available holding force. 2 * mu * squeeze = 2 * 1.6 * 2.4 ≈ 7.7 N   (order-of-magnitude, not exact)
margin over weight...... ~10x
finger effort ceiling... 25 N — squeeze force is well under this
```

If contact solving proves unreliable in practice (README notes: "Gazebo contact solvers are not
always kind"), the documented fallback is a `gz-sim-detachable-joint-system` plugin that welds
`object_min` to `left_palm` on command via `/grasp/attach` and `/grasp/detach` topics. **This
snippet is documented but explicitly untested** — the exact element names have moved across
gz-sim versions and it was written without a running Gazebo to check against.

---

## 8. World / task definition

`worlds/manipulation_task.sdf`: ground plane (50x50 m, mu=1.0), one directional light with
shadows, physics at 1 ms step / real-time factor 1.0, and the `Physics`, `UserCommands`,
`SceneBroadcaster`, `Contact`, and `Sensors` (ogre2) system plugins.

| Model | Pose (world) | Top surface | Purpose |
|---|---|---|---|
| `starting_base` | (0.41, 0.02, 0) | z = 0.20 m, 30x28 cm top on a 12x12x18 cm pedestal | Holds `object_min`; positioned so every pick waypoint stays under 75% reach (see §7) |
| `destination_base` | (0.22, 0.30, 0) | z = 0.20 m, 24x24 cm top on a 10x10x18 cm pedestal | Place target |
| `object_min` | (0.31, 0.09, 0.225) | — | 5 cm dia x 5 cm tall cylinder, 0.08 kg, mu=1.6. **The pick-and-place demo object** — fits the 7 cm jaw with 2 cm margin. |
| `object_max` | (0.44, 0.02, 0.325) | — | 18 cm dia x 25 cm tall cylinder, 0.50 kg (payload ceiling), mu=1.6. **Not part of the demo** — see §9. |

The two objects deliberately sit at the two extremes of the size/mass envelope the design sheet
specifies, so the world doubles as a validation fixture for "smallest reachable/grippable object"
and "largest payload" rather than just a demo prop set.

---

## 9. Open design questions

These are unresolved decisions, not bugs — each needs a spec answer before the corresponding
behavior can be implemented for real.

**Large-object bimanual grasp is structurally blocked.** `object_max` is 18 cm in diameter; the
gripper jaw opens 7 cm; the arm's approach axis is locked inside the J1 plane (§6.1), so a single
arm can't push laterally on an object in front of the robot. Yawing both arms inward gives only a
**partial** pinch, and it gets worse the further out the object sits:

| Object standoff | J1 convergence | Inward force fraction |
|---|---|---|
| 0.50 m | -12.4° | 21% |
| 0.35 m | -19.2° | 33% |
| 0.2675 m | -27.2° | 46% |

0.2675 m is the floor — any closer and the object collides with the deck edge. So the best
available squeeze is ~46% of applied actuator force, with the base pressed right up against the
object, and it's an **angled** friction pinch (contact normal not lateral, separation not fixed) —
not the `p_R - p_L = G * n_hat` opposed-grasp model that force closure normally assumes. Probably
workable at mu=1.6 with a 500 g object, but unproven and should be resolved before locking any
bimanual actuator sizing.

**Arm separation is 2.5 mm short of the object diameter.** `S = 17.75 cm` (arm separation) vs.
`object_max diameter = 18.00 cm` — the grippers physically cannot approach the large object in
parallel even before considering the jaw-opening limit above.

**LiDAR forward-sector self-occlusion.** At 32 cm AGL on the rear mast, the arms occupy roughly
±34° bearings at 15.7 cm range and the mast posts themselves occupy ±90° at 12.1 cm — the forward
sector the LiDAR needs for navigation is partly self-blocked. Needs either a bearing mask in the
LiDAR consumer or relocating the LiDAR to the front deck edge.

**Payload vs. size envelope looks inconsistent.** `object_max` at 18x25 cm and 500 g works out to
~79 kg/m³ — essentially an empty vessel. The same envelope filled with water would be ~6 kg, 12x
the stated payload ceiling. The two spec numbers together only really admit empty/lightweight
containers; this may be intentional (task is "move an empty box," not "move a filled one") but it
is not stated on the design sheet and should be confirmed.

**Shoulder torque margin is thinner than it first looks.** The spec's ~1.6 N·m appears to be just
the payload term (`0.5 kg x 0.32 m x 9.81 = 1.57 N·m`). Including link self-weight, actual required
torque is closer to 2.75 N·m. A common hobby-servo choice (XM430-W350-class, ~4.1 N·m stall at
12 V) still clears this, but the margin drops from ~2.5x to ~1.5x, and stall torque is not
continuous-duty torque — worth re-checking against a duty-cycle spec before committing to a motor.

---

## 10. Known simplifications (deliberate, not gaps)

- **Base is a cylinder primitive, not a Kobuki mesh.** Radius/height/mass are correct; using a
  primitive avoids a hard dependency on `kobuki_description`, whose package naming has shifted
  across ROS distros, and nothing about the arm kinematics depends on the visual mesh.
- **Top deck is a disc, not a hexagonal prism.** URDF has no native hex-prism primitive; the disc
  uses the hexagon's circumradius, so every mount position and the footprint radius are exact —
  only the silhouette in the viewer is wrong.
- **Arms face +X; the sensor mast is behind them.** The original top-view spec doesn't label a
  front. If this is backwards for the intended use, flip `arm_offset_fwd` and `mast_offset_x` in
  `common.xacro` — nothing else needs to change.
- **No wheels, no drive.** The base is a static rigid body sitting on the ground plane; navigation
  is out of scope for this milestone, and the pick-and-place demo has no approach/retreat travel —
  the robot is placed already in position. Adding a `DiffDrive` plugin is estimated at ~15 lines
  when navigation is in scope.
- **Joint-space, not Cartesian, path interpolation** (§6.4) — acceptable here because the demo's
  waypoints are close together in an empty workspace; would need revisiting for any task requiring
  guaranteed straight-line tool paths.
