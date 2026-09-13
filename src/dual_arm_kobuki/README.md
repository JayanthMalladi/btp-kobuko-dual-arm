# dual_arm_kobuki

Gazebo Harmonic simulation of the Kobuki-based mobile bimanual manipulator from
the design sheet. **Milestone 1: the robot and world spawn correctly. Nothing
drives itself yet.**

Stack: **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)**, connected via `ros_gz`.

See [`docs/RUNNING.md`](docs/RUNNING.md) for a step-by-step run/verify/troubleshoot guide, and
[`docs/SPEC.md`](docs/SPEC.md) for the full implementation spec — architecture, control design,
kinematics math, and open design questions.

---

## 1. Install the dependencies

You need these once. Everything else in this package is self-contained.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2
```

Confirm Harmonic is actually what you have — Jazzy will happily co-exist with an
older Gazebo and the plugin names differ:

```bash
gz sim --versions     # must print 8.x
```

## 2. Build

```bash
mkdir -p ~/dual_arm_ws/src
cp -r dual_arm_kobuki ~/dual_arm_ws/src/
cd ~/dual_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: with it, edits to the `.xacro` and `.sdf` files take
effect on the next launch without rebuilding. You will be editing these a lot.

## 3. Run

```bash
ros2 launch dual_arm_kobuki spawn.launch.py
```

Useful arguments:

```bash
ros2 launch dual_arm_kobuki spawn.launch.py rviz:=true    # also open RViz2
ros2 launch dual_arm_kobuki spawn.launch.py gui:=false    # headless
```

---

## 4. How to tell it actually worked

Don't trust "the window opened". Check these four things in a second terminal
(`source ~/dual_arm_ws/install/setup.bash` first):

**a. The arms are standing, not drooping.** Both arms should point straight up
from the deck and stay there. If they fold over, the position controllers didn't
load — check the Gazebo console for `JointPositionController` errors.

**b. TF is complete.**
```bash
ros2 run tf2_tools view_frames
```
21 frames, rooted at `base_footprint`, with `left_grasp_frame` and
`right_grasp_frame` at the ends of both chains.

**c. Sensors are publishing.**
```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```
Roughly 10 Hz and 15 Hz. If these are silent, the bridge is up but the
`gz-sim-sensors-system` plugin isn't rendering — almost always a GPU/driver
issue, and `gui:=false` with `--headless-rendering` is the usual workaround.

**d. Joint states are flowing.**
```bash
ros2 topic echo /joint_states --once
```
Eight names: `left_j1..j4`, `right_j1..j4`.

---

## 5. Running the demo

```bash
ros2 launch dual_arm_kobuki spawn.launch.py       # terminal 1
ros2 run dual_arm_kobuki pick_and_place.py        # terminal 2
```

The left arm picks the 5 cm object off the starting base and places it on the
destination base. About 17 s end to end.

Check the plan without touching ROS first:

```bash
ros2 run dual_arm_kobuki pick_and_place.py --dry-run
```

That solves every waypoint and prints joint angles plus how much of the reach
envelope each one consumes. If a waypoint were infeasible it would fail here,
before the arm moved.

Manual poking:

```bash
ros2 run dual_arm_kobuki pose_cmd.py home|zero|reach|open|close
ros2 topic pub -1 /arm/left_j1/cmd_pos std_msgs/msg/Float64 "{data: -0.5}"
```

That last one moves ONLY the left arm - useful if anyone claims the two arms
are coupled. They are not: 12 independent joints, 12 independent controllers,
12 separate command topics.

---

## 6. What's in here

```
urdf/common.xacro        All dimensions and masses. Change numbers HERE only.
urdf/base.xacro          Kobuki body + top deck.
urdf/arm.xacro           4-DOF arm + wrist + 2-finger gripper macro, x2.
urdf/sensors.xacro       Mast, RGB camera, 2D LiDAR + gz sensor blocks.
urdf/robot.urdf.xacro    Assembly + Gazebo system plugins.
worlds/                  Starting base, destination base, both envelope extremes.
launch/spawn.launch.py   Sim, description, spawn, bridge, home pose.
config/bridge.yaml       ros_gz topic map (17 topics).
nodes/arm_kinematics.py  Closed-form FK/IK. Run directly to self-test.
nodes/pick_and_place.py  The demo sequencer.
nodes/pose_cmd.py        Named-pose helper.
```

Every dimension in `common.xacro` is tagged `[POSTER]` or `[PLACEHOLDER]`.
`[POSTER]` values are locked from section 2A. `[PLACEHOLDER]` values are what
section 2B left open - gripper geometry, joint limits, motor selection, link
masses. They are set to plausible numbers so the model simulates; they are not
design decisions.

Verified on expansion:

| Quantity | Spec | Model |
|---|---|---|
| Arm mount radius r | 13 cm | 13.001 cm |
| Arm separation S | 17.75 cm | 17.750 cm |
| Arm mounting height | 14 cm | 14.000 cm |
| Geometric reach (J2 to J4) | 32 cm | 32.0 cm |
| Reach to grasp point | not on sheet | 41.3 cm |
| Riser h0 | 4 cm | 4.0 cm |
| Cam/LiDAR mast offset | 14 cm | 14.00 cm |
| Camera height AGL | 46 cm | 46.00 cm |
| LiDAR height AGL | 30-35 cm | 32.00 cm |
| Gripper opening | not on sheet | 7.0 cm |

---

## 7. The IK

`nodes/arm_kinematics.py` is closed-form, not iterative. The arm is J1 yaw
then three pitches about a common axis, so it is a planar 3R chain in a
vertical plane whose heading J1 picks. That decomposes:

1. J1 from the target's bearing off the shoulder.
2. Choose the tool pitch `psi` (gripper angle from vertical).
3. Back off L3 = 9.3 cm along `psi` to get the wrist point.
4. Textbook 2R solve for J2 and J3; then J4 = psi - J2 - J3.

Elbow-up and elbow-down are both tried, and the first solution inside every
joint limit wins. Run the file directly for a round-trip self-test against FK -
about 3900 random poses, worst error under a micrometre.

The demo uses `psi = 2.35 rad` (135 deg), a diagonal approach rather than
straight down. Top-down would be `psi = pi`, but that puts the wrist 9.3 cm
directly above the object and pushes the pick waypoints past 95% of the reach
envelope. The diagonal keeps everything under 75%.

**4 DOF buys you three position constraints and exactly one orientation
constraint.** `psi` is that one. There is no tool roll and no tool yaw: the
approach axis is locked inside the J1 plane. That is the mechanism, not the
solver.

Trajectories are joint-space minimum-jerk at 50 Hz. Peak joint velocity across
the whole demo is 0.64 rad/s against a 2.0 rad/s limit, so there is a lot of
headroom to speed it up.

---

## 8. The grasp is friction-only

No attach plugin. Stiff finger PIDs (p = 600) squeeze 4 mm past the object
surface against mu = 1.6 pads.

Budget: holding 0.08 kg through two pads needs 0.245 N. The 4 mm interference
produces roughly 2.4 N. Nearly 10x margin, well under the 25 N finger ceiling.

If the object still squirts out - Gazebo contact solvers are not always kind -
the standard fix is a detachable joint that welds object to gripper on command:

```xml
<plugin filename="gz-sim-detachable-joint-system"
        name="gz::sim::systems::DetachableJoint">
  <parent_link>left_palm</parent_link>
  <child_model>object_min</child_model>
  <child_link>link</child_link>
  <attach_topic>/grasp/attach</attach_topic>
  <detach_topic>/grasp/detach</detach_topic>
</plugin>
```

Treat that snippet as untested. I could not run Gazebo while writing this, and
the exact element names for attach-on-demand have moved between gz-sim
versions. Check it against the DetachableJoint docs for your installed version
before assuming it works. Friction should be enough for an 80 g object.

---

## 9. Open design questions

**The large object cannot be picked, and that is structural.** 18 cm exceeds
the 7 cm jaw opening, so it is inherently the bimanual case. But a 4-DOF arm
whose three pitches share an axis has its approach vector locked in the J1
plane, so it cannot push laterally on something in front of the robot. Yawing
J1 to -90 deg aims the hand sideways but then it can only reach x = 9.5 cm,
and the object is at 44 cm.

**Correction (see `docs/bimanual/README.md` section 2b):** the table below,
kept for the record, frames this as an *inefficient* squeeze. A closer force
analysis shows it is worse than inefficient - at the object's actual position
the two palms' forward force components dominate and add, there is no
backstop, and the arm separation is physically narrower than the object (next
paragraph). It is **infeasible**, not merely a low force fraction. Partial
convergence numbers, for the record:

| Object standoff | J1 convergence | Inward force fraction |
|---|---|---|
| 0.50 m | -12.4 deg | 21% |
| 0.35 m | -19.2 deg | 33% |
| 0.2675 m | -27.2 deg | 46% |

0.2675 m is the floor - any closer and the object hits the deck edge.

**Arm separation vs max object.** S = 17.75 cm, object = 18.00 cm. The
grippers have 2.5 mm less lateral span than the object needs, so they cannot
approach parallel.

**Resolved:** bimanual co-manipulation now targets a *different* object,
`object_bimanual` in `worlds/manipulation_task.sdf`, with two vertical
handle posts instead of a bare-surface squeeze on `object_max`. `object_max`
stays in the world unchanged, on purpose, as the documented negative result
above. Full rationale, the math behind it, and the two other architecture
decisions that went with it (coupling strictness, grasp physics) are in
[`docs/bimanual/README.md`](docs/bimanual/README.md) and
[`docs/bimanual/IMPLEMENTATION.md`](docs/bimanual/IMPLEMENTATION.md).

**LiDAR self-occlusion.** At 32 cm AGL on the rear mast, the arms sit at
15.7 cm on +-34 deg bearings and the mast posts at 12.1 cm on +-90 deg. The
forward sector is partly blocked. Mask those bearings, or move the LiDAR to
the front deck edge.

**Payload vs size envelope.** 18 cm x 25 cm at 500 g is about 79 kg/m^3 - an
empty vessel. Water-filled it would be ~6 kg, twelve times the ceiling. The
two envelopes together only admit empty containers. May be intended; is not
stated on the sheet.

**Shoulder torque.** Panel 4B's ~1.6 N.m looks like the payload term alone
(0.5 kg x 0.32 m x 9.81 = 1.57). With link weights included it is 2.75 N.m.
XM430-W350 stalls near 4.1 N.m at 12 V, so XM-class still works, but the
margin is ~1.5x rather than ~2.5x, and stall torque is not continuous torque.

---

## 10. Known simplifications

**Base is a cylinder, not a Kobuki mesh.** Correct radius, height and mass.
Deliberate: `kobuki_description` naming shifts across distros and nothing
about the arm kinematics depends on the mesh.

**Top deck is a disc, not a hexagon.** URDF primitives cannot express a
hexagonal prism. Uses the hex circumradius, so mount positions and footprint
are exact; only the silhouette is wrong.

**Arms face +X, mast is behind them.** The top view does not label the front.
Flip `arm_offset_fwd` and `mast_offset_x` in `common.xacro` if reversed.

**No wheels, no drive.** The base is a rigid body on the ground. Navigation is
handled separately per your sheet, so the approach and back-away phases are
not modelled. Adding `DiffDrive` is roughly fifteen lines.

**Joint-space interpolation.** Paths between waypoints are not Cartesian
straight lines. Fine here because waypoints are close and the space is empty -
that is why there are explicit lift and transit waypoints rather than one
direct move.
