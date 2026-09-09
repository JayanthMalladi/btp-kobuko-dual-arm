# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A colcon workspace (`~/dual_arm_ws`) with one ROS 2 package, `src/dual_arm_kobuki`: a Gazebo Harmonic
simulation of a Kobuki-class mobile bimanual manipulator (two 4-DOF arms + 2-finger grippers, RGB
camera + 2D LiDAR mast). Stack: **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)** via `ros_gz`.

Milestone 1 scope: the robot and world spawn correctly and a scripted pick-and-place demo runs. There
is no `ros2_control`, no MoveIt, no navigation/drive base — every joint is driven directly by a Gazebo
`JointPositionController` plugin over a `/arm/<joint>/cmd_pos` topic.

Not a git repository — there is no VCS history to consult here.

## Build / run commands

```bash
source /opt/ros/jazzy/setup.bash
cd ~/dual_arm_ws
colcon build --symlink-install      # --symlink-install matters: edits to .xacro/.sdf take
                                     # effect on next launch without rebuilding
source install/setup.bash

ros2 launch dual_arm_kobuki spawn.launch.py                  # full sim + spawn + bridge
ros2 launch dual_arm_kobuki spawn.launch.py rviz:=true       # also open RViz2
ros2 launch dual_arm_kobuki spawn.launch.py gui:=false       # headless (GPU/driver issues)
ros2 launch dual_arm_kobuki spawn.launch.py world:=<name>    # load a different worlds/<name>.sdf

ros2 run dual_arm_kobuki pick_and_place.py            # run the demo (after spawn.launch.py)
ros2 run dual_arm_kobuki pick_and_place.py --dry-run  # solve + print the joint plan, no ROS/Gazebo
ros2 run dual_arm_kobuki pose_cmd.py home|zero|reach|pinch|open|close   # manual named-pose helper

python3 src/dual_arm_kobuki/nodes/arm_kinematics.py   # self-test: FK/IK round-trip over ~4000
                                                        # random poses per arm, reports worst error
                                                        # (this is the closest thing to a test suite)
```

There is no separate lint/test target — `colcon build` (ament_cmake, no compiled code) mainly
validates `package.xml`/`CMakeLists.txt`, and the kinematics self-test above is the correctness check.

### Verifying a change actually worked (from the package README)

Don't trust "the window opened" — check in a second sourced terminal:
- Arms stand straight up and stay there (drooping = position controllers didn't load; check the
  Gazebo console for `JointPositionController` errors).
- `ros2 run tf2_tools view_frames` → 21 frames rooted at `base_footprint`, ending in
  `left_grasp_frame` / `right_grasp_frame`.
- `ros2 topic hz /scan` (~10 Hz) and `ros2 topic hz /camera/image_raw` (~15 Hz) — silence usually
  means a GPU/driver issue with the `gz-sim-sensors-system` plugin; `gui:=false` uses
  `--headless-rendering` as the workaround.
- `ros2 topic echo /joint_states --once` → 8 names, `left_j1..j4` / `right_j1..j4`.

## Architecture

### Package layout (`src/dual_arm_kobuki/`)

```
urdf/common.xacro        All dimensions, masses, joint limits, HOME pose. Change numbers HERE only.
urdf/base.xacro          Kobuki body (cylinder primitive) + top deck.
urdf/arm.xacro           4-DOF arm + wrist + 2-finger gripper, as a reflectable macro (x2: left/right).
urdf/sensors.xacro       Mast, RGB camera, 2D LiDAR links + gz sensor blocks.
urdf/robot.urdf.xacro    Assembles the above + declares all Gazebo system plugins (below).
worlds/manipulation_task.sdf   Starting base, destination base, small/large objects.
launch/spawn.launch.py   Orchestrates: gz-sim, robot_state_publisher, spawn, bridge, home pose, rviz.
config/bridge.yaml       ros_gz_bridge topic map (17 topics: sensors out, joint commands in).
nodes/arm_kinematics.py  Closed-form FK/IK for the 4-DOF arm. No ROS dependency — pure math module.
nodes/pick_and_place.py  Demo sequencer: builds a waypoint plan with arm_kinematics, executes it.
nodes/pose_cmd.py        Publishes a fixed named pose/grip to both arms, then exits.
```

`common.xacro` is the single source of truth for geometry: every other xacro file and
`nodes/arm_kinematics.py` derive their constants from it. **The `MOUNT_X`/`MOUNT_Y`/`J2_Z`/`L1`/`L2`/`L3`
constants at the top of `arm_kinematics.py` are hand-mirrored copies of `common.xacro` values, not
computed from it** — if you change arm geometry in `common.xacro`, update `arm_kinematics.py` to match
or IK will silently target the wrong geometry.

Every `common.xacro` value is tagged `[POSTER]` (locked spec value, verified in the README table) or
`[PLACEHOLDER]` (plausible but not a design decision — gripper geometry, joint limits, link masses).

### Control path (no ros2_control)

Each of the 12 joints (4 per arm x2, 2 gripper fingers x2) gets its own
`gz-sim-joint-position-controller-system` plugin instance, declared in `robot.urdf.xacro`, listening on
its own `/arm/<joint>/cmd_pos` `std_msgs/Float64` topic. These are bridged ROS<->Gazebo by
`ros_gz_bridge` per `config/bridge.yaml`. The 12 joints are fully independent — there is no
synchronization or coupling between the two arms; anything driving motion (`pose_cmd.py`,
`pick_and_place.py`) publishes to each joint's topic directly.

### Launch sequence (`spawn.launch.py`)

Order matters and is enforced with `TimerAction` delays, not readiness checks: gz-sim world load, then
`robot_state_publisher` (needs `/robot_description` from xacro via `Command(["xacro ", ...])`, must use
`ParameterValue(..., value_type=str)` or the type inference fails silently and nothing spawns), then a
4s-delayed `ros_gz_sim create` spawn (world needs time to advertise its spawn service), then an
8s-delayed `pose_cmd.py home` run to drive arms off their zero pose (zero = arms straight up, which
looks like a single fused rod rather than an articulated 4-DOF arm).

### Inverse kinematics (`nodes/arm_kinematics.py`)

The arm is J1 (yaw about Z) then J2/J3/J4 (all pitch about the *same* axis) — a planar 3R chain in a
vertical plane whose heading J1 picks. This is why IK is closed-form rather than iterative:

1. J1 = bearing of the target off the shoulder.
2. Choose tool pitch `psi` (cumulative gripper angle from vertical — the demo uses 2.35 rad / 135°, a
   diagonal approach that costs much less reach envelope than top-down `psi = pi`).
3. Back off `L3` along `psi` to get the wrist point, then solve a textbook 2R problem for J2/J3.
4. J4 = psi - J2 - J3.

4 DOF = 3 position constraints + exactly 1 orientation constraint (`psi`) — there is no tool roll or
yaw; the approach axis is locked inside the J1 plane. `ik()` tries elbow-up and elbow-down (`elbow=
"auto"`) and returns the first solution inside all joint limits, else raises `Unreachable`.
`reach_fraction()` reports how much of the 2R envelope (`L1+L2`) a target consumes; above ~0.9 the
solve is poorly conditioned (near full extension).

### Demo sequencing (`nodes/pick_and_place.py`)

`build_plan()` solves every Cartesian waypoint via `arm_kinematics.ik()` up front — infeasibility is
caught before any motion starts (see `--dry-run`). Waypoints are then interpolated in **joint space**
with a minimum-jerk profile at 50 Hz (`run()`), not Cartesian space — paths between waypoints are not
straight lines, which is why explicit lift/transit/pre-place waypoints exist instead of one direct
pick-to-place move. Only the left arm moves; the idle right arm is parked at `HOME` and held there.

The grasp is friction-only (stiff finger PID squeezing ~4mm past the object surface against mu=1.6
pads) — there is no attach/detach plugin. See README section 8 for the force budget and a documented
(but untested) `DetachableJoint` fallback if contact solving proves unreliable.

### Known structural limits (see README sections 9-10 for full detail)

- The large object (18cm) cannot be picked by a single gripper (7cm jaw) and is *inherently* the
  bimanual case, but the shared-pitch-axis 4-DOF arm can't push laterally on something in front of the
  robot — only a partial (~46% max) angled friction pinch is available by yawing both arms inward.
  Read that section before wiring up any bimanual grasp logic.
- Base is a rigid body with no wheels/drive; navigation/approach phases are not modelled.
- Arms face +X; flip `arm_offset_fwd`/`mast_offset_x` in `common.xacro` if the front is wrong.
