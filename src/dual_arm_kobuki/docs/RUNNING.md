# dual_arm_kobuki — Running Guide

Step-by-step instructions to install, build, launch, verify, and drive the simulation. For what
the system actually does and why, see [`SPEC.md`](SPEC.md).

Target stack: **ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)**, connected via `ros_gz`.

---

## 1. Install dependencies (once per machine)

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

Confirm Harmonic is actually what's installed — Jazzy will happily coexist with an older Gazebo,
and the plugin names in this package (`gz-sim-*`) are Harmonic-specific and will fail to load
against an older gz-sim:

```bash
gz sim --versions     # must print 8.x
```

---

## 2. Build

```bash
mkdir -p ~/dual_arm_ws/src
cp -r dual_arm_kobuki ~/dual_arm_ws/src/     # skip if already checked out there
cd ~/dual_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: with it, edits to `.xacro` / `.sdf` / Python nodes take effect on the
next launch without rebuilding. You'll be editing `urdf/` and `nodes/` a lot — use it.

Every new terminal needs both source lines (or add them to `~/.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/dual_arm_ws/install/setup.bash
```

---

## 3. Launch the simulation

```bash
ros2 launch dual_arm_kobuki spawn.launch.py
```

This brings up Gazebo, publishes the robot description, spawns the robot 4 s later, starts the
ROS<->Gazebo bridge, and drives both arms to their home pose 8 s after that. Expect ~10 s from
launch to a fully posed, sensor-publishing robot.

Useful launch arguments (append as `key:=value`):

| Argument | Default | Effect |
|---|---|---|
| `gui` | `true` | `false` runs headless (server only, `--headless-rendering`) |
| `rviz` | `false` | `true` also opens RViz2 |
| `world` | `manipulation_task` | load a different `.sdf` from `worlds/` |
| `use_sim_time` | `true` | propagated to every node |
| `home_pose` | `true` | `false` skips the automatic home-pose move after spawn |

```bash
ros2 launch dual_arm_kobuki spawn.launch.py rviz:=true    # also open RViz2
ros2 launch dual_arm_kobuki spawn.launch.py gui:=false    # headless
```

---

## 4. Verify it actually worked

Don't trust "the window opened." In a **second** terminal (sourced as in step 2), check all four:

**a. The arms are standing, not drooping.**
Both arms should point up-and-back and hold there once the 8 s home-pose timer fires. If they
droop or fold, the position controllers didn't load — check the Gazebo console output for
`JointPositionController` errors.

**b. TF is complete.**
```bash
ros2 run tf2_tools view_frames
```
Expect 21 frames, rooted at `base_footprint`, with `left_grasp_frame` and `right_grasp_frame` at
the ends of both arm chains.

**c. Sensors are publishing.**
```bash
ros2 topic hz /scan               # expect roughly 10 Hz
ros2 topic hz /camera/image_raw   # expect roughly 15 Hz
```
If these are silent, the bridge is up but the `gz-sim-sensors-system` (ogre2) plugin isn't
rendering — almost always a GPU/driver issue. Relaunch with `gui:=false` to force
`--headless-rendering` as a workaround.

**d. Joint states are flowing.**
```bash
ros2 topic echo /joint_states --once
```
Expect 8 joint names: `left_j1..j4`, `right_j1..j4`.

---

## 5. Run the pick-and-place demo

With the sim already running (step 3):

```bash
ros2 run dual_arm_kobuki pick_and_place.py
```

The left arm picks the 5 cm object off the starting base and places it on the destination base.
About 17 s end to end, including the return to home.

**Check the plan before touching ROS at all:**

```bash
ros2 run dual_arm_kobuki pick_and_place.py --dry-run
```

This solves every waypoint with the analytic IK and prints joint angles, reach-envelope usage, and
total motion time — nothing moves. If a waypoint is kinematically infeasible, this is where it
shows up, before the arm ever moves. Example of what a healthy dry run reports:

```
step        reach   j1      j2      j3      j4      grip
--------------------------------------------------------------
pre-grasp   ...%   +0.xxx  +0.xxx  +0.xxx  +0.xxx  x.x cm
...
--------------------------------------------------------------
total motion time: ~17.0 s
```

Any step flagged `<-- extended` is above 90% of the reach envelope — worth a second look, not
necessarily a failure.

---

## 6. Manual / exploratory control

**Named poses and gripper states**, applied to both arms at once:

```bash
ros2 run dual_arm_kobuki pose_cmd.py home    # parked pose, used at launch
ros2 run dual_arm_kobuki pose_cmd.py zero    # all joints 0 (arms straight up)
ros2 run dual_arm_kobuki pose_cmd.py reach   # forward/down toward the starting base
ros2 run dual_arm_kobuki pose_cmd.py pinch   # yawed outward, bimanual pinch stance
ros2 run dual_arm_kobuki pose_cmd.py open    # grippers to 7.0 cm opening
ros2 run dual_arm_kobuki pose_cmd.py close   # grippers fully closed
```

**Single-joint control**, to confirm the arms are truly independent (12 joints, 12 controllers, 12
separate topics — moving one never affects any other):

```bash
ros2 topic pub -1 /arm/left_j1/cmd_pos std_msgs/msg/Float64 "{data: -0.5}"
```

Every joint has its own `/arm/<side>_<joint>/cmd_pos` topic of type `std_msgs/msg/Float64`:
`left_j1..j4`, `right_j1..j4`, `left_finger_a/b`, `right_finger_a/b`.

**Kinematics sanity check**, no ROS or Gazebo needed:

```bash
python3 src/dual_arm_kobuki/nodes/arm_kinematics.py
```

Runs an FK/IK round-trip self-test over ~4000 random poses per arm and reports the worst position
error (expect sub-micrometre). Use this after touching any geometry constant in `common.xacro` or
`arm_kinematics.py` to confirm the two are still in sync (see `SPEC.md` §2 for why they can drift).

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| World loads, robot never appears | `robot_state_publisher` never started because `robot_description`'s `ParameterValue` lost its `value_type=str` | Check for `robot_state_publisher` errors in the launch output; see `SPEC.md` §5 |
| Arms droop or fold over | Position controller plugins didn't load | Check the Gazebo console for `JointPositionController` errors |
| Arms look like one straight rod at startup | Expected transiently — home pose applies 8 s after launch (`home_pose:=true`) | Wait, or check `home_pose` wasn't set to `false` |
| `/scan` or `/camera/image_raw` silent | GPU/driver issue with `ogre2` rendering | Relaunch with `gui:=false` (forces `--headless-rendering`) |
| `pick_and_place.py` reports `PLAN INFEASIBLE` | A waypoint is outside the arm's reach annulus for the chosen `psi` | Run `--dry-run` to see which step and by how much; see `SPEC.md` §6.3 |
| Object slips out of the gripper during the demo | Friction-only grasp (no attach plugin) losing the contact | See `SPEC.md` §7 for the force budget and the documented (untested) `DetachableJoint` fallback |
| `gz sim --versions` doesn't print `8.x` | Wrong Gazebo version installed alongside ROS 2 Jazzy | Reinstall the Harmonic-matched `ros-jazzy-ros-gz*` packages; this package's plugin names are Harmonic-only |
