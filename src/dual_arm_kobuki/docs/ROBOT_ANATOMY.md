# Robot Anatomy: Every Link, Every Joint

**What this is:** a physical inventory of the robot as it's actually built in
`urdf/` — every link, every joint, every sensor, with its real dimensions,
mass, and where it sits. No kinematics math, no demo logic — just "what is
this robot made of." Pulled directly from `common.xacro`, `base.xacro`,
`arm.xacro`, `sensors.xacro`, and `robot.urdf.xacro`, cross-checked against
each other, not copied from any summary.

For an illustrated version with scale diagrams, see the published artifact
linked from the project conversation. For *why* the numbers are what they
are (the Pythagoras behind the shoulder offset, the derivation of L3, etc.),
see `BEGINNERS_GUIDE.md`. For the robotics concepts underneath all of this,
see `CONCEPTS.md` or `ROBOTICS_PRIMER.md`.

---

## 1. The whole robot, top to bottom

```
                              ● camera_link      (0.46 m AGL, pitched 23° down)
                              │
                       ┌──────┴──────┐
                       │  mast_head  │            (crossbar, 0.45 m AGL)
                       └──┬───────┬──┘
                    mast_left  mast_right         (two posts, r=14cm from centre)
                          │       │
                     ═════╪═══════╪═════          top_deck  (0.1775 m radius, 14 cm AGL)
                    ╱      │       │      ╲
              left_mount   │       │   right_mount     (arm shoulders, ±8.875cm, 9.5cm fwd)
                    │      │       │        │
                 [J1..J4 chains — section 3]
                    │      │       │        │
                    ═══════╪═══════╪══════════       base_link (Ø35.5 cm cylinder, 9 cm tall)
                           │
                     base_footprint                  (ground level, z = 0)
```

Everything hangs off `base_footprint`, a massless frame sitting exactly on
the ground. Every other link's position ultimately traces back to it.

---

## 2. Base assembly

| Link | Shape | Size | Mass | Colour | Parent joint |
|---|---|---|---|---|---|
| `base_footprint` | none (frame only) | — | 0 kg | — | *(root)* |
| `base_link` | cylinder | ⌀0.355 m × 0.09 m | 2.5 kg | `kobuki_dark` (0.15, 0.16, 0.18) | fixed, 1.5 cm off the ground |
| `top_deck` | cylinder | ⌀0.355 m × 0.010 m | 0.30 kg | `deck_grey` (0.55, 0.57, 0.60) | fixed, top surface at 0.14 m AGL |
| `front_marker` | box | 0.02 × 0.06 × 0.008 m | 0.01 kg | `sensor_red` (0.80, 0.20, 0.20) | fixed, on the +X rim of `base_link` |

`base_link` is a plain cylinder standing in for the real Kobuki's rounded
hex body — modelled with primitives only, no mesh files. `front_marker` is
a flat red tab bolted to the front edge purely so the robot's heading is
obvious at a glance in the viewer; it carries no sensor and does nothing
functionally.

The three fixed joints in this group:

| Joint | Parent → Child | Offset |
|---|---|---|
| `base_footprint_to_base_link` | `base_footprint` → `base_link` | z = ground_clearance + base_height/2 = 0.015 + 0.045 = **0.060 m** |
| `base_to_front_marker` | `base_link` → `front_marker` | x = base_radius − 0.01 = **0.1675 m**, z = 0.04 m |
| `base_to_deck` | `base_link` → `top_deck` | z chosen so the deck's *top surface* lands exactly at 0.14 m AGL |

---

## 3. Arm assembly (×2, mirrored)

Each arm is one macro (`four_dof_arm`), instantiated twice — `prefix="left"
reflect="1"` and `prefix="right" reflect="-1"`. The `reflect` flag flips the
sideways mounting offset only; the joint chain itself is identical on both
sides.

### 3.1 The chain

```
top_deck
   │  (fixed, at ±8.875cm lateral, 9.5cm forward, on the deck rim)
   ▼
 mount          — a small grey pad, ⌀0.064 × 0.010 m, 0.05 kg
   │  (fixed, 1 cm up)
   ▼
 J1 (yaw, Z axis) ─── ±90°, 5 N·m, 2 rad/s
   │
   ▼
 riser          — teal post, ⌀0.040 × 0.040 m, 0.15 kg  [+ amber yaw housing]
   │  (fixed, 4 cm up = riser's own length)
   ▼
 J2 (pitch, Y axis) ─── ±90°, 5 N·m, 2 rad/s
   │
   ▼
 upper_arm      — teal rod, ⌀0.036 × 0.160 m, 0.20 kg  [+ amber pitch housing]
   │  (fixed, 16 cm up = upper_arm's own length)
   ▼
 J3 (pitch, Y axis) ─── ±135°, 5 N·m, 2 rad/s
   │
   ▼
 forearm        — teal rod, ⌀0.036 × 0.160 m, 0.18 kg  [+ amber pitch housing]
   │  (fixed, 16 cm up = forearm's own length)
   ▼
 J4 (pitch, Y axis) ─── ±114.6°, 5 N·m, 2 rad/s   ← this IS the wrist
   │
   ▼
 wrist          — teal stub, ⌀0.040 × 0.030 m, 0.06 kg  [+ amber pitch housing]
   │  (fixed, 3 cm up = wrist's own length)
   ▼
 palm           — dark box, 0.050 × 0.075 × 0.030 m, 0.08 kg
   │  (fixed, 3 cm up = palm's own length)
   ├──► finger_a_link ─┐
   └──► finger_b_link ─┤  (both prismatic, mirrored — section 3.4)
                        │
   (fixed, 6.3 cm up)   ▼
                    grasp_frame   — no geometry, mass 0 — the IK target
```

**Every revolute joint carries a small amber cylinder** lying along its own
rotation axis (visual only, ⌀0.052 m, 0.022–0.044 m long) purely so the
articulation reads as distinct joints in the viewer rather than one fused
rod. These housings have no collision geometry and don't affect the
kinematics or dynamics — they're a legibility aid.

### 3.2 Where the arm mounts

The `mount` link sits on the deck at:

```
x = +0.095 m   (forward of centre)
y = ±0.08875 m (left arm +, right arm −)
z = deck top (0.14 m AGL) + half the deck thickness
```

giving a straight-line distance of **0.130 m** from the robot's centreline
to each shoulder, and **0.1775 m** between the two shoulders.

### 3.3 Joint-by-joint reference

| Joint | Type | Axis | Lower | Upper | Effort | Velocity | Damping | Friction |
|---|---|---|---|---|---|---|---|---|
| `{side}_j1` | revolute | Z (yaw) | −1.5708 rad (−90°) | +1.5708 rad (+90°) | 5.0 N·m | 2.0 rad/s | 0.15 | 0.05 |
| `{side}_j2` | revolute | Y (pitch) | −1.5708 rad (−90°) | +1.5708 rad (+90°) | 5.0 N·m | 2.0 rad/s | 0.15 | 0.05 |
| `{side}_j3` | revolute | Y (pitch) | −2.3562 rad (−135°) | +2.3562 rad (+135°) | 5.0 N·m | 2.0 rad/s | 0.15 | 0.05 |
| `{side}_j4` | revolute | Y (pitch) | −2.0000 rad (−114.6°) | +2.0000 rad (+114.6°) | 5.0 N·m | 2.0 rad/s | 0.15 | 0.05 |

`{side}` is `left` or `right`. J2, J3, and J4 all rotate about the *same*
axis direction (Y, in each link's own local frame) — that's what makes the
arm a planar 3-joint mechanism once J1 has picked a heading, and it's the
single fact that shapes the whole kinematics story (see `CONCEPTS.md` §4).

### 3.4 The gripper

Two prismatic finger joints, mounted symmetrically on the palm:

| Joint | Type | Axis | Range | Effort | Velocity | Damping | Friction |
|---|---|---|---|---|---|---|---|
| `{side}_finger_a` | prismatic | +Y | 0 → 0.035 m | 25 N | 0.10 m/s | 2.0 | 0.5 |
| `{side}_finger_b` | prismatic | −Y | 0 → 0.035 m | 25 N | 0.10 m/s | 2.0 | 0.5 |

Both joints read **0 at fully closed**; commanding the *same* value to both
opens them symmetrically. Maximum opening = 2 × 0.035 m = **7.0 cm**.

Each finger link is a small box, 0.028 × 0.012 × 0.055 m, 0.025 kg, dark
grey (`finger_dark`), with a second, purely-visual sliver — 0.85× the
finger's width, 3 mm thick, running the inner 55% of its length — painted
green (`pad_green`) to mark the contact pad. In Gazebo, the finger link's
*collision* geometry (the full box) carries `mu1 = mu2 = 1.6`,
`kp = 1,000,000`, `kd = 100` — high friction, stiff, lightly damped contact.
That surface property is what actually grips; the green paint is only a
visual cue for where.

The **grasp frame** — a massless, geometry-less link — sits 6.3 cm above the
palm's face (`palm_len + 0.6 × finger_len` = 0.030 + 0.6×0.055 = 0.063 m),
roughly between the finger pads. It's the point every IK call actually
targets, and the point `ros2 topic echo /tf` will show moving as the arm
articulates even though nothing is physically there.

---

## 4. Sensor mast assembly

One mast, built once (not mirrored), straddling the deck's centreline
slightly behind the arms:

```
                    ┌── camera_link
                    │     (0.02×0.05×0.02 m box, red, pitched 22.9° down)
                    │
              ┌─────┴─────┐
   mast_left ─┤ mast_head ├─ mast_right
   (post)     │(0.03×0.24×│   (post)
              │ 0.025 box)│
              └─────┬─────┘
                    │
              lidar_bracket
              (0.016×0.016×0.13 m drop bracket)
                    │
               lidar_link
          (⌀0.033×0.035 m cylinder, red)
```

| Link | Shape | Size | Mass | Colour |
|---|---|---|---|---|
| `mast_left` | cylinder | ⌀0.020 × 0.31 m | 0.10 kg | `deck_grey` |
| `mast_right` | cylinder | ⌀0.020 × 0.31 m | 0.10 kg | `deck_grey` |
| `mast_head` | box | 0.03 × 0.2425 × 0.025 m | 0.10 kg | `deck_grey` |
| `camera_link` | box | 0.02 × 0.05 × 0.02 m | 0.03 kg | `sensor_red` |
| `lidar_bracket` | box | 0.016 × 0.016 × 0.13 m | 0.02 kg | `deck_grey` |
| `lidar_link` | cylinder | ⌀0.066 × 0.035 m | 0.12 kg | `sensor_red` |

**Post placement:** each post sits 14 cm from the robot's centreline, 7 cm
*behind* it — `mast_offset_x = −0.07`, `mast_offset_y = ±0.1212`
(`= √(0.14² − 0.07²)`, chosen so the straight-line radius comes out to
exactly 0.14 m). The crossbar (`mast_head`) spans between the two posts'
tops, its centre landing at **0.45 m AGL** (`deck_top_z + mast_height` =
0.14 + 0.31).

**Sensor placement is height-driven, not offset-driven:** the xacro
specifies *camera 0.46 m AGL* and *LiDAR 0.32 m AGL* directly, and derives
the joint offsets from the crossbar height:

```
camera_z_off = 0.46 − 0.45 = +0.01 m   (1 cm above the crossbar)
lidar_z_off  = 0.32 − 0.45 = −0.13 m   (13 cm below the crossbar)
```

The camera is additionally pitched **0.40 rad (≈ 22.9°, documented as 23°)
downward** about its mounting joint's Y axis, and offset 2.5 cm forward of
the crossbar.

### 4.1 Camera (`rgb_camera`, on `camera_link`)

| Property | Value |
|---|---|
| Topic | `camera/image_raw` (bridged to `/camera/image_raw`) |
| Resolution | 640 × 480, `R8G8B8` |
| Horizontal FOV | 1.204 rad (≈ 69°) |
| Clip range | 0.05 m – 12.0 m |
| Update rate | 15 Hz |
| Mount | 0.46 m AGL, pitched 23° down, 2.5 cm forward of the crossbar |

### 4.2 2D LiDAR (`lidar_2d`, on `lidar_link`)

| Property | Value |
|---|---|
| Topic | `scan` (bridged to `/scan`) |
| Type | `gpu_lidar`, single horizontal plane |
| Samples | 360 (1° resolution), full 360° (−π to +π) |
| Range | 0.12 m – 8.0 m, 0.01 m resolution |
| Update rate | 10 Hz |
| Mount | 0.32 m AGL, level (no tilt) |

---

## 5. Named poses

`nodes/pose_cmd.py` defines four named arm configurations (same four
joint angles sent to *both* arms) plus two gripper states:

| Pose | j1 | j2 | j3 | j4 | Description |
|---|---|---|---|---|---|
| `zero` | 0.0 | 0.0 | 0.0 | 0.0 | All joints at zero — arms point straight up, read as one fused rod |
| `home` | 0.0 | −0.55 | 1.35 | 0.50 | Folded back and up — the parking pose after spawn |
| `reach` | 0.0 | 0.70 | 0.55 | 0.30 | Forward and down, toward the starting base |
| `pinch` | 0.0 | 0.85 | 0.45 | 0.25 | Yawed outward, closing on a large object from both sides |

| Gripper state | Per-finger value | Total opening |
|---|---|---|
| `open` | 0.035 m | 7.0 cm (mechanical maximum) |
| `close` | 0.0 m | 0 cm (fully closed) |

`pose_cmd.py` is explicitly a manual jogging tool, not a controller — it
publishes one step setpoint and lets Gazebo's per-joint PID (§6) do the
rest, with no interpolation.

---

## 6. Actuation: how every joint is driven

Every one of the 12 moving joints (4 per arm × 2 arms, plus 2 fingers per
arm × 2 arms) is driven by its own instance of
`gz-sim-joint-position-controller-system`, listening on its own topic:

| Joint | ROS topic | P | I | D | I-clamp | Effort limit |
|---|---|---|---|---|---|---|
| `{side}_j1` | `/arm/{side}_j1/cmd_pos` | 6.0 | 0.05 | 0.6 | ±1.0 | 5 N·m |
| `{side}_j2` | `/arm/{side}_j2/cmd_pos` | 9.0 | 0.10 | 0.9 | ±1.0 | 5 N·m |
| `{side}_j3` | `/arm/{side}_j3/cmd_pos` | 7.0 | 0.08 | 0.7 | ±1.0 | 5 N·m |
| `{side}_j4` | `/arm/{side}_j4/cmd_pos` | 4.0 | 0.05 | 0.4 | ±1.0 | 5 N·m |
| `{side}_finger_a` | `/arm/{side}_finger_a/cmd_pos` | 600.0 | 8.0 | 20.0 | ±12.0 | 25 N |
| `{side}_finger_b` | `/arm/{side}_finger_b/cmd_pos` | 600.0 | 8.0 | 20.0 | ±12.0 | 25 N |

Every message is a single `std_msgs/msg/Float64` — one number, the target
angle (radians) or extension (metres). There is no `ros2_control`, no
trajectory controller, and no coordination between joints at this layer:
twelve independent PID loops, twelve independent topics.

Additionally, `gz-sim-joint-state-publisher-system` publishes the measured
state of every joint to the `joint_states` topic (bridged to
`/joint_states`), which `robot_state_publisher` consumes to compute and
broadcast the TF tree (§8).

---

## 7. Materials

Seven named materials, defined once in `common.xacro` and reused by name
everywhere:

| Material | RGBA | Used on |
|---|---|---|
| `kobuki_dark` | 0.15, 0.16, 0.18 | `base_link` |
| `deck_grey` | 0.55, 0.57, 0.60 | `top_deck`, mount pads, mast posts/head/bracket |
| `arm_teal` | 0.10, 0.55, 0.60 | riser, upper arm, forearm, wrist (the structural link segments) |
| `joint_amber` | 0.90, 0.60, 0.15 | the visual-only housing cylinder at every revolute joint |
| `finger_dark` | 0.22, 0.24, 0.26 | palm, finger bodies |
| `pad_green` | 0.20, 0.65, 0.35 | the visual contact-pad sliver on each finger |
| `sensor_red` | 0.80, 0.20, 0.20 | front marker, camera housing, LiDAR housing |

The colour coding is legible at a glance: **teal is structure, amber is a
joint, red is a sensor, grey is the base/mast/mounting hardware.**

---

## 8. The TF frame tree

Every link above gets its own coordinate frame, published by
`robot_state_publisher`. Counting them directly from the source (not from
any summary):

```
base_footprint                                    (1  — root)
└── base_link                                      (2)
    ├── front_marker                               (3)
    └── top_deck                                   (4)
        ├── mast_left                              (5)
        │   └── mast_head                          (7, via mast_left)
        │       ├── camera_link                    (8)
        │       ├── lidar_bracket                  (9)
        │       └── lidar_link                     (10)
        ├── mast_right                             (6)
        ├── left_mount                             (11)
        │   └── left_riser              [left_j1]  (12)
        │       └── left_upper_arm      [left_j2]  (13)
        │           └── left_forearm    [left_j3]  (14)
        │               └── left_wrist  [left_j4]  (15)
        │                   └── left_palm          (16)
        │                       ├── left_finger_a_link         (17)
        │                       ├── left_finger_b_link         (18)
        │                       └── left_grasp_frame           (19)
        └── right_mount                            (20)
            └── right_riser             [right_j1] (21)
                └── right_upper_arm     [right_j2] (22)
                    └── right_forearm   [right_j3] (23)
                        └── right_wrist [right_j4] (24)
                            └── right_palm         (25)
                                ├── right_finger_a_link        (26)
                                ├── right_finger_b_link        (27)
                                └── right_grasp_frame          (28)
```

**28 frames total**, rooted at `base_footprint`, ending in
`left_grasp_frame` / `right_grasp_frame` at the tips — connected by 27
joints (3 in the base group + 6 in the mast/sensor group + 9 per arm × 2 =
27), broken down by type as 15 fixed + 8 revolute + 4 prismatic.

> **Worth flagging:** `README.md` and `CLAUDE.md` both state "21 frames" as
> the expected `view_frames` output. Counting directly from the current
> `base.xacro` / `arm.xacro` / `sensors.xacro` (all committed, unmodified in
> the working tree) gives **28**, not 21 — every link in the tree above is
> a real, distinct link declaration in the source. If you run
> `ros2 run tf2_tools view_frames` and get a different number than either,
> trust the tool's output over both this document and the older one; if you
> get 28, the "21" in the existing docs is simply stale and worth fixing at
> the source.

---

## 9. Mass and size summary

| Group | Links | Combined mass |
|---|---|---|
| Base (`base_link` + `top_deck` + `front_marker`) | 3 | 2.81 kg |
| Sensor mast (both posts + head + camera + bracket + LiDAR) | 6 | 0.47 kg |
| One arm (mount → riser → upper_arm → forearm → wrist → palm → 2 fingers) | 8 (+1 massless grasp frame) | 0.77 kg |
| **Both arms** | 16 (+2 massless) | **1.54 kg** |
| **Whole robot** | 28 (incl. `base_footprint`) | **≈ 4.82 kg** |

| Overall dimension | Value |
|---|---|
| Footprint (base diameter) | 0.355 m |
| Height, base to deck top | 0.14 m |
| Height, base to mast crossbar | 0.45 m |
| Height, base to camera | 0.46 m |
| Per-arm reach envelope (fully extended, J2+J3 straight) | 0.32 m |
| Shoulder-to-shoulder span | 0.1775 m |
| Gripper max opening | 0.07 m |
| Moving DOF | 12 (8 revolute + 4 prismatic) |
| Total links / frames | 28 |
| Total joints | 27 = 15 fixed + 8 revolute + 4 prismatic |
