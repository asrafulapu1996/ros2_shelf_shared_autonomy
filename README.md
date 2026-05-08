# myCobot 280 Autonomous Medicine Pick-and-Place

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![License](https://img.shields.io/badge/License-BSD--3--Clause-green)

A ROS 2 simulation of an **Elephant Robotics myCobot 280** 6-DOF arm performing autonomous pick-and-place of medicine packets from a two-tier shelf. The robot detects medicines from a live Intel RealSense D435 RGBD point cloud, selects the closest target, and executes a collision-free grasp using MoveIt 2.

> **Stack:** ROS 2 Jazzy · Gazebo Harmonic · MoveIt 2 · Python 3.12 · Ubuntu 24.04

---

## Demo

> *Add a screenshot or screen-recording GIF here (e.g. `docs/demo.gif`) and replace this line with:*
> `![Demo](docs/demo.gif)`

---

## Prerequisites

| Tool | Version |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS 2 | [Jazzy](https://docs.ros.org/en/jazzy/Installation.html) |
| Gazebo | [Harmonic](https://gazebosim.org/docs/harmonic/install) |
| Python | 3.12 |

Make sure you have sourced your ROS 2 installation before proceeding:

```bash
source /opt/ros/jazzy/setup.bash
```

---

## Installation

### 1. Create a workspace and clone

```bash
mkdir -p ~/class_ws/src
cd ~/class_ws/src
git clone https://github.com/asrafulapu1996/ros2_shelf_shared_autonomy.git .
```

### 2. Install ROS 2 dependencies with rosdep

```bash
cd ~/class_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Install Python dependencies not covered by rosdep

```bash
pip3 install pymoveit2
```

> `pymoveit2` provides the Python MoveIt 2 interface used by the picker node and is not yet available through rosdep. All other Python dependencies (`numpy`, `opencv-python`, `scikit-learn`) are installed by rosdep in step 2.

### 4. Build the workspace

```bash
cd ~/class_ws
colcon build --symlink-install
```

### 5. Source the workspace

```bash
source ~/class_ws/install/setup.bash
```

Add to `~/.bashrc` to avoid repeating this step:

```bash
echo "source ~/class_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

Launch the full simulation stack with a single command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/class_ws/install/setup.bash
ros2 launch picker pick_and_place.launch.py
```

Wait approximately **35 seconds** for all nodes to come up. The startup sequence is:

| Time | What starts |
|---|---|
| 0 s | Gazebo + robot description + controllers + D435 camera |
| 5 s | Robot spawned in Gazebo |
| 12 s | Arm moves to home pose |
| 20 s | MoveIt 2 move_group + RViz |
| 25 s | Gamepad driver (`joy_node`) |
| 28 s | Medicine detector |
| 32 s | Picker node |

Launch without RViz:

```bash
ros2 launch picker pick_and_place.launch.py use_rviz:=false
```

---

## How to Use

1. Connect a gamepad (any device supported by the Linux `joy` driver)
2. Wait for the picker log: `Picker ready. Start medicine_detector then press Start`
3. Detected medicines appear as **green** (target) and **orange** (others) boxes in the RViz camera overlay at `/medicine_detection_image`
4. **Press Start (button 7)** to begin a pick-and-place cycle
5. The robot picks the closest medicine and carries it to the drop position above the table
6. The robot holds the medicine and logs: `Holding medicine at drop position. Open gripper manually then press Start to go home.`
7. Open the gripper manually (see below)
8. **Press Start again** — the arm returns home

### Manually open the gripper

```bash
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 50.0}}"
```

---

## Package Overview

```
src/
├── apps/
│   └── picker/                        # Pick-and-place application (Python)
│       ├── launch/
│       │   └── pick_and_place.launch.py   ← master launch (runs everything)
│       └── picker/
│           ├── medicine_detector.py       ← DBSCAN-based RGBD detector
│           ├── picker_node.py             ← motion controller
│           └── keyboard_selector.py       ← keyboard target selector
│
├── core/
│   └── mycobot_description/           # URDF/xacro robot model + meshes
│
├── planning/
│   └── mycobot_moveit_config/         # MoveIt 2 SRDF, kinematics, controllers
│
├── simulation/
│   └── mycobot_gazebo/                # Gazebo world, SDF models, ROS-GZ bridge
│
└── examples/
    └── mycobot_moveit_demos/          # Gamepad & keyboard teleop demos (C++)
```

---

## Configuration

All tunable parameters are at the top of [src/apps/picker/picker/picker_node.py](src/apps/picker/picker/picker_node.py):

| Parameter | Default | Description |
|---|---|---|
| `GRIPPER_REACH` | `0.120` m | Distance offset from `link6_flange` to stop before the medicine face. Increase to grasp earlier; decrease to reach deeper into the medicine |
| `STAGING_Y` | `0.10` m | Y distance in front of the shelf where the arm parks before/after Cartesian moves |
| `CLEAR_Y` | `-0.05` m | Y distance the arm retracts to before moving to the drop position, ensuring the shelf is unreachable |
| `TABLE_DROP_POS` | `[0.208, 0.091, 0.108]` | World-frame `link6_flange` XYZ at the table drop position |
| `CART_SPEED` | `0.04` m/s | Speed for all Cartesian straight-line moves |

### Updating the drop position

Move the arm to the desired position using the RViz **Motion Planning → Joints** panel, then read the live TF:

```bash
ros2 run tf2_ros tf2_echo world link6_flange
```

Copy the `Translation: [x, y, z]` into `TABLE_DROP_POS` and rebuild:

```bash
cd ~/class_ws
colcon build --packages-select picker --symlink-install
```

### Changing the gamepad button

```bash
# Find your button index
ros2 topic echo /joy
```

Then pass the index at launch:

```bash
ros2 run picker picker_node --ros-args -p grasp_button_index:=9
```

---

## Key ROS 2 Interfaces

| Topic / Action / Service | Type | Description |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | Gamepad input |
| `/target_medicine_pose` | `geometry_msgs/PoseStamped` | Closest detected medicine |
| `/detected_medicines_markers` | `visualization_msgs/MarkerArray` | RViz bounding-box overlays |
| `/medicine_detection_image` | `sensor_msgs/Image` | Annotated colour image |
| `/camera_head/depth/color/points` | `sensor_msgs/PointCloud2` | Input point cloud |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Cartesian path execution |
| `/gripper_action_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Gripper open/close |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | Straight-line path planning |

---

## Troubleshooting

**Planning fails / arm freezes**
MoveIt is configured for 5 s planning time and 10 attempts per call. Wait for the `move_group` log `"Ready to take commands"` before pressing Start.

**No medicines detected**
Check the point cloud is arriving:
```bash
ros2 topic hz /camera_head/depth/color/points
```
The camera is only active when `use_camera:=true` (default in the master launch).

**Arm collides with shelf during drop move**
Decrease `CLEAR_Y` (e.g. `-0.10`) so the arm pulls further back before the joint-space move to the drop position.

---

## License

BSD-3-Clause — see [LICENSE](LICENSE) for details.
