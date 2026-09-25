# Prius ADAS — ROS 2 + Gazebo Classic

A ROS 2 Humble simulation project for a Prius-style vehicle implementing:

- Autonomous camera-based lane following
- SLAM-based localization/navigation
- Nav2 autonomous path planning
- Vehicle path following
- Dynamic obstacle detection
- Nav2 replanning and alternate-path following
- Collision monitoring
- ROS 2 integration with the Prius vehicle control interface

The project is developed and tested in simulation using Gazebo Classic.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Main Features](#main-features)
- [System Architecture](#system-architecture)
- [Operating Modes](#operating-modes)
  - [Part 1 — Autonomous Lane Following](#part-1--autonomous-lane-following)
  - [Part 2 — SLAM + Nav2 ADAS](#part-2--slam--nav2-adas)
- [Dynamic Obstacle Avoidance](#dynamic-obstacle-avoidance)
- [ROS 2 Packages](#ros-2-packages)
- [Main Nodes](#main-nodes)
- [Main Launch Files](#main-launch-files)
- [Configuration Files](#configuration-files)
- [Topics and Data Flow](#topics-and-data-flow)
- [Software Environment](#software-environment)
- [Workspace Structure](#workspace-structure)
- [Installation and Build](#installation-and-build)
- [How to Run](#how-to-run)
- [Useful Verification Commands](#useful-verification-commands)
- [Tools](#tools)
- [Documentation](#documentation)
- [Project Verification](#project-verification)
- [Known Development History](#known-development-history)
- [Screenshots and Demonstration Media](#screenshots-and-demonstration-media)
- [Dataset](#dataset)
- [References](#references)
- [Future Work](#future-work)
- [Author](#author)

---

## Project Overview

This project simulates an autonomous Prius-style vehicle in ROS 2 and
Gazebo Classic.

The project contains two distinct operating modes.

### Part 1 — Autonomous Lane Following

The first mode is a camera-based autonomous driving system.

The Prius camera provides image data to the lane detection node.
The detected lane information is then used by the autonomous driver
to generate vehicle control commands.

```text
Camera
   |
   v
lane_detection
   |
   v
Lane Information
   |
   v
autonomous_driver
   |
   v
/prius/control
   |
   v
Prius
```

### Part 2 — SLAM + Nav2 ADAS

The second mode uses LiDAR, SLAM, Nav2, costmaps, collision monitoring,
and a Nav2-to-Prius command adapter.

```text
Gazebo / Prius
      |
      +------------------+
      |                  |
      v                  v
    LiDAR             Odometry / TF
      |                  |
      v                  |
 /prius/scan             |
      |                  |
      +--------+---------+
               |
               v
          SLAM Toolbox
               |
               v
              Map
               |
               v
              Nav2
       +-------+-------+----------------+
       |               |                |
       v               v                v
 Smac Hybrid-A*   Global Costmap   Local Costmap
       |                                  |
       +---------------+------------------+
                       |
                       v
          Regulated Pure Pursuit
                       |
                       v
                Velocity Command
                       |
                       v
               Collision Monitor
                       |
                       v
               nav2_cmd_to_prius
                       |
                       v
                 /prius/control
                       |
                       v
                     Prius
```

---

## Main Features

### Autonomous Lane Following

The vehicle can autonomously follow the road using camera-based lane
detection.

### SLAM

The project includes SLAM Toolbox integration for navigation and
localization-related operation.

### Nav2 Navigation

The project uses Nav2 for autonomous path planning and path following.

The configured navigation stack includes:

- Smac Hybrid-A* planner
- Regulated Pure Pursuit controller
- Global costmap
- Local costmap
- Behavior Tree Navigator
- Collision Monitor

### Vehicle Command Conversion

Nav2 velocity commands are converted into the control interface
required by the Prius simulation.

### Dynamic Obstacle Avoidance

When an obstacle blocks the current route:

```text
Obstacle
   |
   v
LiDAR
   |
   v
Costmap Update
   |
   v
Nav2 Replanning
   |
   v
Alternate Path
   |
   v
Controller
   |
   v
Collision Monitor
   |
   v
nav2_cmd_to_prius
   |
   v
/prius/control
   |
   v
Prius
```

The Prius can follow the newly generated navigation path around the
obstacle.

---

## System Architecture

The project can be viewed as two related control systems.

```text
                       PRIUS ADAS
                           |
              +------------+------------+
              |                         |
              v                         v
      Autonomous Driving         Navigation / ADAS
              |                         |
              v                         v
       Lane Detection                 LiDAR
              |                         |
              v                         v
     Autonomous Driver              SLAM
              |                         |
              |                         v
              |                        Map
              |                         |
              |                         v
              |                        Nav2
              |                  +------+------+------+
              |                  |      |      |      |
              |                  v      v      v      v
              |                Planner Controller Costmaps BT
              |                  |      |
              |                  +------+
              |                         |
              +------------+------------+
                           |
                           v
                    Prius Control
                           |
                           v
                    /prius/control
                           |
                           v
                         Prius
```

---

## Operating Modes

### Part 1 — Autonomous Lane Following

Part 1 uses exactly three terminals:

```text
Terminal 1
Gazebo + Prius

Terminal 2
lane_detection

Terminal 3
autonomous_driver
```

#### Data Flow

```text
Prius Camera
     |
     v
lane_detection
     |
     v
Lane Information
     |
     v
autonomous_driver
     |
     v
/prius/control
     |
     v
Prius
```

There is no separate autonomous-controller launch file in the final
project. The lane detection and autonomous driver nodes are started
directly using `ros2 run`.

---

### Part 2 — SLAM + Nav2 ADAS

Part 2 uses four terminals:

```text
Terminal 1
Gazebo + Prius

Terminal 2
SLAM

Terminal 3
Nav2

Terminal 4
RViz2
```

#### Data Flow

```text
Prius Sensors
     |
     v
LiDAR
     |
     v
SLAM Toolbox
     |
     v
Map / Localization
     |
     v
Nav2
     |
     +----------------------+
     |                      |
     v                      v
Global Planning        Local Control
     |                      |
     v                      v
Smac Hybrid-A*       Regulated Pure Pursuit
     |                      |
     +----------+-----------+
                |
                v
          Costmaps / BT
                |
                v
       Collision Monitor
                |
                v
       nav2_cmd_to_prius
                |
                v
          /prius/control
                |
                v
              Prius
```

RViz2 is used for visualization, navigation goals, maps, paths, and
navigation state.

---

## Dynamic Obstacle Avoidance

The ADAS navigation system was tested by placing an obstacle in the
vehicle's planned route.

The intended navigation process is:

```text
1. Prius follows the original Nav2 path
2. Obstacle appears in the route
3. LiDAR observes the obstacle
4. Costmap is updated
5. Nav2 replans
6. A new path is generated
7. Prius follows the alternate path
```

High-level flow:

```text
Obstacle
   |
   v
LiDAR
   |
   v
/prius/scan
   |
   v
Costmap
   |
   v
Nav2 Planner
   |
   v
New / Alternate Path
   |
   v
Controller
   |
   v
Collision Monitor
   |
   v
nav2_cmd_to_prius
   |
   v
/prius/control
   |
   v
Prius
```

---

## ROS 2 Packages

The workspace contains two ROS 2 packages used by the project.

### 1. `prius_adas_control`

Main project package containing:

- Lane following
- Nav2 launch files
- SLAM launch file
- Vehicle launch environment
- Navigation configuration
- Collision monitoring configuration
- RViz configuration
- Vehicle/world assets

### 2. `prius_adas_navigation`

Supporting navigation package located at:

```text
src/prius_adas_navigation/
```

It contains supporting navigation nodes used by the Prius simulation.

Main executables include:

```text
prius_odom_bridge
laser_scan_merger
```

The Prius world launch file uses the `prius_odom_bridge` executable.

---

## Main Nodes

### `lane_detection`

Processes the Prius camera image and extracts lane information for the
autonomous driving system.

Run with:

```bash
ros2 run prius_adas_control lane_detection
```

### `autonomous_driver`

Uses the lane information to determine vehicle motion and steering
for autonomous lane following.

Run with:

```bash
ros2 run prius_adas_control autonomous_driver
```

### `nav2_cmd_to_prius`

Acts as the command adapter between Nav2 and the Prius vehicle
control interface.

Conceptually:

```text
Nav2 /cmd_vel
      |
      v
nav2_cmd_to_prius
      |
      v
/prius/control
      |
      v
Prius
```

Run with:

```bash
ros2 run prius_adas_control nav2_cmd_to_prius
```

### `prius_odom_bridge`

Supporting navigation node from:

```text
src/prius_adas_navigation/
```

Provides the odometry bridge required by the Prius navigation setup.

### `laser_scan_merger`

Supporting navigation node from:

```text
src/prius_adas_navigation/
```

Provides laser scan processing/merging functionality for the navigation
system.

---

## Main Launch Files

The final project contains three launch files:

```text
prius_adas_control/launch/

├── nav2.launch.py
├── prius_adas_world.launch.py
└── slam.launch.py
```

### `prius_adas_world.launch.py`

Starts the Gazebo Prius simulation environment.

```bash
ros2 launch prius_adas_control prius_adas_world.launch.py
```

Available launch arguments:

```text
gui
server
```

### `slam.launch.py`

Starts the SLAM/localization-related navigation components.

```bash
ros2 launch prius_adas_control slam.launch.py
```

Available arguments:

```text
use_sim_time
slam_params_file
```

### `nav2.launch.py`

Starts the Nav2 navigation system.

```bash
ros2 launch prius_adas_control nav2.launch.py
```

---

## Configuration Files

The main configuration directory is:

```text
prius_adas_control/config/
```

Current configuration files:

```text
ackermann_nav_to_pose.xml
collision_monitor.yaml
nav2_params.yaml
prius_figure8_nav2.rviz
slam_localization.yaml
slam_toolbox.yaml
```

### `nav2_params.yaml`

Contains Nav2 parameters for:

- Planner
- Controller
- Costmaps
- Behavior Tree Navigator
- Waypoint follower
- Navigation behavior

The project uses:

```text
Smac Hybrid-A*
```

for global planning and:

```text
Regulated Pure Pursuit
```

for path following.

### `collision_monitor.yaml`

Contains Collision Monitor configuration.

The project defines safety regions including:

```text
SlowZone
StopZone
```

### `slam_localization.yaml`

Contains SLAM/localization parameters used by the project.

### `slam_toolbox.yaml`

Contains additional SLAM Toolbox configuration retained as part of the
project configuration.

### `prius_figure8_nav2.rviz`

RViz2 configuration for visualizing the navigation system.

### `ackermann_nav_to_pose.xml`

Navigation behavior-tree configuration retained in the project's
configuration directory.

---

## Topics and Data Flow

The project uses ROS 2 topics to connect the main subsystems.

### Vehicle Control

The Prius control interface is:

```text
/prius/control
```

Both the lane-following control system and the Nav2 command adapter
ultimately use the Prius control interface.

### Navigation Velocity Flow

The high-level navigation command flow is:

```text
Nav2
  |
  v
/cmd_vel or navigation velocity chain
  |
  v
Collision Monitor
  |
  v
/cmd_vel
  |
  v
nav2_cmd_to_prius
  |
  v
/prius/control
  |
  v
Prius
```

### LiDAR

The navigation system uses the Prius LiDAR scan topic:

```text
/prius/scan
```

The LiDAR information is used by the navigation and collision-monitor
systems.

### Odometry

The navigation setup includes the supporting:

```text
prius_odom_bridge
```

node in the `prius_adas_navigation` package.

---

## Software Environment

| Component | Environment |
|---|---|
| Operating System | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Simulator | Gazebo Classic |
| Visualization | RViz2 |
| Programming | Python |
| Vision | OpenCV |
| Navigation | Nav2 |
| Localization / Mapping | SLAM Toolbox |

---

## Workspace Structure

```text
prius_adas_ws/
│
├── README.md
├── .gitignore
│
├── docs/
│   ├── architecture.md
│   ├── findings.md
│   ├── how-to-run.md
│   ├── issue-log.md
│   ├── reproduction.md
│   └── troubleshooting.md
│
├── prius_adas_control/
│   ├── config/
│   │   ├── ackermann_nav_to_pose.xml
│   │   ├── collision_monitor.yaml
│   │   ├── nav2_params.yaml
│   │   ├── prius_figure8_nav2.rviz
│   │   ├── slam_localization.yaml
│   │   └── slam_toolbox.yaml
│   │
│   ├── launch/
│   │   ├── nav2.launch.py
│   │   ├── prius_adas_world.launch.py
│   │   └── slam.launch.py
│   │
│   ├── models/
│   ├── prius_adas_control/
│   │   ├── __init__.py
│   │   ├── autonomous_driver.py
│   │   ├── lane_detection.py
│   │   └── nav2_cmd_to_prius.py
│   │
│   ├── resource/
│   ├── test/
│   ├── urdf/
│   ├── worlds/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── src/
│   └── prius_adas_navigation/
│       ├── prius_adas_navigation/
│       │   ├── __init__.py
│       │   ├── laser_scan_merger.py
│       │   └── prius_odom_bridge.py
│       │
│       ├── resource/
│       ├── test/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
│
└── tools/
    ├── add_slam_barriers.py
    └── clean_and_add_lidar.py
```

---

## Installation and Build

### 1. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 2. Source the Prius workspace

```bash
source ~/prius_ws/install/setup.bash
```

### 3. Build the Prius ADAS workspace

```bash
cd ~/prius_adas_ws

colcon build --symlink-install
```

### 4. Source the workspace

```bash
source ~/prius_adas_ws/install/setup.bash
```

---

## How to Run

### Part 1 — Autonomous Lane Following

Use exactly three terminals.

#### Terminal 1 — Gazebo + Prius

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 launch prius_adas_control prius_adas_world.launch.py
```

#### Terminal 2 — Lane Detection

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 run prius_adas_control lane_detection
```

#### Terminal 3 — Autonomous Driver

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 run prius_adas_control autonomous_driver
```

The vehicle should then follow the lane autonomously.

---

### Part 2 — SLAM + Nav2 ADAS

Use four terminals.

#### Terminal 1 — Gazebo + Prius

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 launch prius_adas_control prius_adas_world.launch.py
```

#### Terminal 2 — SLAM

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 launch prius_adas_control slam.launch.py
```

#### Terminal 3 — Nav2

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 launch prius_adas_control nav2.launch.py
```

#### Terminal 4 — RViz2

```bash
source /opt/ros/humble/setup.bash
source ~/prius_adas_ws/install/setup.bash

rviz2
```

Use RViz2 to visualize the map, robot pose, costmaps, navigation path,
and send navigation goals.

---

### Dynamic Obstacle Test

With Part 2 running:

1. Send a navigation goal in RViz2.
2. Allow the Prius to start following the generated path.
3. Introduce an obstacle into the current route in Gazebo.
4. Observe the LiDAR/costmap response.
5. Nav2 replans the route.
6. The Prius follows the alternate path.

Expected flow:

```text
Original Path
      |
      v
Obstacle Appears
      |
      v
LiDAR Detection
      |
      v
Costmap Update
      |
      v
Nav2 Replanning
      |
      v
Alternate Path
      |
      v
Prius
```

---

## Useful Verification Commands

### Check `prius_adas_control` executables

```bash
ros2 pkg executables prius_adas_control
```

Expected:

```text
prius_adas_control autonomous_driver
prius_adas_control lane_detection
prius_adas_control nav2_cmd_to_prius
```

### Check `prius_adas_navigation` executables

```bash
ros2 pkg executables prius_adas_navigation
```

Expected supporting executables include:

```text
prius_adas_navigation laser_scan_merger
prius_adas_navigation prius_odom_bridge
```

### Check world launch arguments

```bash
ros2 launch prius_adas_control prius_adas_world.launch.py --show-args
```

### Check SLAM launch arguments

```bash
ros2 launch prius_adas_control slam.launch.py --show-args
```

### Check Nav2 launch arguments

```bash
ros2 launch prius_adas_control nav2.launch.py --show-args
```

### Check Git status

```bash
cd ~/prius_adas_ws
git status
```

---

## Tools

The repository contains two project utility scripts.

### `add_slam_barriers.py`

Used for generating/adding barrier structures to the SLAM-oriented
Gazebo world.

Location:

```text
tools/add_slam_barriers.py
```

### `clean_and_add_lidar.py`

Used for LiDAR-related modification/cleanup of the Prius simulation
URDF.

Location:

```text
tools/clean_and_add_lidar.py
```

These scripts are development utilities and are kept separately from
the main ROS 2 packages.

---

## Documentation

Additional documentation is located in:

```text
docs/
```

| File | Purpose |
|---|---|
| `architecture.md` | Detailed ROS architecture and node relationships |
| `how-to-run.md` | Practical build and run commands |
| `troubleshooting.md` | Troubleshooting information |
| `findings.md` | Project findings and development observations |
| `issue-log.md` | Development issue history |
| `reproduction.md` | Reproduction information |

The README focuses on the final project architecture and usage.
Development problems and debugging history are documented separately.

---

## Project Verification

The following project functionality has been exercised in the
Gazebo simulation:

```text
✓ prius_adas_control package builds
✓ prius_adas_navigation package builds
✓ Gazebo Prius simulation starts
✓ Autonomous lane following
✓ Camera-based lane detection
✓ Autonomous driver
✓ SLAM launch
✓ Nav2 launch
✓ Nav2 path following
✓ Collision Monitor configuration
✓ Dynamic obstacle detection workflow
✓ Nav2 alternate-path replanning
✓ Prius follows the alternate navigation path
```

---

## Known Development History

During development, several experimental nodes and launch files were
used while building and debugging the project.

The final repository contains the cleaned working architecture rather
than every experimental implementation.

The final `prius_adas_control` Python nodes are:

```text
lane_detection.py
autonomous_driver.py
nav2_cmd_to_prius.py
```

The final launch files are:

```text
nav2.launch.py
prius_adas_world.launch.py
slam.launch.py
```

Older experimental files were removed from the final working package
during repository cleanup.

Troubleshooting and development history remain documented separately
under `docs/`.

---

## Repository Content Policy

The Git repository intentionally excludes:

```text
build/
install/
log/
__pycache__/
*.pyc
*.gv
*.pdf
local Gazebo backup files
datasets/
```

The project dataset is not required for running the core simulation and
is therefore not included in this repository.

---

## Screenshots and Demonstration Media

Screenshots, videos, and demonstration images will be added to this
section after the repository documentation is finalized.

Planned media includes:

1. Gazebo Prius simulation
2. Autonomous lane following
3. Lane detection visualization
4. SLAM map
5. Nav2 planned path
6. Costmaps
7. RViz navigation
8. Dynamic obstacle scenario
9. Alternate Nav2 path
10. Prius following the alternate path

Planned directory:

```text
docs/images/
```

Suggested files:

```text
docs/images/
├── gazebo-prius.png
├── lane-following.png
├── lane-detection.png
├── slam-map.png
├── nav2-path.png
├── costmap.png
├── obstacle-detected.png
├── alternate-path.png
└── final-adas-demo.png
```

---

## Dataset

The project dataset is intentionally **not included** in this
repository.

The dataset is not required to build or run the core Prius ADAS
simulation.

The repository `.gitignore` excludes:

```text
datasets/
```

---

## References

The lane detection implementation was developed with reference to
classical lane recognition and tracking concepts, including image
preprocessing, region-of-interest processing, edge detection, and line
detection.

Reference:

<https://resources.altium.com/p/lane-recognition-and-tracking-nvidia-jetson-nano>

The reference is used as supporting material; the project
implementation is its own ROS 2 implementation.

---

## Future Work

Possible future development areas include:

- Improved lane detection robustness
- Additional sensor fusion
- More complex obstacle scenarios
- Improved vehicle dynamics
- Additional ADAS behaviors
- Better visualization and telemetry
- More automated testing
- Hardware deployment when the simulation system is sufficiently mature

---

## Author

**Vatsal Jha**

Prius ADAS simulation project using:

```text
ROS 2 Humble
Gazebo Classic
Nav2
SLAM Toolbox
RViz2
Python
OpenCV
```

---

## Project Status

The core simulated Prius ADAS workflow is operational.

The repository documents:

```text
Autonomous Driving
        +
SLAM
        +
Nav2
        +
Dynamic Obstacle Avoidance
        +
Prius Vehicle Control
```