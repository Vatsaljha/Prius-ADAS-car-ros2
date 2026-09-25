# Prius ADAS — ROS 2 Autonomous Driving Simulation

A ROS 2 Humble + Gazebo Classic project for autonomous Prius simulation with **camera-based lane following**, **SLAM + Nav2 navigation**, and **dynamic obstacle avoidance**.

## Overview

The project is divided into two main parts:

### Part 1 — Autonomous Lane Following

The Prius uses its camera to detect the lane and an autonomous driver to control the vehicle.

```text
Camera
   ↓
lane_detection
   ↓
Lane Information
   ↓
autonomous_driver
   ↓
/prius/control
   ↓
Prius
```

### Part 2 — ADAS Navigation

The second part uses LiDAR, SLAM Toolbox, Nav2, costmaps, Collision Monitor, and a custom Nav2-to-Prius adapter.

```text
LiDAR / Odometry
       ↓
   SLAM Toolbox
       ↓
      Map
       ↓
      Nav2
   ↙    ↓    ↘
Planner Controller Costmaps
        ↓
 Collision Monitor
        ↓
nav2_cmd_to_prius
        ↓
 /prius/control
        ↓
      Prius
```

## Key Features

- Autonomous camera-based lane following
- SLAM Toolbox integration
- Nav2 autonomous navigation
- Smac Hybrid-A* path planning
- Regulated Pure Pursuit path following
- Global and local costmaps
- Collision Monitor safety layer
- Dynamic obstacle detection through LiDAR
- Nav2 replanning around blocked routes
- Alternate-path following by the Prius
- Gazebo Classic simulation
- RViz2 visualization

## Project Workflow

### Lane Following

```text
Camera → Lane Detection → Autonomous Driver → Prius
```

### ADAS Navigation

```text
LiDAR → SLAM → Nav2 → Collision Monitor → Prius
```

### Obstacle Avoidance

```text
Obstacle
   ↓
LiDAR
   ↓
Costmap Update
   ↓
Nav2 Replanning
   ↓
Alternate Path
   ↓
Prius
```

## Main ROS 2 Nodes

| Node | Purpose |
|---|---|
| `lane_detection` | Camera-based lane detection |
| `autonomous_driver` | Autonomous lane-following control |
| `nav2_cmd_to_prius` | Converts Nav2 commands to Prius control |
| `prius_odom_bridge` | Prius odometry support |
| `laser_scan_merger` | LiDAR scan processing support |

## Main Launch Files

| Launch file | Purpose |
|---|---|
| `prius_adas_world.launch.py` | Starts Gazebo + Prius simulation |
| `slam.launch.py` | Starts SLAM/localization system |
| `nav2.launch.py` | Starts Nav2 navigation |

## How to Run

### Part 1 — Lane Following

**Terminal 1 — Gazebo**

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 launch prius_adas_control prius_adas_world.launch.py
```

**Terminal 2 — Lane Detection**

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 run prius_adas_control lane_detection
```

**Terminal 3 — Autonomous Driver**

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash

ros2 run prius_adas_control autonomous_driver
```

### Part 2 — SLAM + Nav2

**Terminal 1 — Gazebo**

```bash
ros2 launch prius_adas_control prius_adas_world.launch.py
```

**Terminal 2 — SLAM**

```bash
ros2 launch prius_adas_control slam.launch.py
```

**Terminal 3 — Nav2**

```bash
ros2 launch prius_adas_control nav2.launch.py
```

**Terminal 4 — RViz2**

```bash
rviz2
```

Before launching, source:

```bash
source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash
source ~/prius_adas_ws/install/setup.bash
```

## Dynamic Obstacle Avoidance

A navigation goal is sent through RViz2 and the Prius follows the generated Nav2 path.

When an obstacle is placed in the route:

1. LiDAR observes the obstacle.
2. The costmap marks the obstacle.
3. Nav2 replans the route.
4. An alternate path is generated.
5. The Prius follows the new path.

## Package Structure

```text
prius_adas_ws/
├── prius_adas_control/
│   ├── config/
│   ├── launch/
│   ├── models/
│   ├── prius_adas_control/
│   │   ├── autonomous_driver.py
│   │   ├── lane_detection.py
│   │   └── nav2_cmd_to_prius.py
│   ├── urdf/
│   └── worlds/
│
├── src/
│   └── prius_adas_navigation/
│       └── prius_adas_navigation/
│           ├── laser_scan_merger.py
│           └── prius_odom_bridge.py
│
├── tools/
│   ├── add_slam_barriers.py
│   └── clean_and_add_lidar.py
│
└── docs/
```

## Build

```bash
cd ~/prius_adas_ws

source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash

colcon build --symlink-install

source ~/prius_adas_ws/install/setup.bash
```

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- RViz2
- Nav2
- SLAM Toolbox
- Python
- OpenCV

## Demonstration

### Autonomous Lane Following

> Screenshots / video will be added here.

### SLAM + Nav2

> SLAM map, RViz, and Nav2 path screenshots will be added here.

### Dynamic Obstacle Avoidance

> Obstacle detection and alternate-path screenshots/video will be added here.

## Reference

Lane recognition and tracking concepts were studied with:

https://resources.altium.com/p/lane-recognition-and-tracking-nvidia-jetson-nano

## Documentation

Detailed project notes are available in:

- `docs/architecture.md`
- `docs/how-to-run.md`
- `docs/troubleshooting.md`
- `docs/findings.md`
- `docs/issue-log.md`
- `docs/reproduction.md`

## Project Status

✅ Autonomous lane following  
✅ SLAM + Nav2 navigation  
✅ Nav2 path following  
✅ Dynamic obstacle avoidance  
✅ Alternate-path following  

## Author

**Vatsal Jha**