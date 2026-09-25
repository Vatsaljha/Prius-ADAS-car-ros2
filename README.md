# 🚗 Prius ADAS — ROS 2 + Gazebo Classic

> Autonomous Prius ADAS simulation using ROS 2 Humble, Gazebo Classic, SLAM Toolbox, Nav2, camera-based lane detection, LiDAR, and dynamic obstacle avoidance.

## 🎥 Demo

<!-- Add your final project image here -->

![Prius ADAS Demo](docs/images/final-adas-demo.png)

## ✨ Features

- 🛣️ Autonomous camera-based lane following
- 📷 Camera lane detection
- 🧭 SLAM Toolbox localization / mapping support
- 🗺️ Nav2 autonomous navigation
- 🧠 Smac Hybrid-A* path planning
- 🎯 Regulated Pure Pursuit path following
- 🚧 LiDAR-based obstacle detection
- 🔄 Nav2 replanning around obstacles
- 🛑 Collision Monitor safety layer
- 🚗 Prius Ackermann control interface
- 🌐 RViz2 visualization
- 🧪 Gazebo Classic simulation

## 🏗️ System Overview

### Part 1 — Autonomous Lane Following

```text
Camera
  ↓
lane_detection
  ↓
autonomous_driver
  ↓
/prius/control
  ↓
Prius
```

### Part 2 — SLAM + Nav2 ADAS

```text
LiDAR + Odometry
      ↓
SLAM Toolbox
      ↓
Map / Localization
      ↓
Nav2
 ┌────┼───────────────┐
 │    │               │
 ↓    ↓               ↓
Smac  Costmaps   Pure Pursuit
 │    │               │
 └────┴──────┬────────┘
             ↓
     Collision Monitor
             ↓
     nav2_cmd_to_prius
             ↓
       /prius/control
             ↓
           Prius
```

## 🚧 Dynamic Obstacle Avoidance

When an obstacle blocks the current route:

```text
Obstacle
   ↓
LiDAR
   ↓
Costmap update
   ↓
Nav2 replanning
   ↓
Alternate path
   ↓
Prius follows new path
```

## 📦 ROS 2 Packages

### `prius_adas_control`

Main package containing the lane-following nodes, Gazebo world launch,
SLAM launch, Nav2 launch, configuration, URDF, and world assets.

### `prius_adas_navigation`

Supporting navigation package containing:

- `prius_odom_bridge`
- `laser_scan_merger`

The Prius world launch starts `prius_odom_bridge` as part of the
simulation setup. fileciteturn0file3L58-L74 fileciteturn0file0L10-L20

## 🚀 How to Run

### 1. Build

```bash
cd ~/prius_adas_ws

source /opt/ros/humble/setup.bash
source ~/prius_ws/install/setup.bash

colcon build --symlink-install
source ~/prius_adas_ws/install/setup.bash
```

### 2. Part 1 — Lane Following

**Terminal 1**

```bash
ros2 launch prius_adas_control prius_adas_world.launch.py
```

**Terminal 2**

```bash
ros2 run prius_adas_control lane_detection
```

**Terminal 3**

```bash
ros2 run prius_adas_control autonomous_driver
```

### 3. Part 2 — SLAM + Nav2

**Terminal 1**

```bash
ros2 launch prius_adas_control prius_adas_world.launch.py
```

**Terminal 2**

```bash
ros2 launch prius_adas_control slam.launch.py
```

**Terminal 3**

```bash
ros2 launch prius_adas_control nav2.launch.py
```

**Terminal 4**

```bash
rviz2
```

## 🔧 Main Components

| Component | Role |
|---|---|
| `lane_detection` | Detects lane information from the camera |
| `autonomous_driver` | Drives the Prius using lane information |
| `prius_odom_bridge` | Provides Prius odometry support |
| `laser_scan_merger` | Supporting LiDAR scan processing |
| `nav2_cmd_to_prius` | Converts Nav2 commands to Prius control |
| SLAM Toolbox | Localization / mapping |
| Smac Hybrid-A* | Global path planning |
| Regulated Pure Pursuit | Path following |
| Collision Monitor | Final collision-safety layer |

The Nav2 launch connects the controller through the navigation command
chain, while `nav2_cmd_to_prius` converts navigation commands for the
Prius control interface. fileciteturn0file4L24-L44 fileciteturn0file5L21-L36

## 🖥️ Software

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Nav2
- SLAM Toolbox
- RViz2
- Python
- OpenCV

## 📁 Project Structure

```text
prius_adas_ws/
├── README.md
├── prius_adas_control/
│   ├── config/
│   ├── launch/
│   ├── models/
│   ├── prius_adas_control/
│   ├── urdf/
│   └── worlds/
├── src/
│   └── prius_adas_navigation/
├── docs/
└── tools/
```

## 📸 Screenshots

Add your final project screenshots here:

| Lane Following | SLAM + Nav2 |
|---|---|
| `docs/images/lane-following.png` | `docs/images/nav2-path.png` |

| Obstacle Avoidance | Final Simulation |
|---|---|
| `docs/images/obstacle-detected.png` | `docs/images/final-adas-demo.png` |

## 📚 Documentation

- [Architecture](docs/architecture.md)
- [How to Run](docs/how-to-run.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Findings](docs/findings.md)
- [Issue Log](docs/issue-log.md)
- [Reproduction Notes](docs/reproduction.md)

## 📌 Project Status

✅ Autonomous lane following

✅ SLAM + Nav2 navigation

✅ Nav2 path following

✅ Dynamic obstacle detection

✅ Alternate-path replanning

✅ Prius control integration

## 📖 Reference

Lane detection development was informed by lane recognition and tracking
concepts from:

<https://resources.altium.com/p/lane-recognition-and-tracking-nvidia-jetson-nano>

## 👤 Author

**Vatsal Jha**

ROS 2 Humble • Gazebo Classic • Nav2 • SLAM Toolbox • RViz2