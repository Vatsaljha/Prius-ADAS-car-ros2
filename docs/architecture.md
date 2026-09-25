# Prius ADAS Architecture

## 1. Project Overview

The Prius ADAS project is a ROS 2 Humble simulation running in
Gazebo Classic.

The project is organized into two main operating modes:

1. Autonomous lane following
2. SLAM + Nav2 autonomous navigation

The same Prius simulation environment is used by both modes.

---

# 2. Part 1 — Autonomous Lane Following

Part 1 provides basic autonomous driving using the Prius camera,
lane detection, and an autonomous driving node.

## 2.1 Node Flow

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