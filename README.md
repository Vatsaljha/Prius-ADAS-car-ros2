# Prius ADAS — ROS 2 + Gazebo Classic

A ROS 2 Humble simulation of a Prius-style vehicle implementing
autonomous lane following and Nav2-based autonomous driving with
dynamic obstacle avoidance.

## Project Overview

This project has two main operating modes.

### Part 1 — Autonomous Lane Following

The Prius follows the road/lane using the camera-based lane detection system.

```text
Camera
   ↓
lane_detection
   ↓
lane information
   ↓
autonomous_driver
   ↓
/prius/control
   ↓
Prius