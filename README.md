# 🚗 Prius ADAS — Autonomous Driving & Obstacle Avoidance Simulation

A ROS 2 Humble based Prius ADAS and autonomous driving simulation built in Gazebo Classic.

This project combines camera-based lane perception, LiDAR-based obstacle sensing, SLAM/localization, Nav2 autonomous navigation, car-like path planning, path following, collision monitoring, and direct vehicle control into a complete simulated autonomous-driving system.

---

## 📌 Project Overview

The goal of this project is to simulate an autonomous Prius capable of:

- Understanding the road/lane using a camera
- Estimating lane geometry
- Localizing inside a mapped environment
- Planning a drivable path
- Following the planned path
- Detecting obstacles using LiDAR
- Reacting to changing obstacles
- Replanning when required
- Applying collision monitoring
- Converting navigation commands into Prius vehicle control
- Physically driving the simulated Prius inside Gazebo

The project is built around a **car-like Ackermann-steering vehicle** rather than treating the Prius as a differential-drive robot.

---

## 🎯 Project Objective

The overall autonomous-driving pipeline is:

```text
Environment
     ↓
Camera + LiDAR
     ↓
Perception
     ↓
Lane Understanding + Obstacle Perception
     ↓
Localization
     ↓
Path Planning
     ↓
Path Following
     ↓
Safety Monitoring
     ↓
Vehicle Control
     ↓
Prius