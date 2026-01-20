# Autonomous Warehouse Robot (ROS 2 Nav2)

## Overview
This project implements a **mission-level autonomous navigation system**
for a warehouse robot using **ROS 2 Jazzy** and **Nav2**.

Instead of sending a single navigation goal, the robot follows **predefined
warehouse routes** composed of aisle waypoints and stations (dock, loading,
delivery points). A custom Python mission manager coordinates navigation,
monitors safety, and logs mission performance.

The system is designed to be **simulation-to-real-robot ready**: only the
map and configuration files need to change for deployment on real hardware.

---

## System Architecture

