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

- **Nav2** handles navigation and dynamic obstacle avoidance.
- **Mission Manager** handles task sequencing, safety logic, and observability.

---

## Key Features

- Route-based navigation (aisle-following behavior)
- Warehouse layout defined using YAML (no hardcoded paths)
- Multi-stage mission execution
- Dynamic obstacle handling via Nav2
- Recovery monitoring (spin, backup, clear costmap)
- Odometry-based stuck detection
- Near-goal tolerance handling (prevents false mission aborts)
- CSV-based mission performance logging
- Simulation-to-real robot ready architecture

---

## Repository Structure

```

autonomous_warehouse_mission/
├── autonomous_warehouse_mission/
│   ├── warehouse_delivery_mission.py  # Mission manager logic
│   ├── station_recorder.py             # Record station positions from RViz
│
├── config/
│   └── warehouse.yaml                  # Warehouse layout, routes, mission
│
├── launch/
│   └── warehouse_mission.launch.py     # Launch file
│
├── README.md
├── package.xml
├── setup.py
└── setup.cfg

````

---

## Warehouse Configuration (YAML)

The warehouse is defined entirely using a YAML file:

- **Stations** – semantic locations (dock, aisles, stations)
- **Routes** – ordered sequences of stations
- **Mission** – ordered list of routes
- **Behavior** – safety timeouts and tolerances

Example:

```yaml
stations:
  dock: [0.0, 0.0, 0.0]
  aisle_1: [1.6, 0.0, 0.0]
  station_a: [2.0, 1.6, 1.57]

routes:
  dock_to_station_a:
    - dock
    - aisle_1
    - station_a

mission:
  - dock_to_station_a
````

This design allows warehouse layouts to be changed **without modifying code**.

---

## How the System Works

1. The mission manager loads the warehouse configuration.
2. Each route is executed waypoint-by-waypoint.
3. Goals are sent to Nav2 using the `NavigateToPose` action.
4. Nav2 handles planning, control, and obstacle avoidance.
5. The mission manager:

   * monitors odometry for progress
   * monitors Nav2 recovery behaviors
   * applies a near-goal grace period to handle goal tolerances
6. Results are logged to a CSV file for offline analysis.

---

### Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy installed and sourced
- Nav2 installed
- Gazebo Harmonic installed

Verify ROS 2:
```bash
ros2 --help
````

---

### Running the Project (Simulation)

### Step 1 – Clone the Repository

```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src

git clone https://github.com/PrathamRathod14/autonomous-warehouse-robot.git
```

---

### Step 2 – Build the Workspace

```bash
cd ~/nav2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

### Step 3 – Start Nav2 Simulation

Open **Terminal 1**:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py use_sim_time:=true
```

This launches:

* Gazebo simulation
* Nav2 stack
* RViz visualization

---

### Step 4 – Run Warehouse Mission

Open **Terminal 2**:

```bash
source ~/nav2_ws/install/setup.bash

ros2 launch autonomous_warehouse_mission warehouse_mission.launch.py \
  warehouse_yaml:=config/warehouse.yaml \
  csv_log_path:=~/warehouse_mission_log.csv
```

The robot will:

* Follow predefined warehouse routes
* Navigate through aisles
* Handle obstacles using Nav2
* Log mission results to a CSV file

---

### Step 5 – View Mission Logs (Optional)

```bash
cat ~/warehouse_mission_log.csv
```

---

## Recording Stations for Real Deployment

For real robot deployment:

1. Build a map using SLAM.
2. Open RViz and enable the **Publish Point** tool.
3. Run:

```bash
ros2 run autonomous_warehouse_mission station_recorder
```

4. Click on the map to record station coordinates.
5. Paste the generated values into `warehouse.yaml`.

This enables fast adaptation to new warehouse environments.

---

## Learning Outcomes

* Mission-level autonomy design
* ROS 2 action-based navigation
* Understanding Nav2 goal tolerances and recoveries
* Safety watchdog implementation for mobile robots
* Simulation-to-real robot workflows
* Professional ROS 2 package structuring

---

## Limitations and Future Work

* The system assumes a static warehouse map.
* Multi-robot coordination is not implemented.
* Speed-control and keep-out zones can be added for improved safety.
* Future work includes deployment on a real AMR platform (e.g., Clearpath Ridgeback).

## Project Demo

<a href="https://drive.google.com/file/d/1NjuqkEP2FNwurKIU7ik4aA0wbjLFLlMx/preview" target="_blank">
  <img src="https://img.shields.io/badge/▶-Watch%20Demo%20Video-red?style=for-the-badge" />
</a>
