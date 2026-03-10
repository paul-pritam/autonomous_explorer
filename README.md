# Autonomous Frontier Exploration with ROS2 Nav2

This project implements an **autonomous exploration system for mobile robots using ROS2 and Nav2**.
The robot automatically explores an unknown environment by detecting **frontiers (boundaries between known and unknown space)** and navigating to them until the entire map is explored.

The system integrates:

* **ROS2 Navigation Stack (Nav2)**
* **Custom A* Global Planner**
* **Pure Pursuit Controller**
* **Frontier-based exploration algorithm**
* **Behavior Tree navigation**
* **Autonomous goal generation**

The robot continuously analyzes the map, detects unexplored regions, selects the most promising frontier, and navigates toward it.

---

# System Architecture

```
SLAM / Mapping
      │
      ▼
  Occupancy Grid (/map)
      │
      ▼
Explorer Node (Frontier Detection)
      │
      ▼
Goal Selection
      │
      ▼
Nav2 NavigateToPose Action
      │
      ▼
Behavior Tree
      │
      ▼
Planner → Controller → Robot Motion
```

---

# Features

* Frontier-based autonomous exploration
* Custom global planner using **A***
* Custom **Pure Pursuit** path follower
* Behavior Tree navigation pipeline
* Dynamic frontier selection using scoring
* Blacklisting of failed goals
* Automatic recovery behaviors
* Costmap clearing and replanning

---

# Frontier Exploration Algorithm

The explorer node performs the following steps:

1. Receive **Occupancy Grid Map**
2. Detect **frontier cells**
   (unknown cells adjacent to free space)
3. Cluster frontier cells into **frontier regions**
4. Compute centroid of each frontier
5. Score frontiers using

```
score = frontier_size / distance
```

6. Select the **best frontier**
7. Send navigation goal using Nav2
8. If navigation fails → add to blacklist
9. Repeat until no frontiers remain

---

# Navigation Stack Configuration

## Planner

Custom **A*** planner plugin.

```
plugin: my_auto_nav_pkg/AstarPlanner
```

Features:

* Grid-based search
* obstacle avoidance
* costmap integration

---

## Controller

Custom **Pure Pursuit Controller**

```
plugin: my_auto_nav_pkg/PurePursuitController
```

Parameters:

| Parameter          | Value     |
| ------------------ | --------- |
| desired_linear_vel | 0.3 m/s   |
| lookahead_dist     | 0.5 m     |
| max_lookahead_dist | 1.0 m     |
| max_angular_vel    | 1.0 rad/s |

---

# Behavior Tree

Navigation is managed using a **custom Behavior Tree**.

Main pipeline:

```
ComputePathToPose
        ↓
FollowPath
```

Recovery behaviors:

1. Wait
2. Clear costmaps
3. Replan
4. Spin in place

Tree Structure

```
RecoveryNode
 ├── PipelineSequence
 │     ├── ComputePathToPose
 │     └── FollowPath
 │
 └── RecoveryActions
       ├── Wait
       ├── ClearCostmaps
       └── Spin / Replan
```

---

# Explorer Node

The `ExplorerNode` is responsible for:

* Receiving the map
* Detecting frontiers
* Selecting navigation goals
* Sending goals to Nav2
* Handling navigation failures

Important components:

```
/map subscription
TF lookup (map → base_link)
Frontier detection
Goal scoring
NavigateToPose action client
```

Key safety mechanisms:

* **Goal validation**
* **Free-space checking**
* **Blacklist failed goals**
* **Fallback recovery**

---

# Costmaps

## Global Costmap

Layers:

* Static Layer
* Obstacle Layer
* Inflation Layer

Inflation radius:

```
0.35 meters
```

---

## Local Costmap

Configuration:

```
rolling_window: true
size: 3m x 3m
resolution: 0.05
```

Uses laser scans for obstacle detection.

---

# Running the System

## 1. Build Workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 2. Launch Navigation + Explorer

```
ros2 launch autonomous_explorer explorer_launch.py
```

This will start:

* Nav2 Navigation stack
* Custom planner
* Pure pursuit controller
* Explorer node

---

# Required Topics

| Topic   | Description        |
| ------- | ------------------ |
| `/map`  | Occupancy grid map |
| `/scan` | Laser scan data    |
| `/odom` | Robot odometry     |
| `/tf`   | Transform tree     |

---

# Dependencies

Required ROS2 packages:

```
nav2_bringup
nav2_msgs
nav2_costmap_2d
tf2_ros
OpenCV
rclcpp
rclcpp_action
```

---

# Exploration Completion

Exploration stops when:

```
No frontiers are detected
```

The robot prints:

```
No frontiers found. Exploration might be complete!
```

---

# Future Improvements

Possible improvements:

* Information gain based frontier scoring
* Multi-robot exploration
* Dynamic obstacle avoidance
* Semantic exploration
* GPU frontier detection
* Better frontier clustering

---

# Project Structure

```
autonomous_explorer/
│
├── include/
│   └── explorer_node.hpp
│
├── src/
│   └── explorer_node.cpp
│
├── config/
│   ├── nav2_params.yaml
│   └── simple.xml
│
├── launch/
│   └── explorer_launch.py
│
└── README.md
```

---

# Author

**Pritam Paul**

Robotics | ROS2 | Autonomous Navigation | Embedded Systems

