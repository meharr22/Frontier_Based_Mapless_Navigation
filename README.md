# Autonomous Mapless Navigation using LiDAR

> A frontier-cluster-based autonomous navigation system that enables a robot to explore an unknown environment and reach a specified goal using only 2D LiDAR observations.

## Overview

This project implements a **mapless navigation planner** for a mobile robot operating in an unknown environment. The robot starts without any prior knowledge of the map and incrementally builds an internal occupancy map from LiDAR scans while simultaneously navigating towards a target location.

The navigation strategy combines **occupancy-grid mapping**, **frontier-based exploration**, **frontier clustering**, and **Breadth-First Search (BFS)** path planning to efficiently balance exploration and goal completion.



## Features

-  Real-time occupancy grid construction from LiDAR observations
-  Frontier detection for autonomous exploration
-  Frontier clustering to identify coherent unexplored regions
-  Utility-based cluster selection balancing exploration and travel cost
-  Breadth-First Search (BFS) for shortest-path planning over explored space
-  Adaptive replanning as new map information becomes available
-  Automatic transition from exploration to goal-directed navigation
-  Collision-free navigation


##  Navigation Pipeline

```text
LiDAR Scan
      │
      ▼
Occupancy Grid Update
      │
      ▼
Frontier Detection
      │
      ▼
Frontier Clustering
      │
      ▼
Cluster Selection
      │
      ▼
BFS Path Planning
      │
      ▼
Robot Movement
      │
      ▼
Repeat until Goal is Reached
```



##  Algorithm

The planner operates in two phases:

### 1. Exploration Phase

- Continuously updates an occupancy map from incoming LiDAR scans.
- Detects frontier cells separating explored and unexplored space.
- Groups neighbouring frontier cells into connected clusters.
- Selects the most promising cluster using a utility function based on:
  - Cluster size
  - Travel cost
  - Distance to the goal
- Plans the shortest path to the selected frontier using BFS.

### 2. Goal Navigation Phase

Once the goal becomes reachable through the discovered map, the planner immediately computes the shortest path using BFS and switches to goal-directed navigation.


##  Project Structure

```
Project_2_Navigation/
│
├── navigator.py      # Navigation planner
├── world.py          # Environment simulator
├── run.py            # Evaluation script
├── watch.py          # Visual simulator
├── README.md
└── WRITEUP.md
```



##  Technologies Used

- Python
- Breadth-First Search (BFS)
- Occupancy Grid Mapping
- Frontier-Based Exploration
- Graph Search
- Autonomous Robotics
- LiDAR-Based Navigation



##  Performance

The final implementation achieved:

| Metric | Result |
|---------|---------|
| Success Rate | **100%** |
| Mean Coverage | **85%** |
| Mean SPL | **0.42** |
| Collisions | **0** |
| Combined Score | **0.70** |



##  How to Run

Clone the repository

```bash
git clone https://github.com/your-username/Autonomous-Mapless-Navigation.git
cd Autonomous-Mapless-Navigation
```

Run evaluation

```bash
python run.py
```

Visualize robot navigation

```bash
python watch.py
```



##  Future Improvements

Potential extensions include:

- Wavefront Frontier Detection (WFD)
- Information-Gain-Based Frontier Selection
- A* Search for long-distance planning
- Multi-step Look-Ahead Planning
- Persistent Frontier Tracking
- Next Best View (NBV) exploration



##  Learning Outcomes

This project strengthened my understanding of:

- Autonomous exploration in unknown environments
- Occupancy grid mapping
- Frontier-based navigation
- Graph search algorithms
- Online path planning
- Robot decision making under partial observability


## Author
Mehar Arora
