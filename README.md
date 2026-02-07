# Gauntlet Path Planning (ROS 2)

This repository contains the implementation for Assignment 1 (Week 1 & Week 2) of SC627, focusing on environment representation and classical path planning.

---

## Week 1: Digital Twin & Configuration Space

- Generated an occupancy grid map of the Gauntlet environment.
- Inflated obstacles using the Minkowski Sum concept to account for the robot’s physical radius and a safety margin.
- Published both the original map and inflated C-space map using a custom map publisher node.

Topics:
- `/map_stored`
- `/map_inflated`

---

## Week 2: Path Planning Algorithms

Two planners were implemented from scratch on the inflated occupancy grid:

### A* Planner
- Uses an 8-connected grid.
- Euclidean distance heuristic.
- Produces the shortest feasible path but often travels close to obstacles.

### GVD-Based Planner
- Uses a Brushfire algorithm to compute obstacle clearance.
- Modifies A* cost to bias paths toward higher clearance regions.
- Produces safer paths closer to the medial axis of free space.

The start and goal are specified in world coordinates (meters) and converted to grid coordinates internally.

Published paths:
- `/path_astar`
- `/path_gvd`

---

## How to Run

```bash
ros2 launch gauntlet_gazebo map_publisher.launch.py
ros2 launch gauntlet_gazebo planner_server.launch.py
