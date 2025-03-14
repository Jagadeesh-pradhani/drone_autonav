# Drone Autonav

Drone Autonav is a ROS2 package designed for autonomous drone navigation in simulation. The package includes an A* path planner that processes occupancy grid data from a map, checks for collisions using a custom CollisionChecker, and modifies the planned path’s altitude in order to safely navigate around obstacles. A separate node publishes sequential goals for the drone. This package works in conjunction with a launch file that brings up RViz, Gazebo, controllers, and other necessary components.

## Package Structure
```
drone_autonav/
├── package.xml
├── setup.py
├── CMakeLists.txt
└── drone_autonav
    ├── __init__.py
    └── drone_autonav_node.py
    └── collision_checker.py
    └── goal.py
    └── node.py
└── launch
    ├── bringup.launch.py
    └── drone_autonav.launch.py
    └── uav.launch.launch.py

```


## Overview

The package consists of the following components:

- **A\* Path Planner (drone_autonav_node.py):**  
  - Converts the drone’s current position and the goal position into grid indices.
  - Uses the A* algorithm to compute a path through the 2D grid.
  - The heuristic used is the Euclidean distance between grid cells.
  - Once a coarse path is generated, it is densified via linear interpolation to create smooth waypoints.
  - A custom CollisionChecker (from `collision_checker.py`) evaluates whether the planned path is clear.  
  - If a collision is detected, the path altitude is immediately increased (using the parameter `avoid_height`) relative to the current drone altitude. Once the obstacle is passed (and the path is clear), the path is restored to the default altitude.
  - The final path is published as a `nav_msgs/Path` message on the topic `drone_path`.

- **Goal Publisher (goal.py):**  
  - Publishes a series of predefined goals (as `PoseStamped` messages) on the topic `/goal_pose`.
  - Listens for a signal (from `/goal_reached`) to move to the next goal.
  - This node sequentially sends goals for the planner to process.

- **Collision Checker & GridNode:**  
  - The `CollisionChecker` (in `collision_checker.py`) uses an `OccupancyGrid` map to determine whether a given node (converted using the helper `GridNode` class from `node.py`) is free of obstacles.
  - This is used by the planner to modify the path altitude if necessary.

## How A* is Used

The A* algorithm is implemented in the `plan_path()` method of `drone_autonav_node.py`:
  
1. **Grid Conversion:**  
   - The drone’s current and goal positions are converted into 2D grid indices using the `world_to_grid()` method.
  
2. **Search:**  
   - A priority queue (using Python’s `heapq`) is used to maintain a list of nodes to expand.
   - Each node is evaluated with a cost function \( f(n) = g(n) + h(n) \) where:
     - \( g(n) \) is the cost from the start to the current node.
     - \( h(n) \) is the Euclidean distance (heuristic) from the current node to the goal.
  
3. **Path Reconstruction:**  
   - Once the goal is reached, the path is reconstructed by following the recorded parent nodes from the goal back to the start.
  
4. **Path Densification:**  
   - The coarse grid path is then densified using linear interpolation (in the `densify_path()` method) to produce a smooth set of waypoints.

## Usage

### Build and Install

Place the `drone_autonav` package into the `src` folder of your ROS2 workspace (e.g., `~/ros2_ws/src/`). Then build your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select drone_autonav
```

Source the setup file:

```bash
source ~/ros2_ws/install/setup.bash
```

### Launching the System
You will need to open two terminals:

#### Terminal 1 – Bringup Launch
This terminal launches the complete system (RViz, Gazebo, controllers, etc.) using the `bringup.launch.py` file:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch drone_autonav bringup.launch.py
```

#### Terminal 2 – Send Goals
This terminal runs the goal publisher node:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run drone_autonav send_goal
```

The goal publisher will send sequential goals to the planner node.

## Code Organization

- **drone_autonav_node.py:** <br>
 Contains the main planning node that:
    - Subscribes to `/goal_pose`, `/clicked_point`, `/simple_drone/odom`, and `/projected_map`.
    - Uses the A* algorithm to compute and densify the path.
    - Checks the path for collisions using the `CollisionChecker` and modifies the path altitude if necessary.
    - Publishes the final path on the topic `drone_path`.

- **collision_checker.py:** <br>
     Implements a collision checker that uses an OccupancyGrid to determine if grid nodes are free of obstacles.
- **node.py:** <br>
    Provides the `GridNode` class which represents a node in the 2D grid for planning purposes.
- **goal.py:** <br>
     Publishes a series of goals (as `PoseStamped`) to be followed by the planner.

- **Launch Files (in launch folder):**
    - `bringup.launch.py`: Launches the full system including RViz, Gazebo, and controllers.
    - `drone_autonav.launch.py`: Launch file specific to the planner node.
    - `uav.launch.launch.py`: Launch file for UAV-related files.

## Summary
This package uses a structured approach to autonomous navigation by combining A* search for path planning, collision checking via an OccupancyGrid, and dynamic path modification (altering altitude) to avoid obstacles