# Industrial Robotics Course Project

This repository contains the project for the Industrial Robotics course. The system is built around the [Franka ROS 2 environment](https://github.com/frankarobotics/franka_ros2) and is focused on a pick-and-place task in simulation.

## Installation

Install the **Development Tools** package:
  ```bash
  sudo apt install ros-dev-tools
  ```
**Clone the Repositories:**
   ```bash
   git clone https://github.com/frankarobotics/franka_ros2.git src
   ```
**Install the dependencies**
  ```bash
  vcs import src < src/dependency.repos --recursive --skip-existing
  ```
**Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
**Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
**Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
  ```


## Project Goal

The objective is to pick up a cube in simulation and place it autonomously inside a container using:

- Gazebo Ignition for the simulated environment
- MoveIt for motion planning and execution
- A simulated Intel RealSense RGB-D camera for perception

The robot must be able to detect the object, plan a grasp while avoiding collisions with scene objects, pick the cube, and place it into the container without manual intervention.

## Simulation Setup

The simulated world includes:

- A Franka FR3 robotic arm
- A cube to be picked
- A container where the cube must be placed
- A static obstacle positioned between the robot and the cube
- A simulated Intel RealSense RGB-D camera

The camera streams both RGB data and point cloud information. These data streams can be visualized through ROS 2 topics and in RViz.

## Assignment Requirements

Students are expected to implement the following components:

1. A node that reads the camera data and estimates the pose of the cube.
2. A grasping and manipulation pipeline that uses the estimated pose to pick the cube.
3. Motion planning that avoids collisions with an obstacle placed between the robot and the cube.
4. A placement policy that autonomously places the cube inside the container.
5. A Gazebo world containing the cube, the container, and the obstacle.
6. A launch process where the cube spawns in a random reachable pose every time the simulation is started.

The random spawn pose must always remain reachable for the robot so the manipulation task can be completed reliably, while still requiring the planned trajectory to avoid the obstacle.

## Suggested Approach for Obstacle Avoidance

As a suggested implementation strategy, students can use the MoveIt perception pipeline with OctoMap to build a 3D occupancy map from camera data and feed it to the planner for collision-aware trajectory generation.

Reference tutorial:

- https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html

## Data and Visualization

The camera provides:

- RGB images
- Point clouds

These outputs are intended to support object recognition, pose estimation, and debugging. RViz can be used to inspect the camera feeds, point cloud data, and the robot scene.

## Simulation Run Examples

Use this section to document example commands for starting the simulation, launching the robot stack, and opening RViz.

### Start the full simulation

```bash
ros2 launch franka_gazebo_bringup moveit_gazebo_franka_arm_example_controller.launch.py
```

## Perception Topics

Use this section to list the ROS 2 topics used for perception and visualization.

### Camera Topics

- RGB image topic: `TBD`
- Depth image topic: `TBD`
- Point cloud topic: `TBD`

### Suggested RViz Displays

- Camera image
- Point cloud
- Robot model
- TF frames



