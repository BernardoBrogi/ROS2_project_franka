# Industrial Robotics Course Project

This repository contains the project for the Industrial Robotics course. The system is built around the [Franka ROS 2 environment](https://github.com/frankarobotics/franka_ros2) and is focused on a pick-and-place task in simulation.

## Project Goal

The objective is to pick up a cube in simulation and place it autonomously inside a container using:

- Gazebo Ignition for the simulated environment
- MoveIt for motion planning and execution
- A simulated Intel RealSense RGB-D camera for perception

The robot must be able to detect the object, plan a grasp, pick the cube, and place it into the container without manual intervention.

## Simulation Setup

The simulated world includes:

- A Franka FR3 robotic arm
- A cube to be picked
- A container where the cube must be placed
- A simulated Intel RealSense RGB-D camera

The camera streams both RGB data and point cloud information. These data streams can be visualized through ROS 2 topics and in RViz.

## Assignment Requirements

Students are expected to implement the following components:

1. A node that reads the camera data and estimates the pose of the cube.
2. A grasping and manipulation pipeline that uses the estimated pose to pick the cube.
3. A placement policy that autonomously places the cube inside the container.
4. A Gazebo world containing the cube and the container.
5. A launch process where the cube spawns in a random reachable pose every time the simulation is started.

The random spawn pose must always remain reachable for the robot so the manipulation task can be completed reliably.

## Data and Visualization

The camera provides:

- RGB images
- Point clouds

These outputs are intended to support object recognition, pose estimation, and debugging. RViz can be used to inspect the camera feeds, point cloud data, and the robot scene.



