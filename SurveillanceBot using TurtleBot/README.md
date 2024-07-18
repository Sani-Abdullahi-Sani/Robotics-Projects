# SurveillanceBot Project

## Introduction

In this project, we developed a SurveillanceBot using a TurtleBot to explore, map, and navigate within a given environment. The primary tasks included mapping the environment, navigating to specified coordinates, and detecting a green utility cart.

## System Setup

- **Environment:** ROS Kinetic, Gazebo, RViz
- **Mapping Tool:** GMapping SLAM

## The Mapping Process

### Launch Environment

- Open Gazebo world.
- Launch GMapping node.
- Open RViz for visualization.
- Use Kobuki Keyop for navigation.

### Parameter Tuning

We created a new launch file for gmapping, adjusting parameters and recompiling the assignment workspace accordingly. Some changes include:
- `MinimumScore`: 100000 (as per the assignment instructions)
- `angularUpdate`: 0.436
- `resampleThreshold`: 0.5

This adjustment noticeably enhanced mapping accuracy and efficiency. Previously, the robot frequently lost its localization, but after tuning, it maintains localization for longer periods. This improvement ensures high-quality scans and a reliable map.

## Localization and Navigation

- We used "2D Pose Estimate" in RViz to align the robot’s simulated position with the map.
- We tested robot navigation using the "2D Nav Goal" in RViz to play around with the robot.

## Path Planning with RRT

### Node Implementation

The node was designed to read target coordinates from standard input and used the Rapidly-Exploring Random Tree (RRT) algorithm and map generated for path planning.

### Path Planning Process

For each node to be added, the path planning algorithm checks if that point is free in the map and then also checks if there is a valid path from the new node to the nearest node already present in the tree. If there is a valid path, the node is added to the tree. This continues until a node which has a valid path to the goal is found. Both this node and the goal node are then added to the tree. From the goal, parents of nodes are traced back to the current position node; this path is then reversed, becoming the planned path.

`move_base` is utilized for local navigation using the planned path, guiding the robot along the route while ensuring obstacle avoidance. To improve path optimality, we limited the distance of new nodes from their nearest nodes in the tree.

## Object Detection - Green Utility Cart

We integrated the camera by subscribing to the `/camera/rgb/image_raw` topic to access the robot’s raw image data. The images were converted to OpenCV format and then transformed from BGR to HSV color space for easier color detection. To detect the green utility cart, we defined an HSV range for green and counted the green pixels. If the count exceeded 1000, it indicated the cart’s presence. The detection results were published to the `/witsdetector` topic, outputting "Yes" for detection and "No" otherwise.

## Conclusion

In this project, we developed a SurveillanceBot capable of mapping, navigating, and detecting a green utility cart in an environment. Key learnings included the importance of parameter tuning, effective SLAM and navigation tools, and robust path planning and obstacle detection algorithms.

---
