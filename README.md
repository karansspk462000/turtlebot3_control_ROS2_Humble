# TurtleBot3 Control with ROS 2

This repository contains a script to control the TurtleBot3 in a Gazebo simulation using ROS 2.

## Prerequisites

1. ROS 2 installed (tested with [**ROS 2 Humble**](https://docs.ros.org/en/humble/Installation.html).
2. [TurtleBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) installed.
3. [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install) installed.

Make sure that the following environment variables are set:

```bash
export TURTLEBOT3_MODEL=burger  # or waffle, depending on your model
```
Run the following command to launch Gazebo with an empty world

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

After launching the Gazebo simulation, run the control script using the following command
```bash
ros2 run turtlebot3_control turtlebot3_control_node
```

## YouTube Video Tutorial
For more tutorials and content, [visit my YouTube Channel](https://www.youtube.com/@allaboutrobo) and subscribe!

## Acknowledgments
ROS 2 Documentation
TurtleBot3 e-Manual
Gazebo Tutorials
