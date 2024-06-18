# Autonomous Warehouse Management System using ROS

This project implements an Autonomous Warehouse Management System using ROS Noetic, designed for autonomous navigation and object manipulation within a warehouse.

## Overview

- Autonomous navigation in a simulated warehouse
- Detection and manipulation of boxes
- Task coordination through an orchestration script

## Project Structure

- **Launch Files**: Start system nodes
- **Message Definitions**: Custom ROS messages
- **Source Code**: Core functionality scripts
- **World Files**: Simulation environment

## Key Components

- **Orchestrator.py**: Central control unit with a Tkinter GUI
- **BoxCarrier.py**: Handles box carrying
- **BoxMover.py**: Manages box movement logic
- **BoxPositionToRviz.py**: Visualizes box positions in RViz
- **StateMonitor.py**: Tracks robot state and returns to origin if idle

## Running the System
```
roslaunch turtlebot3_custom_warehouse_complete.launch
```
