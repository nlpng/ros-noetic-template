# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context
This is a ROS (Robot Operating System) 1 noetic package template designed to be reusable across robotics projects. The template uses C++17 with the catkin build system and runs inside Docker containers for consistent development environments.

## Core Architecture
- **ROS Framework**: ROS 1 noetic with catkin build system
- **Language**: C++17
- **Containerization**: Docker with ros:noetic-robot base image
- **Visualization**: Foxglove bridge for modern web-based data visualization
- **Workspace Structure**: Standard catkin workspace (workspace_folder/src/package_name/)

## Development Commands

### Docker Operations
```bash
# Build Docker image
docker build -t ros-noetic-template .

# Run with ROS networking (CRITICAL: always use --network host)
docker run --network host -it ros-noetic-template

# Multi-container setup
docker-compose up

# Start with Foxglove Studio
docker-compose --profile studio up
```

### Foxglove Visualization
```bash
# Access web-based Foxglove Studio
# Open https://studio.foxglove.dev and connect to ws://localhost:8765

# Run local Foxglove Studio container
docker-compose --profile studio up
# Access at http://localhost:8080
```

### ROS Package Development
```bash
# Build catkin workspace
catkin build

# Source the workspace
source devel/setup.bash

# Run ROS nodes
rosrun <package_name> <node_name>

# Launch files
roslaunch <package_name> <launch_file.launch>
```

## Key Development Principles

### ROS-Specific Patterns
- Use `ros::Timer` for periodic tasks, never sleep loops in callbacks
- Consider real-time constraints and message queue sizes
- Follow standard ROS package structure with CMakeLists.txt and package.xml
- Use appropriate queue sizes for publishers/subscribers based on message frequency

### Docker Best Practices
- ALWAYS use `--network host` for ROS communication between containers
- Install all dependencies in Dockerfile, never in running containers
- Rebuild Docker image when adding new dependencies
- Keep Docker images compact and optimized

### File Organization
- Maintain catkin workspace structure: `workspace_folder/src/package_name/`
- Keep components modular and decoupled
- Never create files longer than 500 lines - refactor into modules

## Task Management
- Always check and update TASK.md before starting work
- Mark completed tasks immediately in TASK.md
- Add discovered sub-tasks to "Discovered During Work" section

## Critical Notes
- This template is designed for general reuse across robotics projects
- Focus on modularity and simplicity for easy modification
- Always consider ROS message passing patterns and real-time constraints
- Use docker-compose for complex multi-node setups