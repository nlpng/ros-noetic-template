# Project Planning: ROS noetic package template with Docker container and catkin package build

## Project Overview
We're building a simple ROS (Robotic Operating System) 1 noetic package template. The ROS package template should be general enough to be used for other projects as well with simple modifications. The ROS package will be built inside docker container, the function of the package is to launch a simple ROS 1 node that subscribe to topics and publish topics. This ROS package template using C++17 with catkin build system. Always consider ROS-specific patterns, message passing, and real-time constraints when writing code.

## Current Implementation Status
The template includes:
- Complete ROS node with publisher/subscriber patterns
- Docker containerization with host networking
- Foxglove bridge integration for visualization
- Configurable launch parameters
- Timer-based periodic operations

## Architecture

### Core Components:
1. **Docker Environment**
   - Base image: ros:noetic-robot
   - Network: Host networking (network_mode: host) for ROS communication
   - Single container setup with docker-compose
   - Image tag: ros-template:noetic

2. **ROS Package Pattern**
   - Catkin build system with C++17 standard
   - Package name: ros_template_node
   - Node implementation with publishers, subscribers, and timers
   - Launch file with configurable parameters
   - Foxglove bridge integration for visualization
   - Current workspace structure:
    ```
    catkin_ws/                 -- WORKSPACE
        src/                   -- SOURCE SPACE
            CMakeLists.txt     -- Top-level catkin cmake
            ros_template_node/ -- Main package
                CMakeLists.txt -- Build configuration
                package.xml    -- Package manifest
                include/ros_template_node/
                    template_node.h  -- Node header
                src/
                    template_node.cpp -- Node implementation
                launch/
                    template_node.launch -- Launch configuration
    ```

3. **Node Functionality**
   - Publishers: /template_output (std_msgs/String), /cmd_vel (geometry_msgs/Twist)
   - Subscribers: /template_input (std_msgs/String), /scan (sensor_msgs/LaserScan)
   - Timers: Publish timer (heartbeat), Status timer (logging)
   - Foxglove bridge for web-based visualization

### Technology Stack:
- **Language**: C++17
- **ROS Framework**: ROS 1 noetic
- **ROS build system**: catkin
- **Docker base image**: ros:noetic-robot
- **Visualization**: Foxglove bridge with WebSocket support
- **Container orchestration**: Docker Compose with host networking

## Development Process

The development follows a task-based approach. Current implementation status:

1. ✅ **ROS Package Setup**: Complete catkin workspace with ros_template_node package
2. ✅ **Dockerfile Configuration**: Multi-stage build with ros:noetic-robot base
3. ✅ **Docker Compose**: Single service configuration with host networking
4. ✅ **Node Implementation**: Publishers, subscribers, timers, and message handling
5. ✅ **Launch Configuration**: Parameterized launch file with Foxglove integration
6. ✅ **Visualization**: Foxglove bridge for web-based monitoring

## Current Limitations
- Single container setup (no multi-container orchestration)
- Basic testing framework (tests commented out in CMakeLists.txt)
- No GUI tools integration or profiles in docker-compose
- Limited parameter validation in node implementation

## Design Principles

1. **Modularity**: Keep components decoupled for easier maintenance
2. **Simplicity**: Focus on making the system easy to understand and modify
3. **Optimization**: Optimize the Docker image size to be compact

## Notes

When implementing this project, make sure to:
- Mark tasks complete in the **TASK.md** file as you finish them
- Always consider real-time constraints in robotics applications
- Use appropriate queue sizes for publishers/subscribers based on message frequency
- Consider using docker-compose for complex multi-node setups
- REMEMBER to rebuild Docker image when adding new dependencies
- ALWAYS use --network host for ROS communication between containers
- ALWAYS commit the changes with compact and concise messages

When implementing this project, make sure **NOT** to:
 - DON'T install packages inside running containers (use Dockerfile)
