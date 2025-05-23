# Project Planning: ROS noetic package template with Docker container and catkin package build

## Project Overview
We're building a simple ROS (Robotic Operating System) 1 noetic package template. The ROSpackage template should be general enough to be used for other projects as well with simple modifications. The ROS package will be built inside docker container, the function of the package is to launch a simple ROS 1 node that subscribe to topics and publish topics. This ROS package template using C++17 with catkin build system. Always consider ROS-specific patterns, message passing, and real-time constraints when writing code.

## Architecture

### Core Components:
1. **Docker Environment**
   - Base image: Use ros:noetic-robot as base.
   - Network: Use --network host for ROS communication between containers.
   - Development: Use docker-compose for multi-container setups.

2. **ROS Package Pattern**
   - Use the catkin build system.
   - Build catkin package standalone. 
   - Timers: Use ros::Timer for periodic tasks, not sleep loops
   - Using the following catkin workspace to build the ROS package
    ```
    workspace_folder/          -- WORKSPACE
        src/                   -- SOURCE SPACE
            CMakeLists.txt     -- 'Toplevel' CMake file, provided by catkin
            package_1/
            CMakeLists.txt     -- CMakeLists.txt file for package_1
            package.xml        -- Package manifest for package_1
    ```

### Technology Stack:
- **Language**: C++17
- **ROS Framework**: ROS 1 noetic
- **ROS build system**: catkin
- **Docker base image**: ros:noetic-robot

## Development Process

The development will follow a task-based approach where each component will be implemented sequentially. We should:

1. Start by preparing the ROS package directory that will later be copied into the Docker container
2. Setting up the Dockerfile to host the ROS package build
3. Build the Dockerfile
4. Launch the ROS node from the Dockerfile

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

When implementing this project, make sure **NOT** to:
 - DON'T install packages inside running containers (use Dockerfile)
