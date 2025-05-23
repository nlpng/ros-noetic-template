# ROS Noetic Template Node

A comprehensive, reusable ROS (Robot Operating System) 1 noetic package template designed for easy integration into robotics projects. This template demonstrates best practices for ROS node development, Docker containerization, and multi-node communication patterns.

## 🚀 Features

- **Complete ROS Package Structure**: Follows standard catkin workspace organization
- **C++17 Implementation**: Modern C++ with proper memory management and RAII patterns
- **Docker Containerization**: Ready-to-use Docker setup with ros:noetic-robot base
- **Multi-Container Support**: Docker Compose configuration for complex deployments
- **Comprehensive Communication**: Demonstrates publisher/subscriber patterns with multiple message types
- **Timer-Based Operations**: Uses `ros::Timer` for periodic tasks (ROS best practice)
- **Configurable Parameters**: Runtime parameter configuration through launch files
- **Production Ready**: Includes proper error handling, logging, and graceful shutdown

## 📋 Prerequisites

- Docker and Docker Compose
- (Optional) ROS Noetic for native development
- (Optional) X11 forwarding for GUI tools

## 🛠️ Quick Start

### Using Docker (Recommended)

1. **Clone and build**:
   ```bash
   git clone <your-repo-url>
   cd ros-noetic-template
   docker-compose up --build
   ```

2. **Test the system**:
   ```bash
   # In another terminal, check running containers
   docker-compose ps
   
   # View node output
   docker-compose logs -f template_node
   
   # Send test messages
   docker-compose exec template_node rostopic pub -1 /template_input std_msgs/String "data: 'Test message'"
   ```

### Using Native ROS Installation

1. **Setup workspace**:
   ```bash
   cd catkin_ws
   catkin build
   source devel/setup.bash
   ```

2. **Run the node**:
   ```bash
   roslaunch ros_template_node template_node.launch
   ```

## 🏗️ Architecture

### Package Structure
```
catkin_ws/
├── src/
│   ├── CMakeLists.txt          # Top-level catkin cmake
│   └── ros_template_node/      # Main package
│       ├── package.xml         # Package manifest
│       ├── CMakeLists.txt      # Build configuration
│       ├── include/ros_template_node/
│       │   └── template_node.h # Node header
│       ├── src/
│       │   └── template_node.cpp # Node implementation
│       └── launch/
│           └── template_node.launch # Launch configuration
```

### Node Functionality

The template node demonstrates:

- **Publishers**:
  - `/template_output` (std_msgs/String): Processed messages and heartbeats
  - `/cmd_vel` (geometry_msgs/Twist): Example velocity commands

- **Subscribers**:
  - `/template_input` (std_msgs/String): Echo and process input messages
  - `/scan` (sensor_msgs/LaserScan): Process laser scan data

- **Timers**:
  - Publish Timer: Sends periodic heartbeat messages
  - Status Timer: Logs node status and statistics

## ⚙️ Configuration

### Launch Parameters

Configure the node behavior through launch file arguments:

```bash
roslaunch ros_template_node template_node.launch \
  publish_rate:=2.0 \
  status_rate:=0.5 \
  input_topic:=/my_input \
  output_topic:=/my_output
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate` | double | 1.0 | Frequency of heartbeat messages (Hz) |
| `status_rate` | double | 0.2 | Frequency of status logging (Hz) |
| `input_topic` | string | /template_input | Input topic name |
| `output_topic` | string | /template_output | Output topic name |
| `cmd_vel_topic` | string | /cmd_vel | Velocity command topic |
| `laser_topic` | string | /scan | Laser scan topic |

## 🐳 Docker Usage

### Single Container

```bash
# Build the image
docker build -t ros-noetic-template .

# Run with ROS networking
docker run --network host -it ros-noetic-template

# Run with custom parameters
docker run --network host -it ros-noetic-template \
  roslaunch ros_template_node template_node.launch publish_rate:=5.0
```

### Multi-Container Setup

```bash
# Start complete system
docker-compose up

# Start with GUI tools (requires X11)
xhost +local:docker  # Allow Docker to connect to X server
docker-compose --profile gui up

# Scale specific services
docker-compose up --scale subscriber_example=3

# Interactive debugging
docker-compose exec template_node bash
```

## 🔧 Development

### Building from Source

1. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   sudo apt install python3-catkin-tools
   ```

2. **Build workspace**:
   ```bash
   cd catkin_ws
   catkin config --cmake-args -DCMAKE_CXX_STANDARD=17
   catkin build
   ```

3. **Run tests**:
   ```bash
   catkin test ros_template_node
   ```

### Customization

To adapt this template for your project:

1. **Rename the package**:
   - Update `package.xml` name and description
   - Modify `CMakeLists.txt` project name
   - Rename directories and files as needed

2. **Modify message types**:
   - Update includes in the header file
   - Modify callback functions for your data types
   - Adjust publishers/subscribers accordingly

3. **Add custom logic**:
   - Implement your algorithms in the callback functions
   - Add new timers for specific periodic tasks
   - Include additional ROS dependencies in `package.xml`

## 🐛 Troubleshooting

### Common Issues

**Container networking problems**:
```bash
# Ensure proper ROS networking
docker run --network host -it ros-noetic-template
```

**Build failures**:
```bash
# Clean and rebuild
catkin clean
catkin build
```

**Permission issues with GUI**:
```bash
xhost +local:docker
```

### Debugging

**Check ROS connectivity**:
```bash
# List active nodes
rosnode list

# Check topic activity
rostopic list
rostopic hz /template_output

# Monitor node graph
rqt_graph
```

**Container debugging**:
```bash
# Access running container
docker-compose exec template_node bash

# Check logs
docker-compose logs template_node

# Resource usage
docker stats
```

## 📚 Learning Resources

- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Catkin Build System](http://wiki.ros.org/catkin)
- [ROS Best Practices](http://wiki.ros.org/BestPractices)
- [Docker for Robotics](https://roboticseabass.com/2021/04/21/docker-and-ros/)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Follow the existing code style
4. Add tests for new functionality
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🏷️ Version History

- **v1.0.0**: Initial template with basic pub/sub functionality
- Container support with Docker and Docker Compose
- C++17 implementation with modern ROS patterns

---

**Note**: This template is designed to be a starting point. Customize it according to your specific robotics application requirements.