# ROS Noetic Template Dockerfile
# This Dockerfile creates a containerized environment for the ROS template node
# Base image: ros:noetic-robot (includes ROS noetic with robot-specific packages)

FROM ros:noetic-robot

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/catkin_ws

# Install additional system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools and utilities
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    # ROS development tools
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # C++17 support and debugging tools
    gcc-9 \
    g++-9 \
    gdb \
    valgrind \
    # Network and communication tools
    net-tools \
    iputils-ping \
    # Foxglove bridge for modern visualization
    ros-noetic-foxglove-bridge \
    # Clean up to reduce image size
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Set up C++17 as default compiler
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Create catkin workspace
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy the ROS package source code
COPY catkin_ws/src/ $CATKIN_WS/src/

# Initialize rosdep and install dependencies
RUN rosdep init || true \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $CATKIN_WS && \
    catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 && \
    catkin build"

# Set up the workspace environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS environment\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
source $CATKIN_WS/devel/setup.bash\n\
\n\
# Execute the command\n\
exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

# Expose common ROS ports (optional, for documentation)
EXPOSE 11311 11312 11313 11314 11315

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command: run the template node
CMD ["roslaunch", "ros_template_node", "template_node.launch"]

# Metadata
LABEL maintainer="ROS Template Developer <developer@example.com>"
LABEL description="ROS Noetic Template Node - A reusable containerized ROS package"
LABEL version="1.0.0"
LABEL ros.distro="noetic"
LABEL ros.package="ros_template_node"