# Use Ubuntu 20.04 LTS as the base image
FROM ubuntu:20.04

# Avoid prompts from apt during build and set your desired timezone
ARG DEBIAN_FRONTEND=noninteractive

# Update and install curl and lsb-release before adding the ROS repository
RUN apt update && apt install -y \
    curl \
    lsb-release \
    gnupg

# Now that lsb-release is guaranteed to be installed, add the ROS repository
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic Base and examples
RUN apt update && apt install -y \
    ros-noetic-ros-base \
    ros-noetic-rospy-tutorials \
    ros-noetic-turtlesim 

# Dependencies for building packages
RUN apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential 

# Initialize rosdep
RUN apt install python3-rosdep && \
    rosdep init && \
    rosdep update

# Clean up
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a new user 'ros' with sudo privileges
RUN useradd -m ros && \
    echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ros && \
    chmod 0440 /etc/sudoers.d/ros

# Switch to the new user
USER ros

# Environment setup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Use sed to uncomment the force_color_prompt line in ~/.bashrc
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' /home/ros/.bashrc

# Create the Catkin Workspace
RUN mkdir -p /home/ros/catkin_ws/src

# Set working directory to ros user
WORKDIR /home/ros/catkin_ws

# Copy your entrypoint script into the container
COPY entrypoint.sh /usr/local/bin/entrypoint.sh

# Make sure the script is executable
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint script to run when the container starts
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command to run when a container starts
CMD ["/bin/bash"]