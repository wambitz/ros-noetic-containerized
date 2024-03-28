# Use Ubuntu 20.04 LTS as the base image
FROM ubuntu:20.04

# Avoid prompts from apt during build and set your desired timezone
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    sudo \
    lsb-release \
    gnupg2 \
    curl \
    tzdata \
    gcc \
    g++ \
    build-essential \
    # Set up the timezone
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    # Clean up
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a new user 'ros' with sudo privileges
RUN useradd -m ros && \
    echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ros && \
    chmod 0440 /etc/sudoers.d/ros

# Switch to the new user
USER ros

# Set the working directory
WORKDIR /home/ros

# Setup Your Sources.list
RUN echo "Setting up sources.list for ROS Noetic" && \
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set Up Your Keys
RUN echo "Adding ROS package keys" && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Ros Base Installation
RUN echo "Updating package index..." && \
    sudo apt-get update && \
    echo "Installing ROS Noetic Desktop Base..." && \
    # Uncomment the line below for the full desktop installation, or use the ros-base version as needed
    # apt-get install -y ros-noetic-desktop-full
    sudo apt-get install -y ros-noetic-ros-base

# Dependencies for Building Packages
RUN echo "Installing dependencies for building packages" && \
    sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
RUN sudo rosdep init && \
    rosdep update

# Set the ROS environment to be sourced upon every bash session
# NOTE!: ~/.bashrc commands will not work for non-interactive sessions (i.e. docker run -it ...)
RUN echo "Setting up ROS Noetic environment" && \
    echo "# NOTE! source ~/.bashrc will not work for non-interactive sessions" >> /home/ros/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/ros/.bashrc

# Cleanup
RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*    

# Create the Catkin Workspace
RUN mkdir -p /home/ros/catkin_ws/src

# Set working directory to the Catkin workspace
WORKDIR /home/ros/catkin_ws

# TODO: Verify if this hangs the devcontainer and why roscore
# it's not being launched
# Copy your entrypoint script into the container
# COPY entrypoint.sh /usr/local/bin/entrypoint.sh

# Make sure the script is executable
# RUN sudo chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint script to run when the container starts
# ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Add color to the bash session
RUN echo "PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/ros/.bashrc

# Add custom alias to .bashrc
RUN echo "alias ll='ls -la'" >> /home/ros/.bashrc
