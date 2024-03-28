# Use Ubuntu 20.04 LTS as the base image
FROM ubuntu:20.04

# Avoid prompts from apt during build
ARG DEBIAN_FRONTEND=noninteractive

# Set your desired timezone
ENV TZ=America/Los_Angeles

# Update and install necessary packages for ROS installation
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

# Create a new user 'ros' with sudo privileges and no password required for sudo
RUN useradd -m ros \
    && echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Switch to the new user before copying the script to ensure the user has ownership
USER ros

# Set the working directory
WORKDIR /home/ros

# Use /bin/bash as the default container command
CMD ["/bin/bash"]
