# Use Ubuntu 20.04 as the base image
FROM ros-deps:latest

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Step 1: Configure Ubuntu Repositories
# This is generally not necessary in the Dockerfile as the base image should have these enabled
USER ros

# Step 2: Setup Your Sources.list
RUN echo "Setting up sources.list for ROS Noetic" && \
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Step 3: Set Up Your Keys
RUN echo "Adding ROS package keys" && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Step 4: Installation
RUN echo "Updating package index..." && \
    sudo apt-get update && \
    echo "Installing ROS Noetic Desktop Base..." && \
    # Uncomment the line below for the full desktop installation, or use the ros-base version as needed
    # apt-get install -y ros-noetic-desktop-full
    sudo apt-get install -y ros-noetic-ros-base

# Step 5: Environment Setup
# NOTE!: ~/.bashrc commands will not work for non-interactive sessions
RUN echo "Setting up ROS Noetic environment"

# Set the ROS environment to be sourced upon every bash session
RUN echo "# NOTE! source ~/.bashrc will not work for non-interactive sessions" >> /home/ros/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/ros/.bashrc

# Step 6: Dependencies for Building Packages
RUN echo "Installing dependencies for building packages" && \
    sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
RUN sudo rosdep init && \
    rosdep update

# Cleanup
RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*        

# Informative message (this won't actually execute as part of the build; just for documentation)
RUN echo "ROS Noetic installation is complete! Please close and reopen your terminal or source your ~/.bashrc to use ROS Noetic."

# Set the working directory
WORKDIR /home/ros

# Use /bin/bash as the default container command
CMD ["/bin/bash"]