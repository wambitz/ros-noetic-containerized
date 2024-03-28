#!/bin/bash

# ROS Noetic Installation Script for Ubuntu 20.04 (Focal Fossa)

# Step 1: Configure Ubuntu Repositories
# Make sure your Ubuntu is configured to allow "restricted," "universe," and "multiverse."
# You can manually check this in the Software & Updates application.

# Step 2: Setup Your Sources.list
echo "Setting up sources.list for ROS Noetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Step 3: Set Up Your Keys
echo "Adding ROS package keys"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Step 4: Installation
echo "Updating package index..."
sudo apt update 

echo "Installing ROS Noetic Desktop Full..."
# sudo apt install -y ros-noetic-desktop-full

# If you want a different installation, comment out the above line and uncomment one of the following lines
# sudo apt install -y ros-noetic-desktop
sudo apt install -y ros-noetic-ros-base

# Step 5: Environment Setup
echo "Setting up ROS Noetic environment"
echo "This will work only for interactive sessions only" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 6: Dependencies for Building Packages
echo "Installing dependencies for building packages"
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "Initializing rosdep"
sudo rosdep init
rosdep update

echo "ROS Noetic installation is complete!"
echo "Please close and reopen your terminal or source your ~/.bashrc to use ROS Noetic."
