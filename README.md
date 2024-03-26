# ros-noetic-containerized
This is a development environment for ROS using Docker containers

# ROS Noetic Installation Guide for Ubuntu

This README provides instructions for installing ROS Noetic Ninjemys on Ubuntu systems. ROS Noetic is the final ROS 1 version targeted at the Ubuntu 20.04 (Focal Fossa) LTS release. 

## Prerequisites

- Ubuntu 20.04 (Focal Fossa)

## Step 1: Configure Ubuntu Repositories

Ensure your Ubuntu repositories are configured to allow "restricted," "universe," and "multiverse" components. You can set this up in the Software & Updates application under the "Ubuntu Software" tab.

## Step 2: Setup Your Sources.list

Setup your computer to accept software from packages.ros.org by running the following command:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Step 3: Set Up Your Keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

## Step 4: Installation

First, make sure your Debian package index is up-to-date:

```bash
sudo apt update
```

There are several ROS Noetic distributions available. Choose one that fits your needs:

- **Desktop-Full Install**: Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages

  ```bash
  sudo apt install ros-noetic-desktop-full
  ```

- **Desktop Install**: Everything in ROS-Base plus tools like rqt and rviz

  ```bash
  sudo apt install ros-noetic-desktop
  ```

- **ROS-Base**: Bare bones (no GUI tools)

  ```bash
  sudo apt install ros-noetic-ros-base
  ```

## Step 5: Environment Setup

Add ROS Noetic to your ROS environment by sourcing the setup script:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Dependencies for Building Packages

Install `rosdep` for easy dependency management:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Step 7: Create a ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## Additional Tools and Information

- **rosinstall**: A frequently used command-line tool in ROS that is distributed separately. It allows you to easily download many source trees for ROS packages with one command.

  Install it with:

  ```bash
  sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  ```

- For more details and troubleshooting, visit the [official ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

- For tutorial view the [Quickstart](./Quickstart.md) guide.
