[Back to Contents](../README.md)

# ROS Noetic with Docker Containers

This example will guide you through creating two simple ROS nodes in Python: one node to publish the string "Hello, World" and another node to subscribe to this message and print it out. We will use ROS Noetic and have it installed on Ubuntu 20.04, this general approach is similar for other ROS versions.

## Setup your Containerized Environment

### 1. Create your ROS workspace image


This will create a Docker images with the following SW build steps:
- Image is based on Ubuntu 20.04 with all the **required dependencies to install and build ROS1 Noetic** and **creates a user with sudo privileges**.
- Image will install at build time ROS Noetic from the steps of the official page [documentation](http://wiki.ros.org/noetic/Installation/Ubuntu).
- At build time, image **configures the container** `entrypoint` and `source` the ROS global configuration (i.e. `/opt/ros/noetic/setup.bash`).

```bash
docker build -t ros-noetic-workspace . 
```

### Step 2. Test your worskpace image 

Create a user container from the custom image:

```bash
docker run --rm --name ros-master -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src ros-noetic-workspace
```

Alternatively, you can create a root container with ROS installed from the official Docker image. This would pull the image from the Docker Hub automatically instead of building our custom image.

> :warning: This approach is not followed on this tutorial, minor adjustments might be needed if you want to use this image

```bash
docker run --rm --name ros-master -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src -w /root ros:noetic
```

---

[Previous: ROS Noetic Native Installation Guide for Ubuntu](./00_ROS_Noetic_Ubuntu_Installation.md) | [Next: Create ROS Package](./02_Create_ROS_Package.md)
