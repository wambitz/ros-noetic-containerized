[Back to Contents](../README.md)

# ROS Noetic with Docker Containers

This example will guide you through creating two simple ROS nodes in Python: one node to publish the string "Hello, World" and another node to subscribe to this message and print it out. We will use ROS Noetic and have it installed on Ubuntu 20.04, this general approach is similar for other ROS versions.

**NOTE**: Step 3 to 7 can be done natively if needed. The only difference is that 3 terminals will be required and no need to connect to any container.

## Setup your Containerized Environment

### 1. Create your ROS workspace image


This will create 3 Docker images: 1) Firs image is based on Ubuntu 20.04 with all the **required dependencies to install and build ROS1 Noetic** and **creates a user with sudo privileges**. 2) Second image will install ROS Noetic from the steps of the official page [documentation](http://wiki.ros.org/noetic/Installation/Ubuntu). Lastly the 3) third image **configures the container** `entrypoint` and `source` the ROS global configuration (i.e. `/opt/ros/noetic/setup.bash`).

A general note, the order on how this images are created is important since they interdependent.

```bash
docker build -t ros-workspace . 
```

### Step 2. Test your worskpace image 

Create a user container from the custom image:

```bash
docker run --rm --name ros-master -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src ros-workspace
```

Create a root container with ROS installed from the official Docker image.

> :warning: This approach is not followed on this tutorial, minor adjustments might be needed if you want to use this image

```bash
docker run --rm --name ros-master -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src -w /root ros:noetic
```

---

[Previous: ROS Noetic Native Installation Guide for Ubuntu](./00_ROS_Noetic_Ubuntu_Installation.md) | [Next: Create ROS Package](./02_Create_ROS_Package.md)
