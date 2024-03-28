[Back to Contents](../README.md)

# Using ROS with graphical apps

## Install XServer

First, you'll need to [download XServer](https://sourceforge.net/projects/vcxsrv/) for Windows and install it.

Once installed open `xlaunch.exe` and change the display number `0` as shown in the image below.

![xserver](images/image.png)

## ROS GUI apps

> :warninig: **Now to save some time, download an image with docker desktop installed. Otherwise, we would need to install the desktop features in the base image `ros:noetic` or our custom image which takes long time.**


First start a container with the following arguments to support graphic applications

> :warning: This will show an error because roscore it's not running yet.

```bash
docker run --rm --name ros-gui -e DISPLAY=host.docker.internal:0.0 -it osrf/ros:noetic-desktop
rosrun turtlesim turtlesim_node
```

Then start `roscore` from the same container

```bash
docker exec -it ros-gui bash

# inside the container
source opt/ros/noetic/setup.bash
roscore
```

---

[Previous: Use the devcontainer for Debugging](./06_Devcontainer.md) | [Next: Additional notes](./08_Additional_Notes.md)