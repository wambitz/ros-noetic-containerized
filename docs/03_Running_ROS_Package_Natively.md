[Back to Contents](../README.md)

# Running ROS Packages Natively

Start a ROS Master in a terminal instance:

```bash
roscore
```

From your `catking_ws`, open two new terminal windows or tabs. In the first one, run the publisher node:

```bash
source devel/setup.bash
rosrun hello_world_pkg listener.py
```

In the second one, run the subscriber node:

```bash
source devel/setup.bash
rosrun hello_world_pkg talker.py
```

You should see the subscriber node printing the "Hello, world" messages with timestamps that the publisher node sends.

**Troubleshooting**

If is not in `~/.bashrc` or `source ~/.bashrc` does not work, do this and try again:

```bash
source /opt/ros/noetic/setup.bash
```

---

[Previous: Create ROS package](./02_Create_ROS_Package.md) | [Next: Running ROS package with a Single Container](./04_Running_ROS_Package_Single_Container.md)