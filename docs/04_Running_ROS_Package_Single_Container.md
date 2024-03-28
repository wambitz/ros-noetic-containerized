[Back to Contents](../README.md)

# Running ROS package with a Single Container

Now, you can run your ROS nodes inside a single container. Open three new separate terminal windows or tabs for the `ros-master`, `publisher` and `subscriber` nodes. You will need to attach to the running container from different terminals.

First you need to run `roscore`, you can create a new one like this if you don't have one already running:

```bash
docker run --rm --name ros-master -it ros-workspace roscore
```

For the listener, in one of the tabs connect to the `ros-master` container using the container name or `CONTAINER_ID`:

```bash
docker exec -it ros-master bash # or <CONTAINER_ID> instead of the name
source devel/setup.bash
rosrun hello_world_pkg listener.py
```

Likewise, do the same for the talker: 

```bash
docker exec -it ros-master bash  # or <CONTAINER_ID> instead of the name
source devel/setup.bash
rosrun hello_world_pkg talker.py
```

**Troubleshooting**

If the following line is not in `~/.bashrc` or `source ~/.bashrc`, run this in the command line and try again:

```bash
source /opt/ros/noetic/setup.bash
```

---

[Previous: Running ROS Package Natively](./03_Running_ROS_Package_Natively.md) | [Next: Running ROS package with a Multiple Containers](./05_Runnning_ROS_Package_Multiple_Containers.md)