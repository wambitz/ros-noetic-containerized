[Back to Contents](../README.md)

# Running ROS packages with Multiple Containers

## Using docker compose:

The easiest way is to used `docker compose` like this:

Run 
```bash
docker compose up
```

Stop
```bash
docker compose down
```

## Create a network and communicate through separate containers

If you rather to use `docker run` and run all the steps manually you can follow the next steps, also for your convience the python scripts `build`, `run` and `clean` take care for this manual steps.

> :warning: **This needs to be run from the root directory of this project**


With the custom image, first create a network 

```bash
docker network create ros-network
```

> :warning: This only works because of the the entrypoiny. Look at the [Additional Notes](./07_Additional_Notes.md) for further details.

Then create a master container:

```bash
docker run --rm --name ros-master --network ros-network -it ros-workspace roscore
```

Create a listener container: 

```bash
docker run --rm --name ros-listener --network ros-network -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src -e ROS_MASTER_URI=http://ros-master:11311 ros-workspace "catkin_make && source devel/setup.bash && rosrun hello_world_pkg listener.py"
```

Create a talker container:

```bash
docker run --rm --name ros-talker --network ros-network -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src -e ROS_MASTER_URI=http://ros-master:11311 ros-workspace "catkin_make && source devel/setup.bash && rosrun hello_world_pkg talker.py"
```

---

[Previous: Running ROS package with a Single Container](./04_Running_ROS_Package_Single_Container.md) | [Next: Use the devcontainer for Debugging](./06_Devcontainer.md)
