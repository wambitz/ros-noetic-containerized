[Back to Contents](../README.md)

# Additional notes

The images `ros-workspace` help to run `ROS Noetic` commands without any additional configuration. :warning: 

These are the manual steps to achieve the same results if there was no entrypoint with the [official Docker image](https://hub.docker.com/_/ros/tags?page=1&name=noetic)  from the Docker hub

> :warning: These steps assume you have `ros-network` created already

Manual steps:

1. Start ROS Master
    ```bash
    docker run --rm \
            --name ros-master \
            --network ros-network \
            -it \
            ros:noetic \
            bash -c "source /opt/ros/noetic/setup.bash && roscore"
    ```

2. Start ROS Listener:
    ```bash
    docker run --rm \
            --name ros-listener \
            --network ros-network \
            -it \
            -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src \
            -e ROS_MASTER_URI=http://ros-master:11311 \
            ros:noetic \
            bash -c "source /opt/ros/noetic/setup.bash && \
                        catkin_make && \
                        source devel/setup.bash && \
                        rosrun hello_world_pkg listener.py"
    ```


3. Start ROS Talker:
    ```bash
    docker run --rm \
            --name ros-talker \
            --network ros-network \
            -it \
            -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src \
            -e ROS_MASTER_URI=http://ros-master:11311 \
            ros:noetic \
            bash -c "source /opt/ros/noetic/setup.bash && \
                        catkin_make && \
                        source devel/setup.bash && \
                        rosrun hello_world_pkg talker.py"
    ```

---

[Previous: Using ROS with graphical apps](./07_ROS_GUI_Apps.md)
